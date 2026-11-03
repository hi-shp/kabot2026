"""Shared exit path for actuator programs; keep ROS alive until neutral ACK."""
import signal
import sys
import time


class _PersistentConsole:
    """A disconnected SSH terminal must not turn a log write into node failure."""
    def __init__(self, stream):
        self.stream = stream

    def write(self, text):
        try:
            return self.stream.write(text)
        except OSError:
            return len(text)

    def flush(self):
        try:
            return self.stream.flush()
        except OSError:
            pass

    def __getattr__(self, name):
        return getattr(self.stream, name)


def protect_console():
    for name in ('stdout', 'stderr'):
        stream = getattr(sys, name)
        if stream is not None and not isinstance(stream, _PersistentConsole):
            setattr(sys, name, _PersistentConsole(stream))


def keep_terminal_loss():
    protect_console()
    if hasattr(signal, 'SIGHUP'):
        signal.signal(signal.SIGHUP, signal.SIG_IGN)


class ExitSignals:
    """Repeated Ctrl+C requests stop without interrupting neutral cleanup."""
    def __init__(self, ctrl_c_only=False):
        protect_console()
        self.requested = False
        self.previous = {signal.SIGINT: signal.signal(signal.SIGINT, self._request),
                         signal.SIGTERM: signal.signal(signal.SIGTERM, signal.SIG_IGN if ctrl_c_only else self._request)}
        # SSH/terminal loss is not an operator stop. USB watchdog still neutralizes
        # the MCU whenever fresh commands cannot reach it.
        if hasattr(signal, 'SIGHUP'):
            self.previous[signal.SIGHUP] = signal.signal(signal.SIGHUP, signal.SIG_IGN)

    def _request(self, signum, frame):
        self.requested = True

    def restore(self):
        for signum, handler in self.previous.items():
            signal.signal(signum, handler)


def neutralize(node, paired_clients=None):
    """Stop/disarm MCU, or send paired neutral to a legacy endpoint. Bounded wait.

    Caller must first stop producing commands and drain in-flight callbacks.
    This function owns spinning during cleanup; no executor may spin concurrently.
    """
    import rclpy
    from std_srvs.srv import Trigger, SetBool
    from mechaship_interfaces.srv import ThrottlePulseWidth

    def wait(futures, seconds):
        until = time.monotonic() + seconds
        while rclpy.ok() and not all(f.done() for f in futures) and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.02)
        return all(f.done() and not f.cancelled() and f.exception() is None
                   and f.result() is not None for f in futures)

    stop = node.create_client(Trigger, '/actuators/stop')
    disarm = node.create_client(SetBool, '/actuators/arm')
    try:
        if stop.wait_for_service(timeout_sec=.6):
            for _ in range(2):
                future = stop.call_async(Trigger.Request())
                if wait([future], .8) and future.result().success:
                    return True
        if disarm.service_is_ready():
            wait([disarm.call_async(SetBool.Request(data=False))], .4)
        if paired_clients is not None:
            futures = [c.call_async(ThrottlePulseWidth.Request(pulse_width=1500))
                       for c in paired_clients]
            return wait(futures, .8) and all(f.result().status for f in futures)
        return False
    except Exception as exc:
        print('STOP_UNCONFIRMED: ' + str(exc), flush=True)
        return False
    finally:
        node.destroy_client(stop)
        node.destroy_client(disarm)
