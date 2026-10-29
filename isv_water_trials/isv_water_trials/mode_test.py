"""Finite RC/AUTO handover test. AUTO requests 10% of the 400us PWM span."""
import argparse
import json
import math
import signal
import sys
import time


class ModeGate:
    def __init__(self):
        self.mode = None
        self.epoch = 0
        self.received = None
        self.neutral_since = None
        self.active_since = None

    def observe(self, payload, now):
        age = payload.get('age_s')
        if (payload.get('receiving') is not True or payload.get('error') is not None
                or type(age) not in (int, float) or not math.isfinite(age) or not 0 <= age < .3
                or payload.get('rc_valid') is not True or payload.get('estop_active') is not False
                or type(payload.get('mode')) is not int or payload['mode'] not in (1, 2)
                or type(payload.get('armed')) is not bool):
            raise RuntimeError('Invalid/stale Arduino status or RC signal; restart test after checking connection')
        mode = payload['mode']
        if mode != self.mode:
            self.epoch += 1
            self.neutral_since = None
            self.active_since = None
        self.mode, self.received = mode, now - age
        if mode == 1:
            self.neutral_since = None
            self.active_since = None
            return
        if self.active_since is None and payload['armed']:
            raise RuntimeError('AUTO already armed by another controller; stop other command programs')
        if self.active_since is not None and now - self.active_since > .3 and not payload['armed']:
            raise RuntimeError('AUTO disarmed unexpectedly; test will not rearm after a fault')
        neutral = all(type(payload.get(key)) is int and abs(payload[key] - 1500) <= 40
                      for key in ('rc_left_us', 'rc_right_us'))
        if not neutral:
            self.neutral_since = None
        elif self.neutral_since is None:
            self.neutral_since = now

    def action(self, now):
        if self.received is None or now - self.received >= .3:
            raise RuntimeError('Arduino telemetry stopped; test aborted')
        if self.mode == 1:
            return 'rc'
        if self.active_since is not None:
            return 'drive'
        return 'arm' if self.neutral_since is not None and now - self.neutral_since >= .5 else 'neutral'

    def armed(self, now):
        self.active_since = now


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=60., help='Total time including RC/waiting (1..600)')
    parser.add_argument('--side', choices=('both', 'left', 'right'), default='both')
    args = parser.parse_args(argv)
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 600:
        parser.error('--seconds must be 1..600')
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String
    from std_srvs.srv import SetBool
    from mechaship_interfaces.srv import ThrottlePulseWidth

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('rc_auto_mode_test')
    gate = ModeGate()
    fault = []
    stopped = False
    prior_handlers = {}
    def stop(signum, frame):
        nonlocal stopped
        stopped = True
    for signum in (signal.SIGINT, signal.SIGTERM):
        prior_handlers[signum] = signal.signal(signum, stop)
    def receive(message):
        try:
            gate.observe(json.loads(message.data), time.monotonic())
        except (ValueError, TypeError, AttributeError, RuntimeError) as exc:
            fault.append(str(exc))
    subscription = node.create_subscription(String, '/actuators/status', receive, 10)
    arm = node.create_client(SetBool, '/actuators/arm')
    clients = [node.create_client(ThrottlePulseWidth, '/actuators/throttle_' + side + '/set_pulse_width')
               for side in ('left', 'right')]
    pwm = (1540 if args.side in ('left', 'both') else 1500,
           1540 if args.side in ('right', 'both') else 1500)

    def check():
        if fault:
            raise RuntimeError(fault[0])
        gate.action(time.monotonic())

    def wait(futures, timeout=.3, cleanup=False):
        until = time.monotonic() + timeout
        while not all(f.done() for f in futures) and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.01)
            if not cleanup:
                check()
                if stopped:
                    return False
        return all(f.done() and f.exception() is None and f.result() is not None for f in futures)

    def set_arm(value, cleanup=False):
        future = arm.call_async(SetBool.Request(data=value))
        return wait([future], .6 if cleanup else .3, cleanup) and future.result().success

    def drive():
        # Concurrent requests are paired into one atomic MCU frame by the bridge.
        futures = [client.call_async(ThrottlePulseWidth.Request(pulse_width=value))
                   for client, value in zip(clients, pwm)]
        return wait(futures) and all(f.result().status for f in futures)

    failed = False
    try:
        print('AUTO test: left=%dus right=%dus; %.1fs total. RC always has priority.' % (*pwm, args.seconds), flush=True)
        until = time.monotonic() + 10.
        while not stopped and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.05)
            if fault:
                raise RuntimeError(fault[0])
            if gate.received is not None and all(c.service_is_ready() for c in [arm, *clients]):
                break
        if stopped:
            return
        check()
        if not all(c.service_is_ready() for c in [arm, *clients]):
            raise RuntimeError('Start bringup with use_arduino:=true first')
        end = time.monotonic() + args.seconds
        announced = None
        while not stopped and time.monotonic() < end:
            tick = min(end, time.monotonic() + .1)
            rclpy.spin_once(node, timeout_sec=.01)
            check()
            action = gate.action(time.monotonic())
            if action != announced:
                print({'rc': 'RC: receiver controls outputs; test waits.',
                       'neutral': 'AUTO: neutral; hold BOTH sticks at 1500 +/-40us for 0.5s.',
                       'arm': 'AUTO: neutral confirmed; arming.',
                       'drive': 'AUTO: sending 10% PWM at 10Hz.'}[action], flush=True)
                announced = action
            if action in ('arm', 'drive'):
                epoch = gate.epoch
                if action == 'arm':
                    # Mark before spinning so our own arm telemetry is accepted.
                    gate.armed(time.monotonic())
                    ok = set_arm(True)
                else:
                    ok = drive()
                # An in-flight command can be rejected in RC before the next
                # 10Hz ROS mode report arrives. Wait briefly for that report.
                until = time.monotonic() + .15
                while not ok and not stopped and gate.epoch == epoch and time.monotonic() < until:
                    rclpy.spin_once(node, timeout_sec=.01)
                    check()
                if gate.epoch != epoch:
                    continue  # RC took over: never revive the old in-flight command.
                if not ok and not stopped:
                    raise RuntimeError('Actuator request rejected/timed out; test stopped')
            while not stopped and time.monotonic() < tick:
                rclpy.spin_once(node, timeout_sec=.01)
                check()
    except (RuntimeError, OSError) as exc:
        failed = True
        print('TEST_STOPPED: ' + str(exc), file=sys.stderr, flush=True)
    finally:
        # Disarm neutralizes AUTO and deliberately leaves live RC untouched.
        try:
            ok = arm.service_is_ready() and set_arm(False, cleanup=True)
            print('AUTO disarmed; RC remains available.' if ok else
                  'STOP_UNCONFIRMED: no disarm ACK; MCU command watchdog is 400ms.', flush=True)
        finally:
            node.destroy_node()
            rclpy.shutdown()
            for signum, handler in prior_handlers.items():
                signal.signal(signum, handler)
    if failed:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
