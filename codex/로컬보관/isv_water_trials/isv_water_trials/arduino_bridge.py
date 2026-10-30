"""Opt-in bridge for kabot_actuator firmware; never armed on startup.

Legacy left/right requests MUST be concurrent, as in the water trial runner.
The paired frame is atomic. No timer repeats stale propulsion commands.
"""
import argparse
from dataclasses import asdict
import json
import os
import secrets
import threading
import time
from .actuator_protocol import command, status, raw_status
from .shutdown import ExitSignals


class SerialTransport:
    def __init__(self, port):
        import serial
        self.serial = serial.Serial(port=None, baudrate=115200, timeout=.02, write_timeout=.05,
                                    **({'exclusive': True} if os.name == 'posix' else {}))
        self.serial.dtr = False
        self.serial.rts = False
        self.serial.port = port
        self.serial.open()  # Some USB boards still reset on open: connect only after wiring verification.
        self.lock = threading.Lock()
        self.condition = threading.Condition()
        self.latest = None
        self.raw = None
        self.raw_at = 0.
        self.received_at = 0.
        self.error = None
        self.sequence = secrets.randbelow(65536)
        self.closed = threading.Event()
        self.reader = threading.Thread(target=self.read, daemon=True)
        self.reader.start()

    def read(self):
        buffer = bytearray()
        dropping = False
        try:
            while not self.closed.is_set():
                for value in self.serial.read(64):
                    if value == 10:
                        if not dropping:
                            try:
                                decoded = status(bytes(buffer) + b'\n')
                            except ValueError:
                                try:
                                    decoded_raw = raw_status(bytes(buffer) + b'\n')
                                except ValueError:
                                    pass
                                else:
                                    with self.condition:
                                        self.raw = decoded_raw
                                        self.raw_at = time.monotonic()
                            else:
                                with self.condition:
                                    self.latest = decoded
                                    self.received_at = time.monotonic()
                                    self.condition.notify_all()
                        buffer.clear()
                        dropping = False
                    elif not dropping:
                        if len(buffer) >= 63:
                            dropping = True
                        else:
                            buffer.append(value)
        except Exception as exc:
            with self.condition:
                self.error = str(exc)
                self.condition.notify_all()

    def reset_controller(self):
        # UNO reset after ESC power is available, as required by the Basic ESC
        # Arduino startup sequence. Do this once per USB connection, not per arm.
        self.serial.dtr = True
        time.sleep(.05)
        self.serial.dtr = False

    def exchange(self, operation, left=1500, right=1500):
        # Do not queue an old propulsion request behind another in-flight transaction.
        if not self.lock.acquire(timeout=.3 if operation in ('S','R') else .03):
            return False
        try:
            self.sequence = (self.sequence + 1) & 65535
            sequence = self.sequence
            sent = time.monotonic()
            try:
                self.serial.write(command(sequence, operation, left, right))
                with self.condition:
                    deadline = sent + .22
                    while not self.error and not self.closed.is_set():
                        m = self.latest
                        if m is not None and self.received_at >= sent and m.sequence == sequence:
                            if operation == 'S':
                                return not m.armed and (m.left,m.right)==(1500,1500)
                            if operation == 'R':
                                return not m.stop_latched and not m.armed
                            if operation in ('D', 'I'):
                                return not m.armed
                            return m.mode == 2 and m.armed and not m.estop_active and (m.left, m.right) == (left, right)
                        remaining = deadline - time.monotonic()
                        if remaining <= 0:
                            break
                        self.condition.wait(remaining)
                # Invalidate a possibly applied but unacknowledged command. Hardware also times out.
                self.sequence = (self.sequence + 1) & 65535
                self.serial.write(command(self.sequence, 'D'))
            except Exception as exc:
                self.error = str(exc)
            return False
        finally:
            self.lock.release()

    def snapshot(self):
        with self.condition:
            return self.latest, time.monotonic() - self.received_at, self.error

    def raw_snapshot(self):
        with self.condition:
            return self.raw, time.monotonic() - self.raw_at

    def close(self):
        acknowledged = False
        try:
            acknowledged = self.exchange('S') or self.exchange('S')
        finally:
            self.closed.set()
            self.reader.join(timeout=.2)
            self.serial.close()
        return acknowledged


class ReconnectingTransport:
    """Keep ROS endpoints alive across USB failures; never queue/replay commands."""
    def __init__(self, port, log, retry_delay=1., reset_on_connect=True):
        self.port, self.log, self.retry_delay = port, log, retry_delay
        self.reset_on_connect = reset_on_connect
        self.lock = threading.Lock()
        self.current = None
        self.error = 'Waiting for Arduino USB'
        self.closed = threading.Event()
        self.worker = threading.Thread(target=self.reconnect, daemon=True)
        self.worker.start()

    def reconnect(self):
        last_message = None
        while not self.closed.is_set():
            transport = None
            try:
                transport = SerialTransport(self.port)
                if self.reset_on_connect:
                    transport.reset_controller()
                deadline = time.monotonic() + 4.
                while not self.closed.is_set() and not transport.exchange('I'):
                    if transport.snapshot()[2] or time.monotonic() >= deadline:
                        raise OSError(transport.snapshot()[2] or 'No firmware initialization ACK')
                    self.closed.wait(.05)
                if self.closed.is_set():
                    break
                with self.lock:
                    self.current = transport
                    self.error = None
                self.log('Arduino connected/reset; MCU holds calibrated ESC neutral for 7s, then accepts fresh output. Logical command neutral remains 1500us.')
                last_message = None
                while not self.closed.wait(.1):
                    _, age, error = transport.snapshot()
                    if error or age >= 1.:
                        raise OSError(error or 'Arduino telemetry timeout')
            except Exception as exc:
                message = str(exc)
                with self.lock:
                    self.error = message
                if not self.closed.is_set() and message != last_message:
                    self.log('Arduino offline; node/services stay running and USB will reconnect: ' + message)
                    last_message = message
            finally:
                with self.lock:
                    self.current = None
                if transport is not None:
                    try:
                        transport.close()
                    except Exception:
                        pass
            self.closed.wait(self.retry_delay)

    def exchange(self, operation, left=1500, right=1500):
        with self.lock:
            transport = self.current
        if self.closed.is_set() or transport is None:
            return False
        _, age, error = transport.snapshot()
        if operation in ('A', 'C') and (error or age >= .3):
            return False
        return transport.exchange(operation, left, right)

    def snapshot(self):
        with self.lock:
            transport, error = self.current, self.error
        return transport.snapshot() if transport else (None, float('inf'), error)

    def raw_snapshot(self):
        with self.lock:
            transport = self.current
        return transport.raw_snapshot() if transport else (None, float('inf'))

    def close(self):
        with self.lock:
            transport = self.current
        acknowledged = bool(transport and transport.exchange('S'))
        self.closed.set()
        self.worker.join(timeout=6.)
        return acknowledged


class Pairer:
    def __init__(self, send, log):
        self.send, self.log = send, log
        self.lock = threading.Lock()
        self.pending = {}

    def submit(self, side, pwm):
        if side not in ('left', 'right') or type(pwm) is not int or not 1100 <= pwm <= 1900:
            return False
        request = {'pwm': pwm, 'at': time.monotonic(), 'done': threading.Event(), 'ok': False}
        with self.lock:
            for key, old in list(self.pending.items()):
                if key == side or request['at'] - old['at'] > .10:
                    old['done'].set()
                    del self.pending[key]
            self.pending[side] = request
            pair = self.pending.copy() if len(self.pending) == 2 else None
            if pair:
                self.pending.clear()
        if pair:
            l, r = pair['left']['pwm'], pair['right']['pwm']
            self.log('requested', l, r)
            try:
                ok = bool(self.send('C', l, r))
            except Exception:
                ok = False
            self.log('acknowledged' if ok else 'failed', l, r)
            for item in pair.values():
                item['ok'] = ok
                item['done'].set()
        if not request['done'].wait(.4):
            with self.lock:
                if self.pending.get(side) is request:
                    del self.pending[side]
            return False
        return request['ok']


def main(argv=None):
    import rclpy
    from rclpy.utilities import remove_ros_args
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', required=True)
    args = parser.parse_args(remove_ros_args()[1:] if argv is None else argv)
    from rclpy.node import Node
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
    from std_msgs.msg import String, Bool, Empty
    from std_srvs.srv import SetBool, Trigger
    from mechaship_interfaces.srv import ThrottlePulseWidth
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    exits = ExitSignals()
    node = Node('arduino_actuator_bridge')
    group = ReentrantCallbackGroup()
    transport = ReconnectingTransport(args.port, node.get_logger().info)
    output = node.create_publisher(String, '/water_trial/command', 100)
    telemetry = node.create_publisher(String, '/actuators/status', 1)
    estop = node.create_publisher(Bool, '/sensor/emo/status', 10)
    stop_request = node.create_publisher(Bool, '/actuators/stop_requested', 10)
    stop_test = node.create_publisher(Empty, '/water_trial/stop', 10)

    def log(state, left, right):
        output.publish(String(data=json.dumps({'status': state, 'source': 'arduino_bridge',
                                              'left_pwm_us': left, 'right_pwm_us': right})))
    pairer = Pairer(transport.exchange, log)
    def callback(side):
        def handle(request, response):
            response.status = not exits.requested and pairer.submit(side, request.pulse_width)
            return response
        return handle
    def arm(request, response):
        response.success = (not exits.requested or not request.data) and transport.exchange('A' if request.data else 'D')
        response.message = 'Acknowledged' if response.success else 'Rejected: check mode, USB connection and telemetry'
        return response
    services = [node.create_service(ThrottlePulseWidth, '/actuators/throttle_' + side + '/set_pulse_width',
                                   callback(side), callback_group=group) for side in ('left', 'right')]
    services.append(node.create_service(SetBool, '/actuators/arm', arm, callback_group=group))
    def stop_all(request,response):
        stop_test.publish(Empty())
        response.success=transport.exchange('S')
        response.message='Neutral acknowledged; AUTO disarmed. Center RC sticks for 0.5s to resume manual control.' if response.success else 'Stop unconfirmed; use physical EMO'
        return response
    def release_stop(request,response):
        response.success=transport.exchange('D') and transport.exchange('R')
        response.message='AUTO disarmed; no stop latch is used' if response.success else 'Request rejected'
        return response
    services.append(node.create_service(Trigger,'/actuators/stop',stop_all,callback_group=group))
    services.append(node.create_service(Trigger,'/actuators/release_stop',release_stop,callback_group=group))
    def report():
        m, age, error = transport.snapshot()
        payload = {'receiving': m is not None and age < .3 and error is None, 'error': error}
        if m is not None:
            payload.update(asdict(m), age_s=round(age, 3))
            payload['mode_name'] = {0: 'INHIBITED', 1: 'RC', 2: 'AUTO'}[m.mode]
        raw, raw_age = transport.raw_snapshot()
        if raw is not None and raw_age < .3:
            payload.update(raw)
        telemetry.publish(String(data=json.dumps(payload)))
        stop_request.publish(Bool(data=not (payload['receiving'] and m.mode == 2 and m.armed and not m.estop_active)))
        # Never report an unwired or stale emergency-stop circuit as released.
        if m is not None and m.estop_known:
            estop.publish(Bool(data=m.estop_active or age >= .3 or error is not None))
    timer = node.create_timer(.1, report)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        while rclpy.ok() and not exits.requested:
            executor.spin_once(timeout_sec=.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        exits.requested = True
        for service in services:
            node.destroy_service(service)
        executor.shutdown()
        if not transport.close():
            node.get_logger().warning('STOP_UNCONFIRMED: neutral was not acknowledged; use physical EMO')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        exits.restore()
