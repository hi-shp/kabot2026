"""Opt-in bridge for kabot_actuator firmware; never armed on startup.

Legacy left/right requests MUST be concurrent, as in the water trial runner.
The paired frame is atomic. No timer repeats stale propulsion commands.
"""
import argparse
from dataclasses import asdict
import json
import secrets
import threading
import time
from .actuator_protocol import command, status, raw_status


class SerialTransport:
    def __init__(self, port):
        import serial
        self.serial = serial.Serial(port=None, baudrate=115200, timeout=.02, write_timeout=.05)
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

    def exchange(self, operation, left=1500, right=1500):
        # Do not queue an old propulsion request behind another in-flight transaction.
        if not self.lock.acquire(timeout=.03):
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
                            if operation == 'D':
                                return not m.armed
                            return m.mode == 2 and m.armed and m.rc_valid and not m.estop_active and (m.left, m.right) == (left, right)
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
        try:
            self.exchange('D')
        finally:
            self.closed.set()
            self.reader.join(timeout=.2)
            self.serial.close()


class Pairer:
    def __init__(self, send, log):
        self.send, self.log = send, log
        self.lock = threading.Lock()
        self.pending = {}

    def submit(self, side, pwm):
        if side not in ('left', 'right') or type(pwm) is not int or not 1400 <= pwm <= 1600:
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
    from rclpy.executors import MultiThreadedExecutor
    from std_msgs.msg import String, Bool
    from std_srvs.srv import SetBool
    from mechaship_interfaces.srv import ThrottlePulseWidth
    rclpy.init()
    node = Node('arduino_actuator_bridge')
    group = ReentrantCallbackGroup()
    transport = SerialTransport(args.port)
    output = node.create_publisher(String, '/water_trial/command', 100)
    telemetry = node.create_publisher(String, '/actuators/status', 10)
    estop = node.create_publisher(Bool, '/sensor/emo/status', 10)
    stop_request = node.create_publisher(Bool, '/actuators/stop_requested', 10)

    def log(state, left, right):
        output.publish(String(data=json.dumps({'status': state, 'source': 'arduino_bridge',
                                              'left_pwm_us': left, 'right_pwm_us': right})))
    pairer = Pairer(transport.exchange, log)
    def callback(side):
        def handle(request, response):
            response.status = pairer.submit(side, request.pulse_width)
            return response
        return handle
    def arm(request, response):
        response.success = transport.exchange('A' if request.data else 'D')
        response.message = 'Acknowledged' if response.success else 'Rejected: check RC mode, neutral sticks, wiring and telemetry'
        return response
    services = [node.create_service(ThrottlePulseWidth, '/actuators/throttle_' + side + '/set_pulse_width',
                                   callback(side), callback_group=group) for side in ('left', 'right')]
    services.append(node.create_service(SetBool, '/actuators/arm', arm, callback_group=group))
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
        stop_request.publish(Bool(data=not (payload['receiving'] and m.mode == 2 and m.armed and m.rc_valid and not m.estop_active)))
        # Never report an unwired or stale emergency-stop circuit as released.
        if m is not None and m.estop_known:
            estop.publish(Bool(data=m.estop_active or age >= .3 or error is not None))
    timer = node.create_timer(.1, report)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        deadline = time.monotonic() + 3.
        acknowledged = transport.exchange('D')
        while not acknowledged and time.monotonic() < deadline:
            time.sleep(.05)
            acknowledged = transport.exchange('D')
        if not acknowledged:
            raise RuntimeError('No kabot_actuator firmware acknowledgement; bridge not started')
        node.get_logger().info('Disarmed. Explicit /actuators/arm and valid RC AUTO selection required.')
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        transport.close()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
