"""ROS handover fixture; localhost domain 218, no serial/hardware access."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
import unittest


@unittest.skipUnless(importlib.util.find_spec('rclpy'), 'ROS runtime required')
class ModeTestRosTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ.update(ROS_DOMAIN_ID='218', ROS_LOCALHOST_ONLY='1')
        import rclpy
        from rclpy.node import Node
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.executors import MultiThreadedExecutor
        from std_msgs.msg import String
        from std_srvs.srv import SetBool
        from mechaship_interfaces.srv import ThrottlePulseWidth
        cls.rclpy, cls.String = rclpy, String
        rclpy.init()
        cls.node = Node('mode_test_fixture')
        cls.publisher = cls.node.create_publisher(String, '/actuators/status', 10)
        cls.lock = threading.RLock()
        cls.start = None
        cls.kind = 'handover'
        cls.mode, cls.armed = 1, False
        cls.commands, cls.arms, cls.transitions = [], [], []
        from isv_water_trials.arduino_bridge import Pairer
        def send(operation, left, right):
            with cls.lock:
                accepted = cls.mode == 2 and cls.armed and cls.kind != 'reject'
                cls.commands.append((time.monotonic() - cls.start, left, right, accepted))
                return accepted
        cls.pairer = Pairer(send, lambda *args: None)
        group = ReentrantCallbackGroup()
        def throttle(side):
            def callback(request, response):
                response.status = cls.pairer.submit(side, request.pulse_width)
                return response
            return callback
        def arm(request, response):
            with cls.lock:
                cls.arms.append((time.monotonic() - cls.start, request.data))
                response.success = not request.data or cls.mode == 2
                cls.armed = request.data and response.success
            return response
        cls.services = [cls.node.create_service(ThrottlePulseWidth,
                        '/actuators/throttle_' + side + '/set_pulse_width', throttle(side), callback_group=group)
                        for side in ('left', 'right')]
        cls.services.append(cls.node.create_service(SetBool, '/actuators/arm', arm, callback_group=group))
        def publish():
            with cls.lock:
                if cls.publisher.get_subscription_count() == 0:
                    return
                if cls.start is None:
                    cls.start = time.monotonic()
                elapsed = time.monotonic() - cls.start
                mode = (1 if elapsed < .4 or 1.9 <= elapsed < 2.8 else 2) if cls.kind == 'handover' else 2
                if mode != cls.mode:
                    cls.armed = False
                    cls.transitions.append((elapsed, mode))
                cls.mode = mode
                if cls.kind == 'gap' and elapsed > 1.4:
                    return
                message = dict(receiving=True, error=None, age_s=0., mode=mode, armed=cls.armed,
                               rc_valid=True, estop_active=False, rc_left_us=1500, rc_right_us=1500)
                cls.publisher.publish(String(data=json.dumps(message)))
        cls.timer = cls.node.create_timer(.04, publish)
        cls.executor = MultiThreadedExecutor(num_threads=4)
        cls.executor.add_node(cls.node)
        cls.thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.thread.join(3)
        cls.node.destroy_node()
        cls.rclpy.shutdown()

    def run_case(self, kind, side='both', seconds=4):
        cls = type(self)
        with cls.lock:
            cls.start, cls.kind, cls.mode, cls.armed = None, kind, 1, False
            cls.commands, cls.arms, cls.transitions = [], [], []
        env = dict(os.environ)
        env['PYTHONPATH'] = str(Path(__file__).resolve().parents[1]) + os.pathsep + env.get('PYTHONPATH', '')
        result = subprocess.run([sys.executable, '-u', '-m', 'isv_water_trials.mode_test',
                                 '--seconds', str(seconds), '--side', side],
                                env=env, capture_output=True, text=True, timeout=15)
        self.assertEqual(result.returncode, 1 if kind in ('gap', 'reject') else 0, result.stdout + result.stderr)
        self.assertTrue(cls.arms)
        self.assertFalse(cls.arms[-1][1], result.stdout)
        return list(cls.commands), list(cls.arms), result.stdout + result.stderr

    def test_handover_and_neutral_before_resume(self):
        commands, arms, logs = self.run_case('handover')
        rc_at = next(t for t, mode in type(self).transitions if mode == 1)
        auto_at = [t for t, mode in type(self).transitions if mode == 2][-1]
        accepted = [(t, l, r) for t, l, r, ok in commands if ok]
        self.assertTrue(any(.9 <= t < 1.9 for t, _, _ in accepted), logs)
        self.assertTrue(any(t >= 3.3 for t, _, _ in accepted), logs)
        self.assertFalse(any(rc_at <= t < auto_at for t, _, _ in accepted))
        self.assertTrue(all((l, r) == (1540, 1540) for _, l, r in accepted))
        self.assertEqual(len([1 for _, armed in arms if armed]), 2)
        self.assertGreaterEqual([t for t, armed in arms if armed][1], auto_at + .5)

    def test_left_only_leaves_right_neutral(self):
        commands, _, logs = self.run_case('normal', side='left', seconds=1.5)
        self.assertTrue(commands, logs)
        self.assertTrue(all((l, r) == (1540, 1500) for _, l, r, _ in commands))

    def test_telemetry_gap_stops_without_rearming(self):
        commands, arms, logs = self.run_case('gap')
        self.assertTrue(commands, logs)
        self.assertEqual(len([1 for _, armed in arms if armed]), 1)
        self.assertIn('telemetry stopped', logs)

    def test_command_rejection_stops_without_rearming(self):
        commands, arms, logs = self.run_case('reject')
        self.assertTrue(commands, logs)
        self.assertFalse(any(ok for _, _, _, ok in commands))
        self.assertEqual(len([1 for _, armed in arms if armed]), 1)
        self.assertIn('rejected/timed out', logs)


if __name__ == '__main__':
    unittest.main()
