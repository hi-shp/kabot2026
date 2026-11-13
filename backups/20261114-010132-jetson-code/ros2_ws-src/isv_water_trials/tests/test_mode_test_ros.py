"""ROS handover fixture; localhost domain 218, no serial/hardware access."""
import importlib.util
import json
import os
import signal
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
        from std_msgs.msg import String, Empty
        from std_srvs.srv import SetBool, Trigger
        from mechaship_interfaces.srv import ThrottlePulseWidth
        cls.rclpy, cls.String = rclpy, String
        rclpy.init()
        cls.node = Node('mode_test_fixture')
        cls.publisher = cls.node.create_publisher(String, '/actuators/status', 10)
        cls.stop_publisher = cls.node.create_publisher(Empty, '/water_trial/stop', 10)
        cls.stop_sent = False
        cls.stop_process = None
        cls.lock = threading.RLock()
        cls.start = None
        cls.kind = 'handover'
        cls.mode, cls.armed = 1, False
        cls.commands, cls.arms, cls.transitions, cls.stops = [], [], [], []
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
        def stop_all(request,response):
            with cls.lock:
                cls.armed=False
                cls.stops.append(time.monotonic() - cls.start if cls.start is not None else 0)
                response.success=True
            return response
        cls.services.append(cls.node.create_service(Trigger,'/actuators/stop',stop_all,callback_group=group))
        def publish():
            with cls.lock:
                if cls.start is None:
                    cls.start = time.monotonic()
                elapsed = time.monotonic() - cls.start
                mode = (1 if elapsed < .4 or 1.9 <= elapsed < 2.8 else 2) if cls.kind == 'handover' else 2
                if cls.kind == 'handover' and (.4 <= elapsed < .55 or 2.8 <= elapsed < 2.95):
                    mode = 0
                if mode != cls.mode:
                    cls.armed = False
                    cls.transitions.append((elapsed, mode))
                cls.mode = mode
                if cls.kind in ('stop', 'stop_cli', 'reinitialize') and elapsed >= 1.6 and not cls.stop_sent:
                    if cls.kind == 'reinitialize':
                        cls.stop_process = subprocess.Popen(
                            [sys.executable, '-m', 'isv_water_trials.reinitialize_esc'],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                    elif cls.kind == 'stop_cli':
                        cls.stop_process = subprocess.Popen(
                            [sys.executable, '-m', 'isv_water_trials.stop_thrusters',
                             '--port', '/tmp/isv-mode-test-no-hardware-serial'],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                    else:
                        cls.stop_publisher.publish(Empty())
                    cls.stop_sent = True
                if cls.kind == 'sigint' and elapsed >= 1.6 and cls.commands and not cls.stop_sent:
                    cls.test_process.send_signal(signal.SIGINT)
                    cls.stop_sent = True
                if cls.kind == 'hangup' and elapsed >= 1.2 and cls.commands and not cls.stop_sent:
                    cls.test_process.send_signal(signal.SIGHUP)
                    cls.stop_sent = True
                if cls.kind == 'terminate' and elapsed >= 1.2 and cls.commands and not cls.stop_sent:
                    cls.test_process.send_signal(signal.SIGTERM)
                    cls.stop_sent = True
                if cls.kind == 'gap' and elapsed > 1.4:
                    return
                message = dict(receiving=True, error=None, age_s=0., mode=mode, armed=cls.armed,
                               rc_valid=cls.kind != 'absent', estop_active=False, left=1500, right=1500,
                               rc_left_us=0 if cls.kind == 'absent' else 1500,
                               rc_right_us=0 if cls.kind == 'absent' else 1500)
                if cls.kind != 'no_telemetry':
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

    def run_case(self, kind, side='both', seconds=4, percent=10, legacy_seconds=None):
        cls = type(self)
        with cls.lock:
            cls.start, cls.kind, cls.mode, cls.armed = time.monotonic(), kind, 1 if kind == 'handover' else 2, False
            cls.commands, cls.arms, cls.transitions, cls.stops = [], [], [], []
            cls.stop_sent = False
            cls.stop_process = None
        env = dict(os.environ)
        env['PYTHONPATH'] = str(Path(__file__).resolve().parents[1]) + os.pathsep + env.get('PYTHONPATH', '')
        process = subprocess.Popen([sys.executable, '-u', '-m', 'isv_water_trials.mode_test',
                                 '--seconds', str(seconds if legacy_seconds is None else legacy_seconds), '--side', side, '--percent', str(percent)],
                                # Only localhost fixture services are available here.
                                env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        cls.test_process = process
        def interrupt():
            if process.poll() is None:process.send_signal(signal.SIGINT)
        timer=threading.Timer(seconds,interrupt)
        started=time.monotonic()
        timer.start()
        try:
            stdout, stderr = process.communicate(timeout=15)
        finally:
            timer.cancel()
            if process.poll() is None:
                process.kill()
                process.wait()
        self.assertEqual(process.returncode, 0, stdout + stderr)
        cls.elapsed=time.monotonic()-started
        if cls.stop_process is not None:
            stop_logs, _ = cls.stop_process.communicate(timeout=6)
            self.assertEqual(cls.stop_process.returncode, 0, stop_logs)
        self.assertTrue(cls.arms)
        self.assertTrue(cls.stops, stdout)
        self.assertFalse(cls.armed, stdout)
        self.assertIn('Neutral acknowledged.', stdout)
        self.assertNotIn('MCU mode:', stdout)
        self.assertNotIn('Waiting for actuator services',stdout)
        self.assertNotIn('Actuator services available',stdout)
        return list(cls.commands), list(cls.arms), stdout + stderr

    def test_host_sends_in_rc_and_mcu_rejects_until_auto(self):
        commands, arms, logs = self.run_case('handover')
        rc_at = next(t for t, mode in type(self).transitions if mode == 1)
        auto_at = [t for t, mode in type(self).transitions if mode == 2][-1]
        accepted = [(t, l, r) for t, l, r, ok in commands if ok]
        self.assertTrue(any(.4 <= t < 1.9 for t, _, _ in accepted), logs)
        self.assertTrue(any(t >= 2.95 for t, _, _ in accepted), logs)
        self.assertFalse(any(rc_at <= t < auto_at for t, _, _ in accepted))
        self.assertTrue(all((l, r) == (1540, 1540) for _, l, r in accepted))
        self.assertTrue(any(rc_at <= t < auto_at for t, _, _, ok in commands if not ok))
        self.assertGreaterEqual(len([1 for _, armed in arms if armed]), 2)

    def test_cold_auto_start_without_receiver(self):
        commands, _, logs = self.run_case('absent', seconds=1.5, percent=30)
        self.assertTrue(commands, logs)
        self.assertTrue(all(ok and (l, r) == (1620, 1620) for _, l, r, ok in commands), logs)
        self.assertNotIn('staging', logs)

    def test_sending_does_not_require_status_topic(self):
        commands, _, logs = self.run_case('no_telemetry', seconds=1.5, percent=30)
        self.assertTrue(commands, logs)
        self.assertTrue(all(ok for _, _, _, ok in commands), logs)

    def test_legacy_seconds_does_not_end_stream(self):
        commands, _, logs = self.run_case('normal', seconds=2, legacy_seconds=.2)
        self.assertTrue(any(t>1 for t, _, _, ok in commands if ok), logs)
        self.assertIn('automatic time-based exit is disabled',logs)

    def test_left_only_leaves_right_neutral(self):
        commands, _, logs = self.run_case('normal', side='left', seconds=1.5)
        self.assertTrue(commands, logs)
        self.assertTrue(all((l, r) == (1540, 1500) for _, l, r, _ in commands))

    def test_ctrl_c_stops_right_only_thirty_percent_before_exit(self):
        commands, arms, logs = self.run_case('sigint', side='right', seconds=10, percent=30)
        self.assertTrue(commands, logs)
        self.assertTrue(all((l, r) == (1500, 1620) for _, l, r, _ in commands))
        self.assertGreaterEqual(type(self).stops[-1], commands[-1][0])
        self.assertEqual(len([1 for _, armed in arms if armed]), 1)

    def test_external_stop_topic_does_not_pause_sender(self):
        commands, arms, logs = self.run_case('stop', seconds=5, percent=30)
        self.assertTrue(type(self).stop_sent, logs)
        self.assertTrue(commands, logs)
        self.assertTrue(all((l, r) == (1620, 1620) for _, l, r, _ in commands))
        self.assertTrue(any(t > 4 for t, _, _, ok in commands if ok), logs)
        self.assertEqual(len([1 for _, armed in arms if armed]), 1)
        self.assertNotIn('STOP_REQUESTED', logs)
        self.assertGreaterEqual(type(self).elapsed,4.9,logs)

    def test_cli_stop_does_not_pause_sender(self):
        commands, arms, logs = self.run_case('stop_cli', seconds=7, percent=30)
        self.assertTrue(type(self).stop_sent, logs)
        self.assertTrue(commands, logs)
        self.assertTrue(all((l, r) == (1620, 1620) for _, l, r, _ in commands))
        self.assertGreaterEqual(len([1 for _, armed in arms if armed]), 2)
        self.assertTrue(any(t > 5.5 for t,_,_,ok in commands if ok),logs)
        self.assertNotIn('STOP_REQUESTED', logs)
        self.assertGreaterEqual(type(self).elapsed,6.9,logs)

    def test_terminal_disconnect_does_not_end_stream(self):
        commands,_,logs=self.run_case('hangup',seconds=2.5,percent=30)
        self.assertTrue(type(self).stop_sent,logs)
        self.assertTrue(any(t>1.8 and ok for t,_,_,ok in commands),logs)

    def test_sigterm_does_not_end_stream_before_ctrl_c(self):
        commands,_,logs=self.run_case('terminate',seconds=2.5,percent=30)
        self.assertTrue(type(self).stop_sent,logs)
        self.assertTrue(any(t>1.8 and ok for t,_,_,ok in commands),logs)

    def test_reinitialize_pauses_and_resumes_same_sender(self):
        commands,arms,logs=self.run_case('reinitialize',seconds=12,percent=30)
        accepted=[t for t,_,_,ok in commands if ok]
        neutral=[t for t,l,r,ok in commands if ok and (l,r)==(1500,1500)]
        driven=[t for t,l,r,ok in commands if ok and (l,r)==(1620,1620)]
        self.assertTrue(type(self).stop_sent,logs)
        self.assertTrue(neutral and driven,logs)
        self.assertGreaterEqual(max(neutral)-min(neutral),6.8,logs)
        self.assertTrue(any(t>max(neutral) for t in driven),logs)
        self.assertEqual(len([1 for _,armed in arms if armed]),1,logs)
        self.assertNotIn('STOP_REQUESTED',logs)
        self.assertGreaterEqual(type(self).elapsed,11.9,logs)

    def test_missing_status_does_not_gate_command_stream(self):
        commands, arms, logs = self.run_case('gap')
        self.assertTrue(commands, logs)
        self.assertEqual(len([1 for _, armed in arms if armed]), 1)
        self.assertTrue(any(t > 2 for t, _, _, ok in commands if ok), logs)

    def test_mcu_rejection_does_not_stop_sender(self):
        commands, arms, logs = self.run_case('reject')
        self.assertTrue(commands, logs)
        self.assertFalse(any(ok for _, _, _, ok in commands))
        self.assertGreater(len([1 for _, armed in arms if armed]), 1)
        self.assertNotIn('TEST_STOPPED', logs)


if __name__ == '__main__':
    unittest.main()
