"""No hardware: ROS clients, signals, initialization and neutral stop are mocked."""
from concurrent.futures import Future
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import thruster_output

class PairedOutputTests(unittest.TestCase):
    def test_pwm_limits_and_integer_units(self):
        self.assertEqual(thruster_output.validate_pair(1100,1900),(1100,1900))
        for pair in [(1000,1500),(1500,2000),(True,1500),(1500.,1500)]:
            with self.assertRaises(ValueError): thruster_output.validate_pair(*pair)
    def simulate_sender(self,stop_ack,competing=False):
        calls=[];exits=SimpleNamespace(requested=False,restore=lambda:None)
        class Client:
            def __init__(self,name): self.name=name
            def service_is_ready(self): return True
            def call_async(self,request):
                calls.append((self.name,request))
                future=Future();future.set_result(SimpleNamespace(status=True,success=True))
                if self.name.endswith('throttle_right/set_pulse_width'): exits.requested=True
                return future
        class Node:
            def __init__(self,name): self.name=name;self.destroyed=False
            def create_service(self,*args): return object()
            def create_client(self,kind,name): return Client(name)
            def get_node_names(self):return ['rc_auto_mode_test','course_mission'] if competing else ['rc_auto_mode_test']
            def destroy_node(self): self.destroyed=True
        with patch('rclpy.init'),patch('rclpy.shutdown') as shutdown,patch('rclpy.ok',return_value=True),patch('rclpy.node.Node',Node),patch('isv_water_trials.shutdown.ExitSignals',return_value=exits),patch.object(thruster_output,'neutralize',return_value=stop_ack) as stop:
            if competing:
                with self.assertRaisesRegex(RuntimeError,'Competing actuator sender'):
                    thruster_output._send_pair((1580,1460))
                stop.assert_not_called();shutdown.assert_called_once()
                self.assertEqual(calls,[])
                return
            if stop_ack: thruster_output._send_pair((1580,1460))
            else:
                with self.assertRaises(SystemExit) as error: thruster_output._send_pair((1580,1460))
                self.assertEqual(error.exception.code,1)
            stop.assert_called_once();self.assertTrue(stop.call_args.args[0].destroyed);shutdown.assert_called_once()
        commands={name:request.pulse_width for name,request in calls if hasattr(request,'pulse_width')}
        self.assertEqual(commands['/actuators/throttle_left/set_pulse_width'],1580)
        self.assertEqual(commands['/actuators/throttle_right/set_pulse_width'],1460)
        self.assertTrue(calls[0][1].data)
    def test_distinct_outputs_and_acknowledged_stop(self): self.simulate_sender(True)
    def test_unacknowledged_stop_is_failure(self): self.simulate_sender(False)
    def test_competing_mission_prevents_arm_and_does_not_stop_its_owner(self): self.simulate_sender(True,True)

if __name__=='__main__': unittest.main()
