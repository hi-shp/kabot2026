"""Manual test annotations only. No actuator publishers or ARM implementation."""
import time
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String
from .common import FieldNode, MODES, spin

INSTRUCTIONS = {
    'stationary': 'Keep propulsion safely off; record IMU bias and RF2O drift for 30-60 s.',
    'straight': 'Operator: verified servo neutral; brief low throttle in a clear area.',
    'coast': 'Operator: low steady speed, then verified neutral; record residual motion.',
    'port_turn': 'Operator: low speed, small verified port steering steps.',
    'starboard_turn': 'Operator: low speed, small verified starboard steering steps.',
    'zigzag': 'Operator: low speed, small alternating verified steering steps.',
    'manual_log': 'Use existing RC/manual control; observe commands without publishing.',
}


class CalibrationNode(FieldNode):
    def __init__(self):
        super().__init__('calibration_node')
        mode = self.param('test_mode', 'manual_log')
        if mode not in MODES:
            raise ValueError('unsupported test_mode')
        self.param('motor_control', 'PASSIVE')
        self.param('automatic_drive', False)
        if self.get_parameter('motor_control').value != 'PASSIVE' or self.get_parameter('automatic_drive').value:
            raise ValueError('automatic drive unavailable: MCU watchdog and RC priority unverified')
        self.started = time.monotonic()
        self.pub = self.json_publisher('/field/test_profile')
        self.add_on_set_parameters_callback(self.validate)
        self.create_timer(1.0, self.tick)

    def validate(self, params):
        for p in params:
            if p.name == 'test_mode' and p.value not in MODES:
                return SetParametersResult(successful=False, reason='unknown test_mode')
            if p.name == 'automatic_drive' and p.value or p.name == 'motor_control' and p.value != 'PASSIVE':
                return SetParametersResult(successful=False, reason='hardware safety unavailable; PASSIVE only')
        return SetParametersResult(successful=True)

    def tick(self):
        mode = self.get_parameter('test_mode').value
        self.publish_json(self.pub, dict(stamp=self.now(), mode=mode, motor_control='PASSIVE',
                                        automatic_drive=False, elapsed=time.monotonic()-self.started,
                                        instruction=INSTRUCTIONS[mode]))


def main(args=None):
    spin(CalibrationNode, args)
