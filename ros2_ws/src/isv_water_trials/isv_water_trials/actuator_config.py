"""Shared actuator calibration; firmware header is generated from this same JSON."""
import json
from pathlib import Path


def load_actuator_config():
    path = Path(__file__).resolve().parents[1] / 'config' / 'actuators.json'
    if not path.exists():
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('isv_water_trials')) / 'config' / 'actuators.json'
    config = json.loads(path.read_text(encoding='utf-8'))
    if not config['min_pwm_us'] < config['neutral_pwm_us'] < config['max_pwm_us']:
        raise ValueError('Invalid ESC range')
    for side in ('left', 'right'):
        channel = config[side]
        if not channel['rc_min_us'] < channel['rc_center_us'] < channel['rc_max_us']:
            raise ValueError('Invalid RC calibration: ' + side)
    return config


CONFIG = load_actuator_config()
NEUTRAL_US = CONFIG['neutral_pwm_us']
MIN_US = CONFIG['command_min_pwm_us']
MAX_US = CONFIG['command_max_pwm_us']


def command_pwm(percent, side='both'):
    import math
    if not math.isfinite(percent) or not -100 <= percent <= 100:
        raise ValueError('PWM percentage must be -100..100')
    if side not in ('both', 'left', 'right'):
        raise ValueError('Unknown thruster side')
    span = MAX_US - NEUTRAL_US if percent >= 0 else NEUTRAL_US - MIN_US
    value = NEUTRAL_US + round(span * percent / 100)
    return (value if side in ('left', 'both') else NEUTRAL_US,
            value if side in ('right', 'both') else NEUTRAL_US)
