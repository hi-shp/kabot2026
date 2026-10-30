import json
import math
from pathlib import Path


def config_path():
    source = Path(__file__).resolve().parents[1] / 'config/boat.json'
    if source.exists():
        return source
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('isv_water_trials')) / 'config/boat.json'


def load_config(path=None):
    cfg = json.loads(Path(path or config_path()).read_text())
    for key in ('mass_kg', 'length_m', 'beam_m', 'thruster_spacing_m',
                'max_sensor_gap_s', 'derivative_window_s', 'command_max_age_s'):
        value = cfg[key]
        if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
            raise ValueError('Invalid positive configuration: ' + key)
    if not 1100 <= cfg['neutral_pwm_us'] <= 1900:
        raise ValueError('Invalid neutral PWM')
    return cfg


def clean_json(value):
    if isinstance(value, dict):
        return {str(k): clean_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean_json(v) for v in value]
    if hasattr(value, 'tolist'):
        return clean_json(value.tolist())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def dump_json(path, value):
    Path(path).write_text(json.dumps(clean_json(value), ensure_ascii=False,
                                    indent=2, allow_nan=False) + '\n', encoding='utf-8')
