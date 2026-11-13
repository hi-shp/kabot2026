"""Separate response-model identity from navigation and display tuning."""
import hashlib
import json
import math
from pathlib import Path


def validate_arrival_radius(value):
    if isinstance(value,bool):raise ValueError('GPS 도달 범위는 0보다 큰 m 값으로 입력하세요')
    radius=float(value)
    if not math.isfinite(radius) or radius<=0:raise ValueError('GPS 도달 범위는 0보다 큰 m 값으로 입력하세요')
    return radius


def read_arrival_radius(path,default):
    try:data=json.loads(Path(path).read_text(encoding='utf-8'))
    except FileNotFoundError:return float(default)
    return validate_arrival_radius(data['arrival_radius_m'])


def validate_drive_limits(values):
    result={}
    for name in ('point','dock','path'):
        value=values[name]
        if isinstance(value,bool):raise ValueError('출력은 0~100%로 입력하세요')
        value=float(value)
        if not math.isfinite(value) or not 0<=value<=100:raise ValueError('출력은 0~100%로 입력하세요')
        result[name]=value
    return result


def read_drive_limits(path,default=20.):
    try:return validate_drive_limits(json.loads(Path(path).read_text(encoding='utf-8')))
    except FileNotFoundError:return dict(point=default,dock=default,path=default)

IDENTITY_KEYS = (
    'mass_kg','length_m','width_m','thruster_spacing_m',
    'esc_neutral_us','esc_min_us','esc_max_us','auto_reversed',
    'imu_source','heading_offset_deg','magnetic_declination_deg','gps_lever_arm_m',
    'gps_position_sigma_limit_m','gps_velocity_sigma_limit_m_s',
    'gps_max_age_s','imu_max_age_s','prediction_dt_s',
)


def learning_identity(config):
    result = {key:config[key] for key in IDENTITY_KEYS}
    # Old records retain their original signature and unknown hardware mode.
    for key in ('imu_heading_mode','imu_calibration_id'):
        if key in config:
            result[key] = config[key]
    return result


def fingerprint(config):
    return hashlib.sha256(json.dumps(learning_identity(config),sort_keys=True).encode()).hexdigest()


def load_config(path):
    path=Path(path); data=json.loads(path.read_text(encoding='utf-8'))
    if 'learning_file' not in data: return data  # Existing flat custom configs.
    result={}
    for name in ('learning_file','navigation_file'):
        part=json.loads((path.parent/data[name]).read_text(encoding='utf-8'))
        overlap=result.keys() & part.keys()
        if overlap: raise ValueError('설정 항목 중복: '+', '.join(sorted(overlap)))
        result.update(part)
    learning_identity(result)  # Fail clearly on missing calibration keys.
    return result


def compatible_signatures(root, config):
    """Read old calibration snapshots; never rewrite historical data/models."""
    signatures={fingerprint(config)}
    for path in (Path(root)/'configs').glob('*.json'):
        try:
            old=json.loads(path.read_text(encoding='utf-8'))
            if learning_identity(old)==learning_identity(config): signatures.add(path.stem)
        except (OSError,ValueError,KeyError,TypeError): continue
    return sorted(signatures)
