"""YAML GPS waypoints -> ENU targets -> relative heading error.

Heading convention: north=0 degrees, clockwise positive.
This class computes guidance only. Callers must stop output when done() is True.
"""
import math
from numbers import Real
from pathlib import Path
from typing import List, Optional, Tuple, Union
import pymap3d as pm
import yaml

PathLike = Union[str, Path]

def _number(value, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(name + ' must be a finite number')
    return float(value)

def _latlon(value, name: str) -> Tuple[float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError(name + ' must be [latitude, longitude]')
    lat = _number(value[0], name + '.latitude')
    lon = _number(value[1], name + '.longitude')
    if not -90 <= lat <= 90 or not -180 <= lon <= 180:
        raise ValueError(name + ' latitude/longitude is out of range')
    return lat, lon

def _norm180(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0

def default_config_path() -> Path:
    source = Path(__file__).resolve().parents[2] / '설정' / 'path_follower.yaml'
    if source.is_file():
        return source
    try:
        from ament_index_python.packages import get_package_share_directory
        installed = Path(get_package_share_directory('isv_shape_navigation')) / 'config' / 'path_follower.yaml'
    except (ImportError, LookupError) as exc:
        raise FileNotFoundError('path_follower.yaml not found; provide config_path explicitly') from exc
    if not installed.is_file():
        raise FileNotFoundError(installed)
    return installed

class PathFollower:
    def __init__(self, origin_latlonalt: Tuple[float, float, float],
                 arrive_radius_m: Optional[float] = None,
                 config_path: Optional[PathLike] = None):
        if not isinstance(origin_latlonalt, (tuple, list)) or len(origin_latlonalt) != 3:
            raise ValueError('origin_latlonalt must be (latitude, longitude, altitude_m)')
        lat, lon = _latlon(origin_latlonalt[:2], 'origin')
        self.origin = (lat, lon, _number(origin_latlonalt[2], 'origin.altitude_m'))
        self.config_path = Path(config_path).expanduser() if config_path is not None else default_config_path()
        with self.config_path.open(encoding='utf-8') as stream:
            data = yaml.safe_load(stream)
        if not isinstance(data, dict) or not isinstance(data.get('path_follower'), dict):
            raise ValueError('YAML must contain a path_follower mapping')
        config = data['path_follower']
        radius = config.get('arrive_radius_m', 1.0) if arrive_radius_m is None else arrive_radius_m
        self.arrive_radius = _number(radius, 'arrive_radius_m')
        if self.arrive_radius <= 0:
            raise ValueError('arrive_radius_m must be greater than zero')
        goals = config.get('goal_gps_coords')
        if not isinstance(goals, list):
            raise ValueError('goal_gps_coords must be a list of [latitude, longitude] pairs')
        self.goal_gps_coords: List[Tuple[float, float]] = [
            _latlon(point, 'goal_gps_coords[%d]' % i) for i, point in enumerate(goals)]
        self.goal_xy: List[Tuple[float, float]] = []
        for goal_lat, goal_lon in self.goal_gps_coords:
            e, n, _ = pm.geodetic2enu(goal_lat, goal_lon, 0.0, *self.origin)
            self.goal_xy.append((float(e), float(n)))
        self.idx = 0

    def done(self) -> bool:
        return self.idx >= len(self.goal_xy)

    def current_target_xy(self) -> Optional[Tuple[float, float]]:
        return None if self.done() else self.goal_xy[self.idx]

    def current_target_gps(self) -> Optional[Tuple[float, float]]:
        return None if self.done() else self.goal_gps_coords[self.idx]

    def update(self, cur_e: float, cur_n: float, now_heading_deg: float) -> float:
        """Return clockwise heading error in [-180, 180), advancing reached targets.

        Inputs use this follower's fixed ENU origin. Heading is north-zero,
        clockwise-positive, in degrees. A completed route returns 0.0 for API
        compatibility; this is an angle, not a motor-stop command. Check done().
        """
        cur_e = _number(cur_e, 'cur_e')
        cur_n = _number(cur_n, 'cur_n')
        heading = _number(now_heading_deg, 'now_heading_deg')
        while not self.done():
            tx, ty = self.goal_xy[self.idx]
            dx, dy = tx - cur_e, ty - cur_n
            if math.hypot(dx, dy) <= self.arrive_radius:
                self.idx += 1
                continue
            angle_to_goal = math.degrees(math.atan2(dx, dy)) % 360.0
            return _norm180(angle_to_goal - heading)
        return 0.0
