"""Follow one learned marker using the existing Jetson fusion/guidance inputs.

No waypoints, U-dock geometry, replacement YOLO, or actuator calibration here.
Guidance and percent-to-PWM conversion are supplied by isv_water_trials.
"""
from dataclasses import dataclass
import math
import numpy as np

from .core import Scan


MODEL_CLASSES = ('blue circle', 'blue cross', 'blue triangle', 'green circle',
                 'green cross', 'green triangle', 'red circle', 'red cross', 'red triangle')


def label_name(value):
    return ' '.join(str(value).lower().replace('_', ' ').split())


@dataclass(frozen=True)
class FusedTarget:
    u: float
    v: float
    class_id: int
    track_id: int
    x: float
    y: float
    z: float
    distance: float
    stamp: float


def fusion_rows(data, stamp, rows=None, cols=8, offset=0):
    """Validate /fusion/roi_center's actual [u,v,class,track,x,y,z,R] layout."""
    if cols != 8 or type(offset) is not int or offset < 0 or offset > len(data):
        raise ValueError('Fusion requires 8 columns and a valid data_offset')
    values = list(data)[offset:]
    if len(values) % 8 or (rows is not None and rows * 8 != len(values)):
        raise ValueError('Fusion dimensions do not match data length')
    if not math.isfinite(stamp):
        raise ValueError('Invalid observation time')
    result = []
    for start in range(0, len(values), 8):
        u, v, cid, tid, x, y, z, distance = values[start:start+8]
        if not all(math.isfinite(value) for value in (u, v, cid, tid, x, y, z, distance)):
            continue
        if (cid != int(cid) or cid < 0 or tid != int(tid) or tid < -1
                or u < 0 or v < 0 or x <= 0 or not .2 <= distance <= 50):
            continue
        norm = math.sqrt(x*x+y*y+z*z)
        if abs(norm-distance) > max(.2, distance*.15):
            continue
        result.append(FusedTarget(u, v, int(cid), int(tid), x, y, z, distance, stamp))
    return result


@dataclass
class FollowConfig:
    target_label: str = 'red triangle'
    class_names: tuple = MODEL_CLASSES
    stable_observations: int = 4
    target_timeout: float = .8
    scan_timeout: float = .5
    sync_tolerance: float = .25
    target_standoff: float = 1.5
    resume_margin: float = .2
    max_percent: float = 16.
    association_distance: float = .8
    footprint_radius: float = .62
    clearance: float = .15
    stop_distance: float = .4
    camera_yaw_deg: float = 0.
    camera_x: float = 0.
    camera_y: float = 0.

    def __post_init__(self):
        self.target_label = label_name(self.target_label)
        self.class_names = tuple(label_name(name) for name in self.class_names)
        if (not self.target_label or self.target_label not in self.class_names
                or len(set(self.class_names)) != len(self.class_names)):
            raise ValueError('target_label must match one unique model class: '+str(self.class_names))
        if type(self.stable_observations) is not int or self.stable_observations < 2:
            raise ValueError('stable_observations must be an integer >= 2')
        for name in ('target_timeout', 'scan_timeout', 'sync_tolerance', 'target_standoff',
                     'resume_margin', 'max_percent', 'association_distance', 'footprint_radius',
                     'clearance', 'stop_distance'):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError('Invalid '+name)
        if self.max_percent > 30:
            raise ValueError('Prototype max_percent must be <= 30')
        if self.target_standoff <= self.footprint_radius+self.clearance+self.stop_distance:
            raise ValueError('target_standoff must exceed the obstacle stop corridor')
        for value in (self.camera_yaw_deg, self.camera_x, self.camera_y):
            if not math.isfinite(value):
                raise ValueError('Invalid camera transform')

    @property
    def target_class_id(self):
        return self.class_names.index(self.target_label)


class Follower:
    def __init__(self, config, guidance):
        self.config, self.guidance = config, guidance
        self.reset()

    def reset(self):
        self.target = None
        self.count = 0
        self.holding = False
        self.last_stamp = -math.inf
        self.vision = 'WAIT_TARGET'

    def observe(self, candidates, now):
        cfg = self.config
        candidates = [target for target in candidates if target.class_id == cfg.target_class_id
                      and -.1 <= now-target.stamp <= cfg.target_timeout]
        if len(candidates) != 1:
            self.target, self.count, self.holding = None, 0, False
            self.vision = 'AMBIGUOUS_TARGET' if candidates else 'WAIT_TARGET'
            return
        candidate = candidates[0]
        if candidate.stamp <= self.last_stamp:
            return  # Repeated fusion packets cannot refresh an old detector frame.
        previous = self.target
        same = previous is not None and candidate.stamp-previous.stamp <= cfg.target_timeout
        if same and previous.track_id >= 0 and candidate.track_id >= 0:
            same = previous.track_id == candidate.track_id
        if same:
            same = math.hypot(candidate.x-previous.x, candidate.y-previous.y) <= cfg.association_distance
        self.count = self.count+1 if same else 1
        if not same:
            self.holding = False
        self.target, self.last_stamp = candidate, candidate.stamp
        self.vision = 'TARGET_STABLE' if self.count >= cfg.stable_observations else 'VERIFY_TARGET'

    def step(self, now, scan):
        cfg = self.config
        neutral = (0., 0.)
        if scan is None or not -.1 <= now-scan.stamp <= cfg.scan_timeout:
            return 'WAIT_SCAN', neutral
        ranges = scan.ranges
        angles = scan.angle_min + scan.angle_increment * np.arange(len(ranges)) + scan.yaw
        angles = np.arctan2(np.sin(angles), np.cos(angles))
        front = np.abs(angles) <= math.radians(45)
        observed = (np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)) | np.isposinf(ranges)
        if np.count_nonzero(front) < 3 or np.count_nonzero(front & observed) < .5*np.count_nonzero(front):
            return 'SCAN_UNOBSERVED', neutral
        points = scan.points(scan.yaw, scan.x, scan.y)
        radius = cfg.footprint_radius+cfg.clearance
        if (np.any(np.linalg.norm(points, axis=1) < radius)
                or np.any((points[:, 0] > 0) & (points[:, 0] < radius+cfg.stop_distance)
                          & (np.abs(points[:, 1]) < radius))):
            return 'OBSTACLE_STOP', neutral
        if self.target is None:
            return self.vision, neutral
        if not -.1 <= now-self.target.stamp <= cfg.target_timeout:
            self.count, self.holding = 0, False
            return 'TARGET_LOST', neutral
        if abs(scan.stamp-self.target.stamp) > cfg.sync_tolerance:
            return 'WAIT_SYNC', neutral
        if self.count < cfg.stable_observations:
            return 'VERIFY_TARGET', neutral
        yaw = math.radians(cfg.camera_yaw_deg)
        c, s = math.cos(yaw), math.sin(yaw)
        x = cfg.camera_x+c*self.target.x-s*self.target.y
        y = cfg.camera_y+s*self.target.x+c*self.target.y
        distance = math.hypot(x, y)
        if distance <= cfg.target_standoff:
            self.holding = True
        elif distance > cfg.target_standoff+cfg.resume_margin:
            self.holding = False
        if self.holding:
            return 'HOLD_DISTANCE', neutral
        # Call the actual installed Jetson guidance function; do not copy a PWM table.
        left, right = self.guidance((0., 0.), 0., (x, y), percent=cfg.max_percent)
        if not all(math.isfinite(value) and abs(value) <= cfg.max_percent+1e-6 for value in (left, right)):
            return 'GUIDANCE_INVALID', neutral
        state = 'ALIGN_TARGET' if abs(math.atan2(y, x)) > math.radians(55) else 'FOLLOW_TARGET'
        return state, (float(left), float(right))


def native_pwm(percentages, command_pwm):
    """Use the installed calibration; firmware alone applies ESC offsets/reversal."""
    left, right = percentages
    result = (command_pwm(left, side='left')[0], command_pwm(right, side='right')[1])
    if any(type(value) is not int or not 1100 <= value <= 1900 for value in result):
        raise ValueError('Native PWM conversion returned an invalid service command')
    return result
