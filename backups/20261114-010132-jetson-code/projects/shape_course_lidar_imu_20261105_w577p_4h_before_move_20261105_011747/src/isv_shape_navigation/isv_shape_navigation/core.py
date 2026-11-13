"""Sensor fusion and shape-first mission, in metres and CCW radians.

The navigation reference is the upstream SLAM reference (velodyne by default).
Camera/scan extrinsics are relative to that reference, not a guessed boat TF.
"""
from dataclasses import dataclass
import math
import numpy as np


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float
    stamp: float

    def world(self, x, y):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return np.array([self.x + c*x - s*y, self.y + s*x + c*y])


@dataclass(frozen=True)
class Detection:
    label: str
    score: float
    cx: float
    cy: float
    width: float
    height: float
    stamp: float


@dataclass(frozen=True)
class Scan:
    ranges: np.ndarray
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float
    stamp: float
    yaw: float = 0.
    x: float = 0.
    y: float = 0.

    def points(self, yaw=0., tx=0., ty=0.):
        r = np.asarray(self.ranges, dtype=float)
        a = self.angle_min + np.arange(r.size)*self.angle_increment + yaw
        valid = np.isfinite(r) & (r >= self.range_min) & (r <= self.range_max)
        return np.column_stack((r[valid]*np.cos(a[valid])+tx,
                                r[valid]*np.sin(a[valid])+ty))


def fuse_detection(detection, scan, pose, fx, cx, *, camera_yaw=0.,
                   camera_x=0., camera_y=0., scan_yaw=0., scan_x=0., scan_y=0.,
                   sync_tolerance=.20, min_range=.4, max_range=20.,
                   min_returns=3, range_spread=.35):
    """Associate returns inside the CENTRAL bbox rays; reject mixed depths.

    Input pixels must be rectified. A ray alone has no metric depth. No scan
    return means no target position; neither infinity nor box size is depth.
    """
    if (not math.isfinite(fx) or fx <= 0 or not math.isfinite(cx)
            or max(abs(detection.stamp-scan.stamp),
                   abs(detection.stamp-pose.stamp)) > sync_tolerance):
        return None
    if not all(math.isfinite(v) for v in (detection.cx, detection.width,
                                         pose.x, pose.y, pose.yaw)) or detection.width <= 0:
        return None
    points = scan.points(scan_yaw, scan_x, scan_y)
    if len(points) == 0:
        return None
    relative = points - [camera_x, camera_y]
    c, s = math.cos(camera_yaw), math.sin(camera_yaw)
    forward = relative[:, 0]*c + relative[:, 1]*s
    left = -relative[:, 0]*s + relative[:, 1]*c
    distance = np.linalg.norm(relative, axis=1)
    projected_u = cx - fx*left/np.maximum(forward, 1e-9)
    # Narrow association limits returns from the adjacent wall/bay.
    half = max(2., detection.width*.30)
    selected = points[(forward > 0) & (distance >= min_range) & (distance <= max_range)
                      & (abs(projected_u-detection.cx) <= half)]
    if len(selected) < min_returns:
        return None
    depths = np.linalg.norm(selected-[camera_x, camera_y], axis=1)
    if np.percentile(depths, 90)-np.percentile(depths, 10) > range_spread:
        return None
    # Keep the measured centroid; do not put a point on an unmeasured camera ray.
    point = np.median(selected, axis=0)
    return pose.world(float(point[0]), float(point[1]))


@dataclass
class MissionConfig:
    target_label: str = 'triangle'
    stable_observations: int = 4
    association_distance: float = .65
    target_timeout: float = 2.
    target_memory_timeout: float = 30.
    approach_standoff: float = 2.5
    entry_standoff: float = 1.2
    goal_tolerance: float = .25
    arrival_hold: float = 1.
    pose_timeout: float = .5
    scan_timeout: float = .5
    max_forward: float = .16
    max_turn: float = .12
    turn_gain: float = .32
    turn_in_place_angle: float = .6
    footprint_radius: float = .62
    clearance: float = .15
    stop_distance: float = .40
    lookahead: float = .65

    def __post_init__(self):
        for key, value in vars(self).items():
            if isinstance(value, (int, float)) and (not math.isfinite(value) or value <= 0):
                raise ValueError(key+' must be finite and positive')
        if self.stable_observations < 2 or int(self.stable_observations) != self.stable_observations:
            raise ValueError('stable_observations must be an integer >= 2')
        if self.approach_standoff <= self.entry_standoff + self.goal_tolerance:
            raise ValueError('approach_standoff must exceed entry_standoff + tolerance')
        if self.entry_standoff <= self.footprint_radius + self.clearance + self.goal_tolerance:
            raise ValueError('entry_standoff must leave room for the full vessel')
        if max(self.max_forward, self.max_turn) > .3:
            raise ValueError('Prototype output limits must be <= 0.3')
        if self.target_memory_timeout < self.target_timeout:
            raise ValueError('target_memory_timeout must be >= target_timeout')


class Mission:
    """A marker defines a region's approach, then a closer entry goal.

    This does NOT infer a doorway or a U-shaped dock. The marked region must
    already be observed as traversable in the SLAM occupancy grid.
    """
    def __init__(self, config=None):
        self.config = config or MissionConfig()
        self.reset()

    def reset(self):
        self.target = None
        self.direction = None
        self.count = 0
        self.last_seen = -math.inf
        self.last_observation = -math.inf
        self.phase = 'SEARCH_SHAPE'
        self.arrival_since = None
        self.completed = False

    def observe(self, point, pose, stamp):
        if self.completed or stamp <= self.last_observation:
            return False
        point = np.asarray(point, dtype=float)
        if point.shape != (2,) or not np.isfinite(point).all():
            return False
        cfg = self.config
        delta = point-np.array([pose.x, pose.y])
        length = np.linalg.norm(delta)
        if length < .4:
            return False
        if self.count < cfg.stable_observations and length <= cfg.entry_standoff+cfg.goal_tolerance:
            return False
        if self.target is None or (self.count < cfg.stable_observations and
                                   stamp-self.last_seen > cfg.target_timeout):
            self.target, self.count = point.copy(), 1
            self.direction = delta/length
        elif np.linalg.norm(point-self.target) <= cfg.association_distance:
            # Freeze world goals once acquired, avoiding moving-goal drift.
            if self.count < cfg.stable_observations:
                self.target = .7*self.target + .3*point
            self.count += 1
        else:
            return False
        self.last_seen = self.last_observation = stamp
        if self.count >= cfg.stable_observations and self.phase == 'SEARCH_SHAPE':
            self.phase = 'ENTER_REGION' if length < cfg.approach_standoff else 'APPROACH_SHAPE'
        return True

    def goal(self):
        if self.target is None or self.count < self.config.stable_observations:
            return None
        offset = (self.config.entry_standoff if self.phase == 'ENTER_REGION'
                  else self.config.approach_standoff)
        return self.target-self.direction*offset

    def step(self, now, pose, scan, grid, *, localization_ready=True):
        """Return status, bounded normalized thrust (forward, CCW turn), path."""
        cfg = self.config
        def hold(reason):
            self.arrival_since = None
            return reason, (0., 0.), []
        if self.completed:
            return 'REGION_REACHED', (0., 0.), []
        if not localization_ready:
            return hold('WAIT_LOCALIZATION')
        if (pose is None or not -.1 <= now-pose.stamp <= cfg.pose_timeout
                or not all(math.isfinite(v) for v in (pose.x, pose.y, pose.yaw))):
            return hold('WAIT_POSE')
        if scan is None or not -.1 <= now-scan.stamp <= cfg.scan_timeout:
            return hold('WAIT_SCAN')
        if grid is None:
            return hold('WAIT_MAP')
        if self.count < cfg.stable_observations:
            return hold('SEARCH_SHAPE')
        if not -.1 <= now-self.last_seen <= cfg.target_memory_timeout:
            return hold('TARGET_LOST')
        goal = self.goal()
        start = np.array([pose.x, pose.y])
        path = grid.plan(start, goal)
        if not path:
            return hold('NO_SAFE_PATH')
        if np.linalg.norm(goal-start) <= cfg.goal_tolerance:
            if self.arrival_since is None:
                self.arrival_since = now
            if now-self.arrival_since >= cfg.arrival_hold:
                self.arrival_since = None
                if self.phase == 'APPROACH_SHAPE':
                    self.phase = 'ENTER_REGION'
                else:
                    self.completed = True
            return ('REGION_REACHED' if self.completed else self.phase), (0., 0.), path
        self.arrival_since = None
        # Follow only the adjacent segment. Never skip a corner by selecting
        # a Euclidean-near waypoint on the far side of an obstacle.
        point = np.array(path[1] if len(path) > 1 else path[0])
        vector = point-start
        length = np.linalg.norm(vector)
        if length > cfg.lookahead:
            point = start+vector/length*cfg.lookahead
        error = wrap(math.atan2(point[1]-pose.y, point[0]-pose.x)-pose.yaw)
        turn = float(np.clip(cfg.turn_gain*error, -cfg.max_turn, cfg.max_turn))
        forward = 0. if abs(error) > cfg.turn_in_place_angle else cfg.max_forward*max(0., math.cos(error))
        # A circular footprint covers the vessel even while turning. Screen
        # obstacle points in the full forward swept corridor, not only a ray.
        points = scan.points(scan.yaw, scan.x, scan.y)
        radius = cfg.footprint_radius+cfg.clearance
        if len(points) < 3:
            return hold('SCAN_UNOBSERVED')
        if np.any(np.linalg.norm(points, axis=1) < radius):
            return hold('OBSTACLE_STOP')
        swept = (points[:, 0] > 0) & (points[:, 0] < radius+cfg.stop_distance)
        swept &= abs(points[:, 1]) < radius
        if forward > 0 and np.any(swept):
            return hold('OBSTACLE_STOP')
        # The map must also cover the actual short-horizon motion. A safe
        # waypoint does not by itself prove a curved/turning command safe.
        if forward > 0:
            ahead = pose.world(min(cfg.lookahead, float(length)), 0.)
            if not grid.segment_free(start, ahead):
                if abs(error) > .05:
                    return 'ALIGN_PATH', (0., turn), path
                return hold('MAP_CORRIDOR_BLOCKED')
        return self.phase, (forward, turn), path


def thrust_to_pwm(forward, turn, limit=.25):
    """CCW left turn: right motor stronger. Upstream service units 1100..1900."""
    if not all(math.isfinite(v) for v in (forward, turn, limit)) or not 0 < limit <= 1:
        return 1500, 1500
    left, right = forward-turn, forward+turn
    peak = max(abs(left), abs(right), limit)
    left, right = left*limit/peak, right*limit/peak
    return 1500+round(400*left), 1500+round(400*right)
