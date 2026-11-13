"""GPS staging point -> visual acquisition -> direct approach -> region entry.

Pure control logic, with injected native Jetson guidance. GPS and SLAM frames
are never equated: GPS is used only to reach the observation point; a local
SLAM pose/map may screen the subsequent visual approach corridor.
"""
from dataclasses import dataclass
import math
import numpy as np

from .core import wrap
from .follow import FollowConfig, Follower


@dataclass(frozen=True)
class Fix:
    latitude: float
    longitude: float
    sigma: float
    stamp: float


@dataclass
class GPSConfig:
    goal_configured: bool = False
    goal_latitude: float = 0.
    goal_longitude: float = 0.
    gps_arrival_radius: float = 3.
    gps_arrival_hold: float = 2.
    gps_timeout: float = 2.
    max_gps_sigma: float = 2.
    max_gps_leg: float = 500.
    heading_timeout: float = .5
    gps_percent: float = 12.
    entry_percent: float = 8.
    align_tolerance_deg: float = 12.
    entry_start_distance: float = 3.
    region_stop_distance: float = 1.5
    region_arrival_hold: float = 1.
    search_sweep_deg: float = 60.
    search_percent: float = 6.
    search_timeout: float = 60.
    use_slam: bool = False
    slam_pose_timeout: float = .5
    slam_map_timeout: float = 5.
    pose_yaw_offset_deg: float = 0.

    def __post_init__(self):
        if (not math.isfinite(self.goal_latitude) or not -85 <= self.goal_latitude <= 85
                or not math.isfinite(self.goal_longitude) or not -180 <= self.goal_longitude <= 180):
            raise ValueError('Invalid GPS goal coordinates')
        for name, value in vars(self).items():
            if isinstance(value, bool) or name in ('goal_latitude', 'goal_longitude', 'pose_yaw_offset_deg'):
                continue
            if not math.isfinite(value) or value <= 0:
                raise ValueError('Invalid '+name)
        if not math.isfinite(self.pose_yaw_offset_deg):
            raise ValueError('Invalid SLAM heading offset')
        if max(self.gps_percent, self.entry_percent, self.search_percent) > 30:
            raise ValueError('Prototype output must be <=30 percent')
        if self.search_sweep_deg > 120 or self.align_tolerance_deg > 45:
            raise ValueError('Invalid search/alignment angle')
        if self.entry_start_distance <= self.region_stop_distance+.2:
            raise ValueError('Entry start must precede final stop distance')
        if self.max_gps_sigma > self.gps_arrival_radius:
            raise ValueError('GPS uncertainty limit must not exceed arrival radius')


def gps_vector(fix, latitude, longitude):
    """Great-circle distance and east/north vector, in metres (local mission)."""
    lat1, lat2 = math.radians(fix.latitude), math.radians(latitude)
    dlat = lat2-lat1
    dlon = wrap(math.radians(longitude-fix.longitude))
    h = math.sin(dlat/2)**2+math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
    distance = 6371008.8*2*math.asin(math.sqrt(min(1., max(0., h))))
    bearing = math.atan2(math.sin(dlon)*math.cos(lat2),
                         math.cos(lat1)*math.sin(lat2)-math.sin(lat1)*math.cos(lat2)*math.cos(dlon))
    return distance*math.sin(bearing), distance*math.cos(bearing)


class GPSDockMission(Follower):
    def __init__(self, config: FollowConfig, guidance, gps_config: GPSConfig):
        self.gps_config = gps_config
        radius = config.footprint_radius+config.clearance+config.stop_distance
        if gps_config.region_stop_distance <= radius:
            raise ValueError('Region stop distance must leave the complete hull/stop corridor clear')
        super().__init__(config, guidance)

    def reset(self):
        super().reset()
        self.phase = 'GPS_TRANSIT'
        self.fix = None
        self.heading, self.heading_stamp = None, -math.inf
        self.gps_inside_since, self.gps_inside_last = None, -math.inf
        self.gps_distance = None
        self.search_started, self.search_heading = None, None
        self.search_direction = 1.
        self.region_since = None
        self.pose, self.map = None, None
        self.map_received = -math.inf
        self.last_motion_reason = None

    @property
    def motion_states(self):
        return ('GPS_TRANSIT', 'ALIGN_GPS', 'GPS_ARRIVAL_HOLD', 'SEARCH_SHAPE',
                'VERIFY_TARGET', 'ALIGN_SHAPE', 'APPROACH_SHAPE', 'ENTER_REGION', 'REGION_ARRIVAL_HOLD')

    def observe(self, candidates, now):
        if self.phase == 'GPS_TRANSIT' or self.phase == 'REGION_REACHED':
            return  # Early detections cannot bypass the GPS observation point.
        candidates = [target for target in candidates if target.stamp > self.search_started]
        if len(candidates) != 1:
            self.region_since = None
        super().observe(candidates, now)

    def clear_vision(self):
        Follower.reset(self)
        self.region_since = None

    def _guide(self, target, percent):
        error = math.atan2(target[1], target[0])
        if abs(error) > math.radians(self.gps_config.align_tolerance_deg):
            # Rotate in place first; do not cut an arc toward the marker.
            signed = math.copysign(min(percent, self.config.max_percent), error)
            return (-signed, signed), True
        left, right = self.guidance((0., 0.), 0., target, percent=percent)
        if not all(math.isfinite(v) and abs(v) <= self.config.max_percent+1e-6 for v in (left, right)):
            raise ValueError('Invalid native guidance output')
        return (float(left), float(right)), False

    def step(self, now, scan):
        cfg, gps, neutral = self.config, self.gps_config, (0., 0.)
        if not gps.goal_configured:
            return 'WAIT_GPS_GOAL', neutral
        if self.phase == 'REGION_REACHED':
            return 'REGION_REACHED', neutral
        reason = self.scan_reason(now, scan)
        if reason:
            self.region_since = None
            self.gps_inside_since = None
            return reason, neutral
        if self.phase == 'GPS_TRANSIT':
            fix = self.fix
            if (fix is None or not all(math.isfinite(v) for v in
                    (fix.latitude, fix.longitude, fix.sigma, fix.stamp))
                    or not -90 <= fix.latitude <= 90 or not -180 <= fix.longitude <= 180
                    or not -.1 <= now-fix.stamp <= gps.gps_timeout):
                self.gps_inside_since = None
                return 'WAIT_GPS', neutral
            if not 0 <= fix.sigma <= gps.max_gps_sigma:
                self.gps_inside_since = None
                return 'GPS_UNCERTAIN', neutral
            if self.heading is None or not -.1 <= now-self.heading_stamp <= gps.heading_timeout:
                self.gps_inside_since = None
                return 'WAIT_HEADING', neutral
            east, north = gps_vector(fix, gps.goal_latitude, gps.goal_longitude)
            distance = math.hypot(east, north)
            self.gps_distance = distance
            if distance > gps.max_gps_leg:
                self.gps_inside_since = None
                return 'GPS_GOAL_TOO_FAR', neutral
            if distance <= gps.gps_arrival_radius:
                if fix.stamp > self.gps_inside_last:
                    self.gps_inside_last = fix.stamp
                    if self.gps_inside_since is None:
                        self.gps_inside_since = fix.stamp
                    if fix.stamp-self.gps_inside_since >= gps.gps_arrival_hold:
                        self.phase = 'SEARCH_SHAPE'
                        self.search_started, self.search_heading = now, self.heading
                        self.clear_vision()
                        return 'SEARCH_SHAPE', neutral
                return 'GPS_ARRIVAL_HOLD', neutral
            self.gps_inside_since = None
            c, s = math.cos(self.heading), math.sin(self.heading)
            wheels, aligning = self._guide((c*east+s*north, -s*east+c*north), gps.gps_percent)
            return ('ALIGN_GPS' if aligning else 'GPS_TRANSIT'), wheels
        if self.target is None or self.count < cfg.stable_observations:
            if self.phase != 'SEARCH_SHAPE':
                return 'TARGET_LOST', neutral
            if self.target is not None:
                return 'VERIFY_TARGET', neutral
            if now-self.search_started > gps.search_timeout:
                return 'SEARCH_TIMEOUT', neutral
            if self.heading is None or not -.1 <= now-self.heading_stamp <= gps.heading_timeout:
                return 'WAIT_HEADING', neutral
            offset = wrap(self.heading-self.search_heading)
            if offset >= math.radians(gps.search_sweep_deg):
                self.search_direction = -1.
            elif offset <= -math.radians(gps.search_sweep_deg):
                self.search_direction = 1.
            turn = gps.search_percent*self.search_direction
            return 'SEARCH_SHAPE', (-turn, turn)
        if not -.1 <= now-self.target.stamp <= cfg.target_timeout:
            self.count = 0
            self.region_since = None
            return 'TARGET_LOST', neutral
        if abs(scan.stamp-self.target.stamp) > cfg.sync_tolerance:
            self.region_since = None
            return 'WAIT_SYNC', neutral
        yaw = math.radians(cfg.camera_yaw_deg)
        c, s = math.cos(yaw), math.sin(yaw)
        x = cfg.camera_x+c*self.target.x-s*self.target.y
        y = cfg.camera_y+s*self.target.x+c*self.target.y
        distance = math.hypot(x, y)
        if x <= 0:
            return 'TARGET_BEHIND', neutral
        if gps.use_slam:
            if self.pose is None or not -.1 <= now-self.pose.stamp <= gps.slam_pose_timeout:
                self.region_since = None
                return 'WAIT_SLAM_POSE', neutral
            if self.map is None or not 0 <= now-self.map_received <= gps.slam_map_timeout:
                self.region_since = None
                return 'WAIT_SLAM_MAP', neutral
            if abs(self.pose.stamp-self.target.stamp) > cfg.sync_tolerance:
                self.region_since = None
                return 'WAIT_SLAM_SYNC', neutral
            goal = self.pose.world(x*(1-gps.region_stop_distance/distance),
                                   y*(1-gps.region_stop_distance/distance)) if distance > gps.region_stop_distance else np.array([self.pose.x, self.pose.y])
            if not self.map.segment_free(np.array([self.pose.x, self.pose.y]), goal):
                self.region_since = None
                return 'SLAM_CORRIDOR_BLOCKED', neutral
        self.phase = 'ENTER_REGION' if distance <= gps.entry_start_distance else 'APPROACH_SHAPE'
        aligned = abs(math.atan2(y, x)) <= math.radians(gps.align_tolerance_deg)
        if distance <= gps.region_stop_distance:
            if not aligned:
                return 'STOP_TOO_CLOSE_UNALIGNED', neutral
            if self.region_since is None:
                self.region_since = now
            if now-self.region_since >= gps.region_arrival_hold:
                self.phase = 'REGION_REACHED'
                return 'REGION_REACHED', neutral
            return 'REGION_ARRIVAL_HOLD', neutral
        self.region_since = None
        percent = gps.entry_percent if self.phase == 'ENTER_REGION' else cfg.max_percent
        wheels, aligning = self._guide((x, y), percent)
        return ('ALIGN_SHAPE' if aligning else self.phase), wheels
