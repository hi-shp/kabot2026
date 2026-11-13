"""GPS 목표 반경 안에서 중립을 유지하고, 벗어나면 다시 접근한다."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Fix:
    latitude: float
    longitude: float
    sigma: float
    stamp: float


def gps_vector(fix, latitude, longitude):
    """Return east/north displacement in metres from great-circle geometry."""
    lat1, lat2 = math.radians(fix.latitude), math.radians(latitude)
    dlat = lat2-lat1
    dlon = (math.radians(longitude-fix.longitude)+math.pi) % (2*math.pi)-math.pi
    h = math.sin(dlat/2)**2+math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
    distance = 6371008.8*2*math.asin(math.sqrt(min(1., max(0., h))))
    bearing = math.atan2(math.sin(dlon)*math.cos(lat2),
                        math.cos(lat1)*math.sin(lat2)-math.sin(lat1)*math.cos(lat2)*math.cos(dlon))
    return distance*math.sin(bearing), distance*math.cos(bearing)


@dataclass
class GPSHoppingConfig:
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
    align_tolerance_deg: float = 12.

    def __post_init__(self):
        if not math.isfinite(self.goal_latitude) or not -90 <= self.goal_latitude <= 90:
            raise ValueError('Invalid goal latitude')
        if not math.isfinite(self.goal_longitude) or not -180 <= self.goal_longitude <= 180:
            raise ValueError('Invalid goal longitude')
        for name, value in vars(self).items():
            if name in ('goal_configured', 'goal_latitude', 'goal_longitude'): continue
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError('Invalid '+name)
        if self.gps_percent > 30 or self.align_tolerance_deg > 45:
            raise ValueError('Invalid heading/output limit')


class GPSHoppingMission:
    motion_states = ('GPS_TRANSIT', 'ALIGN_GPS', 'GPS_ARRIVAL_HOLD', 'GPS_HOLD')

    def __init__(self, config, guidance, gps_config):
        self.config, self.guidance, self.gps_config = config, guidance, gps_config
        if gps_config.gps_percent > config.max_percent:
            raise ValueError('gps_percent exceeds max_percent')
        self.reset()

    def reset(self):
        self.fix = None
        self.heading, self.heading_stamp = None, -math.inf
        self.target, self.count = None, 0
        self.phase = 'GPS_TRANSIT'
        self.gps_distance = None
        self.gps_inside_since = None

    def set_goal(self, latitude, longitude, radius):
        values = dict(vars(self.gps_config))
        values.update(goal_configured=True, goal_latitude=latitude,
                      goal_longitude=longitude, gps_arrival_radius=radius)
        self.gps_config = GPSHoppingConfig(**values)
        self.phase, self.gps_distance, self.gps_inside_since = 'GPS_TRANSIT', None, None

    def clear_goal(self):
        self.gps_config.goal_configured = False
        self.phase, self.gps_distance, self.gps_inside_since = 'WAIT_GPS_GOAL', None, None

    def step(self, now, scan=None):
        gps, neutral, fix = self.gps_config, (0., 0.), self.fix
        if not gps.goal_configured: return 'WAIT_GPS_GOAL', neutral
        if (fix is None or not all(math.isfinite(v) for v in
                (fix.latitude, fix.longitude, fix.sigma, fix.stamp))
                or not -90 <= fix.latitude <= 90 or not -180 <= fix.longitude <= 180
                or not -.1 <= now-fix.stamp <= gps.gps_timeout):
            self.gps_inside_since = None
            return 'WAIT_GPS', neutral
        if not 0 <= fix.sigma <= gps.max_gps_sigma:
            self.gps_inside_since = None
            return 'GPS_UNCERTAIN', neutral
        east, north = gps_vector(fix, gps.goal_latitude, gps.goal_longitude)
        self.gps_distance = math.hypot(east, north)
        if self.gps_distance > gps.max_gps_leg:
            self.gps_inside_since = None
            return 'GPS_GOAL_TOO_FAR', neutral
        if self.gps_distance <= gps.gps_arrival_radius:
            if self.gps_inside_since is None: self.gps_inside_since = fix.stamp
            self.phase = ('GPS_HOLD' if fix.stamp-self.gps_inside_since >= gps.gps_arrival_hold
                          else 'GPS_ARRIVAL_HOLD')
            return self.phase, neutral
        self.gps_inside_since = None
        if (self.heading is None or not math.isfinite(self.heading)
                or not -.1 <= now-self.heading_stamp <= gps.heading_timeout):
            return 'WAIT_HEADING', neutral
        c, s = math.cos(self.heading), math.sin(self.heading)
        target = (c*east+s*north, -s*east+c*north)
        error = math.atan2(target[1], target[0])
        if abs(error) > math.radians(gps.align_tolerance_deg):
            turn = math.copysign(gps.gps_percent, error)
            self.phase = 'ALIGN_GPS'
            return self.phase, (-turn, turn)
        wheels = self.guidance((0., 0.), 0., target, percent=gps.gps_percent)
        if not all(math.isfinite(v) and abs(v) <= self.config.max_percent+1e-6 for v in wheels):
            raise ValueError('Invalid native guidance output')
        self.phase = 'GPS_TRANSIT'
        return self.phase, tuple(float(v) for v in wheels)
