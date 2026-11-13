"""Three GPS points, measured-neutral 10s hold, then the existing visual dock.

No learned dynamics. User PWM tables plus configured forward-base increase.
Unknown scan sectors are blocked, and narrow fallback gaps are never selected.
"""
from dataclasses import dataclass
import math
import numpy as np

from .core import wrap
from .gps_mission import GPSDockMission, GPSConfig, gps_vector
from .supplied_pwm import heading_pwm, obstacle_pwm
from .local_dock import LocalDockConfig, LocalDock, motion_clear


@dataclass
class RouteConfig(LocalDockConfig):
    wp_latitudes: tuple = ()
    wp_longitudes: tuple = ()
    wp_radius: float = 1.
    arrival_hold: float = 1.
    wp2_stop_seconds: float = 10.
    wp2_brake_pwm: int = 1400
    wp2_brake_seconds: float = .5
    wp3_confirmation_radius: float = 1.
    avoidance_range: float = 3.
    avoidance_horizon: float = 2.5
    avoidance_fov_deg: float = 75.
    forward_pwm_boost: int = 50
    avoidance_distance_weight: float = 1.0
    avoidance_angle_weight: float = 1.0

    def __post_init__(self):
        super().__post_init__()
        if type(self.wp2_brake_pwm) is not int or not 1300<=self.wp2_brake_pwm<1500:raise ValueError('Invalid WP2 brake PWM')
        if not math.isfinite(self.wp2_brake_seconds) or not 0<=self.wp2_brake_seconds<=1.:raise ValueError('Invalid WP2 brake time')
        if type(self.forward_pwm_boost) is not int or not 0<=self.forward_pwm_boost<=100:
            raise ValueError('forward_pwm_boost must be an integer in 0..100')
        self.wp_latitudes, self.wp_longitudes = tuple(self.wp_latitudes), tuple(self.wp_longitudes)
        if len(self.wp_latitudes) != 3 or len(self.wp_longitudes) != 3:
            raise ValueError('Exactly three waypoints required')
        for lat, lon in zip(self.wp_latitudes, self.wp_longitudes):
            if not math.isfinite(lat) or not -85 < lat < 85 or not math.isfinite(lon) or not -180 <= lon <= 180:
                raise ValueError('Invalid waypoint')
        for name in ('wp_radius','arrival_hold','wp2_stop_seconds','wp3_confirmation_radius',
                     'avoidance_range','avoidance_horizon','avoidance_fov_deg'):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError('Invalid '+name)
        for name in ('avoidance_distance_weight', 'avoidance_angle_weight'):
            value = getattr(self, name)
            if isinstance(value, bool) or not math.isfinite(value) or value < 0:
                raise ValueError('Invalid '+name)
        if self.avoidance_distance_weight + self.avoidance_angle_weight <= 0:
            raise ValueError('At least one avoidance weight must be positive')
        if self.avoidance_fov_deg > 85 or self.avoidance_horizon > self.avoidance_range:
            raise ValueError('Invalid avoidance field of view/range')


def choose_corridor(scan, desired, config, radius):
    """Inflated free-gap selection in body coordinates; +angle is CCW left.

This scores observed free directions with adjustable obstacle-spacing and GPS
heading-error weights while screening the full hull and unknown rays.
Only observed rays may establish a free corridor. Infinity is a clear return.
"""
    ranges = np.asarray(scan.ranges)
    angles = scan.angle_min+np.arange(len(ranges))*scan.angle_increment+scan.yaw
    angles = np.arctan2(np.sin(angles),np.cos(angles))
    valid = (np.isfinite(ranges)&(ranges >= scan.range_min)&(ranges <= scan.range_max)) | np.isposinf(ranges)
    points = scan.points(scan.yaw, scan.x, scan.y)
    if len(points) and np.min(np.linalg.norm(points,axis=1)) < radius:
        return None, {'reason':'OBSTACLE_STOP','gaps':[], 'obstacles':points[::max(1,len(points)//250)].tolist()}
    candidates = np.radians(np.arange(-config.avoidance_fov_deg, config.avoidance_fov_deg+.1, 3.))
    free, clearance, widths = [], [], []
    for direction in candidates:
        distance_angle = np.abs(np.arctan2(np.sin(angles-direction),np.cos(angles-direction)))
        sector = distance_angle <= math.radians(8)
        usable = sector & valid
        if np.count_nonzero(sector) < 3 or np.count_nonzero(usable) < .8*np.count_nonzero(sector):
            free.append(False); clearance.append(0.); widths.append(0.); continue
        # Coverage must extend past the checked short-horizon line segment.
        if scan.range_max < config.avoidance_horizon+radius:
            free.append(False); clearance.append(0.); widths.append(0.); continue
        c,s = math.cos(direction),math.sin(direction)
        if len(points):
            along = points[:,0]*c+points[:,1]*s
            cross = -points[:,0]*s+points[:,1]*c
            closest = np.column_stack((np.clip(along,0,config.avoidance_horizon),np.zeros(len(points))))
            distance = np.hypot(along-closest[:,0],cross)
            safe = not np.any(distance < radius)
            opening = float(np.min(along[(along>0)&(np.abs(cross)<radius)])) if np.any((along>0)&(np.abs(cross)<radius)) else config.avoidance_range
        else:
            safe,opening=True,config.avoidance_range
        # Unknown rays anywhere across the hull width, not just the centre, block it.
        half = math.atan2(radius,config.avoidance_horizon)
        envelope = distance_angle <= half
        safe &= np.count_nonzero(envelope)>2 and np.all(valid[envelope])
        # Width transverse to this candidate, using forward obstacle returns.
        # Each missing side is capped at avoidance_range; this is observed
        # obstacle spacing, separate from the hard swept-hull collision test.
        width = 2. * config.avoidance_range
        if len(points):
            forward = (along > 0) & (along <= config.avoidance_range)
            left = cross[forward & (cross > 0)]
            right = -cross[forward & (cross < 0)]
            left_distance = min(config.avoidance_range, float(np.min(left))) if len(left) else config.avoidance_range
            right_distance = min(config.avoidance_range, float(np.min(right))) if len(right) else config.avoidance_range
            width = left_distance + right_distance
        free.append(bool(safe));clearance.append(min(config.avoidance_range,opening));widths.append(width)
    free=np.asarray(free,dtype=bool)
    regions=[];start=None
    for index,available in enumerate([*free,False]):
        if available and start is None:start=index
        elif not available and start is not None:
            regions.append({'left_deg':float(np.degrees(candidates[index-1])),
                            'right_deg':float(np.degrees(candidates[start]))})
            start=None
    report={'gaps':regions,'obstacles':points[::max(1,len(points)//250)].tolist()}
    available=np.flatnonzero(free)
    if not len(available):
        report['reason']='NO_SAFE_GAP';return None,report
    differences=np.abs(np.arctan2(np.sin(candidates[available]-desired),np.cos(candidates[available]-desired)))
    errors_deg = np.degrees(differences)
    distances = np.asarray(widths)[available]  # metres / 1 metre
    scores = (config.avoidance_distance_weight * distances
              + config.avoidance_angle_weight * errors_deg)
    # Preserve closest-heading selection on equal costs.
    winner = min(range(len(available)), key=lambda i: (round(float(scores[i]), 12), errors_deg[i], int(available[i])))
    index = int(available[winner])
    report['objective'] = dict(distance_weight=config.avoidance_distance_weight,
        angle_weight=config.avoidance_angle_weight, distance_unit='m', angle_unit='deg',
        selected_width=float(distances[winner]), selected_error_deg=float(errors_deg[winner]),
        selected_score=float(scores[winner]), candidates=[dict(angle_deg=float(np.degrees(candidates[j])),
            width=float(distances[i]), error_deg=float(errors_deg[i]), score=float(scores[i]))
            for i, j in enumerate(available)])
    report.update(selected_angle_deg=float(np.degrees(candidates[index])),
                  selected_distance=float(clearance[index]),reason='CLEAR')
    return float(candidates[index]),report


class RouteMission(GPSDockMission):
    def __init__(self,follow_config,guidance,gps_config,route_config):
        self.route_config=route_config
        super().__init__(follow_config,guidance,gps_config)

    def reset(self):
        super().reset()
        self.stage='TO_WP1'
        self.phase=self.stage
        self.wp_index=0
        self.arrival_since=None
        self.arrival_last=-math.inf
        self.hold_elapsed=0.
        self.hold_last=None
        self.hold_remaining=self.route_config.wp2_stop_seconds
        self.avoidance={}
        self.heading_error_deg=None
        self.distance_to_wp=None
        self.intended_pwm=(1500,1500)
        self.executing=False
        self.neutral_confirmed=False
        self.completed=False
        self.command_kind='NEUTRAL'
        self.brake_elapsed=0.;self.brake_started=None
        self.brake_aborted=False
        self.local_dock=LocalDock()
        self.dock_track_id=None

    @property
    def motion_states(self):
        return ('TO_WP1','WP1_TO_WP2','AVOID_OBSTACLE','WAYPOINT_ARRIVAL','WP2_BRAKE', 'WP2_HOLD',
                'WP2_TO_WP3_ALIGN','SEARCH_SHAPE','VERIFY_TARGET','ALIGN_SHAPE',
                'APPROACH_SHAPE','ENTER_REGION','REGION_ARRIVAL_HOLD',
                'DOCK_OBSERVE_APPROACH','DOCK_ACQUIRE','DOCK_TO_Q','DOCK_ALIGN','DOCK_ENTER','DOCK_HOLD','DOCK_REVERSE')

    def observe(self,candidates,now):
        if self.stage!='DOCK_TO_WP3':return
        super().observe(candidates,now)

    def geo_state(self,now):
        gps=self.gps_config
        if (self.fix is None or not -.1<=now-self.fix.stamp<=gps.gps_timeout):return 'WAIT_GPS'
        if not 0<=self.fix.sigma<=gps.max_gps_sigma:return 'GPS_UNCERTAIN'
        if self.heading is None or not -.1<=now-self.heading_stamp<=gps.heading_timeout:return 'WAIT_HEADING'
        lat=self.route_config.wp_latitudes[self.wp_index];lon=self.route_config.wp_longitudes[self.wp_index]
        east,north=gps_vector(self.fix,lat,lon)
        self.distance_to_wp=math.hypot(east,north)
        if self.distance_to_wp>gps.max_gps_leg:return 'GPS_GOAL_TOO_FAR'
        self.heading_error_deg=math.degrees(wrap(math.atan2(north,east)-self.heading))
        return None

    def step(self,now,scan):
        self.intended_pwm=(1500,1500);self.command_kind='NEUTRAL'
        if self.completed:return 'MISSION_COMPLETE',(0.,0.)
        reason=self.geo_state(now)
        if reason:
            self.arrival_since=None;self.hold_last=None
            if self.stage=='WP2_BRAKE' and self.brake_started is not None:self.brake_aborted=True
            return reason,(0.,0.)
        self.avoidance={}
        if self.stage in ('TO_WP1','WP1_TO_WP2'):
            if scan is None or not -.1<=now-scan.stamp<=self.config.scan_timeout:
                self.arrival_since=None;return 'WAIT_SCAN',(0.,0.)
            r=self.config.footprint_radius+self.config.clearance
            angle,report=choose_corridor(scan,math.radians(self.heading_error_deg),self.route_config,r)
            self.avoidance=report
            if angle is None:
                self.arrival_since=None;return report['reason'],(0.,0.)
            if self.distance_to_wp<=self.route_config.wp_radius:
                if self.fix.stamp>self.arrival_last:
                    self.arrival_last=self.fix.stamp
                    if self.arrival_since is None:self.arrival_since=self.fix.stamp
                    if self.fix.stamp-self.arrival_since>=self.route_config.arrival_hold and self.executing:
                        if self.wp_index==0:
                            self.wp_index=1;self.stage=self.phase='WP1_TO_WP2'
                        else:
                            self.stage=self.phase='WP2_BRAKE' if self.route_config.wp2_brake_seconds>0 else 'WP2_HOLD';self.hold_last=None
                        self.arrival_since=None
                return 'WAYPOINT_ARRIVAL',(0.,0.)
            self.arrival_since=None
            desired=math.radians(self.heading_error_deg)
            blocked=abs(wrap(angle-desired))>math.radians(5) and abs(desired)<=math.radians(self.route_config.avoidance_fov_deg)
            self.command_kind='AVOIDANCE' if blocked else 'HEADING'
            boost=self.route_config.forward_pwm_boost
            self.intended_pwm=obstacle_pwm(report['selected_distance'],-math.degrees(angle),forward_boost=boost) if blocked else heading_pwm(self.distance_to_wp,-math.degrees(angle) if abs(desired)<=math.radians(self.route_config.avoidance_fov_deg) else -self.heading_error_deg,forward_boost=boost)
            if not blocked and abs(desired)>math.radians(self.route_config.avoidance_fov_deg):
                # The supplied heading table reverses a motor for large turns.
                # Require full-circle clearance before permitting this sweep.
                reason=self.scan_reason(now,scan)
                if reason:self.intended_pwm=(1500,1500);return reason,(0.,0.)
            return ('AVOID_OBSTACLE' if blocked else self.stage),self.percentages()
        if self.stage=='WP2_BRAKE':
            cfg=self.route_config
            if self.brake_aborted:return 'WP2_BRAKE_ABORTED',(0.,0.)
            if self.brake_started is not None and not self.executing:
                self.brake_aborted=True;return 'WP2_BRAKE_ABORTED',(0.,0.)
            if not self.executing:return 'WP2_BRAKE',(0.,0.)
            if scan is None or not -.1<=now-scan.stamp<=self.config.scan_timeout or not motion_clear(scan,np.array([-1.,0.]),self.config.footprint_radius+self.config.clearance):
                self.brake_aborted=True;return 'WP2_BRAKE_BLOCKED',(0.,0.)
            if self.distance_to_wp>cfg.wp_radius+self.gps_config.max_gps_sigma:
                self.brake_aborted=True;return 'WP2_DRIFT',(0.,0.)
            if self.brake_started is None:self.brake_started=now
            # Bounded command pulse, never prolonged while waiting for feedback.
            # Physical application still depends on the existing MCU ACK driver.
            self.brake_elapsed=max(0.,now-self.brake_started)
            if self.brake_elapsed+1e-8>=cfg.wp2_brake_seconds:
                self.stage=self.phase='WP2_HOLD';self.hold_last=None
                return 'WP2_HOLD',(0.,0.)
            self.intended_pwm=(cfg.wp2_brake_pwm,cfg.wp2_brake_pwm);self.command_kind='BRAKE'
            return 'WP2_BRAKE',self.percentages()
        if self.stage=='DOCK_TO_WP3' and self.route_config.dock_geometry_enabled:
            return self.step_local_dock(now,scan)
        reason=self.scan_reason(now,scan)
        if reason:
            self.hold_last=None;return reason,(0.,0.)
        if self.stage=='WP2_HOLD':
            if self.distance_to_wp>self.route_config.wp_radius+self.gps_config.max_gps_sigma:
                self.hold_last=None
                return 'WP2_DRIFT',(0.,0.)
            # Count actual neutral feedback time only while the operator's run is active.
            if self.executing and self.neutral_confirmed:
                if self.hold_last is not None:self.hold_elapsed+=max(0.,min(.25,now-self.hold_last))
                self.hold_last=now
            else:self.hold_last=None
            self.hold_remaining=max(0.,self.route_config.wp2_stop_seconds-self.hold_elapsed)
            if self.hold_remaining<=1e-6:
                self.wp_index=2;self.stage=self.phase='WP2_TO_WP3_ALIGN'
            return 'WP2_HOLD',(0.,0.)
        if self.stage=='WP2_TO_WP3_ALIGN':
            if abs(self.heading_error_deg)>self.gps_config.align_tolerance_deg:
                self.intended_pwm=heading_pwm(self.distance_to_wp,-self.heading_error_deg,holding=True)
                self.command_kind='ALIGNMENT'
                return 'WP2_TO_WP3_ALIGN',self.percentages()
            if self.executing:
                self.stage='DOCK_TO_WP3';self.phase='SEARCH_SHAPE'
                self.search_started=now;self.search_heading=self.heading;self.clear_vision()
            return 'WP2_TO_WP3_ALIGN',(0.,0.)
        reason,wheels=super().step(now,scan)
        if reason=='REGION_REACHED':
            if self.distance_to_wp>self.route_config.wp3_confirmation_radius:
                return 'WP3_POSITION_MISMATCH',(0.,0.)
            if self.executing:self.completed=True;self.stage='COMPLETE'
            return 'MISSION_COMPLETE',(0.,0.)
        if reason=='SEARCH_SHAPE':
            self.intended_pwm=heading_pwm(0.,-20.*self.search_direction,holding=True)
        elif reason=='ALIGN_SHAPE' and self.target:
            self.intended_pwm=heading_pwm(self.target.distance,-self.target_bearing(),holding=True)
        elif reason in ('APPROACH_SHAPE','ENTER_REGION') and self.target:
            self.intended_pwm=heading_pwm(self.target.distance,-self.target_bearing(),forward_boost=self.route_config.forward_pwm_boost)
        self.command_kind='DOCKING' if self.intended_pwm!=(1500,1500) else 'NEUTRAL'
        return reason,self.percentages()

    def target_bearing(self):
        yaw=math.radians(self.config.camera_yaw_deg)
        x=self.config.camera_x+math.cos(yaw)*self.target.x-math.sin(yaw)*self.target.y
        y=self.config.camera_y+math.sin(yaw)*self.target.x+math.cos(yaw)*self.target.y
        return math.degrees(math.atan2(y,x))

    def step_local_dock(self,now,scan):
        if not self.route_config.dock_calibration_verified:
            self.local_dock.report={'reason':'카메라·LiDAR 보정값 미확인: 도킹 출력을 중단함'}
            return 'DOCK_CALIBRATION_REQUIRED',(0.,0.)
        if scan is None or not -.1<=now-scan.stamp<=self.config.scan_timeout:return 'WAIT_SCAN',(0.,0.)
        if abs(scan.stamp-self.heading_stamp)>self.config.sync_tolerance:return 'WAIT_SYNC',(0.,0.)
        if self.target is None or self.count<self.config.stable_observations:
            # The existing visual search remains usable before geometry lock.
            if self.local_dock.locked_yaw is not None:return 'TARGET_LOST',(0.,0.)
            reason,_=super().step(now,scan)
            if reason=='SEARCH_SHAPE':
                self.intended_pwm=heading_pwm(0.,-20.*self.search_direction,holding=True)
                self.command_kind='DOCKING'
            return reason,self.percentages()
        if not -.1<=now-self.target.stamp<=self.config.target_timeout:return 'TARGET_LOST',(0.,0.)
        if abs(scan.stamp-self.target.stamp)>self.config.sync_tolerance:return 'WAIT_SYNC',(0.,0.)
        yaw=math.radians(self.config.camera_yaw_deg);c,s=math.cos(yaw),math.sin(yaw)
        target=np.array([self.config.camera_x+c*self.target.x-s*self.target.y,
                         self.config.camera_y+s*self.target.x+c*self.target.y])
        if self.dock_track_id is not None and self.target.track_id!=self.dock_track_id:
            return 'DOCK_ID_CHANGED',(0.,0.)
        if self.local_dock.locked_yaw is None and np.linalg.norm(target)>self.route_config.dock_observation_distance:
            error=math.degrees(math.atan2(target[1],target[0]));pivot=abs(error)>self.gps_config.align_tolerance_deg
            if not motion_clear(scan,np.zeros(2) if pivot else target,self.config.footprint_radius+self.config.clearance):
                return 'DOCK_PATH_BLOCKED',(0.,0.)
            self.intended_pwm=heading_pwm(float(np.linalg.norm(target)),-error,holding=pivot,
                                         forward_boost=self.route_config.forward_pwm_boost)
            self.command_kind='DOCKING';self.local_dock.report={'phase':'DOCK_OBSERVE_APPROACH',
                'turn_command_deg':error,'reason':'카메라 표식 방향으로 LiDAR 관측 거리까지 접근'}
            return 'DOCK_OBSERVE_APPROACH',self.percentages()
        reason,pwm,done=self.local_dock.step(now,scan,target,self.heading,self.route_config,self.config,
            self.executing,self.neutral_confirmed,self.distance_to_wp<=self.route_config.wp3_confirmation_radius)
        if self.local_dock.locked_yaw is not None and self.target.track_id>=0 and self.dock_track_id is None:
            self.dock_track_id=self.target.track_id
        self.intended_pwm=pwm;self.phase=self.local_dock.phase
        self.command_kind='DOCKING' if pwm!=(1500,1500) else 'NEUTRAL'
        if done:self.completed=True;self.stage='COMPLETE';return 'MISSION_COMPLETE',(0.,0.)
        return reason,self.percentages()

    def percentages(self):
        return tuple((pwm-1500)/4. for pwm in self.intended_pwm)
