"""Relative docking state machine. Pure logic; no ROS or hardware side effects."""
import math
import numpy as np

NEUTRAL=(1500,1500)
TERMINAL={'IDLE','DONE','ABORT','EMERGENCY','TRACKING_LOST'}


def validate_config(c):
    for name in ('dry_run','enable_output','hardware_output_authorized','reverse_output_authorized'):
        if type(c[name]) is not bool:raise ValueError(name+' must be boolean')
    for name in ('approach_percent','max_pwm_deviation','pwm_slew_per_s','tracking_timeout_s',
                 'initial_wait_s','heading_tolerance_deg'):
        if isinstance(c[name],bool) or not math.isfinite(c[name]) or c[name]<=0:raise ValueError('Invalid '+name)
    if c['approach_percent']>10 or c['max_pwm_deviation']>80 or c['tracking_timeout_s']>2:
        raise ValueError('Docking low-speed/freshness limits exceeded')
    tolerance=c.get('approach_align_tolerance_deg',12.)
    if isinstance(tolerance,bool) or not math.isfinite(tolerance) or not 0<tolerance<=45:
        raise ValueError('Invalid approach alignment tolerance')
    r=c['reverse'];corridor=c['corridor'];t=c['tracking']
    for section,names in [(r,('enabled','path_verified')),(corridor,('envelope_verified',))]:
        for name in names:
            if type(section[name]) is not bool:raise ValueError(name+' must be boolean')
    for section,names in [(r,('percent','distance_m','max_seconds','heading_abort_deg','max_lateral_error_m')),
                          (corridor,('hull_half_length_m','half_width_m','forward_length_m','reverse_length_m')),
                          (t,('voxel_m','min_range_m','max_range_m','roi_margin_m','reacquire_gap_s'))]:
        for name in names:
            if isinstance(section[name],bool) or not math.isfinite(section[name]) or section[name]<=0:
                raise ValueError('Invalid '+name)
    if r['percent']>c['approach_percent'] or r['max_seconds']>15 or r['distance_m']>corridor['reverse_length_m']:
        raise ValueError('Reverse limits exceed observed corridor/low speed bounds')
    if r['heading_dock_deg'] is not None and not math.isfinite(r['heading_dock_deg']):raise ValueError('Invalid reverse heading')
    for low,high in [('z_min_m','z_max_m'),('visibility_z_min_m','visibility_z_max_m')]:
        if not all(math.isfinite(corridor[k]) for k in (low,high)) or corridor[low]>=corridor[high]:
            raise ValueError('Invalid corridor heights')
    for section,name in [(corridor,'coverage_bins'),(corridor,'min_returns_per_bin'),(t,'max_points')]:
        if type(section[name]) is not int or section[name]<1:raise ValueError('Invalid '+name)
    if t['min_range_m']>=t['max_range_m'] or t['max_points']>5000:raise ValueError('Invalid tracking point bounds')
    if r['enabled'] and (not r['path_verified'] or r['heading_dock_deg'] is None):
        raise ValueError('Enabled reverse requires verified path and dock_ref heading')
    return c


def auto_ready(telemetry,age):
    return (math.isfinite(age) and 0<=age<=.5 and telemetry.get('mode')==2
            and telemetry.get('receiving') is True and telemetry.get('estop_active') is False
            and telemetry.get('stop_latched',False) is False)


class DockingControl:
    def __init__(self,config,guidance,pwm_converter,turn_controller):
        self.config=validate_config(config);self.guidance=guidance;self.convert=pwm_converter;self.turn=turn_controller
        self.state='IDLE';self.reason='';self.pwm=NEUTRAL
        self.hold_since=None;self.reverse_since=None;self.reverse_origin=None
        self.last_tick=None;self.wait_since=None;self.reverse_progress_m=0.

    @property
    def active(self):return self.state not in TERMINAL

    def start(self,now):
        if self.active:raise ValueError('AUTO docking is already active')
        self.state='LOAD_REFERENCE_MAP';self.reason='';self.pwm=NEUTRAL
        self.hold_since=None;self.reverse_since=None;self.reverse_origin=None
        self.last_tick=now;self.wait_since=now;self.turn.reset();self.reverse_progress_m=0.

    def stop(self,reason='operator stop',emergency=False):
        self.state='EMERGENCY' if emergency else 'ABORT';self.reason=reason
        self.pwm=NEUTRAL;self.hold_since=None;self.turn.reset()
        return NEUTRAL

    def fail_tracking(self,reason):
        self.state='TRACKING_LOST';self.reason=reason;self.pwm=NEUTRAL
        self.hold_since=None;self.turn.reset();return NEUTRAL

    def _slew(self,desired,dt):
        limit=self.config['max_pwm_deviation'];step=self.config['pwm_slew_per_s']*dt
        result=tuple(int(round(a+np.clip(np.clip(b,1500-limit,1500+limit)-a,-step,step)))
                     for a,b in zip(self.pwm,desired))
        self.pwm=result;return result

    def tick(self,now,telemetry,telemetry_age,tracking,hopping,*,yaw_rate=0.,
             forward_clear=False,rear_clear=False,hardware_healthy=True):
        dt=0. if self.last_tick is None else max(0.,min(.2,now-self.last_tick));self.last_tick=now
        if not self.active:self.pwm=NEUTRAL;return NEUTRAL
        if telemetry.get('estop_active') is not False:return self.stop('ESTOP',True)
        if not auto_ready(telemetry,telemetry_age):return self.stop('AUTO deselected or MCU telemetry stale')
        if not hardware_healthy:return self.stop('Hardware output fault')
        if self.state=='LOAD_REFERENCE_MAP':self.state='WAIT_FOR_REGISTRATION'
        fresh=(tracking is not None and tracking.valid and tracking.stamp is not None
               and 0<=now-tracking.stamp<=self.config['tracking_timeout_s'])
        if not fresh:
            self.hold_since=None;self.pwm=NEUTRAL
            if self.state=='WAIT_FOR_REGISTRATION' and now-self.wait_since<=self.config['initial_wait_s']:
                return NEUTRAL
            return self.fail_tracking('Registration missing, rejected, or stale')
        goal=tracking.goal
        if self.state=='WAIT_FOR_REGISTRATION':self.state='APPROACH_HOPPING_POINT'
        distance=goal['distance_m'];heading=goal['heading_error_deg']
        inside=distance<=hopping.radius_m
        aligned=heading is None or abs(heading)<=self.config['heading_tolerance_deg']
        if self.state in ('APPROACH_HOPPING_POINT','HOPING'):
            if inside and aligned:
                self.state='HOPING';self.reason=''
                if self.hold_since is None:self.hold_since=now
                if now-self.hold_since>=hopping.hold_seconds:
                    reverse=self.config['reverse']
                    if not reverse['enabled']:
                        self.state='DONE';self.reason='Hold complete; reverse disabled';self.pwm=NEUTRAL;return NEUTRAL
                    if (not reverse['path_verified'] or reverse['heading_dock_deg'] is None or not rear_clear):
                        return self.stop('Reverse path/heading unverified or rear corridor unobserved/blocked')
                    direction=tracking.base_from_dock[:3,:3]@np.array([
                        math.cos(math.radians(reverse['heading_dock_deg'])),
                        math.sin(math.radians(reverse['heading_dock_deg'])),0.])
                    error=math.degrees(math.atan2(direction[1],direction[0]))
                    if abs(error)>self.config['heading_tolerance_deg']:
                        return self.stop('Reverse heading differs from current heading')
                    self.state='DOCKING_REVERSE';self.reverse_since=now
                    self.reverse_origin=np.linalg.inv(tracking.base_from_dock)[:3,3].copy()
                    self.pwm=NEUTRAL;self.turn.reset()
                else:
                    # Within tolerance: neutral or yaw-only correction, no forward creep.
                    desired=self.turn.adjust(NEUTRAL,0. if heading is None else heading,yaw_rate,now,
                        max_difference=2*self.config['max_pwm_deviation'])
                    return self._slew(desired,dt)
            else:
                self.state='APPROACH_HOPPING_POINT';self.hold_since=None
                # A differential-thrust hull cannot independently control sway and yaw.
                # Outside radius, steer toward position; inside, align heading in place.
                error=goal['bearing_deg'] if not inside else (heading or 0.)
                if not forward_clear:return self.stop('Approach corridor unobserved/blocked')
                # Same align-then-transit rule as GPS호핑/gps_hopping.py.
                # Input is the registered target in base FLU, not raw GPS.
                percent=self.config['approach_percent']
                tolerance=self.config.get('approach_align_tolerance_deg',12.)
                if inside:
                    desired=NEUTRAL
                    self.reason='ALIGN_HOPPING_HEADING'
                elif abs(error)>tolerance:
                    turn=math.copysign(percent,error)
                    desired=self.convert((-turn,turn))
                    self.reason='ALIGN_HOPPING_TARGET'
                else:
                    left,right=self.guidance((0.,0.),0.,goal['point_base'][:2],percent=percent)
                    desired=self.convert((left,right))
                    self.reason='TRANSIT_TO_HOPPING_TARGET'
                desired=self.turn.adjust(desired,error,yaw_rate,now,max_difference=2*self.config['max_pwm_deviation'])
                return self._slew(desired,dt)
        if self.state=='DOCKING_REVERSE':
            if not rear_clear:return self.stop('Rear corridor became unobserved/blocked')
            reverse=self.config['reverse']
            position=np.linalg.inv(tracking.base_from_dock)[:3,3]
            angle=math.radians(reverse['heading_dock_deg']);axis=np.array([math.cos(angle),math.sin(angle),0.])
            displacement=position-self.reverse_origin
            self.reverse_progress_m=float(-displacement@axis)
            lateral=float(np.linalg.norm(displacement+axis*self.reverse_progress_m))
            direction=tracking.base_from_dock[:3,:3]@axis
            error=math.degrees(math.atan2(direction[1],direction[0]))
            if abs(error)>reverse['heading_abort_deg'] or lateral>reverse['max_lateral_error_m']:
                return self.stop('Reverse heading/path deviation')
            if self.reverse_progress_m < -reverse['max_lateral_error_m']:
                return self.stop('Reverse motion has wrong direction')
            if self.reverse_progress_m>=reverse['distance_m']:
                self.state='DONE';self.reason='Reverse distance reached';self.pwm=NEUTRAL;return NEUTRAL
            if now-self.reverse_since>=reverse['max_seconds']:
                return self.stop('Reverse timed out before target distance')
            desired=self.convert((-reverse['percent'],-reverse['percent']))
            desired=self.turn.adjust(desired,error,yaw_rate,now,max_difference=2*self.config['max_pwm_deviation'])
            return self._slew(desired,dt)
        return NEUTRAL

    def snapshot(self,now):
        return dict(state=self.state,reason=self.reason,pwm=list(self.pwm),
                    hold_elapsed_s=0. if self.hold_since is None else max(0.,now-self.hold_since),
                    reverse_elapsed_s=0. if self.reverse_since is None else max(0.,now-self.reverse_since),
                    reverse_progress_m=self.reverse_progress_m)
