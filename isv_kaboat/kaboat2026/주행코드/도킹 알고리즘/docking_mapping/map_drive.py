"""Explicit operator start for map navigation using the existing output driver."""
import math

MOVING_STATES={'APPROACH_TARGET','AVOID_OBSTACLE','ALIGN_TARGET'}
HOLD_STATES={'WAIT_DOCK_ALIGNMENT'}


class MapDrive:
    def __init__(self,tracking_timeout,scan_timeout,gps_timeout=2.,prediction_timeout=1.):
        self.output=None;self.requested=False;self.reason='';self.commanded_pwm=(1500,1500)
        self.tracking_timeout=tracking_timeout;self.scan_timeout=scan_timeout;self.gps_timeout=gps_timeout;self.prediction_timeout=prediction_timeout;self.commands_submitted=0

    def prepare(self,output):
        if self.requested or self.owned:raise ValueError('Stop output and wait for stop acknowledgement')
        if not output.enabled:raise ValueError('Map output adapter is disabled')
        self.output=output;self.reason='';self.commanded_pwm=(1500,1500)
        # Client creation only. No arm, publish, ownership or service request.

    @property
    def owned(self):return self.output is not None and self.output.owner is not None

    def reason_not_ready(self,plan,telemetry,age,ros_now,monotonic_now):
        if self.output is None:return 'MAP_OUTPUT_NOT_PREPARED'
        if not isinstance(telemetry,dict) or telemetry.get('rc_valid') is not True:return 'RC_RECEIVER_INVALID'
        reason=self.output.readiness.check(monotonic_now,telemetry,age,self.output.ready())
        if reason:return reason
        return self.plan_reason(plan,ros_now)

    def plan_reason(self,plan,ros_now):
        if plan is None:return 'WAIT_FRESH_MAP_PLAN'
        if plan.get('state') not in MOVING_STATES|HOLD_STATES:return 'MAP_'+str(plan.get('state','NOT_READY'))
        for key,timeout in [('pose_stamp_s',self.tracking_timeout),('imu_stamp_s',self.tracking_timeout),('scan_stamp_s',self.scan_timeout)]:
            stamp=plan.get(key)
            if not isinstance(stamp,(int,float)) or not math.isfinite(stamp) or not -.1<=ros_now-stamp<=timeout:
                return 'STALE_'+key.upper()
        if plan.get('state') in HOLD_STATES and (plan.get('navigation_mode')!='GPS_APPROACH' or list(plan.get('pwm',[]))!=[1500,1500]):return 'INVALID_HOLD_PLAN'
        if plan.get('navigation_mode')=='GPS_APPROACH':
            stamp=plan.get('gps_stamp_s')
            if not isinstance(stamp,(int,float)) or not math.isfinite(stamp) or not -.1<=ros_now-stamp<=self.gps_timeout:return 'STALE_GPS'
        if plan.get('navigation_mode')=='LIDAR_DOCKING':
            stamp=plan.get('last_registration_stamp_s')
            if not isinstance(stamp,(int,float)) or not math.isfinite(stamp) or not -.1<=ros_now-stamp<=self.prediction_timeout:return 'STALE_REGISTRATION'
        pwm=plan.get('pwm')
        if not isinstance(pwm,(list,tuple)) or len(pwm)!=2 or any(type(v) is not int or not 1100<=v<=1900 for v in pwm):
            return 'INVALID_MAP_PWM'
        return ''

    def start(self,plan,telemetry,age,ros_now,monotonic_now):
        if self.requested:raise ValueError('Map drive already started')
        reason=self.reason_not_ready(plan,telemetry,age,ros_now,monotonic_now)
        if reason:raise ValueError(reason)
        self.output.start(telemetry,age)
        self.requested=True;self.reason='';self.commanded_pwm=(1500,1500)

    def stop(self,reason='operator stop'):
        self.requested=False;self.reason=reason;self.commanded_pwm=(1500,1500)
        if self.output is not None:self.output.request_stop(reason)

    def tick(self,plan,telemetry,age,ros_now,monotonic_now):
        if self.output is None:return
        try:
            if self.requested:
                reason=self.reason_not_ready(plan,telemetry,age,ros_now,monotonic_now)
                if reason:self.stop(reason)
                else:
                    self.commanded_pwm=tuple(plan['pwm'])
                    self.output.publish(self.commanded_pwm,True)
                    self.commands_submitted+=1
            self.output.tick(telemetry,age)
            if self.requested and self.output.fault:self.stop('OUTPUT_FAULT: '+self.output.fault)
        except Exception as exc:self.stop('MAP_OUTPUT_ERROR: '+str(exc))

    def snapshot(self):
        status={} if self.output is None else self.output.snapshot()
        return dict(prepared=self.output is not None,requested=self.requested,owned=self.owned,
            armed=status.get('armed',False),stop_confirmed=status.get('stop_confirmed'),fault=status.get('fault',''),
            reason=self.reason,commanded_pwm=list(self.commanded_pwm),command_updates_submitted=self.commands_submitted)
