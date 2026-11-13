"""ROS sensor adapter and short independent control tick for relative docking."""
from copy import deepcopy
from types import SimpleNamespace
import math
import sys
import threading
import time
import numpy as np
from sensor_msgs_py import point_cloud2
from .pointcloud import read_scalar_fields
from .geometry import ENU,boat_pose,lidar_extrinsic,apply
from .tracking import DockTracker
from .control import DockingControl,auto_ready
from .actuation import DockingOutput
from .hopping import read_hopping
from .preprocessing import scan_motion_reason
from .safety import corridor_clear


class DockingAutonomy:
    def __init__(self,node):
        self.node=node;self.config=node.config['autonomy'];self.lock=threading.RLock()
        # Reuse the project's guidance, calibrated PWM conversion and yaw feedback.
        sys.path.insert(0,str(node.root.parents[1]/'공통모듈'/'isv_shape_navigation'))
        from isv_water_trials.tank_core import guidance
        from isv_water_trials.actuator_config import command_pwm
        from isv_shape_navigation.follow import native_pwm
        from isv_shape_navigation.turn_control import YawTurnController
        self.controller=DockingControl(self.config,guidance,lambda p:native_pwm(p,command_pwm),YawTurnController())
        self.output=DockingOutput(node,self.config)
        self.tracker=None;self.sample=None;self.generation=0;self.last_scan=-math.inf
        self.world_from_dock=None;self.world_stamp=None;self.reason='';self.raw_pose=None
        self.world_scan=np.empty((0,3))
        self.forward_clear=False;self.rear_clear=False;self.yaw_rate=0.;self.sensor_stamp=None
        self.timer=node.create_timer(.05,self.control_tick)
        self.output_timer=node.create_timer(.02,self.output_tick)

    def telemetry(self):
        with self.node.data_lock:
            return dict(self.node.rc),time.monotonic()-self.node.arrivals.get('rc',-math.inf)

    @property
    def active(self):
        with self.lock:return self.controller.active

    def start(self,expected_generation=None):
        n=self.node;telemetry,age=self.telemetry()
        if getattr(n,'map_navigation',None) is not None and n.map_navigation.active:raise ValueError('Stop map Dry-run before AUTO')
        if n.config['sensors']['heading_reference']=='relative':
            raise ValueError('RELATIVE_MAP_PREVIEW_ONLY: true-north alignment not verified')
        if not auto_ready(telemetry,age):raise ValueError('AUTO_NOT_READY')
        with n.map_lock:
            if n.session.active:raise ValueError('Finish RC mapping before AUTO')
            if n.reference is None or n.reference_folder is None:raise ValueError('Load/save reference before AUTO')
            if n.hopping_dirty:raise ValueError('Save hopping changes before AUTO')
            hopping=read_hopping(n.reference_folder,n.reference)
            if hopping is None:raise ValueError('No saved map-bound hopping point')
            # Validate sensor convention/extrinsics before accepting the start command.
            boat_pose([0.,0.,0.],[0.,0.,0.,1.],n.config['sensors'])
            tracker=DockTracker(n.reference,hopping,lidar_extrinsic(n.config['sensors']),self.config['tracking'])
            with self.lock:
                if expected_generation is not None and expected_generation!=self.generation:
                    raise ValueError('AUTO start cancelled by stop/override')
                telemetry,age=self.telemetry()
                if not auto_ready(telemetry,age):raise ValueError('AUTO_NOT_READY')
                if self.controller.active:raise ValueError('AUTO already active')
                self.output.start(telemetry,age)
                self.generation+=1;self.tracker=tracker;self.sample=None;self.last_scan=-math.inf
                self.world_from_dock=None;self.world_stamp=None;self.raw_pose=None
                self.world_scan=np.empty((0,3))
                self.forward_clear=False;self.rear_clear=False;self.sensor_stamp=None
                self.controller.start(n.now_s());self.reason=''

    def stop(self,reason='operator stop',emergency=False):
        with self.lock:
            self.generation+=1;self.controller.stop(reason,emergency)
            self.sample=None;self.output.request_stop(reason)

    def on_telemetry(self,telemetry):
        # Fast callback latches manual override even if GICP is still running.
        if not auto_ready(telemetry,0.):
            with self.lock:
                if self.controller.active:self.stop('RC/manual override or MCU stop',telemetry.get('estop_active') is not False)
                else:self.generation+=1

    def invalidate(self,reason):
        with self.lock:self.sample=None;self.reason=reason

    def process_latest(self):
        n=self.node;m=n.config['mapping'];s=n.config['sensors']
        with self.lock:tracker=self.tracker;generation=self.generation
        if tracker is None:return
        with n.data_lock:
            cloud,n.latest_cloud=n.latest_cloud,None;imus=list(n.imu);fixes=list(n.gps)
        if cloud is None:return
        stamp=n.stamp(cloud)
        if stamp<=self.last_scan:return
        self.last_scan=stamp
        try:
            if not n.fresh(stamp):raise ValueError('STALE_LIDAR')
            if cloud.header.frame_id!=s['lidar_frame']:raise ValueError('LIDAR_FRAME_MISMATCH')
            if not imus:raise ValueError('WAIT_IMU')
            imu=min(imus,key=lambda message:abs(n.stamp(message)-stamp))
            if (not n.fresh(n.stamp(imu)) or abs(n.stamp(imu)-stamp)>m['imu_sync_tolerance_s']
                    or imu.header.frame_id!=s['imu_frame']):raise ValueError('WAIT_IMU_SYNC')
            q=imu.orientation;quaternion=[q.x,q.y,q.z,q.w]
            rotation=boat_pose([0.,0.,0.],quaternion,s)[:3,:3]
            rate=imu.angular_velocity;rate_vector=np.array([rate.x,rate.y,rate.z])
            if not np.isfinite(rate_vector).all():raise ValueError('INVALID_IMU_RATE')
            yaw_rate=float((rotation.T@boat_pose([0.,0.,0.],quaternion,
                dict(s,imu_from_base_rpy_deg=[0.,0.,0.]))[:3,:3]@rate_vector)[2])
            motion=n.config['scan_motion']
            if motion.get('enabled',False):
                duration=None
                if any(f.name=='time' for f in cloud.fields):
                    offsets=read_scalar_fields(cloud,('time',)).ravel()
                    offsets=offsets[np.isfinite(offsets)]
                    if len(offsets)>1:duration=float(np.ptp(offsets))
                if duration is None or duration<=0:duration=motion.get('fallback_duration_s')
                reason=scan_motion_reason(duration,float(np.linalg.norm(rate_vector)),m['max_boat_speed_mps'],motion)
                if reason:raise ValueError(reason)
            world_pose=None
            valid=[f for f in fixes if f.status.status>=0 and n.fresh(n.stamp(f))
                   and abs(n.stamp(f)-stamp)<=m['gps_sync_tolerance_s'] and f.position_covariance_type!=0]
            if valid:
                fix=min(valid,key=lambda f:abs(n.stamp(f)-stamp))
                covariance=np.asarray(fix.position_covariance)[[0,4,8]]
                if (np.isfinite(covariance).all() and np.all(covariance>=0)
                        and max(covariance[:2])<=m['max_gps_sigma_m']**2):
                    world_pose=boat_pose(ENU(tracker.reference.origin_lla).to_enu([fix.latitude,fix.longitude,fix.altitude]),quaternion,s)
            points=read_scalar_fields(cloud,('x','y','z')).reshape(-1,3)
            began=time.monotonic()
            accepted=tracker.process(points,stamp,world_pose,rotation)
            base_points=apply(points,tracker.extrinsic)
            forward=corridor_clear(base_points,self.config['corridor'])
            rear=corridor_clear(base_points,self.config['corridor'],reverse=True)
            with self.lock:
                if generation!=self.generation or not self.controller.active:return
                self.reason=tracker.metrics.get('reason','')
                self.sample=(SimpleNamespace(valid=True,stamp=stamp,goal=deepcopy(tracker.goal),
                    base_from_dock=tracker.base_from_dock.copy()) if accepted else None)
                self.yaw_rate=yaw_rate;self.sensor_stamp=stamp
                self.forward_clear=forward;self.rear_clear=rear
                if accepted:
                    # Freeze the last display anchor when GPS is unavailable. Never
                    # snap a live map back to its original saved ENU pose.
                    if tracker.world_from_dock is not None:
                        self.world_from_dock=tracker.world_from_dock.copy()
                        self.world_stamp=stamp;self.raw_pose=world_pose.copy()
                    if self.world_from_dock is not None:
                        self.world_scan=apply(tracker.target,self.world_from_dock@np.linalg.inv(tracker.lidar_from_dock))
                n.processing_ms=(time.monotonic()-began)*1000
        except (ValueError,TypeError,RuntimeError) as exc:self.invalidate(str(exc))

    def control_tick(self):
        telemetry,age=self.telemetry();now=self.node.now_s()
        with self.lock:
            try:
                pwm=self.controller.tick(now,telemetry,age,self.sample,
                    None if self.tracker is None else self.tracker.hopping,yaw_rate=self.yaw_rate,
                    forward_clear=self.forward_clear,rear_clear=self.rear_clear,hardware_healthy=not self.output.fault)
                active=self.controller.active
                if not active:self.output.request_stop(self.controller.reason or self.controller.state)
                self.output.publish(pwm,active,self.controller.state=='DOCKING_REVERSE')
            except Exception as exc:
                self.stop('CONTROL_ERROR: '+str(exc))

    def output_tick(self):
        telemetry,age=self.telemetry()
        self.output.tick(telemetry,age)

    def snapshot(self):
        telemetry,age=self.telemetry()
        with self.lock:
            result=self.controller.snapshot(self.node.now_s())
            result.update(self.output.snapshot(),active=self.controller.active,registration_reason=self.reason,
                registration={} if self.tracker is None else deepcopy(self.tracker.metrics),
                relative_goal=None if self.sample is None else deepcopy(self.sample.goal),
                tracking_stamp_s=None if self.sample is None else self.sample.stamp,
                world_anchor_stamp_s=self.world_stamp,forward_clear=self.forward_clear,rear_clear=self.rear_clear)
            result['auto_ready']=auto_ready(telemetry,age) and self.node.config['sensors']['heading_reference']!='relative'
            result['world_anchor_age_s']=(None if self.world_stamp is None else max(0.,self.node.now_s()-self.world_stamp))
            return result
