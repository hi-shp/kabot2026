"""GPS approach -> stable fixed-map GICP handoff -> GPS-free LiDAR docking."""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
from dataclasses import fields
import math,sys,threading,time
import numpy as np
import yaml
from .geometry import boat_pose,lidar_extrinsic
from .pointcloud import read_scalar_fields
from .preprocessing import scan_motion_reason
from .map_navigation import plan_v3
from .hybrid_localization import BestMapMatcher,HybridLocalization,settings
from .map_drive import MapDrive


class MapNavigationDryRun:
    def __init__(self,node):
        runtime=node.root.parents[1]/'.runtime/build/isv_shape_navigation'
        if runtime.is_dir() and str(runtime) not in sys.path:sys.path.append(str(runtime))
        from kaboat_avoidance.route_mission import RouteConfig
        from sensor_msgs.msg import LaserScan
        from rclpy.qos import QoSProfile,ReliabilityPolicy
        self.node=node;self.lock=threading.RLock();self.generation=0;self.active=False
        self.options=settings(node.config);self.primary=None;self.raw=None
        self.scan_history={False:deque(maxlen=16),True:deque(maxlen=16)}
        self.target=None;self.origin=None;self.matcher=None;self.localization=None;self.tracker=None
        self.state='IDLE';self.phase='IDLE';self.reason='';self.plan=None
        self.pose=None;self.pose_stamp=None;self.current_scan=np.empty((0,3));self.used_gps_seed=False
        self.last_scan=-math.inf;self.last_job=-math.inf;self.job=None;self.job_generation=None
        self.executor=ThreadPoolExecutor(max_workers=1,thread_name_prefix='map_gicp')
        self.uncertainty=None;self.processing_ms=0.;self.gps_approach_distance=None
        self.last_gps=None
        self.parameters=yaml.safe_load((node.root.parents[1]/'설정/course_mission.yaml').read_text())['course_mission']['ros__parameters']
        self.route=RouteConfig(**{f.name:self.parameters[f.name] for f in fields(RouteConfig) if f.name in self.parameters})
        self.radius=self.parameters['footprint_radius']+self.parameters['clearance']
        self.drive=MapDrive(node.config['autonomy']['tracking_timeout_s'],self.parameters['scan_timeout'],
            self.parameters['gps_timeout'],self.options['max_prediction_age_s'])
        self.output_timer=node.create_timer(.02,self.output_tick)
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscriptions=[node.create_subscription(LaserScan,self.parameters['scan_topic'],lambda m:self.on_scan(m,False),qos),
            node.create_subscription(LaserScan,self.parameters['scan_fallback_topic'],lambda m:self.on_scan(m,True),qos)]

    def on_scan(self,message,raw):
        from isv_shape_navigation.core import Scan
        p=self.parameters;stamp=self.node.stamp(message)
        if (message.header.frame_id!=p['scan_frame'] or not len(message.ranges)
            or not -.1<=self.node.now_s()-stamp<=p['scan_timeout']
            or not all(math.isfinite(v) for v in (message.angle_min,message.angle_increment,message.range_min,message.range_max))
            or message.angle_increment==0 or not 0<=message.range_min<message.range_max):return
        scan=Scan(np.asarray(message.ranges,dtype=float),message.angle_min,message.angle_increment,
            max(p['avoidance_lidar_min_range'],message.range_min),message.range_max,stamp,
            math.radians(p['scan_yaw_deg']),p['scan_x'],p['scan_y'])
        with self.lock:
            previous=self.raw if raw else self.primary
            if previous is not None and stamp<=previous.stamp:return
            self.scan_history[raw].append(scan)
            if raw:self.raw=scan
            else:self.primary=scan

    def start(self,expected_generation=None):
        n=self.node
        with n.map_lock:
            if n.session.active or n.autonomy.active:raise ValueError('Stop mapping/AUTO before hybrid docking')
            if not n.gps_map_loaded or not n.gps_map_name:raise ValueError('Load a saved GPS map first')
            target=n.geographic_maps.read_target(n.gps_map_name)
            if target is None:raise ValueError('Save a target in this map first')
            matcher=BestMapMatcher(n.mapper.map.points(),lidar_extrinsic(n.config['sensors']),n.config)
            with self.lock:
                if expected_generation is not None and expected_generation!=self.generation:raise ValueError('Start cancelled by stop')
                if self.active:raise ValueError('Hybrid docking already active')
                if self.drive.owned:raise ValueError('WAIT_OUTPUT_STOP_ACK')
                self.generation+=1;self.active=True;self.matcher=matcher;self.localization=HybridLocalization(n.config)
                self.tracker=self.localization;self.target=deepcopy(target);self.origin=deepcopy(n.mapper.enu)
                self.state='WAIT_GPS_APPROACH';self.phase='GPS_APPROACH';self.reason='';self.plan=None
                self.pose=None;self.pose_stamp=None;self.used_gps_seed=False;self.uncertainty=None
                self.last_scan=-math.inf;self.last_job=-math.inf;self.gps_approach_distance=None
                self.last_gps=None

    def stop(self,reason='operator stop'):
        with self.lock:
            self.generation+=1;self.active=False;self.state='STOPPED';self.phase='STOPPED';self.reason=reason
            self.drive.stop(reason);self.plan=None;self.pose=None;self.pose_stamp=None
            self.current_scan=np.empty((0,3))

    def fail(self,reason,generation):
        with self.lock:
            if generation!=self.generation:return
            driving=self.drive.requested
            self.drive.stop(reason);self.reason=reason;self.plan=None;self.pose=None;self.pose_stamp=None
            if (self.localization is not None and self.localization.locked) or driving:
                self.active=False;self.generation+=1;self.state='TRACKING_LOST' if self.phase=='LIDAR_DOCKING' else 'APPROACH_ABORTED'
            else:self.state=reason

    def gps_pose(self,fixes,imu,stamp):
        n=self.node;p=self.parameters
        valid=[f for f in fixes if f.status.status>=0 and -.1<=n.now_s()-n.stamp(f)<=p['gps_timeout']
               and f.position_covariance_type!=0]
        if not valid:raise ValueError('WAIT_VALID_GPS')
        fix=max(valid,key=n.stamp);cov=np.asarray(fix.position_covariance)[[0,4,8]]
        if not np.isfinite(cov).all() or np.any(cov<0) or max(cov[:2])>p['max_gps_sigma']**2:
            raise ValueError('GPS_COVARIANCE')
        q=imu.orientation;pose=boat_pose(self.origin.to_enu([fix.latitude,fix.longitude,fix.altitude]),[q.x,q.y,q.z,q.w],n.config['sensors'])
        distance=float(np.linalg.norm(np.asarray(self.target['point_map_xy_m'])-pose[:2,3]))
        if distance>p['max_gps_leg']:raise ValueError('GPS_GOAL_TOO_FAR')
        sigma=float(np.sqrt(max(cov[:2])));fix_stamp=n.stamp(fix)
        if self.last_gps is not None and fix_stamp>self.last_gps[0]:
            old_stamp,old_pose,old_sigma=self.last_gps
            bound=n.config['mapping']['gps_jump_margin_m']+n.config['mapping']['max_boat_speed_mps']*(fix_stamp-old_stamp)+2*np.hypot(sigma,old_sigma)
            if np.linalg.norm(pose[:2,3]-old_pose[:2,3])>bound:raise ValueError('GPS_APPROACH_POSITION_JUMP')
        self.last_gps=(fix_stamp,pose.copy(),sigma)
        return pose,fix_stamp,sigma

    def _consume_job(self,now):
        if self.job is None or not self.job.done():return
        job=self.job;generation=self.job_generation;self.job=None
        if generation!=self.generation or not self.active:return
        try:result=job.result()
        except Exception as exc:result=dict(accepted=False,reason='REGISTRATION_ERROR: '+str(exc),stamp=now)
        self.localization.accept(result,now)
        if self.localization.locked:
            self.phase='LIDAR_DOCKING';self.used_gps_seed=True

    def _submit_job(self,cloud,imus,gps_pose,gps_stamp):
        n=self.node;m=n.config['mapping'];stamp=n.stamp(cloud)
        if (self.job is not None or stamp<=self.last_scan or stamp-self.last_job<1/self.options['registration_hz']):return
        self.last_scan=stamp
        if not n.fresh(stamp) or cloud.header.frame_id!=n.config['sensors']['lidar_frame']:return
        imu=min(imus,key=lambda message:abs(n.stamp(message)-stamp))
        if (not n.fresh(n.stamp(imu)) or abs(n.stamp(imu)-stamp)>m['imu_sync_tolerance_s']
            or imu.header.frame_id!=n.config['sensors']['imu_frame']):return
        q=imu.orientation;rotation=boat_pose([0.,0.,0.],[q.x,q.y,q.z,q.w],n.config['sensors'])[:3,:3]
        motion=n.config['scan_motion'];rate=imu.angular_velocity
        if motion.get('enabled',False):
            duration=None
            if any(f.name=='time' for f in cloud.fields):
                values=read_scalar_fields(cloud,('time',)).ravel();values=values[np.isfinite(values)]
                if len(values)>1:duration=float(np.ptp(values))
            if duration is None or duration<=0:duration=motion.get('fallback_duration_s')
            reason=scan_motion_reason(duration,float(np.linalg.norm([rate.x,rate.y,rate.z])),m['max_boat_speed_mps'],motion)
            if reason:self.reason=reason;return
        if self.localization.locked:
            seed,reason,_=self.localization.predict(stamp,rotation)
            if seed is None:return
        else:
            if gps_pose is None or abs(gps_stamp-stamp)>m['gps_sync_tolerance_s']:return
            seed=gps_pose
            # Out of sight: keep GPS guidance running rather than queue a futile fit.
            low=self.matcher.points_map.min(0);high=self.matcher.points_map.max(0)
            margin=n.config['autonomy']['tracking']['max_range_m']+self.options['initial_search_xy_m']
            if np.any(seed[:2,3]<low[:2]-margin) or np.any(seed[:2,3]>high[:2]+margin):return
        points=read_scalar_fields(cloud,('x','y','z')).reshape(-1,3).copy()
        generation=self.generation;matcher=self.matcher;tracking=self.localization.locked
        self.last_job=stamp;self.job_generation=generation
        self.job=self.executor.submit(matcher.match,points,rotation,seed,stamp,tracking,
            lambda:generation!=self.generation or not self.active)

    def process_latest(self):
        from isv_shape_navigation.core import select_navigation_scan
        n=self.node;p=self.parameters
        with self.lock:
            if not self.active:return
            generation=self.generation;self._consume_job(n.now_s());locked=self.localization.locked
        with n.data_lock:
            cloud=n.latest_cloud;imus=list(n.imu)
            # GPS no longer enters guidance or localization after handoff.
            fixes=None if locked else list(n.gps)
        try:
            if not imus:raise ValueError('WAIT_IMU')
            with self.lock:
                scan,source=select_navigation_scan(self.primary,self.raw,n.now_s(),p['scan_timeout'])
            if scan is None or source=='unavailable':raise ValueError('WAIT_AVOIDANCE_SCAN')
            # Use attitude at the scan's measurement time, not the newest
            # attitude callback. Keep both freshness and the stricter IMU skew.
            imu=min(imus,key=lambda message:abs(n.stamp(message)-scan.stamp));imu_stamp=n.stamp(imu)
            if (not -.1<=n.now_s()-imu_stamp<=p['heading_timeout']
                or abs(imu_stamp-scan.stamp)>n.config['mapping']['imu_sync_tolerance_s']
                or imu.header.frame_id!=n.config['sensors']['imu_frame']):raise ValueError('WAIT_IMU_SYNC')
            q=imu.orientation;rotation=boat_pose([0.,0.,0.],[q.x,q.y,q.z,q.w],n.config['sensors'])[:3,:3]
            with self.lock:
                if generation!=self.generation or not self.active:return
                gps_pose=None;gps_stamp=None;sigma=None
                if not locked:gps_pose,gps_stamp,sigma=self.gps_pose(fixes,imu,imu_stamp)
                if cloud is not None:self._submit_job(cloud,imus,gps_pose,gps_stamp)
                raw=self.raw
                if abs(scan.stamp-imu_stamp)>p['sync_tolerance']:raise ValueError('WAIT_SCAN_SYNC')
                raw=raw if raw is not None and -.1<=n.now_s()-raw.stamp<=p['scan_timeout'] and abs(raw.stamp-scan.stamp)<=p['sync_tolerance'] else None
                if locked:
                    # Predict to the fresh scan, not to a stale optimization start.
                    pose,source_mode,self.uncertainty=self.localization.predict(scan.stamp,rotation)
                    if pose is None:raise ValueError(source_mode)
                    plan=plan_v3(pose,self.target['point_map_xy_m'],self.matcher.points_map,scan,self.route,self.radius,raw)
                    self.pose=pose.copy();self.pose_stamp=scan.stamp
                    plan.update(navigation_mode='LIDAR_DOCKING',pose_source=source_mode,
                        last_registration_stamp_s=self.localization.poses[-1][0],prediction_uncertainty_m=self.uncertainty)
                else:
                    plan=plan_v3(gps_pose,self.target['point_map_xy_m'],np.empty((0,3)),scan,self.route,self.radius,raw)
                    self.gps_approach_distance=plan['goal']['distance_m']
                    # No GPS-based final arrival. Wait outside the uncertainty margin.
                    near=max(self.options['gps_approach_stop_m'],2*sigma+self.radius)
                    if self.gps_approach_distance<=near:
                        plan.update(state='WAIT_DOCK_ALIGNMENT',pwm=[1500,1500])
                    plan.update(navigation_mode='GPS_APPROACH',pose_source='LIVE_GPS_IMU',gps_stamp_s=gps_stamp)
                    self.pose=None;self.pose_stamp=None
                plan.update(pose_stamp_s=scan.stamp,imu_stamp_s=imu_stamp,scan_stamp_s=scan.stamp)
                self.plan=plan;self.state=plan['state'];self.reason=''
                if locked and plan['state']=='ARRIVED':self.active=False
        except Exception as exc:self.fail(str(exc),generation)

    def telemetry(self):
        with self.node.data_lock:return dict(self.node.rc),time.monotonic()-self.node.arrivals.get('rc',-math.inf)

    def prepare_output(self):
        from .actuation import DockingOutput
        n=self.node
        with n.map_lock:
            if n.session.active or n.autonomy.active:raise ValueError('Stop mapping/old AUTO first')
            if not n.gps_map_loaded or n.gps_target is None:raise ValueError('Load the map and saved target first')
            with self.lock:
                if self.drive.requested or self.drive.owned:raise ValueError('Wait for output stop acknowledgement')
                if self.drive.output is None:
                    self.drive.prepare(DockingOutput(n,dict(n.config['autonomy'],dry_run=False,enable_output=True,hardware_output_authorized=True)))

    def start_drive(self,expected_generation=None):
        # HTTP status is a 1Hz snapshot; recompute from current sensor buffers
        # before the one-shot arm decision. Never arm on a cached GUI plan.
        self.process_latest()
        telemetry,age=self.telemetry()
        with self.lock:
            if expected_generation is not None and expected_generation!=self.generation:raise ValueError('Map drive start cancelled by stop')
            if not self.active:raise ValueError('Start GPS approach / LiDAR preview first')
            self.drive.start(self.plan,telemetry,age,self.node.now_s(),time.monotonic())

    def on_telemetry(self,telemetry):
        with self.lock:
            if self.drive.requested and (telemetry.get('rc_valid') is not True or telemetry.get('mode')!=2
                or telemetry.get('receiving') is not True or telemetry.get('estop_active') is not False or telemetry.get('stop_latched',False)):
                self.stop('RC/manual override or MCU stop')

    def output_tick(self):
        telemetry,age=self.telemetry()
        with self.lock:
            driving=self.drive.requested;self.drive.tick(self.plan,telemetry,age,self.node.now_s(),time.monotonic())
            if driving and not self.drive.requested:
                if self.plan is not None and self.plan.get('state')=='ARRIVED':self.active=False;self.state='ARRIVED'
                else:self.stop(self.drive.reason)

    def snapshot(self):
        telemetry,age=self.telemetry()
        with self.lock:
            plan=deepcopy(self.plan);now=self.node.now_s()
            if plan is not None and self.drive.plan_reason(plan,now).startswith('STALE_'):plan=None
            blocker=self.drive.reason_not_ready(plan,telemetry,age,now,time.monotonic())
            localization=self.localization
            return dict(active=self.active,state=self.state,phase=self.phase,reason=self.reason,plan=plan,
                dry_run=not self.drive.requested,hardware_output_enabled=self.drive.output is not None,
                actuator_commands_sent=0 if self.drive.output is None else None,
                drive=self.drive.snapshot(),drive_ready=not blocker,drive_blocker=blocker,
                calculated_pwm=[1500,1500] if plan is None else plan['pwm'],
                registration={} if localization is None else deepcopy(localization.metrics),
                confirmation_count=0 if localization is None else localization.confirmed,confirmation_required=self.options['confirm_scans'],
                gps_policy='APPROACH_THEN_EXCLUDED',gps_seed_used=self.used_gps_seed,
                pose_stamp_s=self.pose_stamp,pose_age_s=None if self.pose_stamp is None else max(0.,now-self.pose_stamp),
                prediction_uncertainty_m=self.uncertainty,registration_busy=self.job is not None and not self.job.done(),
                gps_approach_distance_m=self.gps_approach_distance,reference_updated=False,
                map_name=None if self.target is None else self.target['map_name'])

    def close(self):
        self.stop('node shutdown');self.executor.shutdown(wait=True,cancel_futures=True)
