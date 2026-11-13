"""Passive global localization; no motor commands, fixed map remains untouched."""
from concurrent.futures import ThreadPoolExecutor
import json
from copy import deepcopy,copy
import math
import os
import signal
import threading
import time
from .tank_global_match import GlobalMatcher,ScanEvidence,DisplayTrack,MotionHistory,handoff_ready,partial_handoff_ready,retain_display_candidate,identify_candidates,scan_points,scan_stable,wrap
from .tank_heading_fusion import ImuLidarFusion, ned_yaw_rate
from .tank_map_file import read_bundle
from .outdoor_route import waypoint_kind,gps_docking_version
from .tank_localization_prior import PosePriorStore, project_prior
from .tank_localization_region import LocalizationRegionStore,region_allows


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.executors import ExternalShutdownException
    from sensor_msgs.msg import LaserScan,Imu
    from vectornav_msgs.msg import CommonGroup
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from std_msgs.msg import String,Empty
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stopped=threading.Event()
    signal.signal(signal.SIGINT,lambda *_:stopped.set())
    signal.signal(signal.SIGTERM,lambda *_:stopped.set())
    try:os.nice(10)
    except OSError:pass

    class Localizer(Node):
        def __init__(self):
            super().__init__('tank_global_localizer')
            self.declare_parameter('file','');self.declare_parameter('automatic',True)
            self.map_correction=bool(self.declare_parameter('map_correction',False).value)
            self.min_score=float(self.declare_parameter('accept_score',.60).value)
            self.min_margin=float(self.declare_parameter('accept_margin',.0375).value)
            if not (.5<=self.min_score<=1 and .01<=self.min_margin<=1):
                raise ValueError('accept_score: 0.5..1; accept_margin: 0.01..1')
            self.map_id,_,files=read_bundle(self.get_parameter('file').value)
            self.partial_match=bool(self.declare_parameter('partial_match',False).value)
            self.matcher=GlobalMatcher(json.loads(files['grid.json']),files['grid.bin'],partial=self.partial_match,
                navigation_info=json.loads(files['navigation_grid.json']) if 'navigation_grid.json' in files else None,
                navigation_cells=files.get('navigation_grid.bin'))
            self.pool=ThreadPoolExecutor(max_workers=1);self.cancel=threading.Event();self.future=None
            self.scan=None;self.session={};self.session_at=0.;self.trial={};self.trial_at=0.
            self.generation=0;self.future_generation=0;self.next_search=time.monotonic()+2
            self.requested=bool(self.get_parameter('automatic').value)
            self.manual_locked=False
            self.navigation_active=False;self.navigation_at=0.;self.match_lost_at=None
            self.navigation_context=None;self.docking_hold=False
            self.gps_docking_active=False
            self.observe_registered_gps=False
            self.prior_store=PosePriorStore();self.expected_prior=None;self.prior_anchor=None;self.prior_pivot=False
            self.region_store=LocalizationRegionStore();self.region_check_at=0.;self.region_key=None
            self.refresh_region()
            self.tracking_future=None;self.tracking_generation=0;self.tracking_stamp=0.;self.tracking_reference=None
            self.next_tracking=0.;self.local_good_at=0.;self.local_confirmations=0;self.imu=None;self.imu_at=0.
            self.local_reason='주변 지도 정합 대기'
            self.last_display=None;self.odom_outliers=0;self.actuators={};self.actuators_at=0.
            self.report={'state':'WAITING','reason':'자동 위치 탐색 준비 중','map_id':self.map_id,'candidates':[],
                         'partial_match':self.partial_match,'docking_match_ready':False}
            self.confirmations=0;self.last_confirm=0;self.pending=None
            self.sent_stamp=None;self.reference=None
            self.evidence=ScanEvidence();self.odom=None;self.scan_pose=None;self.evidence_info={};self.future_info={}
            self.display=DisplayTrack();self.future_pose=None;self.background_period=5. if self.partial_match else 60.
            self.motion=MotionHistory();self.trail_at=0.;self.trail_key=None
            self.fusion=ImuLidarFusion();self.raw_odom=None
            self.review_period=2.;self.next_review=0.;self.review_candidates=[];self.review_pose=None;self.future_kind='global'
            self.status=self.create_publisher(String,'/tank/localization_candidates',10)
            self.fused_pub=self.create_publisher(Odometry,'/tank/fused_odom',qos_profile_sensor_data)
            self.initial_pub=self.create_publisher(PoseWithCovarianceStamped,'/initialpose',1)
            self.create_subscription(LaserScan,'/scan',self.receive_scan,qos_profile_sensor_data)
            self.create_subscription(Imu,'/vectornav/imu',self.receive_imu,qos_profile_sensor_data)
            self.create_subscription(CommonGroup,'/vectornav/raw/common',self.receive_raw_imu,qos_profile_sensor_data)
            self.create_subscription(Odometry,'/tank/odom',self.receive_odom,qos_profile_sensor_data)
            self.create_subscription(String,'/tank/session',self.receive_session,10)
            self.create_subscription(String,'/tank/status',self.receive_trial,qos_profile_sensor_data)
            self.create_subscription(String,'/outdoor/status',self.receive_navigation,qos_profile_sensor_data)
            self.create_subscription(String,'/actuators/status',self.receive_actuators,qos_profile_sensor_data)
            self.create_subscription(Empty,'/tank/localize/request',self.request,1)
            self.create_subscription(PoseWithCovarianceStamped,'/initialpose',self.initial_seen,1)
            self.create_timer(.5,self.tick)
            self.create_timer(.1,self.broadcast)

        def receive_odom(self,msg):
            if msg.header.frame_id!='tank_odom' or msg.child_frame_id!='velodyne':return
            q=msg.pose.pose.orientation;p=msg.pose.pose.position
            if not .9<math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)<1.1:return
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            current=(p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
            if not all(math.isfinite(v) for v in current):return
            previous=self.raw_odom
            dt=stamp-previous[0] if previous is not None else 0.
            jump=previous is not None and (math.hypot(current[0]-previous[1][0],current[1]-previous[1][1])>max(.2,dt*1.5) or abs(wrap(current[2]-previous[1][2]))>max(.2,dt*2))
            if previous is not None:
                if dt<=0:return
                if jump and self.manual_locked:
                    self.odom_outliers+=1
                    if self.odom_outliers<3:return
                else:self.odom_outliers=0
            self.raw_odom=(stamp,current)
            effective=self.fusion.update(stamp,current,rebase=jump or dt>1.) if self.manual_locked else current
            if previous is not None and (dt>1 or jump):
                if not self.manual_locked:self.display=DisplayTrack()
                self.generation+=1;self.pending=None;self.next_search=0;self.evidence.clear()
                self.review_candidates=[];self.review_pose=None;self.motion.clear()
                if self.manual_locked and jump:
                    prior=self.display.pose(self.odom[1])
                    self.manual_odom_reset=prior is None
                    if prior is not None:
                        self.display.adopt(dict(candidates=[prior],display_fallback=True,path_checked=True),effective,effective,self.odom[0])
                    self.local_good_at=0.;self.local_confirmations=0;self.next_tracking=0.;self.scan_pose=None
                    self.local_reason='오도메트리 변경 · 마지막 위치 주변 재정합 중'
            old=self.odom
            self.odom=(stamp,effective)
            self.motion.add(effective)
            if self.manual_locked and self.fusion.active:
                fused=deepcopy(msg)
                fused.pose.pose.position.x=effective[0];fused.pose.pose.position.y=effective[1]
                q=fused.pose.pose.orientation;q.x=q.y=0.;q.z=math.sin(effective[2]/2);q.w=math.cos(effective[2]/2)
                if old is not None and stamp>old[0]:
                    fused.twist.twist.angular.z=wrap(effective[2]-old[1][2])/(stamp-old[0])
                self.fused_pub.publish(fused)
        def receive_scan(self,msg):
            self.scan=msg;self.scan_pose=None
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            if self.odom is None or abs(stamp-self.odom[0])>.15:return
            if msg.header.frame_id!='velodyne':return
            self.scan_pose=self.odom[1]
            if self.manual_locked or not self.search_allowed():return
            try:self.evidence.add(stamp,self.scan_pose,scan_points(msg.ranges,msg.angle_min,msg.angle_increment,msg.range_min,msg.range_max))
            except ValueError:pass
        def receive_session(self,msg):
            try:self.session=json.loads(msg.data);self.session_at=time.monotonic()
            except ValueError:pass
        def receive_imu(self,msg):
            self.imu=msg;self.imu_at=time.monotonic()
        def receive_raw_imu(self,msg):
            if not msg.group_fields & 0x0020 or not msg.group_fields & 0x0008:return
            rate=ned_yaw_rate(msg.yawpitchroll.z,msg.yawpitchroll.y,msg.angularrate.x,msg.angularrate.y,msg.angularrate.z)
            if rate is None:return
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            self.fusion.add_gyro(stamp,rate)
        def release_fusion(self):
            if self.fusion.active and self.raw_odom is not None and self.odom is not None:
                pose=self.display.pose(self.odom[1])
                self.odom=self.raw_odom;self.scan_pose=None;self.motion.clear()
                if pose is not None:
                    self.display.adopt(dict(candidates=[pose],display_fallback=True,path_checked=True),self.odom[1],self.odom[1],self.odom[0])
            self.fusion.stop()
        def receive_trial(self,msg):
            try:
                self.trial=json.loads(msg.data);self.trial_at=time.monotonic()
                if self.observe_registered_gps and self.trial.get('position_source')=='gps':
                    self.trial=dict(self.trial,execute=False)
            except ValueError:pass
        def receive_navigation(self,msg):
            try:status=json.loads(msg.data)
            except ValueError:return
            if not isinstance(status,dict):return
            route=status.get('route') or {};goal=status.get('goal')
            self.observe_registered_gps=bool(status.get('execute') and isinstance(goal,int) and gps_docking_version(route,goal))
            gps_docking=status.get('gps_docking')
            self.gps_docking_active=bool(status.get('execute') and isinstance(gps_docking,dict) and
                gps_docking.get('active') and gps_docking.get('map_id')==self.map_id and not self.observe_registered_gps)
            if self.gps_docking_active:
                if self.requested or self.manual_locked or self.navigation_context is not None:self.reset_search()
                self.navigation_active=False;self.docking_hold=False;self.navigation_at=time.monotonic()
                return
            goal=status.get('goal')
            route=status.get('route') or {}
            stage=(status.get('priority_route') or {}).get('state','gps')
            self.docking_hold=bool(status.get('execute') and stage in ('neutralizing','docking','aborting'))
            active=bool(status.get('execute') and (stage=='gps' or self.observe_registered_gps and stage in ('neutralizing','docking')) and
                        status.get('point_follow_position_source','gps')=='gps' and
                        isinstance(goal,int) and waypoint_kind(route,goal)=='dock' and
                        status.get('docking_search_id'))
            context=(status.get('docking_search_id'),goal) if active else None
            started=active and context!=self.navigation_context
            self.navigation_active=active;self.navigation_at=time.monotonic()
            docking=bool(self.trial.get('execute') and self.navigation_at-self.trial_at<3)
            if started and not docking:
                self.reset_search()
                self.navigation_context=context
                try:self.expected_prior=self.prior_store.read(self.map_id)
                except (OSError,ValueError,TypeError):self.expected_prior=None
                self.request(None)
            elif not active and not self.docking_hold and not docking:
                if self.navigation_context is not None:self.reset_search()
            pivot=bool(active and (status.get('waypoint_mission') or {}).get('searching_for_map'))
            if pivot and not self.prior_pivot and self.odom is not None:
                self.prior_anchor=self.odom[1]
            self.prior_pivot=pivot
        def search_allowed(self):
            return self.navigation_active and time.monotonic()-self.navigation_at<3
        def refresh_region(self):
            if time.monotonic()<self.region_check_at:return
            self.region_check_at=time.monotonic()+1.
            try:region=self.region_store.read(self.map_id)
            except (OSError,ValueError,TypeError,KeyError):return
            key=json.dumps(region,sort_keys=True)
            if key==self.region_key:return
            self.region_key=key
            matcher=copy(self.matcher);matcher.region=region;self.matcher=matcher
            # In-flight searches retain their immutable matcher; discard their
            # results if the operator changes the area during acquisition.
            if hasattr(self,'display') and not self.manual_locked and not self.gps_docking_active:
                if self.future is not None:self.cancel.set()
                self.generation+=1;self.pending=None;self.confirmations=0
                self.review_candidates=[];self.review_pose=None
                self.display=DisplayTrack();self.last_display=None;self.next_search=0.
                self.report.update(candidates=[],live_candidates=[],display_pose=None,docking_match_ready=False)
        def reset_search(self):
            self.generation+=1
            if self.future is not None:self.cancel.set()
            self.pending=None;self.confirmations=0;self.requested=False
            self.release_fusion()
            self.manual_locked=False;self.lidar_tracking_ready=False
            self.navigation_context=None;self.match_lost_at=None
            self.expected_prior=None;self.prior_anchor=None;self.prior_pivot=False
            self.review_candidates=[];self.review_pose=None
            self.display=DisplayTrack();self.last_display=None
            self.evidence=ScanEvidence();self.motion=MotionHistory()
            self.report.update(candidates=[],live_candidates=[],display_pose=None,display_trail_m=[],
                               docking_match_ready=False,docking_match_stamp_s=None,matched_fraction=None,
                               map_tracking_ready=False,docking_search_id=None,docking_goal_index=None)
        def receive_actuators(self,msg):
            try:self.actuators=json.loads(msg.data);self.actuators_at=time.monotonic()
            except ValueError:pass
        def request(self,_):
            if not self.search_allowed():
                self.publish('WAITING','도킹 웨이포인트 대기 · 자동 탐색 꺼짐')
                return
            reset=getattr(self,'manual_odom_reset',False)
            self.release_fusion()
            self.manual_locked=False
            self.manual_odom_reset=False
            self.match_lost_at=None
            self.report['docking_match_ready']=False
            self.generation+=1;self.pending=None;self.confirmations=0;self.requested=True
            if self.future is not None:self.cancel.set()
            if self.odom is not None:
                current=self.odom[1];incumbent=getattr(self,'last_display',None) if reset else self.display.pose(current)
                if reset:
                    self.review_candidates=[];self.review_pose=None
                    self.display=DisplayTrack()
                if incumbent is not None:
                    candidates=[dict(c,**DisplayTrack.propagate(c,self.review_pose,current)) for c in self.review_candidates] if self.review_pose is not None else []
                    self.review_candidates=identify_candidates([incumbent]+[c for c in candidates if math.hypot(c['x']-incumbent['x'],c['y']-incumbent['y'])>.3 or abs(wrap(math.radians(c['yaw_deg']-incumbent['yaw_deg'])))>.1],candidates)
                    self.review_pose=current
            self.next_review=0
            self.next_search=time.monotonic()+self.background_period if self.review_candidates and not self.partial_match else 0
            self.publish('REVIEWING','현재 위치를 유지하며 주변 정합 보정 중')
        def initial_seen(self,msg):
            stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            if stamp==self.sent_stamp:return
            if msg.header.frame_id!='tank_map' or self.odom is None:return
            p=msg.pose.pose.position;q=msg.pose.pose.orientation
            candidate=dict(x=p.x,y=p.y,yaw_deg=math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))),score=0.,inliers=0.,known=0.,free=0.)
            if not region_allows(self.matcher.region,candidate):return
            if self.raw_odom is not None:
                self.fusion.start(self.raw_odom[0],self.raw_odom[1],self.odom[1])
            self.manual_locked=True;self.requested=False
            self.lidar_tracking_ready=False
            self.local_good_at=0.;self.local_confirmations=0;self.next_tracking=0.;self.tracking_stamp=0.
            self.manual_odom_reset=False
            if self.future is not None:self.cancel.set()
            self.generation+=1;self.pending=None;self.next_search=time.monotonic()+self.background_period
            self.review_candidates=identify_candidates([candidate],[]);self.review_pose=self.odom[1]
            self.display.adopt(dict(candidates=self.review_candidates,display_fallback=True,path_checked=True),self.odom[1],self.odom[1],self.odom[0])
            self.review_candidates=[];self.review_pose=None
            self.publish('MANUAL_TRACKING','선택한 위치·방향 기준으로 추적 중',candidates=[],live_candidates=[])
        def broadcast(self):
            now=self.get_clock().now().nanoseconds/1e9
            valid=self.odom is not None and -.1<=now-self.odom[0]<.5 and self.session.get('session_id')==self.map_id and time.monotonic()-self.session_at<3 and not getattr(self,'manual_odom_reset',False)
            display=self.display.pose(self.odom[1]) if valid else None
            if display is not None:
                display.update(stamp_s=self.odom[0],source_age_s=now-display['matched_stamp_s'],manual_locked=self.manual_locked,held=False)
                self.last_display=dict(display)
            elif getattr(self,'last_display',None) is not None:
                display=dict(self.last_display,held=True)
            live=[dict(c,**DisplayTrack.propagate(c,self.review_pose,self.odom[1]),stamp_s=self.odom[0]) for c in self.review_candidates] if valid and self.review_pose is not None else []
            if not self.manual_locked:
                live=[c for c in live if region_allows(self.matcher.region,c)]
                if display is not None and not region_allows(self.matcher.region,display):display=None
            key=(self.motion.revision,self.display.revision)
            if display is None:
                self.report.update(display_trail_m=[],trail_revision=None);self.trail_key=None
            elif not display.get('held') and key!=self.trail_key and (time.monotonic()-self.trail_at>=1 or self.report.get('trail_revision')!=self.display.revision):
                self.report.update(display_trail_m=self.display.trail(self.motion.snapshot(self.odom[1])),trail_revision=self.display.revision)
                self.trail_key=key;self.trail_at=time.monotonic()
            self.report.update(heading_source=self.fusion.source,imu_lidar_fusion=self.fusion.active,
                               imu_yaw_correction_deg=math.degrees(self.fusion.correction),localization_region=self.matcher.region,
                               registered_gps_observation=self.observe_registered_gps)
            self.report.update(display_pose=display,live_candidates=live,manual_locked=self.manual_locked,tracking_valid=valid and display is not None and not display.get('held'),background_period_s=self.background_period,review_period_s=self.review_period)
            if self.manual_locked:
                if getattr(self,'map_correction',True):
                    self.report['map_tracking_ready']=bool(valid and time.monotonic()-getattr(self,'local_good_at',0)<2.)
                else:
                    scan=getattr(self,'scan',None)
                    scan_fresh=scan is not None and -.1<=now-(scan.header.stamp.sec+scan.header.stamp.nanosec/1e9)<.5
                    self.report['map_tracking_ready']=bool(valid and scan_fresh and getattr(self,'lidar_tracking_ready',False))
            context=self.navigation_context
            self.report.update(docking_search_id=context[0] if context else None,
                               docking_goal_index=context[1] if context else None,
                               docking_search_enabled=self.search_allowed())
            if not self.search_allowed():self.report['docking_match_ready']=False
            self.status.publish(String(data=json.dumps(self.report,allow_nan=False)))
        def publish(self,state,reason,**extra):
            self.report.update(state=state,reason=reason,map_id=self.map_id,accept_score=self.min_score,accept_margin=self.min_margin,acceptance_scale=.75,**extra)
            self.broadcast()
        def points(self):
            m=self.scan
            points=scan_points(m.ranges,m.angle_min,m.angle_increment,m.range_min,m.range_max,partial=self.partial_match)
            self.evidence_info=dict(observation_count=1,observation_span_s=0.,observation_baseline_m=0.)
            if self.scan_pose is not None:
                points,self.evidence_info=self.evidence.collect(m.header.stamp.sec+m.header.stamp.nanosec/1e9,self.scan_pose,points)
            return points

        def expected_pose(self,reference):
            return project_prior(self.expected_prior,self.prior_anchor,reference)

        def match(self,kind,candidates,points,history,reference,incumbent):
            matcher=self.matcher
            prior=self.expected_pose(reference)
            result=(matcher.search(points,45.,self.cancel,prior=prior) if kind=='global'
                    else matcher.refine(candidates,points,prior=prior))
            result['expected_pose']=prior
            if self.partial_match:return dict(result,point_count=len(points))
            seeds=candidates or ([incumbent] if incumbent is not None else [])
            if not result.get('candidates') and seeds:
                result=matcher.validate(seeds,points)
                result['display_fallback']=True
            return matcher.constrain_path(result,history,reference,points,incumbent)

        def acquire_partial(self,result):
            if not self.search_allowed():return False
            if not partial_handoff_ready(result) or self.scan_pose is None or self.future_pose is None or self.odom is None:return False
            scan=self.scan;ros_now=self.get_clock().now().nanoseconds/1e9
            stamp=scan.header.stamp.sec+scan.header.stamp.nanosec/1e9
            if not -.1<=ros_now-stamp<.5 or abs(stamp-self.odom[0])>.15:return False
            candidate=DisplayTrack.propagate(result['candidates'][0],self.future_pose,self.scan_pose)
            try:latest=scan_points(scan.ranges,scan.angle_min,scan.angle_increment,scan.range_min,scan.range_max,partial=True)
            except ValueError:return False
            prior=self.expected_pose(self.scan_pose)
            verified=self.matcher.validate([candidate],latest,prior=prior);verified['point_count']=len(latest)
            if not partial_handoff_ready(verified):return False
            candidate=DisplayTrack.propagate(verified['candidates'][0],self.scan_pose,self.odom[1])
            if not region_allows(self.matcher.region,candidate):return False
            msg=PoseWithCovarianceStamped();msg.header.frame_id='tank_map';msg.header.stamp=self.get_clock().now().to_msg()
            msg.pose.pose.position.x=candidate['x'];msg.pose.pose.position.y=candidate['y']
            a=math.radians(candidate['yaw_deg']);msg.pose.pose.orientation.z=math.sin(a/2);msg.pose.pose.orientation.w=math.cos(a/2)
            self.map_correction=False
            self.initial_seen(msg);self.sent_stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            self.report.update(partial_match=True,matched_fraction=verified['candidates'][0]['inliers'],
                               docking_match_ready=True,docking_match_stamp_s=stamp)
            self.initial_pub.publish(msg)
            return True

        def manual_tick(self):
            now=time.monotonic();ros_now=self.get_clock().now().nanoseconds/1e9
            # Acquisition area is not a fence for an already confirmed boat.
            matcher=copy(self.matcher);matcher.region=None
            valid=self.odom is not None and -.1<=ros_now-self.odom[0]<.5 and not getattr(self,'manual_odom_reset',False)
            valid=valid and self.session.get('session_id')==self.map_id and now-self.session_at<3
            if not getattr(self,'map_correction',True):
                # The acquisition anchor stays fixed; gyro/LiDAR fused odometry
                # supplies coherent rotation and translation from that anchor.
                m=self.scan
                scan_valid=m is not None and m.header.frame_id=='velodyne' and -.1<=ros_now-(m.header.stamp.sec+m.header.stamp.nanosec/1e9)<.5
                if scan_valid:
                    scan_valid=sum(math.isfinite(v) and max(.8,m.range_min)<=v<min(25.,m.range_max) for v in m.ranges)>=(9 if self.partial_match else 36)
                self.lidar_tracking_ready=bool(valid and scan_valid and self.display.anchor is not None)
                self.report['docking_match_ready']=False
                if self.partial_match and self.lidar_tracking_ready and self.scan_pose is not None:
                    try:
                        points=scan_points(m.ranges,m.angle_min,m.angle_increment,m.range_min,m.range_max,partial=True)
                        current=self.display.pose(self.scan_pose)
                        result=matcher.validate([current],points,prior=self.expected_pose(self.scan_pose));result['point_count']=len(points)
                        self.report.update(docking_match_ready=partial_handoff_ready(result),
                            docking_match_stamp_s=m.header.stamp.sec+m.header.stamp.nanosec/1e9,
                            matched_fraction=result['candidates'][0]['inliers'])
                    except (ValueError,IndexError,TypeError):pass
                if self.tracking_future is not None:
                    self.tracking_future.cancel();self.tracking_future=None
                self.report.update(local_correction=None,tracking_source=self.fusion.source,map_correction_enabled=False)
                docking=bool(self.trial.get('execute') and now-self.trial_at<3)
                searching=bool(self.partial_match and self.navigation_active and now-self.navigation_at<3 and
                               not docking and self.lidar_tracking_ready)
                if searching and not self.report['docking_match_ready']:
                    if self.match_lost_at is None:self.match_lost_at=now
                    elif now-self.match_lost_at>=2.:
                        self.request(None)
                        return
                else:self.match_lost_at=None
                reason=('IMU 회전·라이다 이동 융합 추적 중' if self.fusion.source=='imu_lidar' else '라이다 추적 중 · IMU 회전 수신 대기') if self.lidar_tracking_ready else '라이다 수신·이동 추정 확인 중'
                self.publish('MANUAL_TRACKING',reason,candidates=[],live_candidates=[])
                return
            imu=self.imu
            if imu is None or now-self.imu_at>.5:
                valid=False;self.local_reason='IMU 수신 대기 · 지도 보정 보류'
            elif not -.1<=ros_now-(imu.header.stamp.sec+imu.header.stamp.nanosec/1e9)<.5:
                valid=False;self.local_reason='IMU 시각 확인 · 지도 보정 보류'
            elif not all(math.isfinite(getattr(imu.angular_velocity,k)) for k in ('x','y','z')) or math.sqrt(sum(getattr(imu.angular_velocity,k)**2 for k in ('x','y','z')))>1.2:
                valid=False;self.local_reason='빠른 회전·흔들림 안정화 대기'
            if self.tracking_future is not None and self.tracking_future.done():
                try:result=self.tracking_future.result()
                except Exception as exc:result=dict(accepted=False,reason='주변 정합 오류: '+str(exc))
                self.tracking_future=None
                if valid and self.tracking_generation==self.generation and -.1<=ros_now-self.tracking_stamp<1.:
                    self.local_reason=result['reason']
                    if result.get('accepted'):
                        self.display.adopt(dict(candidates=[result['candidate']],display_fallback=True,path_checked=True),
                                           self.tracking_reference,self.odom[1],self.tracking_stamp)
                        self.display.fallback=not result['supported']
                        self.local_confirmations=self.local_confirmations+1 if result['supported'] else 0
                        if self.local_confirmations>=2:self.local_good_at=now
                        elif not result['supported']:self.local_good_at=0.
                        self.report['local_correction']=dict(score=result['candidate']['score'],translation_m=result['translation_m'],rotation_deg=result['rotation_deg'],stamp_s=self.tracking_stamp)
                    else:
                        # No pose was adopted: retain the last confirmed track
                        # for the existing two-second freshness window only.
                        self.local_confirmations=0
                        if self.local_good_at>0 and now-self.local_good_at<2.:
                            self.local_reason='최근 정합 위치 유지 · 다음 스캔으로 재확인 중'
            if not valid:
                self.local_good_at=0.;self.local_confirmations=0
                if getattr(self,'manual_odom_reset',False):self.local_reason='오도메트리 좌표 변경 · 현재 위치 재지정 필요'
            m=self.scan
            if valid and self.tracking_future is None and now>=self.next_tracking and m is not None and self.scan_pose is not None:
                stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
                if m.header.frame_id=='velodyne' and -.1<=ros_now-stamp<.5 and stamp>self.tracking_stamp:
                    seed=self.display.pose(self.scan_pose)
                    if seed is not None:
                        try:points=scan_points(m.ranges,m.angle_min,m.angle_increment,m.range_min,m.range_max)
                        except ValueError:self.local_reason='평면 스캔 관측 부족';self.local_good_at=0.
                        else:
                            self.tracking_reference=self.scan_pose;self.tracking_stamp=stamp;self.tracking_generation=self.generation
                            self.tracking_future=self.pool.submit(matcher.track,seed,points,self.min_score)
                    self.next_tracking=now+.5
            self.publish('MANUAL_TRACKING',self.local_reason,candidates=[],live_candidates=[])

        def tick(self):
            now=time.monotonic()
            self.refresh_region()
            if self.gps_docking_active and now-self.navigation_at<3:
                self.publish('GPS_TRACKING','정합 완료 · GPS 위치·IMU 방향으로 도킹 중')
                return
            if not self.search_allowed():
                docking=(self.docking_hold and now-self.navigation_at<3) or (self.trial.get('execute') and now-self.trial_at<3)
                if docking and self.manual_locked:
                    self.manual_tick()
                    return
                if self.requested or self.manual_locked or self.navigation_context is not None:
                    self.reset_search()
                self.publish('WAITING','도킹 웨이포인트 대기 · 자동 탐색 꺼짐')
                return
            if self.future is not None and self.future.done():
                try:result=self.future.result()
                except Exception as exc:result={'state':'ERROR','reason':str(exc),'candidates':[]}
                self.future=None
                self.cancel=threading.Event()
                if self.future_generation==self.generation:
                    previous=[]
                    if self.future_pose is not None:
                        if self.review_pose is not None:
                            previous=[dict(c,**DisplayTrack.propagate(c,self.review_pose,self.future_pose)) for c in self.review_candidates]
                        incumbent=self.display.pose(self.future_pose)
                        if incumbent is not None:previous.append(incumbent)
                    result=retain_display_candidate(result,previous,region=self.matcher.region)
                    result['candidates']=identify_candidates(result.get('candidates',[]),previous)
                    result.update(map_id=self.map_id,candidate_stamp_s=time.time(),accept_score=self.min_score,accept_margin=self.min_margin,**self.future_info)
                    if self.partial_match and self.acquire_partial(result):return
                    current=self.odom[1] if self.odom is not None else None
                    adopted=self.display.adopt(result,self.future_pose,current,self.get_clock().now().nanoseconds/1e9,self.min_score)
                    accepted=self.future_kind=='global' and handoff_ready(result,self.min_score,self.min_margin)
                    if accepted:result.update(state='VERIFYING',reason='점수 기준 통과; 새 스캔으로 위치 확인 중')
                    self.report=result;self.pending=result if accepted else None
                    self.review_candidates=result.get('candidates',[]);self.review_pose=self.future_pose;self.next_review=now+self.review_period
                    self.confirmations=0;self.last_confirm=0
                    if self.future_kind=='global':self.next_search=now+(.5 if self.partial_match else 12 if result.get('display_fallback') else self.background_period if self.display.anchor or self.session.get('localization_ready') else 12)
                    if not self.review_candidates:self.next_search=min(self.next_search,now+2)
            if self.manual_locked:
                self.manual_tick();return
            if self.session.get('session_id')!=self.map_id or now-self.session_at>3:
                self.publish('WAITING','고정 지도 세션 수신 대기');return
            rc=self.actuators.get('receiving') and self.actuators.get('mode')==1 and now-self.actuators_at<.5
            if self.trial.get('execute') and now-self.trial_at<3 and not rc:
                self.pending=None;self.generation+=1
                self.publish('PAUSED','RC로 전환하거나 왕복을 종료하면 위치를 다시 찾습니다');return
            if self.session.get('localization_ready'):
                self.pending=None
                if self.future is None and now<min(self.next_search,self.next_review):
                    self.publish('TRACKING','위치 추적 중 · 백그라운드 후보 재검토 대기');return
            m=self.scan
            if m is None:
                self.publish('WAITING','라이다 수신 대기');return
            stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
            age=(self.get_clock().now().nanoseconds-stamp)/1e9
            if m.header.frame_id!='velodyne' or not -.1<=age<.5:
                self.pending=None;self.publish('WAITING','최신 velodyne 스캔 대기');return
            if self.pending is not None:
                if self.reference is None or not scan_stable(self.reference,m.ranges):
                    self.pending=None;self.confirmations=0;self.next_search=now+(self.background_period if self.display.anchor else 2)
                    self.publish('WAITING','관측이 바뀌었습니다. 배가 잠시 정지하면 다시 탐색합니다');return
                if stamp<=self.last_confirm:return
                try:verified=self.matcher.validate(self.pending['candidates'],self.points())
                except ValueError as exc:self.pending=None;self.publish('WAITING',str(exc));return
                if not verified['candidates']:
                    self.pending=None;self.next_search=now;return
                previous=self.pending['candidates'][0];current=verified['candidates'][0]
                # Old keyframes alone must never authorize a pose unsupported now.
                latest=scan_points(m.ranges,m.angle_min,m.angle_increment,m.range_min,m.range_max)
                support=self.matcher.validate([current],latest)['candidates'][0]
                supported=support['inliers']>=.4875 and support['known']>=.6 and support['free']>=.6375 and support['score']>=self.min_score
                same=math.hypot(previous['x']-current['x'],previous['y']-current['y'])<.3 and abs(wrap(math.radians(previous['yaw_deg']-current['yaw_deg'])))<.1
                if not handoff_ready(verified,self.min_score,self.min_margin) or not same or not supported:
                    self.pending=None;self.publish('AMBIGUOUS','새 스캔에서 후보 구분이 유지되지 않았습니다',candidates=verified['candidates']);return
                if not self.matcher.path_metrics([current],self.motion.snapshot(self.scan_pose),self.scan_pose)[0]['valid']:
                    self.pending=None;self.next_search=now;self.publish('WAITING','이동 경로와 맞는 후보 재탐색');return
                self.confirmations+=1;self.last_confirm=stamp
                if self.confirmations<3:
                    self.publish('VERIFYING',f'새 스캔으로 후보 확인 {self.confirmations}/3');return
                msg=PoseWithCovarianceStamped();msg.header.frame_id='tank_map';msg.header.stamp=self.get_clock().now().to_msg()
                msg.pose.pose.position.x=current['x'];msg.pose.pose.position.y=current['y']
                a=math.radians(current['yaw_deg']);msg.pose.pose.orientation.z=math.sin(a/2);msg.pose.pose.orientation.w=math.cos(a/2)
                msg.pose.covariance[0]=msg.pose.covariance[7]=.25;msg.pose.covariance[35]=math.radians(15)**2
                if not self.map_correction:
                    # Automatic acquisition also becomes a fixed odometry anchor
                    # once confirmed; background matching resumes only on request.
                    self.initial_seen(msg)
                    self.sent_stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
                    self.initial_pub.publish(msg)
                    return
                self.sent_stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
                self.initial_pub.publish(msg);self.pending=None;self.next_search=now+self.background_period
                self.publish('ALIGNING','위치·방향 후보 전달 완료; SLAM 정합 확인 중',confirmations=3);return
            if self.future is not None:
                self.publish('SEARCHING' if self.future_kind=='global' else 'REVIEWING','지도 전체에서 위치·방향 탐색 중' if self.future_kind=='global' else '이동량을 반영해 기존 후보 빠르게 비교 중');return
            if now<self.next_search or not self.requested:
                if self.requested and now>=self.next_review and self.review_candidates and self.review_pose is not None and self.scan_pose is not None:
                    try:points=self.points()
                    except ValueError as exc:self.next_review=now+2;self.publish('WAITING',str(exc));return
                    candidates=[dict(c,**DisplayTrack.propagate(c,self.review_pose,self.scan_pose)) for c in self.review_candidates]
                    self.future_generation=self.generation;self.future_pose=self.scan_pose;self.future_info=dict(self.evidence_info);self.future_kind='local'
                    incumbent=self.display.pose(self.scan_pose)
                    self.future=self.pool.submit(self.match,'local',candidates,points,self.motion.snapshot(self.scan_pose),self.scan_pose,incumbent)
                    self.publish('REVIEWING','이동량을 반영해 기존 후보 빠르게 비교 중');return
                self.broadcast();return
            try:points=self.points()
            except ValueError as exc:self.next_search=now+3;self.publish('WAITING',str(exc));return
            self.reference=list(m.ranges);self.future_generation=self.generation
            self.future_pose=self.scan_pose
            self.future_info=dict(self.evidence_info)
            self.future_kind='global'
            incumbent=self.display.pose(self.scan_pose) if self.scan_pose is not None else None
            self.future=self.pool.submit(self.match,'global',[],points,self.motion.snapshot(self.scan_pose),self.scan_pose,incumbent)
            self.publish('SEARCHING','지도 전체에서 위치·방향 탐색 중')

    node=Localizer()
    try:
        while rclpy.ok() and not stopped.is_set():rclpy.spin_once(node,timeout_sec=.2)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        node.cancel.set();node.pool.shutdown(wait=True,cancel_futures=True)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
