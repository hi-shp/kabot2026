"""Passive global localization; no motor commands, fixed map remains untouched."""
from concurrent.futures import ThreadPoolExecutor
import json
import math
import os
import signal
import threading
import time
from .tank_global_match import GlobalMatcher,ScanEvidence,DisplayTrack,MotionHistory,handoff_ready,scan_points,scan_stable,wrap
from .tank_map_file import read_bundle


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.executors import ExternalShutdownException
    from sensor_msgs.msg import LaserScan
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
            self.min_score=float(self.declare_parameter('accept_score',.80).value)
            self.min_margin=float(self.declare_parameter('accept_margin',.05).value)
            if not (.5<=self.min_score<=1 and .01<=self.min_margin<=1):
                raise ValueError('accept_score: 0.5..1; accept_margin: 0.01..1')
            self.map_id,_,files=read_bundle(self.get_parameter('file').value)
            self.matcher=GlobalMatcher(json.loads(files['grid.json']),files['grid.bin'])
            self.pool=ThreadPoolExecutor(max_workers=1);self.cancel=threading.Event();self.future=None
            self.scan=None;self.session={};self.session_at=0.;self.trial={};self.trial_at=0.
            self.generation=0;self.future_generation=0;self.next_search=time.monotonic()+2
            self.requested=bool(self.get_parameter('automatic').value)
            self.report={'state':'WAITING','reason':'자동 위치 탐색 준비 중','map_id':self.map_id,'candidates':[]}
            self.confirmations=0;self.last_confirm=0;self.pending=None
            self.sent_stamp=None;self.reference=None
            self.evidence=ScanEvidence();self.odom=None;self.scan_pose=None;self.evidence_info={};self.future_info={}
            self.display=DisplayTrack();self.future_pose=None;self.background_period=60.
            self.motion=MotionHistory();self.trail_at=0.;self.trail_key=None
            self.review_period=2.;self.next_review=0.;self.review_candidates=[];self.review_pose=None;self.future_kind='global'
            self.status=self.create_publisher(String,'/tank/localization_candidates',10)
            self.initial_pub=self.create_publisher(PoseWithCovarianceStamped,'/initialpose',1)
            self.create_subscription(LaserScan,'/scan',self.receive_scan,qos_profile_sensor_data)
            self.create_subscription(Odometry,'/tank/odom',self.receive_odom,qos_profile_sensor_data)
            self.create_subscription(String,'/tank/session',self.receive_session,10)
            self.create_subscription(String,'/tank/status',self.receive_trial,qos_profile_sensor_data)
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
            if self.odom is not None:
                dt=stamp-self.odom[0];old=self.odom[1]
                if dt<=0:return
                if dt>1 or math.hypot(current[0]-old[0],current[1]-old[1])>max(.2,dt*1.5) or abs(wrap(current[2]-old[2]))>max(.2,dt*2):
                    self.display=DisplayTrack();self.generation+=1;self.pending=None;self.next_search=0;self.evidence.clear()
                    self.review_candidates=[];self.review_pose=None
                    self.motion.clear()
            self.odom=(stamp,current)
            self.motion.add(current)
        def receive_scan(self,msg):
            self.scan=msg;self.scan_pose=None
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            if self.odom is None or abs(stamp-self.odom[0])>.15:return
            if msg.header.frame_id!='velodyne':return
            self.scan_pose=self.odom[1]
            try:self.evidence.add(stamp,self.scan_pose,scan_points(msg.ranges,msg.angle_min,msg.angle_increment,msg.range_min,msg.range_max))
            except ValueError:pass
        def receive_session(self,msg):
            try:self.session=json.loads(msg.data);self.session_at=time.monotonic()
            except ValueError:pass
        def receive_trial(self,msg):
            try:self.trial=json.loads(msg.data);self.trial_at=time.monotonic()
            except ValueError:pass
        def request(self,_):
            self.generation+=1;self.pending=None;self.confirmations=0;self.requested=True;self.next_search=0
            self.evidence.clear()
            self.display=DisplayTrack()
            self.review_candidates=[];self.review_pose=None
        def initial_seen(self,msg):
            stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            if stamp==self.sent_stamp:return
            self.generation+=1;self.pending=None;self.next_search=time.monotonic()+30
            self.evidence.clear()
            self.display=DisplayTrack()
            self.review_candidates=[];self.review_pose=None
        def broadcast(self):
            now=self.get_clock().now().nanoseconds/1e9
            valid=self.odom is not None and -.1<=now-self.odom[0]<.5 and self.session.get('session_id')==self.map_id and time.monotonic()-self.session_at<3
            display=self.display.pose(self.odom[1]) if valid else None
            if display is not None:display.update(stamp_s=self.odom[0],source_age_s=now-display['matched_stamp_s'])
            key=(self.motion.revision,self.display.revision)
            if display is None:
                self.report.update(display_trail_m=[],trail_revision=None);self.trail_key=None
            elif key!=self.trail_key and (time.monotonic()-self.trail_at>=1 or self.report.get('trail_revision')!=self.display.revision):
                self.report.update(display_trail_m=self.display.trail(self.motion.snapshot(self.odom[1])),trail_revision=self.display.revision)
                self.trail_key=key;self.trail_at=time.monotonic()
            self.report.update(display_pose=display,background_period_s=self.background_period,review_period_s=self.review_period)
            self.status.publish(String(data=json.dumps(self.report,allow_nan=False)))
        def publish(self,state,reason,**extra):
            self.report.update(state=state,reason=reason,map_id=self.map_id,accept_score=self.min_score,accept_margin=self.min_margin,**extra)
            self.broadcast()
        def points(self):
            m=self.scan
            points=scan_points(m.ranges,m.angle_min,m.angle_increment,m.range_min,m.range_max)
            self.evidence_info=dict(observation_count=1,observation_span_s=0.,observation_baseline_m=0.)
            if self.scan_pose is not None:
                points,self.evidence_info=self.evidence.collect(m.header.stamp.sec+m.header.stamp.nanosec/1e9,self.scan_pose,points)
            return points

        def match(self,kind,candidates,points,history,reference,incumbent):
            result=self.matcher.search(points,45.,self.cancel) if kind=='global' else self.matcher.refine(candidates,points)
            return self.matcher.constrain_path(result,history,reference,points,incumbent)

        def tick(self):
            now=time.monotonic()
            if self.future is not None and self.future.done():
                try:result=self.future.result()
                except Exception as exc:result={'state':'ERROR','reason':str(exc),'candidates':[]}
                self.future=None
                if self.future_generation==self.generation:
                    result.update(map_id=self.map_id,candidate_stamp_s=time.time(),accept_score=self.min_score,accept_margin=self.min_margin,**self.future_info)
                    current=self.odom[1] if self.odom is not None else None
                    adopted=self.display.adopt(result,self.future_pose,current,self.get_clock().now().nanoseconds/1e9,self.min_score)
                    if not adopted and result.get('incumbent_path_valid') is False:self.display=DisplayTrack()
                    accepted=self.future_kind=='global' and handoff_ready(result,self.min_score,self.min_margin)
                    if accepted:result.update(state='VERIFYING',reason='점수 기준 통과; 새 스캔으로 위치 확인 중')
                    self.report=result;self.pending=result if accepted else None
                    self.review_candidates=result.get('candidates',[]);self.review_pose=self.future_pose;self.next_review=now+self.review_period
                    self.confirmations=0;self.last_confirm=0
                    if self.future_kind=='global':self.next_search=now+(self.background_period if self.display.anchor or self.session.get('localization_ready') else 12)
                    if not self.review_candidates:self.next_search=min(self.next_search,now+2)
            if self.session.get('session_id')!=self.map_id or now-self.session_at>3:
                self.publish('WAITING','고정 지도 세션 수신 대기');return
            if self.trial.get('execute') and now-self.trial_at<3:
                self.pending=None;self.generation+=1
                self.publish('PAUSED','왕복을 종료하면 자동 위치 탐색을 계속합니다');return
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
                previous=self.pending['candidates'][0];current=verified['candidates'][0]
                # Old keyframes alone must never authorize a pose unsupported now.
                latest=scan_points(m.ranges,m.angle_min,m.angle_increment,m.range_min,m.range_max)
                support=self.matcher.validate([current],latest)['candidates'][0]
                supported=support['inliers']>=.65 and support['known']>=.8 and support['free']>=.85 and support['score']>=self.min_score
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
