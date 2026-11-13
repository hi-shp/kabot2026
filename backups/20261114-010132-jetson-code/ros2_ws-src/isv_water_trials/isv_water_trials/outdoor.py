"""Outdoor RC recorder/learner and explicitly started GPS A/B verification."""
import argparse
import uuid
from copy import deepcopy
from collections import deque
from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor
import multiprocessing
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import shutil
import signal
import sqlite3
import subprocess
import sys
import time
import uuid
import numpy as np
from .outdoor_model import ResponseModel, LiveComparison, applied_command, fit_model, fingerprint, latlon, local_xy, logical_pwm
from .outdoor_state import OutdoorState
from .outdoor_store import OutdoorStore
from .outdoor_planner import OutdoorPlanner
from .outdoor_config import load_config, read_arrival_radius, read_drive_limits, validate_drive_limits
from .outdoor_layers import LidarLayers
from .outdoor_test_course import latest_path_blocked, PlanComparison
from .outdoor_regions import gps_regions
from .outdoor_follow import point_follow_command, path_follow_target, forward_arrival, maneuver_space, LearnedFollowPlanner, limit_output, buoy_goal_command, WaypointLineGuide
from .outdoor_overlay import FeasibleOverlay, measured_rollout
from .outdoor_recovery import recovery_command
from .codex_nav.gpu_clearance import gpu_status
from .outdoor_anticipation import EarlyAvoidance
from .tank_storage import atomic_json
from .tank_mission import DockingMission, docking_points
from .gps_docking import GpsDocking
from .outdoor_route import normalize_route, waypoint_kind, waypoint_output, gps_docking_version, KINDS, WaypointProgress, docking_search_command, ReverseStop, DOCK_SEARCH_SECONDS
from .docking_observation import DockingAudit,read_profile,compare_targets
from .buoy_vision import VisionRuntime
from .gps_recording import GpsRecordingGate
from .marine_lidar_runtime import MarineLidarRuntime
from .path_avoidance import PathAvoidanceController
from .buoy_orbit import BuoyOrbit,orbit_choice
from .live_route import live_goal, leg_signature
from .waypoint_timeouts import WaypointTimeouts, read_limits as read_waypoint_timeouts


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', default='~/water_trials/outdoor')
    p.add_argument('--config', default=None)
    p.add_argument('--no-bag', action='store_true', help='Only for local integration fixtures')
    return p


def main(argv=None):
    # launch_ros appends --ros-args/-r; keep them out of our application parser.
    from rclpy.utilities import remove_ros_args
    args = parser().parse_args(remove_ros_args()[1:] if argv is None else argv)
    from ament_index_python.packages import get_package_share_directory
    config_path = Path(args.config) if args.config else Path(get_package_share_directory('isv_water_trials'))/'config/outdoor.json'
    config = load_config(config_path)
    import fcntl
    root = Path(args.root).expanduser(); root.mkdir(parents=True, exist_ok=True)
    try:config['arrival_radius_m']=read_arrival_radius(root/'gps_arrival.json',config['arrival_radius_m'])
    except (OSError,ValueError,KeyError,TypeError):pass
    lock = (root/'owner.lock').open('w')
    try: fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError: print('outdoor already running'); return
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import NavSatFix, LaserScan
    from geometry_msgs.msg import TwistWithCovarianceStamped
    from vectornav_msgs.msg import CommonGroup
    from std_msgs.msg import String, Empty
    from std_srvs.srv import SetBool
    from mechaship_interfaces.srv import ThrottlePulseWidth
    from .shutdown import ExitSignals, neutralize
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO); exits = ExitSignals()

    class Outdoor(Node):
        def __init__(self):
            super().__init__('outdoor_water_trial')
            self.store = OutdoorStore(root, config); self.state = OutdoorState(config)
            self.map_layers=LidarLayers()
            self.display_pool=ThreadPoolExecutor(max_workers=1)
            self.feasible_overlay=FeasibleOverlay()
            self.display_job=None;self.display_result=None;self.display_at=0.
            self.display_key=None;self.display_job_key=None;self.display_frame=None;self.display_position=None;self.display_revision=0
            self.rows = deque(self.store.rows(), maxlen=30000); self.commands = deque(maxlen=600)
            self.chart = deque(maxlen=300); self.trail = deque(maxlen=20000)
            self.chart.extend({'t':row['t'],'command':applied_command(row.get('actuator',{}),config),
                'measured':row['velocity'],'predicted':None,'horizon_s':None} for row in list(self.rows)[-300:])
            try: trail_after = json.loads((root/'display.json').read_text())['trail_after_t']
            except (FileNotFoundError, ValueError, KeyError): trail_after = 0.
            self.gps_recording=GpsRecordingGate()
            recording=self.gps_recording.session()
            self.gps_recording_id=recording['id'] if recording else None
            if recording:self.trail.extend(self.store.trail(max(trail_after,recording['started_stamp'])))
            self.last_fix_t = None
            self.comparison = LiveComparison(); self.model = None; self.validation_model = None; self.frozen = False; self.learning = False
            self.learning_started_t = float('inf')
            self.lineage=None; self.baseline_model=None
            self.test_course=None;self.test_comparison=None;self.pre_test_route=None
            self.fit_job = None; self.fit_generation = 0; self.fit_at = time.monotonic()
            self.fit_pool = ProcessPoolExecutor(max_workers=1, mp_context=multiprocessing.get_context('spawn'))
            self.plan_pool = ThreadPoolExecutor(max_workers=1)
            self.plan_job = None; self.plan_generation = 0; self.plan_started = 0.; self.plan_at = 0.
            self.execute = False; self.planner = None; self.plan_result = None; self.goal = 0; self.direction = 1; self.arrivals = 0
            self.direct_follow = False; self.point_follow_status = None;self.recovery_requested=False
            self.restart_request=None;self.run_start_index=0
            self.orbit_abort_token=None;self.orbit_abort_status=None
            self.buoy_orbit=None;self.orbit_status=None;self.orbit_input_scan=None;self.leg_from_current_index=None
            self.follow_path=False;self.follow_track=[];self.buoy_course=False
            self.waypoint_line=WaypointLineGuide()
            self.path_controller=PathAvoidanceController()
            self.marine_diagnostic_at=0.
            self.waypoint_progress=WaypointProgress()
            self.stop_brake=ReverseStop()
            self.arrival_settings_at=-float('inf');self.arrival_settings_stamp=None
            self.waypoint_timeouts=WaypointTimeouts();self.waypoint_timeouts_at=-float('inf')
            self.waypoint_timeout_last=None
            self.drive_limits=read_drive_limits(root/'drive_limits.json',config['percent'])
            self.drive_limits_at=-float('inf');self.drive_limits_stamp=None
            self.follow_reverse=False;self.follow_mpc=LearnedFollowPlanner()
            self.early_avoidance=EarlyAvoidance();self.avoidance_info=None
            self.priority_stage='gps';self.priority_used_maps=set();self.priority_mission=None
            self.docking_complete=False
            self.docking_aborted=False;self.abort_child_done_at=None
            self.docking_slam_job=None;self.docking_slam_target=None;self.docking_slam_auto_off=False
            self.docking_slam_attempt=-float('inf');self.docking_slam_error=''
            self.docking_search_id=uuid.uuid4().hex
            self.docking_run_route=None
            self.priority_process=None;self.priority_started=0.;self.priority_completion=None
            self.gps_docking=None;self.gps_docking_lock=None;self.docking_line=WaypointLineGuide()
            self.tank_session_data={};self.tank_session_at=0.;self.tank_status_data={};self.tank_status_at=0.
            self.tank_match_data={};self.tank_match_at=0.
            self.follow_job=None;self.follow_job_key=None;self.follow_result=None;self.follow_result_key=None;self.follow_plan_at=0.
            self.virtual_fix = None; self.follow_position_source = 'gps'; self.follow_origin = None
            self.actuator = {}; self.actuator_at = 0.; self.scan = None; self.scan_at = 0.
            self.command = [0., 0.]; self.command_at = 0.; self.pending = []; self.pending_at = 0.
            self.error = ''; self.fit_status = '학습 시작 대기'; self.ack = None; self.ready_since = None
            self.last_frame_t = None; self.last_valid = False; self.route = None; self.bag = None; self.bag_at = 0.
            self.bag_log = None; self.bag_status = '학습 중지'; self.bag_stopping = False; self.last_disk = 0.; self.disk_ok = True
            try:
                current,validation,self.learning=self.store.live_selection()
                self.model=ResponseModel(current) if current else None
                self.validation_model=ResponseModel(validation) if validation else None
            except (ValueError, KeyError, OSError) as exc:self.error='저장 설정 확인: '+str(exc)
            try: self.route = self.validate_route(json.loads((root/'route.json').read_text(encoding='utf-8')))
            except (FileNotFoundError, ValueError, KeyError): pass
            self.regions=[]
            try:
                saved=json.loads((root/'regions.json').read_text(encoding='utf-8'))
                gps_regions(saved);self.regions=saved
            except FileNotFoundError: pass
            except (ValueError, TypeError, KeyError, ImportError) as exc:self.error='저장 구역 확인: '+str(exc)
            self.status_pub = self.create_publisher(String, '/outdoor/status', 5)
            self.navigation_scan_pub=self.create_publisher(LaserScan,'/outdoor/navigation_scan',qos_profile_sensor_data)
            self.tank_status_pub=self.create_publisher(String,'/tank/status',10)
            self.docking_audit=None;self.docking_observation={'active':False};self.observation_scan_stamp=None
            self.registered_profiles={};self.observation_run=None
            self.observation_pub=self.create_publisher(String,'/outdoor/docking_observation',10)
            self.heading_pub = self.create_publisher(String, '/navigation/heading', 5)
            self.command_pub = self.create_publisher(String, '/outdoor/command', 10)
            self.marine_lidar = MarineLidarRuntime(self, config, root)
            self.arm = self.create_client(SetBool, '/actuators/arm')
            self.throttles = [self.create_client(ThrottlePulseWidth, '/actuators/throttle_'+side+'/set_pulse_width') for side in ('left', 'right')]
            for topic, kind, handler in [('/tank/localization_candidates',String,self.on_tank_match),('/tank/session',String,self.on_tank_session),
                    ('/tank/status',String,self.on_tank_status),('/gps/fix', NavSatFix, self.on_fix),
                    ('/gps/virtual/fix', NavSatFix, self.on_virtual_fix),
                    ('/gps/fix_velocity', TwistWithCovarianceStamped, self.on_velocity),
                    ('/vectornav/raw/common', CommonGroup, self.on_imu),
                    ('/scan', LaserScan, self.on_scan), ('/actuators/status', String, self.on_actuator)]:
                self.create_subscription(kind, topic, handler, qos_profile_sensor_data)
            self.create_subscription(String, '/outdoor/request', self.request, 10)
            self.create_subscription(Empty, '/water_trial/stop', lambda _: self.stop_trial(), 10)
            self.buoy_vision=VisionRuntime(self,config)
            self.create_timer(.1, self.tick); self.create_timer(.5, self.report)
            self.create_timer(.2,self.observe_registered_docking)
            self.store.event('session_started', learning=self.learning, model=self.model.data['id'] if self.model else None)

        def registered_profile(self,version):
            if version in self.registered_profiles:return self.registered_profiles[version]
            return read_profile(Path.home(),self.tank_session_data.get('session_id'),version)

        def observe_registered_docking(self):
            now=time.monotonic();version=gps_docking_version(self.route,self.goal) if self.route else None
            active=bool(self.execute and version and self.priority_stage not in ('cancelled','error'))
            if not active:
                if self.observation_run and self.docking_audit is not None:
                    self.docking_audit.append(self.observation_run,dict(kind='end',stamp_s=self.ros_now(),goal_index=self.goal))
                self.observation_run=None;self.docking_observation=dict(active=False,log=self.docking_audit.status() if self.docking_audit else None)
                return
            if self.docking_audit is None:self.docking_audit=DockingAudit(root/'docking_audit')
            self.observation_run=self.docking_search_id+'_'+str(self.goal)
            measured=self.state.snapshot(now) or self.state.display_pose(now)
            selected=None;failure='';comparison=None;aligned=None
            try:selected=self.registered_profile(version)
            except (OSError,ValueError,KeyError,TypeError) as exc:failure=str(exc)
            match=deepcopy(self.tank_match_data);map_id=self.tank_session_data.get('session_id')
            display=match.get('display_pose') if match.get('map_id')==map_id else None
            target=local_xy(*self.route['points'][self.goal],self.state.origin) if self.state.origin else None
            if selected and selected.get('session_id')!=map_id:
                failure='선택 도킹 경로와 현재 SLAM 지도 불일치';selected=None
            if selected and display and target is not None and now-self.tank_match_at<2 and not display.get('held'):
                try:
                    aligned=self.state.pose_at(float(display['stamp_s']))
                    if aligned is not None:
                        comparison=compare_targets(selected,display,aligned,self.state.origin,target,
                            config.get('lidar_lever_arm_m',[0.,0.]),config.get('lidar_yaw_deg',0.))
                except (ValueError,KeyError,TypeError,IndexError) as exc:failure=str(exc)
            summary=dict(active=True,version=version,map_id=map_id,goal_index=self.goal,
                registered_a_gps=list(self.route['points'][self.goal]),phase='B' if self.gps_docking is not None else 'A',
                route=selected,position_source='gps',heading_source='imu',slam_display_only=True,
                comparison_error_m=comparison.get('a_gps_error_m') if comparison else None,
                slam_tracking_valid=bool(match.get('tracking_valid')),error=failure,log=self.docking_audit.status())
            self.docking_observation=summary
            row=dict(kind='sample',stamp_s=self.ros_now(),wall_unix_s=time.time(),run_id=self.docking_search_id,
                goal_index=self.goal,version=version,stage=self.priority_stage,registered_a_gps=summary['registered_a_gps'],
                gps_origin=self.state.origin,gps_imu=measured,aligned_gps_imu=aligned,
                profile=selected,slam=match,slam_report_age_s=now-self.tank_match_at,
                session=deepcopy(self.tank_session_data),comparison=comparison,
                control_target_source='registered_gps_a' if self.gps_docking is None else 'frozen_slam_b',
                control_status=deepcopy(self.point_follow_status),sent_command=list(self.last_sent_command),
                actuator=deepcopy(self.actuator),gps_docking=self.gps_docking_report(now),error=failure)
            scan=self.scan
            if scan is not None and scan['stamp']!=self.observation_scan_stamp and now-getattr(self,'observation_scan_at',0.)>=.5:
                self.observation_scan_stamp=scan['stamp'];self.observation_scan_at=now
                count=len(scan['ranges']);step=max(1,math.ceil(count/1440))
                row['scan']=dict(stamp_s=scan['stamp'],source='navigation_scan_with_marine_obstacles',angles_rad=list(scan['angles'][::step]),
                    ranges_m=list(scan['ranges'][::step]),range_min=scan['range_min'],range_max=scan['range_max'],sample_step=step)
                row['marine_lidar']=self.marine_lidar.diagnostic()
            self.docking_audit.append(self.observation_run,row)
            self.observation_pub.publish(String(data=json.dumps(summary,allow_nan=False)))

        def ros_now(self): return self.get_clock().now().nanoseconds/1e9

        def stamp(self, msg, max_age=None):
            value = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            age = self.ros_now()-value
            if not math.isfinite(value) or age < -.15 or age > (config['gps_max_age_s'] if max_age is None else max_age):
                raise ValueError('센서 측정 시각이 오래되었거나 젯슨 시각과 다릅니다')
            return value

        def on_fix(self, msg):
            try:
                stamp = self.stamp(msg)
                self.state.fix(stamp, time.monotonic(), msg.latitude, msg.longitude, msg.status.status, msg.position_covariance, msg.position_covariance_type)
                if self.state.fixes and self.state.fixes[-1][0]==stamp and stamp != self.last_fix_t:
                    if self.last_fix_t is not None and (stamp-self.last_fix_t > 1.6 or stamp < self.last_fix_t): self.store.new_segment('gps_gap')
                    self.last_fix_t = stamp
                    recording=self.gps_recording.session()
                    if recording:
                        if recording['id']!=self.gps_recording_id:
                            self.trail.clear();self.gps_recording_id=recording['id']
                        if stamp>=recording['started_stamp']:
                            self.trail.append([msg.latitude, msg.longitude, self.store.segment, stamp])
                            if self.disk_ok:self.store.position(stamp, msg.latitude, msg.longitude)
            except ValueError as exc: self.error = str(exc)
            except (OSError, sqlite3.Error) as exc: self.disk_ok = False; self.error = '위치 기록 저장 오류: '+str(exc)

        def on_virtual_fix(self, msg):
            # Synthetic positions are used only by explicitly selected point following.
            # They never enter OutdoorState, the real trail, or the learner.
            if msg.status.status<0:
                self.virtual_fix=None;return
            try:
                stamp=self.stamp(msg)
                point=[float(msg.latitude),float(msg.longitude)]
                if not all(math.isfinite(v) for v in point) or not -85<point[0]<85 or not -180<=point[1]<=180:
                    raise ValueError('가상 GPS 좌표 오류')
                self.virtual_fix={'point':point,'received':time.monotonic(),'stamp':stamp}
            except (ValueError,TypeError):self.virtual_fix=None

        def on_imu(self, msg):
            self.marine_lidar.imu(msg)
            if msg.group_fields & 0x0008 and msg.group_fields & 0x0020:
                try:
                    now=time.monotonic()
                    self.state.imu(self.stamp(msg, config['imu_max_age_s']), now, msg.yawpitchroll.x, msg.angularrate.z)
                    heading=self.state.heading(now)
                    if heading is not None:
                        self.heading_pub.publish(String(data=json.dumps(heading,allow_nan=False)))
                except ValueError as exc: self.error = str(exc)

        def on_actuator(self, msg):
            try:
                value = json.loads(msg.data); self.actuator = value; self.actuator_at = time.monotonic()
                command = applied_command(value, config)
                if command is not None:
                    stamp = self.ros_now()
                    if self.commands and stamp <= self.commands[-1][0]:
                        self.commands.clear(); self.store.new_segment('clock_reset'); self.ready_since = None
                    self.commands.append([stamp, *command])
            except (ValueError, TypeError): pass

        def on_scan(self, msg):
            try: stamp = self.stamp(msg, config['scan_max_age_s'])
            except ValueError: return
            # Preserve uniform full-circle angular bins and the nearest return
            # in each bin. Do not throw away narrow obstacles while decimating.
            bins = 720; ranges = np.full(bins, np.nan)
            for i, distance in enumerate(msg.ranges):
                angle = msg.angle_min+i*msg.angle_increment
                slot = int(round((angle+math.pi)/(2*math.pi)*bins)) % bins
                if math.isnan(distance) or distance < msg.range_min: continue
                if math.isnan(ranges[slot]) or distance < ranges[slot]: ranges[slot] = distance
            self.scan = {'angles': np.linspace(-math.pi, math.pi, bins, endpoint=False),
                         'ranges': ranges, 'range_min': msg.range_min,
                         'range_max': msg.range_max, 'stamp': stamp, 'frame_id': msg.header.frame_id}
            self.orbit_input_scan=self.scan
            self.scan = self.marine_lidar.merge(self.scan, self.state.pose_at(stamp))
            self.scan_at = time.monotonic()
            if self.execute and self.plan_result and self.plan_result.get('safety_path'):
                pose=self.state.pose_at(stamp)
                if pose and latest_path_blocked(self.scan,pose,self.plan_result['safety_path'],self.planner.config):
                    self.plan_generation+=1;self.plan_result=None;self.recovery_requested=True
                    self.error='새 장애물 감지 · 경로 재계산'

        def on_tank_session(self,msg):
            try:
                data=json.loads(msg.data)
                if isinstance(data,dict):self.tank_session_data=data;self.tank_session_at=time.monotonic()
            except (TypeError,ValueError):pass

        def on_tank_status(self,msg):
            try:
                data=json.loads(msg.data)
                if isinstance(data,dict):self.tank_status_data=data;self.tank_status_at=time.monotonic()
            except (TypeError,ValueError):pass

        def on_tank_match(self,msg):
            try:
                data=json.loads(msg.data)
                if isinstance(data,dict):self.tank_match_data=data;self.tank_match_at=time.monotonic()
            except (TypeError,ValueError):pass

        def cancel_docking(self):
            process=getattr(self,'priority_process',None)
            if process is not None and process.poll() is None:
                try:os.killpg(process.pid,signal.SIGINT)
                except ProcessLookupError:pass
            if self.gps_docking is not None:
                self.publish_gps_docking('STOPPED','도킹 종료',time.monotonic(),execute=False)
                self.gps_docking=None
            if self.gps_docking_lock is not None:
                self.gps_docking_lock.close();self.gps_docking_lock=None
            if getattr(self,'priority_stage','gps')!='gps':self.priority_stage='cancelled'

        def abort_docking(self,data):
            if self.priority_stage=='aborting':return
            if not self.execute or not self.route:raise ValueError('진행 중인 도킹이 없습니다.')
            if data.get('goal_index')!=self.goal or data.get('docking_search_id')!=self.docking_search_id:
                raise ValueError('목표가 변경됐습니다. 현재 도킹 상태에서 다시 눌러 주세요.')
            if self.priority_stage=='gps':
                if waypoint_kind(self.route,self.goal)!='dock':raise ValueError('도킹 웨이포인트 추종 중에 사용할 수 있습니다.')
                self.priority_mission=DockingMission.begin(self.tank_session_data.get('session_id',''),self.goal,
                    len(self.route['points']),uuid.uuid4().hex,loop=not getattr(self,'direct_follow',False))
                self.priority_process=None
            elif self.priority_stage not in ('neutralizing','docking','completing','error'):
                raise ValueError('진행 중인 도킹이 없습니다.')
            process=self.priority_process
            if process is not None and process.poll() is None:
                try:os.killpg(process.pid,signal.SIGINT)
                except ProcessLookupError:pass
            self.priority_stage='aborting';self.abort_child_done_at=None
            self.plan_generation+=1;self.plan_result=None;self.follow_result=None
            self.command=[0.,0.];self.command_at=time.monotonic()
            self.store.event('docking_abort_requested',goal_index=self.goal)

        def abort_orbit(self,data):
            token=(data.get('docking_search_id'),data.get('goal_index'))
            if token==self.orbit_abort_token:return
            if type(token[1]) is not int or token!=(self.docking_search_id,self.goal):
                raise ValueError('목표가 변경됐습니다. 현재 선회 상태에서 다시 눌러 주세요.')
            if not self.execute or waypoint_kind(self.route,self.goal)!='orbit':
                raise ValueError('선회 웨이포인트 진행 중에 사용할 수 있습니다.')
            skipped=self.goal;now=time.monotonic()
            self.orbit_abort_token=token
            self.orbit_abort_status=dict(skipped_index=skipped,next_index=skipped+1 if skipped+1<len(self.route['points']) else None)
            self.buoy_orbit=None;self.orbit_status=None
            self.plan_generation+=1;self.plan_result=None;self.follow_result=None;self.follow_track=[]
            self.display_revision+=1;self.display_key=None;self.display_result=None
            self.waypoint_progress.reset();self.waypoint_line=WaypointLineGuide();self.path_controller=PathAvoidanceController()
            self.follow_arrived=False;self.follow_reverse=False;self.recovery_requested=False
            self.corridor_scan=None;self.avoidance_info=None;self.command=[0.,0.];self.command_at=now
            self.arrivals+=1
            try:self.store.event('buoy_orbit_aborted',**self.orbit_abort_status)
            except (OSError,sqlite3.Error):pass
            if skipped+1<len(self.route['points']):
                self.goal=skipped+1;self.leg_from_current_index=self.goal
                self.error='';self.point_follow_status=dict(status='선회 중단 · 다음 지점으로 진행',tracking='point')
                self.send(self.command,now)
            else:self.stop_trial()

        def skip_waypoint(self,data):
            token=(data.get('docking_search_id'),data.get('goal_index'),data.get('arrivals'))
            if token==getattr(self,'waypoint_skip_token',None):return
            if (type(token[1]) is not int or type(token[2]) is not int or
                token!=(self.docking_search_id,self.goal,self.arrivals)):
                raise ValueError('목표가 변경됐습니다. 현재 목표를 확인하고 다시 눌러 주세요.')
            if not self.execute or not self.route or self.restart_request is not None:
                raise ValueError('현재 웨이포인트를 주행 중일 때 사용할 수 있습니다.')
            if self.priority_stage in ('neutralizing','docking','completing','aborting','error'):
                # Use the existing neutral/child-exit handshake before GPS resumes.
                self.abort_docking(data)
                self.priority_mission.resume_index=self.goal+1 if self.goal+1<len(self.route['points']) else None
                self.waypoint_skip_token=token
                self.waypoint_skip_status=dict(skipped_index=self.goal,next_index=self.priority_mission.resume_index,
                    message='도달 처리 요청 · 도킹 종료 후 다음 지점으로 진행')
                return
            skipped=self.goal;now=time.monotonic()
            next_index=skipped+1 if skipped+1<len(self.route['points']) else None
            counted=(next_index is None and getattr(self,'follow_arrived',False) and
                     waypoint_kind(self.route,skipped) not in ('hold','dock','orbit'))
            self.buoy_orbit=None;self.orbit_status=None;self.priority_mission=None
            self.plan_generation+=1;self.plan_result=None;self.follow_result=None;self.follow_track=[]
            self.follow_job_key=None;self.follow_result_key=None;self.follow_plan_at=0.
            self.display_revision+=1;self.display_key=None;self.display_result=None
            self.waypoint_progress.reset();self.waypoint_line=WaypointLineGuide();self.path_controller=PathAvoidanceController()
            self.follow_arrived=False;self.follow_reverse=False;self.recovery_requested=False
            self.corridor_scan=None;self.avoidance_info=None;self.stop_brake.reset()
            self.command=[0.,0.];self.command_at=now
            if not counted:self.arrivals+=1
            if next_index is None:
                self.stop_trial()
            else:
                self.goal=next_index;self.leg_from_current_index=next_index
                self.follow_percent=self.follow_output_limit();self.arrival_threshold=config['arrival_radius_m']
                self.docking_search_id=uuid.uuid4().hex;self.docking_start_at=0.
                self.send(self.command,now)
            self.error=''
            message='마지막 지점 도달 처리 · 주행 종료' if next_index is None else f'{skipped+1}번 도달 처리 · {next_index+1}번으로 진행'
            self.point_follow_status=dict(status=message,tracking='gps')
            self.waypoint_skip_token=token
            self.waypoint_skip_status=dict(skipped_index=skipped,next_index=next_index,message=message)

        def resume_after_docking(self,aborted=False):
            mission=self.priority_mission
            if self.gps_docking is not None:
                self.publish_gps_docking('ABORTED' if aborted else 'COMPLETE',
                    '도킹 중단' if aborted else '도킹 A/B 도달 완료',time.monotonic(),execute=False)
            self.gps_docking=None
            if self.gps_docking_lock is not None:
                self.gps_docking_lock.close();self.gps_docking_lock=None
            self.arrivals+=1;self.priority_stage='gps';self.priority_process=None
            self.docking_complete=not aborted;self.docking_aborted=aborted;self.docking_slam_target=False
            self.plan_generation+=1;self.plan_result=None;self.follow_result=None;self.follow_track=[]
            self.command=[0.,0.];self.last_sent_command=[0.,0.];self.command_at=0.;self.follow_arrived=False
            self.follow_job_key=None;self.follow_result_key=None
            self.waypoint_line=WaypointLineGuide()
            if mission.resume_index is None:
                self.goal=mission.skipped_index;self.execute=False;self.direct_follow=False
            else:self.goal=mission.resume_index
            self.waypoint_progress.reset();self.error=''
            self.point_follow_status={'status':'도킹 중단 · 다음 지점으로 진행' if aborted else '도킹 완료','tracking':'gps'}
            self.store.event('docking_aborted' if aborted else 'docking_completed',
                goal_index=mission.skipped_index,next_index=mission.resume_index)

        def gps_docking_report(self,now):
            docking=self.gps_docking
            if docking is None:return None
            value=docking.snapshot()
            value.update(active=bool(self.execute and self.priority_stage in ('neutralizing','docking','completing','aborting')),
                goal=docking.names[docking.index],complete=docking.complete,
                targets_gps=[list(latlon(*p,docking.origin)) for p in docking.targets])
            pose=self.state.snapshot(now) or self.state.display_pose(now)
            value['display_pose']=docking.map_pose(pose) if pose is not None and pose.get('yaw') is not None else None
            return value

        def publish_gps_docking(self,state,reason,now,execute=None):
            docking=self.gps_docking
            if docking is None:return
            report=self.gps_docking_report(now);p=report['display_pose']
            position=[p['x'],p['y']] if p else None
            row=dict(state=state,reason=reason,goal=docking.names[docking.index],legs=docking.legs,
                mission_id=self.priority_mission.mission_id,execute=self.execute if execute is None else execute,
                map_position_m=position,heading_rad=math.radians(p['yaw_deg'])-docking.lidar_yaw if p else None,
                goal_point_m=list(docking.map_points[docking.index]),mission_waypoints_m=[list(p) for p in docking.map_points],
                path_m=[position,list(docking.map_points[docking.index])] if position else [],
                position_source='gps',heading_source='imu',odom_source=None,
                output_limit_percent=self.follow_output_limit(),gps_targets=report['targets_gps'])
            self.tank_status_data=row;self.tank_status_at=now
            self.tank_status_pub.publish(String(data=json.dumps(row,ensure_ascii=False,allow_nan=False)))

        def tick_gps_docking(self,now):
            docking=self.gps_docking
            if docking is None:
                self.priority_stage='error';self.error='GPS 도킹 좌표 대기';return
            measured=self.state.snapshot(now) or self.state.display_pose(now)
            ready=(measured is not None and measured.get('pose_fresh',True) and measured.get('yaw') is not None and
                measured['position_sigma_m']<=config.get('navigation_position_sigma_limit_m',3.) and
                now-self.actuator_at<.3 and self.actuator.get('mode_name')=='AUTO' and applied_command(self.actuator,config) is not None)
            if not ready:
                self.command=[0.,0.];self.command_at=now;self.error='GPS·IMU·추진 상태 수신 대기'
                self.send(self.command,now);self.publish_gps_docking('WAITING',self.error,now);return
            state=docking.control_state(measured)
            if docking.advance(state['position'],float(config['arrival_radius_m'])):
                self.store.event('gps_docking_arrived',mission_id=self.priority_mission.mission_id,legs=docking.legs)
                self.command=[0.,0.];self.command_at=now;self.docking_line=WaypointLineGuide()
                if docking.complete:
                    self.priority_stage='completing';self.abort_child_done_at=None
                    self.send(self.command,now,disarm=True)
                    self.publish_gps_docking('COMPLETE','GPS 도킹 A/B 도달 완료',now)
                else:
                    self.send(self.command,now);self.publish_gps_docking('RUNNING','GPS 도킹 다음 지점',now)
                return
            if state.get('velocity') is None:
                heading=self.state.heading(now)
                state['yaw_rate_rad_s']=heading.get('yaw_rate_rad_s',0.) if heading else 0.
            state=self.docking_line.motion(state)
            goal=docking.targets[docking.index]
            previous=docking.targets[docking.index-1] if docking.index else docking.anchor
            target,cross=self.docking_line.target(state,goal,(self.priority_mission.mission_id,docking.index),previous)
            if docking.names[docking.index]=='B':
                command,_,error=buoy_goal_command(state,target,self.follow_output_limit(),reverse_only=True)
                delta=np.asarray(target)-state['position']
                stern_error=math.atan2(delta[1],delta[0])-state['yaw']-math.pi
                error=math.degrees(math.atan2(math.sin(stern_error),math.cos(stern_error)))
            else:
                command,_,error=point_follow_command(state,target,self.follow_output_limit(),self.last_sent_command)
            self.command=command;self.command_at=now;self.follow_result=None;self.avoidance_info=None;self.error=''
            self.follow_track=[list(latlon(*previous,docking.origin)),list(latlon(*goal,docking.origin))]
            self.point_follow_status=dict(status='GPS 도킹 '+docking.names[docking.index]+' 추종 중',
                distance_m=math.dist(state['position'],goal),heading_error_deg=error,
                cross_track_error_m=cross,tracking='gps_docking',position_source='gps',heading_source='imu',
                control_target_m=target.tolist(),controller='gps_docking_line_v1')
            self.send(command,now);self.publish_gps_docking('RUNNING','GPS 위치·IMU 방향 추종 중',now)

        def buoy_route_display(self):
            corridor=self.buoy_vision.corridor_data
            origin=getattr(self,'follow_origin',None)
            if corridor is None or origin is None or not self.buoy_vision.options['corridor']:return None
            center,fences=corridor;n=len(center)
            def preview(points):
                step=max(1,math.ceil(len(points)/50))
                return [latlon(*p,origin) for p in points[::step]]
            return dict(left=preview(fences[:n]),right=preview(fences[n:]),center=preview(center))

        def vision_nav(self):
            return dict(execute=self.execute,path=bool(getattr(self,'direct_follow',False) and self.follow_path),
                        gps=getattr(self,'follow_position_source','gps')=='gps',goal=self.goal,route=self.route,
                        dock=bool(self.route and waypoint_kind(self.route,self.goal)=='dock'),
                        priority=self.priority_stage)

        def docking_phase(self):
            if self.priority_stage=='aborting':return 'aborting'
            if self.priority_stage in ('neutralizing','docking','completing'):return 'docking'
            if self.execute and self.route and waypoint_kind(self.route,self.goal)=='dock':return 'searching'
            if self.docking_aborted:return 'aborted'
            return 'complete' if self.docking_complete else 'waiting'

        def manage_docking_slam(self,now):
            from .tank_session import running
            if (self.execute and self.route and waypoint_kind(self.route,self.goal)=='dock'
                and self.priority_stage=='gps' and self.docking_slam_auto_off):
                self.docking_slam_target=True
            job=self.docking_slam_job
            if job is not None:
                if job.poll() is None:return
                self.docking_slam_job=None
                actual=bool(running('mapping'))
                if actual==self.docking_slam_target:
                    self.docking_slam_auto_off=not actual;self.docking_slam_target=None;self.docking_slam_error=''
                else:self.docking_slam_error='도킹 후 SLAM 전환 재시도 대기'
            target=self.docking_slam_target
            if target is None or now-self.docking_slam_attempt<3.:return
            self.docking_slam_attempt=now
            runtime=Path.home()/'water_trials/runtime'
            try:
                runtime.mkdir(parents=True,exist_ok=True)
                with (runtime/'docking_slam.log').open('a',encoding='utf-8') as log:
                    self.docking_slam_job=subprocess.Popen([sys.executable,'-m','isv_water_trials.tank_session',
                        'start' if target else 'stop','mapping'],stdin=subprocess.DEVNULL,stdout=log,
                        stderr=subprocess.STDOUT,start_new_session=True,close_fds=True)
            except OSError as exc:self.docking_slam_error=str(exc)

        def tick_priority_route(self,now):
            if not self.execute:return False
            if self.priority_stage in ('aborting','completing'):
                aborting=self.priority_stage=='aborting'
                process=self.priority_process
                if process is not None and process.poll() is None:return True
                if self.abort_child_done_at is None:self.abort_child_done_at=now
                pending_done=not self.pending or all(f.done() for f in self.pending)
                neutral=(pending_done and self.actuator_at>self.abort_child_done_at and now-self.actuator_at<.3 and
                    self.actuator.get('receiving') and not self.actuator.get('armed') and
                    self.actuator.get('left')==1500 and self.actuator.get('right')==1500)
                if neutral:self.resume_after_docking(aborted=aborting)
                else:self.send([0.,0.],now,disarm=True)
                return True
            if getattr(self,'test_course',None) and self.test_course.get('running'):return False
            if getattr(self,'priority_stage','gps')=='gps':
                if waypoint_kind(self.route,self.goal)!='dock':return False
                version=gps_docking_version(self.route,self.goal)
                if version and not self.waypoint_progress.reached:return False
                self.docking_complete=False;self.docking_aborted=False
                if now-self.actuator_at>=.3 or self.actuator.get('mode_name')!='AUTO':return False
                session=getattr(self,'tank_session_data',{});map_id=session.get('session_id')
                if (now-getattr(self,'tank_session_at',0.)>1.5 or not session.get('fixed_map') or
                    not session.get('localization_ready') or not map_id or (map_id,self.goal) in self.priority_used_maps):return False
                match=getattr(self,'tank_match_data',{})
                if (now-getattr(self,'tank_match_at',0.)>1. or match.get('map_id')!=map_id or
                    not match.get('docking_match_ready')):return False
                if (match.get('docking_search_id')!=self.docking_search_id or
                    match.get('docking_goal_index')!=self.goal):return False
                stamp=match.get('docking_match_stamp_s')
                if not isinstance(stamp,(int,float)) or not math.isfinite(stamp) or not -.1<=self.ros_now()-stamp<.6:return False
                if getattr(self,'direct_follow',False) and self.follow_position_source!='gps':return False
                try:
                    route_path=Path.home()/'water_trials/tank/route.json'
                    if self.buoy_vision.options['objects'] and getattr(self,'docking_run_route',None) is None:
                        choice=self.buoy_vision.docking_choice(map_id)
                        if not choice:
                            self.error='도킹 객체·클래스별 경로 연결 확인 대기';return False
                        from .docking_profile_select import select_profile
                        if not select_profile(map_id,choice,route_path):
                            self.error='인식한 도킹 구간의 저장 A/B 경로 대기';return False
                    saved=self.registered_profile(version) if version else self.docking_route_for_run(route_path);docking_points(saved,map_id)
                    map_pose=match.get('display_pose')
                    if not isinstance(map_pose,dict) or map_pose.get('held'):return False
                    matched_pose=self.state.pose_at(float(map_pose['stamp_s']))
                    if matched_pose is None or self.state.origin is None:
                        self.error='정합 시점의 GPS·IMU 수신 대기';return False
                    gps_docking=GpsDocking(saved,map_pose,matched_pose,self.state.origin,
                        config.get('lidar_lever_arm_m',[0.,0.]),config.get('lidar_yaw_deg',0.))
                    if version:
                        gps_docking.registered_return(local_xy(*self.route['points'][self.goal],self.state.origin))
                    if 'tank_round_trip' in self.get_node_names():return False
                    mission=DockingMission.begin(map_id,self.goal,len(self.route['points']),uuid.uuid4().hex,
                        loop=not getattr(self,'direct_follow',False))
                    lease=open('/tmp/kabot_tank_trial_'+os.environ.get('ROS_DOMAIN_ID','0')+'.lock','w')
                    try:fcntl.flock(lease,fcntl.LOCK_EX | fcntl.LOCK_NB)
                    except OSError:lease.close();return False
                except (OSError,ValueError,KeyError,TypeError):return False
                self.gps_docking=gps_docking;self.gps_docking_lock=lease;self.docking_line=WaypointLineGuide()
                self.priority_process=None;self.priority_completion=None
                self.priority_mission=mission;self.priority_stage='neutralizing';self.priority_started=now
                self.store.event('gps_docking_frozen',mission_id=mission.mission_id,**gps_docking.snapshot())
                self.tank_status_data={};self.tank_status_at=0.
                self.priority_used_maps.add((map_id,self.goal));self.plan_generation+=1;self.plan_result=None
                self.follow_result=None;self.command=[0.,0.];self.command_at=now
                self.send([0.,0.],now,disarm=True)
                self.publish_gps_docking('WAITING','정합 완료 · A/B GPS 좌표 고정',now)
                return True
            if self.priority_stage=='neutralizing':
                pending_done=not self.pending or all(f.done() for f in self.pending)
                neutral=(pending_done and self.actuator_at>self.priority_started and
                    now-self.actuator_at<.3 and self.actuator.get('receiving') and not self.actuator.get('armed'))
                if not neutral:
                    self.send([0.,0.],now,disarm=True);return True
                if self.gps_docking is None:
                    self.priority_stage='error';self.error='GPS 도킹 좌표 변환 대기'
                    return True
                self.priority_stage='docking';self.error=''
                return True
            if self.priority_stage=='docking':
                self.tick_gps_docking(now);return True
            if self.priority_stage=='error':
                self.send([0.,0.],now,disarm=True);return True
            return False

        def on_velocity(self, msg):
            now = time.monotonic()
            try:
                stamp = self.stamp(msg)
                frame = self.state.velocity(stamp, now, msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.covariance,
                                            sample_age_s=max(0., self.ros_now()-stamp))
            except ValueError as exc: self.error = str(exc); return
            if frame is None: self.last_valid = False; return
            if self.last_frame_t is not None and (frame['t']-self.last_frame_t > 1.6 or frame['t'] <= self.last_frame_t):
                self.store.new_segment('sensor_gap'); self.comparison = LiveComparison(); self.ready_since = None
                if getattr(self,'test_comparison',None):self.test_comparison.plans.clear()
            self.last_frame_t = frame['t']
            valid = now-self.actuator_at < .3 and applied_command(self.actuator, config) is not None
            row = {'t': frame['t'], 'velocity': frame['velocity'], 'position': frame['position'], 'yaw': frame['yaw'],
                   'latlon': latlon(*frame['position'], self.state.origin),
                   'mode': self.actuator.get('mode_name'), 'valid': valid and self.learning,
                   'learning': self.learning, 'actuator': self.actuator,
                   'branch_id':self.lineage['id'] if self.lineage else None,
                   'commands': [c for c in self.commands if c[0] >= max(frame['t']-3,getattr(self,'learning_started_t',-math.inf) if self.learning else -math.inf)],
                   'gps_sigma_m': frame['position_sigma_m'], 'gps_velocity_sigma_m_s': frame['velocity_sigma_m_s'],
                   'imu_skew_s': frame['imu_skew_s']}
            if self.execute and self.route:
                info=dict(self.point_follow_status or {})
                row['navigation']=dict(goal_index=self.goal,goal_gps=self.route['points'][self.goal],
                    waypoint_kind=waypoint_kind(self.route,self.goal),waypoint_type=self.route.get('waypoint_types',[])[self.goal],tracking=info,
                    sent_command=list(self.last_sent_command),error=self.error)
                if self.docking_observation.get('active'):
                    row['docking_observation']={k:self.docking_observation.get(k) for k in ('version','phase','registered_a_gps','comparison_error_m','log')}
                if self.marine_lidar.active and now-self.marine_diagnostic_at >= 1.:
                    row['marine_lidar']=self.marine_lidar.diagnostic()
                    self.marine_diagnostic_at=now
                planned=self.display_result or {}
                reference=planned.get('smooth') or []
                if reference:
                    step=max(1,math.ceil(len(reference)/16))
                    row['navigation'].update(plan_stamp=planned.get('plan_stamp'),
                        path_m=reference[::step]+([reference[-1]] if (len(reference)-1)%step else []))
            if self.disk_ok:
                try: row = self.store.add(row); self.rows.append(row)
                except (OSError, sqlite3.Error) as exc: self.disk_ok = False; self.error = '기록 저장 오류: '+str(exc)
            if getattr(self,'test_course',None) and self.test_course['running']:
                if valid and self.actuator.get('mode_name')=='AUTO':
                    for event in self.test_comparison.observe(frame['t'],frame['position']):
                        self.record_test_event('model_test_comparison',test_id=self.test_course['id'],**event)
                else:self.test_comparison.plans.clear()
            comparison = self.comparison.update(self.model, frame['t'], frame['velocity'], list(self.commands)) if self.commands else None
            applied = applied_command(self.actuator, config)
            self.chart.append({'t': frame['t'], 'command': applied, 'measured': frame['velocity'],
                               'predicted': comparison['predicted'] if comparison else None,
                               'horizon_s': comparison['horizon_s'] if comparison else None})
            if self.ready_since is None: self.ready_since = now
            self.last_valid = valid

        def validate_route(self, data):
            return normalize_route(data)

        def load_model(self, identity):
            if len(identity) != 16 or any(c not in '0123456789abcdef' for c in identity): raise ValueError('모델 ID 오류')
            # Compatibility for older CLI scripts: choose validation only.
            self.validation_model=ResponseModel(self.store.load_compatible_model(identity))

        def save_selection(self):
            self.store.save_live_selection(self.model.data if self.model else None,
                self.validation_model.data if self.validation_model else None,self.learning)

        def record_test_event(self, action, **values):
            try:self.store.event(action,**values)
            except (OSError,sqlite3.Error) as exc:
                self.disk_ok=False;self.command_at=0.;self.error='시험 기록 저장 오류: '+str(exc)

        def follow_output_limit(self):
            if self.gps_docking is not None and self.priority_stage in ('neutralizing','docking','completing','aborting'):
                return waypoint_output(self.route,self.goal,self.drive_limits['dock'])
            kind=waypoint_kind(self.route,self.goal)
            default=float(config['percent']) if kind=='hold' else self.drive_limits['dock' if kind=='dock' else 'path' if kind in ('path','orbit') else 'point']
            return waypoint_output(self.route,self.goal,default)

        def refresh_drive_limits(self,now):
            if now-self.drive_limits_at<.2:return
            self.drive_limits_at=now
            try:
                info=(root/'drive_limits.json').stat();stamp=(info.st_mtime_ns,info.st_size)
                if stamp==self.drive_limits_stamp:return
                limits=read_drive_limits(root/'drive_limits.json',config['percent'])
            except (OSError,ValueError,KeyError,TypeError):return
            self.drive_limits_stamp=stamp;self.drive_limits=limits
            self.follow_percent=self.follow_output_limit()
            self.plan_generation+=1;self.plan_result=None;self.follow_result=None

        def refresh_arrival_radius(self,now):
            if now-self.arrival_settings_at<.2:return
            self.arrival_settings_at=now
            path=root/'gps_arrival.json'
            try:
                info=path.stat();stamp=(info.st_mtime_ns,info.st_size)
                if stamp==self.arrival_settings_stamp:return
                radius=read_arrival_radius(path,config['arrival_radius_m'])
            except (OSError,ValueError,KeyError,TypeError):return
            self.arrival_settings_stamp=stamp
            if radius==config['arrival_radius_m']:return
            config['arrival_radius_m']=radius;self.arrival_threshold=radius
            self.plan_generation+=1;self.plan_result=None;self.follow_result=None
            self.follow_mpc.key=None
            if self.planner is not None:self.planner.config['arrival_radius_m']=radius

        def refresh_waypoint_timeouts(self,now):
            if now-self.waypoint_timeouts_at<.2:return
            self.waypoint_timeouts_at=now
            try:self.waypoint_timeouts.limits=read_waypoint_timeouts(root/'waypoint_timeouts.json')
            except (OSError,ValueError,KeyError,TypeError):pass

        def tick_waypoint_timeout(self,now):
            if not self.waypoint_timeouts.update(self.route,self.goal,self.arrivals,self.execute,now):return False
            expired=self.waypoint_timeouts.snapshot(now);skipped=self.goal
            self.skip_waypoint(dict(goal_index=self.goal,arrivals=self.arrivals,docking_search_id=self.docking_search_id))
            self.waypoint_timeout_last=dict(goal_index=skipped,elapsed_s=expired['elapsed_s'],
                limit_s=expired['limits']['point7_s' if skipped==6 else 'orbit8_s'],next_index=self.goal if self.execute else None)
            self.waypoint_timeouts.reset()
            try:self.store.event('waypoint_timeout',**self.waypoint_timeout_last)
            except (OSError,sqlite3.Error):pass
            return True

        def brake_ready(self,now):
            return (0<=now-self.actuator_at<.3 and self.actuator.get('receiving')
                    and self.actuator.get('mode_name')=='AUTO' and self.actuator.get('armed')
                    and all(c.service_is_ready() for c in [self.arm,*self.throttles]))

        def tick_stop_brake(self,now):
            if not self.stop_brake.active:return False
            if not self.brake_ready(now) or self.stop_brake.finished(now):
                interrupted=not self.brake_ready(now)
                self.stop_brake.reset();self.command=[0.,0.];self.command_at=now
                # Neutral must not wait behind the last reverse service reply.
                for future in self.pending:
                    if not future.done():future.cancel()
                self.pending=[]
                self.send(self.command,now,disarm=interrupted or getattr(self,'stopping',False))
            else:
                self.command=[-.7,-.7];self.command_at=now
                if self.send(self.command,now,braking=True):self.stop_brake.sent(now)
            return True

        def stop_trial(self,brake=False):
            self.waypoint_timeouts.reset()
            self.restart_request=None
            self.buoy_orbit=None;self.orbit_status=None;self.leg_from_current_index=None
            self.cancel_docking()
            was_running = self.execute
            was_braking=self.stop_brake.active;self.stop_brake.reset()
            self.recovery_requested=False
            self.direct_follow = False
            self.buoy_course = False
            self.execute = False; self.command = [0., 0.]; self.command_at = time.monotonic()
            self.plan_generation += 1; self.plan_result = None
            if was_running or was_braking:
                self.stopping = True
                for future in self.pending:
                    if not future.done():future.cancel()
                self.pending=[]
                if brake and was_running and not was_braking and self.brake_ready(self.command_at):
                    self.stop_brake.begin(self.command_at);self.tick_stop_brake(self.command_at)
                else:self.send([0.,0.],self.command_at,disarm=True)
            if getattr(self,'test_course',None) and self.test_course['running']:
                try:self.save_test_map()
                except OSError as exc:self.error='관측 지도 저장 오류: '+str(exc)
                self.map_layers.end_survey()
                self.test_course['running']=False
                self.record_test_event('model_test_stopped',test_id=self.test_course['id'],metrics=self.test_comparison.summary())
                self.route=self.pre_test_route;self.goal=0

        def save_test_map(self):
            if getattr(self,'test_course',None) and getattr(self,'map_layers',None):
                course=self.test_course
                atomic_json(root/'maps'/(course['id']+'.json'),dict(
                    schema='gps-lidar-observation-map-v1',origin=course['origin'],
                    cell_m=self.map_layers.cell,points=self.map_layers.survey_points(),
                    stamp=self.ros_now(),reference='GPS/IMU aligned; no loop closure'))


        def docking_start_status(self,refresh=False,route=None,start_index=None):
            route=self.route if route is None else route
            if (route and (start_index is not None or route.get('mission')=='tournament') and
                not any(waypoint_kind(route,i)=='dock' for i in range(start_index or 0,len(route['points'])))):
                return {'ready':True,'reason':'','required':False}
            from .docking_start import docking_start_state
            from .tank_session import running
            now=time.monotonic()
            if refresh or now-getattr(self,'docking_start_checked',0.)>=.5:
                self.docking_start_checked=now
                self.docking_start_value=docking_start_state(Path.home(),running('mapping'),
                    self.tank_session_data,now-self.tank_session_at<3.)
            return {k:v for k,v in self.docking_start_value.items() if k!='route'}

        def capture_docking_route(self):
            self.registered_profiles={}
            status=self.docking_start_status(refresh=True)
            if not status['ready']:raise ValueError(status['reason'])
            if status.get('required') is False:
                self.docking_run_route=None
                return
            self.docking_run_route=deepcopy(self.docking_start_value['route'])
            for version in {gps_docking_version(self.route,i) for i in range(len((self.route or {}).get('points',[])))}-{None}:
                self.registered_profiles[version]=read_profile(Path.home(),status['map_id'],version)

        def docking_route_for_run(self,path):
            selected=getattr(self,'docking_run_route',None)
            return deepcopy(selected) if selected is not None else json.loads(path.read_text(encoding='utf-8'))

        def prepare_point_follow(self,data,route):
            if not route:raise ValueError('목적지를 추가하고 경로 저장을 누르세요')
            route=self.validate_route(route)
            index=data.get('start_index',0)
            if type(index) is not int or not 0<=index<len(route['points']):
                raise ValueError('시작할 웨이포인트를 선택하세요')
            for i in range(index,len(route['points'])):
                if waypoint_kind(route,i)=='orbit':orbit_choice(route,i)
            percent=float(data.get('percent',config['percent']))
            if not math.isfinite(percent) or not 1<=percent<=100:raise ValueError('출력 상한은 1~100%로 지정하세요')
            source=data.get('position_source','gps');tracking=data.get('tracking','point')
            if source not in ('gps','virtual'):raise ValueError('위치 입력 선택 오류')
            if tracking not in ('point','path'):raise ValueError('추종 방식 선택 오류')
            if type(data.get('buoy_course',False)) is not bool:raise ValueError('buoy_course must be boolean')
            conflicts={'rc_auto_mode_test','tank_round_trip','water_trial_runner','water_trial_avoid_preview','course_mission'} & set(self.get_node_names())
            # An owned docking child is stopped by the restart transaction.
            if data['action']=='restart_point_follow' and self.priority_process is not None and self.priority_process.poll() is None:
                conflicts.discard('tank_round_trip')
            if conflicts:raise ValueError('기존 추진 노드를 먼저 종료하세요: '+', '.join(sorted(conflicts)))
            limits=validate_drive_limits(data['drive_limits']) if 'drive_limits' in data else None
            status=self.docking_start_status(refresh=True,route=route,
                start_index=index if data['action']=='restart_point_follow' else None)
            if not status['ready']:raise ValueError(status['reason'])
            dock_route=None;profiles={}
            if status.get('required') is not False:
                dock_route=deepcopy(self.docking_start_value['route'])
                for version in {gps_docking_version(route,i) for i in range(index,len(route['points']))}-{None}:
                    profiles[version]=read_profile(Path.home(),status['map_id'],version)
            return dict(route=route,start_index=index,percent=percent,source=source,tracking=tracking,
                buoy_course=data.get('buoy_course',False),limits=limits,dock_route=dock_route,profiles=profiles)

        def begin_point_follow(self,prepared):
            self.waypoint_timeouts.reset()
            self.buoy_orbit=None;self.orbit_status=None;self.leg_from_current_index=None
            if prepared['limits'] is not None:
                atomic_json(root/'drive_limits.json',prepared['limits']);self.drive_limits=prepared['limits']
            route=prepared['route'];self.route=route;self.goal=prepared['start_index'];self.run_start_index=self.goal
            self.arrivals=0;self.direction=1;self.follow_percent=prepared['percent']
            self.waypoint_progress.reset();self.waypoint_line=WaypointLineGuide();self.path_controller=PathAvoidanceController()
            self.follow_path=prepared['tracking']=='path';self.follow_track=[]
            self.follow_position_source=prepared['source'];fix=getattr(self,'virtual_fix',None)
            self.follow_origin=list(fix['point'] if prepared['source']=='virtual' and fix else route['points'][0])
            self.direct_follow=True;self.follow_arrived=False;self.follow_reverse=False
            self.recovery_requested=False;self.avoidance_info=None
            self.follow_result=None;self.follow_plan_at=0.;self.follow_job_key=None;self.follow_result_key=None
            self.point_follow_status={'status':'센서 수신 대기'}
            self.planner=None;self.plan_result=None;self.test_course=None;self.test_comparison=None
            self.command=[0.,0.];self.last_sent_command=[0.,0.];self.command_at=0.
            self.priority_stage='gps';self.priority_used_maps=set();self.priority_mission=None;self.priority_process=None
            self.docking_complete=False;self.docking_aborted=False;self.abort_child_done_at=None
            self.docking_search_id=uuid.uuid4().hex
            self.docking_run_route=prepared['dock_route'];self.registered_profiles=prepared['profiles']
            self.buoy_course=prepared['buoy_course']
            self.display_revision+=1;self.display_key=None;self.display_result=None
            self.execute=True;self.stopping=False;self.plan_generation+=1

        def queue_point_restart(self,data):
            if self.restart_request is not None:raise ValueError('선택 지점 재시작 처리 중입니다')
            prepared=self.prepare_point_follow(data,data.get('route',self.route))
            self.stop_trial()
            self.restart_request=dict(prepared=prepared,request_id=data.get('request_id'),neutral_after=None)
            self.store.event('waypoint_restart_queued',start_index=prepared['start_index'])

        def tick_point_restart(self,now):
            request=self.restart_request
            if request is None:return False
            process=self.priority_process
            if process is not None and process.poll() is None:
                self.send([0.,0.],now,disarm=True);return True
            # Require telemetry newer than the child's exit and our disarm.
            if request['neutral_after'] is None:
                request['neutral_after']=now;self.send([0.,0.],now,disarm=True);return True
            neutral=(all(f.done() for f in self.pending) and self.actuator_at>request['neutral_after'] and
                0<=now-self.actuator_at<.3 and self.actuator.get('receiving') and
                not self.actuator.get('armed') and self.actuator.get('left')==1500 and self.actuator.get('right')==1500)
            if not neutral:
                self.send([0.,0.],now,disarm=True);return True
            self.restart_request=None
            try:
                atomic_json(root/'route.json',request['prepared']['route'])
                self.begin_point_follow(request['prepared'])
                self.error='';self.ack=dict(id=request['request_id'],ok=True,message=f'{self.goal+1}번 지점부터 주행 시작됨')
                self.store.event('waypoint_restart_started',start_index=self.goal)
            except (ValueError,TypeError,KeyError,OSError) as exc:
                self.stop_trial();self.error=str(exc)
                self.ack=dict(id=request['request_id'],ok=False,message=self.error)
            return True

        def follow_previous(self,origin):
            # The first leg begins at the boat, never at a skipped waypoint.
            if not self.goal or self.goal==self.leg_from_current_index or (self.direct_follow and self.goal==self.run_start_index):return None
            return local_xy(*self.route['points'][self.goal-1],origin)

        def apply_route(self, route):
            if self.restart_request is not None:
                raise ValueError('선택 지점 재시작 처리 후 경로를 저장하세요')
            running=self.execute
            previous=self.route;old_goal=self.goal;arrivals=self.arrivals
            index=live_goal(previous,route,old_goal) if running else 0
            changed=running and (index is None or leg_signature(previous,old_goal)!=leg_signature(route,index))
            target_changed=running and (index is None or leg_signature(previous,old_goal)[:-1]!=leg_signature(route,index)[:-1])
            # Persist first: a failed save must leave the active mission untouched.
            atomic_json(root/'route.json',route)
            if not running:
                self.stop_trial();self.route=route;self.goal=0;self.direction=1;self.arrivals=0
                self.run_start_index=0;self.follow_arrived=False;self.waypoint_progress.reset()
                self.follow_result=None;self.follow_track=[];self.arrival_threshold=config['arrival_radius_m']
            elif index is None:
                self.stop_trial();self.route=route;self.goal=len(route['points'])-1;self.arrivals=arrivals
            else:
                self.route=route;self.goal=index
                if getattr(self,'test_course',None) and self.test_course.get('running'):
                    self.pre_test_route=deepcopy(route)
                start=live_goal(previous,route,self.run_start_index)
                self.run_start_index=start if start is not None else index
                used=set()
                for map_id,old in self.priority_used_maps:
                    if not 0<=old<len(previous['points']):continue
                    mapped=live_goal(previous,route,old)
                    if mapped is not None and previous['points'][old]==route['points'][mapped]:used.add((map_id,mapped))
                self.priority_used_maps=used
                if not target_changed:
                    if self.waypoint_progress.index==old_goal:self.waypoint_progress.index=index
                    if self.buoy_orbit is not None and index!=old_goal:
                        key=list(self.buoy_orbit.key);key[1]=index;self.buoy_orbit.key=tuple(key)
                mission=self.priority_mission
                if mission is not None:
                    mission.skipped_index=index
                    mission.resume_index=index+1 if index+1<len(route['points']) else (None if self.direct_follow else 0)
                if changed:
                    docking=self.gps_docking
                    registered=(docking is not None and getattr(docking,'registered_a',None) is not None and
                        gps_docking_version(previous,old_goal)==gps_docking_version(route,index) and gps_docking_version(route,index) is not None)
                    if registered and target_changed:
                        progress=(docking.index,docking.legs,docking.complete)
                        docking.registered_return(local_xy(*route['points'][index],docking.origin))
                        docking.index,docking.legs,docking.complete=progress
                        self.docking_line=WaypointLineGuide()
                    elif target_changed and self.priority_stage in ('neutralizing','docking','completing','aborting','error'):
                        self.cancel_docking();self.priority_stage='gps';self.priority_mission=None
                    self.plan_generation+=1;self.plan_result=None;self.follow_result=None
                    self.follow_job_key=None;self.follow_result_key=None;self.follow_plan_at=0.
                    self.display_revision+=1;self.display_key=None;self.display_result=None
                    if target_changed:self.waypoint_progress.reset()
                    self.waypoint_line=WaypointLineGuide();self.path_controller=PathAvoidanceController()
                    self.follow_track=[];self.follow_arrived=False;self.follow_reverse=False
                    if target_changed:self.buoy_orbit=None;self.orbit_status=None
                    self.corridor_scan=None;self.avoidance_info=None
                    self.stop_brake.reset();self.command_at=0.;self.recovery_requested=False
                    self.leg_from_current_index=index;self.arrival_threshold=config['arrival_radius_m']
                    if target_changed and not registered:self.docking_search_id=uuid.uuid4().hex
                self.docking_start_at=0.
            self.route_update=dict(live=running,goal=self.goal,arrivals=self.arrivals,
                target_changed=bool(target_changed),remaining=0 if running and index is None else max(0,len(route['points'])-self.goal),
                finished=running and index is None)

        def request(self, msg):
            data = {}; ok = False
            try:
                data = json.loads(msg.data); action = data['action']
                if action in ('start_point_follow','start','start_model_test') and self.restart_request is not None:
                    raise ValueError('선택 지점 재시작 처리 중입니다')
                if action in ('start','start_model_test'):
                    if self.execute:raise ValueError('현재 주행을 정지한 뒤 시작하세요')
                    self.waypoint_line=WaypointLineGuide()
                    self.path_controller=PathAvoidanceController()
                    self.capture_docking_route()
                    if type(data.get('buoy_course',False)) is not bool:raise ValueError('buoy_course must be boolean')
                    process=getattr(self,'priority_process',None)
                    if process is not None and process.poll() is None:raise ValueError('도킹 종료·중립 확인 후 다시 시작하세요.')
                    if 'drive_limits' in data:
                        self.drive_limits=validate_drive_limits(data['drive_limits'])
                        atomic_json(root/'drive_limits.json',self.drive_limits)
                if action in ('pause', 'resume'):
                    self.fit_generation += 1; self.learning = action == 'resume'
                    if self.learning:
                        self.learning_started_t=self.ros_now();self.last_disk=0.;self.bag_at=0.
                    else:self.stop_recording()
                    self.store.new_segment(action); self.save_selection()
                    self.fit_status='학습 중지' if not self.learning else '주행 기록 수집 중'
                elif action in ('apply_model','freeze'):
                    if self.model is None: raise ValueError('먼저 주행 기록으로 모델을 학습하세요')
                    if self.execute:raise ValueError('주행 정지 후 새 검증 모델을 적용하세요')
                    self.store.finalize_model(self.model.data)
                    self.validation_model=ResponseModel(deepcopy(self.model.data))
                    self.save_selection()
                elif action == 'load':
                    self.stop_trial(); self.load_model(data['id']); self.save_selection()
                elif action == 'route':
                    route = self.validate_route(data)
                    self.apply_route(route)
                elif action == 'regions':
                    if self.execute:raise ValueError('주행을 정지한 뒤 구역을 변경하세요')
                    regions=data.get('regions',[])
                    gps_regions(regions)
                    atomic_json(root/'regions.json',regions);self.regions=deepcopy(regions)
                elif action == 'restart_point_follow':
                    self.queue_point_restart(data)
                elif action == 'start_point_follow':
                    if self.execute:raise ValueError('현재 주행을 정지한 뒤 시작하세요')
                    process=self.priority_process
                    if process is not None and process.poll() is None:raise ValueError('도킹 종료 후 다시 시작하세요')
                    prepared=self.prepare_point_follow(data,self.route)
                    if self.stop_brake.active:self.stop_trial()
                    self.begin_point_follow(prepared)
                elif action == 'point_follow_limit':
                    percent=float(data['percent'])
                    if not math.isfinite(percent) or not 1<=percent<=100:raise ValueError('출력 상한은 1~100%로 지정하세요')
                    if not getattr(self,'direct_follow',False):raise ValueError('지점 추종 실행 대기')
                    self.follow_percent=percent
                    self.command=np.clip(self.command,-percent/100,percent/100).tolist()
                elif action in ('start','start_model_test'):
                    if self.execute:raise ValueError('현재 주행을 정지한 뒤 시작하세요')
                    if self.stop_brake.active:self.stop_trial()
                    if not self.validation_model:raise ValueError('현재 모델을 먼저 적용하세요')
                    if not self.route:raise ValueError('지도에서 웨이포인트를 지정하고 경로 저장을 누르세요')
                    route=self.validate_route(self.route)
                    if len(route['points'])<2:raise ValueError('순환 주행은 지점을 2개 이상 지정하세요')
                    percent=float(data.get('percent',config['percent']))
                    conflicts={'rc_auto_mode_test','tank_round_trip','water_trial_runner','water_trial_avoid_preview'} & set(self.get_node_names())
                    if conflicts:raise ValueError('기존 추진 노드를 먼저 종료하세요: '+', '.join(sorted(conflicts)))
                    # Follow the operator's saved route. Live lidar remains the
                    # obstacle source; no generated targets or virtual discs.
                    drive_config=dict(config,obstacle_margin_m=.1,prediction_margin_m=0.)
                    planner=OutdoorPlanner(self.validation_model.data,drive_config,percent)
                    self.planner=planner;self.route=route;self.goal=0;self.arrivals=0;self.recovery_requested=False
                    self.arrival_threshold=config['arrival_radius_m']
                    self.command=[0.,0.];self.command_at=0.;self.last_sent_command=[0.,0.]
                    self.test_course=None;self.test_comparison=None
                    if action=='start_model_test':
                        if self.state.origin is None:raise ValueError('GPS 위치 수신 대기')
                        course=dict(id=uuid.uuid4().hex,model_id=self.validation_model.data['id'],running=True,
                            navigation='saved_route',origin=list(self.state.origin),percent=percent,
                            obstacles=[],area=None,regions=None,
                            goals=[local_xy(*p,self.state.origin).tolist() for p in route['points']])
                        self.store.event('model_test_started',course=course)
                        self.pre_test_route=deepcopy(route);self.test_course=course;self.test_comparison=PlanComparison()
                        self.map_layers.begin_survey(course['id']);self.map_save_at=0.
                    self.priority_stage='gps';self.priority_used_maps=set();self.priority_mission=None
                    self.docking_complete=False;self.docking_aborted=False
                    self.docking_search_id=uuid.uuid4().hex
                    self.buoy_course=data.get('buoy_course',False)
                    self.waypoint_timeouts.reset()
                    self.execute=True;self.stopping=False;self.plan_generation+=1
                elif action == 'skip_waypoint':self.skip_waypoint(data)
                elif action == 'abort_orbit':
                    self.abort_orbit(data)
                elif action == 'abort_docking':self.abort_docking(data)
                elif action == 'stop': self.stop_trial(brake=True)
                elif action == 'clear_trail':
                    self.trail.clear()
                    atomic_json(root/'display.json', {'trail_after_t': self.ros_now()})
                else: raise ValueError('알 수 없는 요청')
                self.store.event(action, model=self.model.data['id'] if self.model else None); self.error = ''; ok = True
            except (ValueError, TypeError, KeyError, OSError,sqlite3.Error) as exc: self.error = str(exc)
            self.ack = {'id': data.get('request_id'), 'ok': ok, 'message': self.error if not ok else
                self.waypoint_skip_status['message'] if data.get('action')=='skip_waypoint' else
                f"기존 주행 종료 · {self.restart_request['prepared']['start_index']+1}번 지점 시작 대기"
                if data.get('action')=='restart_point_follow' and self.restart_request else
                f"경로 저장 · {self.goal+1}번부터 주행 중 변경 반영" if data.get('action')=='route' and self.execute else '적용됨'}

        def stop_recording(self):
            bag=getattr(self,'bag',None)
            if bag and bag.poll() is None and not getattr(self,'bag_stopping',False):
                try:os.killpg(bag.pid,signal.SIGINT)
                except ProcessLookupError:pass
                self.bag_stopping=True
            self.bag_status='학습 중지'

        def manage_bag(self, now):
            if not self.learning:
                self.stop_recording();return
            if args.no_bag: self.bag_status = 'fixture: bag disabled'; return
            free = shutil.disk_usage(root).free
            self.disk_ok = free > 256*1024**2
            if self.bag and self.bag.poll() is None:
                if self.bag_stopping:
                    self.bag_status='이전 기록 마무리 중';return
                self.bag_status = 'recording'
                if free < 1024**3: os.killpg(self.bag.pid, signal.SIGINT); self.bag_status = '원본 기록: 디스크 여유 대기'
                return
            if now-self.bag_at < 10 or free < 2*1024**3: return
            self.bag_at = now
            self.bag_stopping=False
            folder = root/'bags'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ'); folder.parent.mkdir(exist_ok=True)
            if self.bag_log: self.bag_log.close()
            self.bag_log = (root/'bag.log').open('a', encoding='utf-8')
            topics = ['/gps/fix', '/gps/fix_velocity', '/vectornav/imu', '/vectornav/raw/common', '/scan',
                      '/velodyne_points', '/actuators/status', '/outdoor/command', '/tf', '/tf_static']
            self.bag = subprocess.Popen(['ros2', 'bag', 'record', '-o', str(folder), '--max-bag-duration', '60', *topics],
                                        stdin=subprocess.DEVNULL, stdout=self.bag_log, stderr=subprocess.STDOUT, start_new_session=True)

        def send(self, command, now, disarm=False, braking=False):
            if (not disarm and not braking and not self.buoy_course and waypoint_kind(self.route,self.goal) not in ('point','hold','orbit') and hasattr(self,'buoy_vision')
                and self.buoy_vision.options['corridor']):
                self.buoy_vision.context=self.vision_nav()
                current=self.state.display_pose(now)
                if current is not None:
                    guarded=self.buoy_vision.guard(command,current,self.validation_model or self.model,self.map_layers.hits)
                    if not np.allclose(command,guarded):
                        command=guarded;self.command=list(command)
                        self.error='라이다 통로 경계/장애물 확인 · 출력 대기'

            if not disarm and not braking:
                cap=self.follow_output_limit()
                command=limit_output(command,cap)
                self.command=list(command)
            if self.pending and not all(f.done() for f in self.pending):
                if now-self.pending_at <= .6: return
                self.error = '추진 응답 재연결 대기'
                for future in self.pending:
                    if not future.done(): future.cancel()
                self.pending = []; self.command_at = 0.
                command = [0., 0.]
            if not all(c.service_is_ready() for c in [self.arm, *self.throttles]): return
            self.pending_at = now
            if disarm:
                # D itself neutralizes AUTO; never race it against a C packet.
                self.pending=[self.arm.call_async(SetBool.Request(data=False))]
                command=[0.,0.]
            elif now-self.actuator_at>=.3 or not self.actuator.get('receiving'):
                self.last_sent_command=[0.,0.];return
            elif self.actuator.get('mode_name')!='AUTO':
                # MCU owns RC priority. Keep planning, without flooding serial
                # with arms that it will reject while the receiver owns output.
                self.last_sent_command=[0.,0.];return
            elif not self.actuator.get('armed'):
                # Firmware A resets outputs to neutral. Wait for telemetry to
                # confirm this one arm before sending paired throttle commands.
                self.pending=[self.arm.call_async(SetBool.Request(data=True))]
                self.last_sent_command=[0.,0.];return
            else:
                self.pending=[c.call_async(ThrottlePulseWidth.Request(pulse_width=p)) for c,p in zip(self.throttles,logical_pwm(command))]
            self.last_sent_command=list(command)
            self.command_pub.publish(String(data=json.dumps({'t': self.ros_now(), 'command': command, 'pwm': logical_pwm(command), 'disarm': disarm})))
            return not disarm and list(command)==[-.7,-.7]

        def anticipate_command(self,command,state,goal,percent,now,selected=None):
            selected=selected or self.validation_model or self.model
            if not config.get('early_avoidance_enabled',True):
                self.avoidance_info={'available':False,'reason':'선제 회피 비활성'};return command
            if selected is None or state.get('velocity') is None:
                self.avoidance_info={'available':False,'reason':'선택된 실측 모델·실측 속도 대기'};return command
            if getattr(self,'direct_follow',False) and getattr(self,'follow_position_source','gps')=='virtual':
                self.avoidance_info={'available':False,'reason':'실제 GPS 위치에서 모델 선제 회피 사용'};return command
            try:
                if self.scan is None or now-self.scan_at>=config['scan_max_age_s']:raise ValueError('선제 회피 · 라이다 최신값 대기')
                pose=self.state.pose_at(self.scan['stamp'])
                if not isinstance(pose,dict):raise ValueError('선제 회피 · 라이다 시각 자세 대기')
                if not hasattr(self,'early_avoidance'):self.early_avoidance=EarlyAvoidance()
                command,self.avoidance_info=self.early_avoidance.adjust(selected,state,goal,
                    dict(self.scan,pose=pose),list(self.commands),config,percent,command,
                    getattr(self,'last_sent_command',[0.,0.]),self.ros_now())
            except (ValueError,RuntimeError,KeyError,TypeError) as exc:
                self.avoidance_info={'available':False,'model_id':selected.data.get('id'),'reason':str(exc)}
            return command

        def publish_navigation_scan(self,scan):
            # A small copy of the controller input; never rerun perception here.
            if not self.navigation_scan_pub.get_subscription_count():return
            message=LaserScan()
            source=scan if scan is not None else self.scan
            stamp=float(source['stamp']) if source is not None else self.ros_now()
            message.header.stamp.sec=int(stamp)
            message.header.stamp.nanosec=int((stamp-int(stamp))*1e9)
            message.header.frame_id=source.get('frame_id','velodyne') if source is not None else 'velodyne'
            message.range_min=float(source.get('range_min',0.)) if source is not None else 0.
            message.range_max=float(source.get('range_max',100.)) if source is not None else 100.
            message.scan_time=.1
            if scan is not None and len(scan['angles']):
                message.angle_min=float(scan['angles'][0]);message.angle_max=float(scan['angles'][-1])
                message.angle_increment=float(scan['angles'][1]-scan['angles'][0]) if len(scan['angles'])>1 else 0.
                message.ranges=[float(value) for value in scan['ranges']]
            self.navigation_scan_pub.publish(message)

        def recover_follow(self,state,goal,now,reason):
            scan=self.scan if self.scan is not None and now-self.scan_at<config['scan_max_age_s'] else None
            self.publish_navigation_scan(scan)
            command,error,maneuver=self.path_controller.command(state,state.get('gps_line_target_m',goal),
                scan,config,self.follow_percent,now,
                (self.docking_search_id,self.goal,self.actuator.get('mode_name')),blocked=True,
                buoy_contact=self.buoy_course)
            self.command=command;self.command_at=now;self.error=reason
            self.follow_reverse=maneuver['maneuver_phase']=='reverse'
            self.follow_result=None
            self.point_follow_status=dict(status=maneuver['maneuver'],control=reason,
                tracking='path',heading_error_deg=error,**maneuver)
            self.send(self.command,now)

        def tick_buoy_orbit(self,state,origin,now):
            try:settings,choice=orbit_choice(self.route,self.goal)
            except ValueError as exc:
                self.error=str(exc);self.command=[0.,0.];self.command_at=now;self.send(self.command,now);return
            seed=local_xy(*choice['point'],origin)
            key=(self.docking_search_id,self.goal,choice['id'],tuple(seed),settings['radius_m'],choice['entry_side'],choice['direction'])
            if self.buoy_orbit is None or self.buoy_orbit.key!=key:
                dock=next((local_xy(*self.route['points'][i],origin) for i in reversed(range(self.goal)) if waypoint_kind(self.route,i)=='dock'),np.asarray(state['position']))
                neighbors=[local_xy(*p['point'],origin) for p in settings['options'] if p['id']!=choice['id'] and p['point'] is not None]
                self.buoy_orbit=BuoyOrbit(seed,dock,choice['entry_side'],choice['direction'],settings['radius_m'],neighbors,key)
                self.path_controller=PathAvoidanceController();self.waypoint_line=WaypointLineGuide()
            orbit=self.buoy_orbit;stamp=self.ros_now()
            scan=self.orbit_input_scan;scan_pose=self.state.pose_at(scan['stamp']) if scan is not None else None
            virtual=getattr(self,'follow_position_source','gps')=='virtual'
            if not virtual:
                marine=self.marine_lidar.result if self.marine_lidar.origin==tuple(origin) else None
                orbit.observe(scan,scan_pose,config,marine,stamp)
            allowed=self.actuator.get('mode_name')=='AUTO' and bool(self.actuator.get('armed'))
            state=self.waypoint_line.motion(state)
            status=orbit.step(state,stamp,allowed=allowed);status.update(selected=choice['id'],waypoint_index=self.goal)
            status['centre_gps']=latlon(*orbit.centre,origin);self.orbit_status=status
            if status['complete']:
                try:self.store.event('buoy_orbit_completed',**status)
                except (OSError,sqlite3.Error):pass
                self.arrivals+=1;self.command=[0.,0.];self.command_at=now
                if self.goal+1<len(self.route['points']):
                    self.goal+=1;self.leg_from_current_index=self.goal
                    self.waypoint_progress.reset();self.waypoint_line=WaypointLineGuide();self.path_controller=PathAvoidanceController()
                    self.follow_arrived=False;self.follow_result=None;self.follow_track=[]
                    self.plan_generation+=1;self.display_revision+=1;self.display_key=None;self.display_result=None
                    self.error='';self.send(self.command,now)
                else:self.stop_trial()
                return
            scan=self.scan if self.scan is not None and now-self.scan_at<config['scan_max_age_s'] else None
            self.publish_navigation_scan(scan)
            command,error,maneuver=self.path_controller.command(state,np.asarray(orbit.target),scan,config,self.follow_percent,now,
                (self.docking_search_id,self.goal,'orbit'),buoy_contact=self.buoy_course)
            self.command=command;self.command_at=now;self.follow_reverse=maneuver['maneuver_phase']=='reverse'
            self.follow_result=None;self.follow_track=[latlon(*point,origin) for point in orbit.preview(state)]
            self.avoidance_info=None;self.error=''
            label={'entry':'선회 부표 진입','search':'선회 부표 탐색','orbit':'부표 한 바퀴 선회'}[status['phase']]
            self.point_follow_status=dict(status=label,tracking='orbit',control='GPS·IMU 주행 / 라이다 부표 중심 보정',
                heading_error_deg=error,orbit=status,**maneuver)
            if stamp-orbit.last_log>=1.:
                orbit.last_log=stamp
                try:self.store.event('buoy_orbit_sample',**status,position_m=list(map(float,state['position'])),
                    position_stamp_s=state.get('position_stamp_s'),yaw_rad=float(state['yaw']),command=list(command),actuator=deepcopy(self.actuator))
                except (OSError,sqlite3.Error):pass
            self.send(command,now)

        def tick_point_follow(self, now):
            self.follow_percent=self.follow_output_limit()
            virtual=getattr(self,'follow_position_source','gps')=='virtual'
            if virtual:
                fix=getattr(self,'virtual_fix',None);heading=self.state.heading(now)
                state=None
                if fix and 0<=now-fix['received']<=config['gps_max_age_s'] and heading is not None:
                    real=self.state.snapshot(now)
                    state={'position':local_xy(*fix['point'],self.follow_origin).tolist(),
                           'yaw':math.radians(90-heading['heading_deg']),
                           'velocity':real.get('velocity') if real else None,
                           'yaw_rate_rad_s':heading.get('yaw_rate_rad_s',0.),'position_sigma_m':.5}
                origin=self.follow_origin
            else:
                state=self.state.snapshot(now) or self.state.display_pose(now)
                origin=self.state.origin
            ready=(state is not None and state.get('pose_fresh',True) and state.get('yaw') is not None and
                   state['position_sigma_m']<=config.get('navigation_position_sigma_limit_m',3.) and
                   now-self.actuator_at<.3 and applied_command(self.actuator,config) is not None)
            if not ready:
                if not (waypoint_kind(self.route,self.goal)=='hold' and self.waypoint_progress.reached):
                    self.waypoint_progress.entered=None
                self.command=[0.,0.];self.command_at=now
                if waypoint_kind(self.route,self.goal) in ('path','orbit'):self.publish_navigation_scan(None)
                self.error=('가상 GPS' if virtual else 'GPS')+'·IMU·추진 상태 수신 대기'
                self.point_follow_status={'status':self.error};self.send(self.command,now);return
            while True:
                goal=local_xy(*self.route['points'][self.goal],origin)
                distance=float(np.linalg.norm(goal-state['position']))
                threshold=config['arrival_radius_m']+(.3 if getattr(self,'follow_arrived',False) else 0.)
                kind=waypoint_kind(self.route,self.goal)
                if kind=='orbit':
                    self.tick_buoy_orbit(state,origin,now);return
                allowed=True
                progress=self.waypoint_progress.update(self.route,self.goal,distance<=threshold,now,allowed)
                if progress=='approach':
                    self.follow_arrived=False;break
                if progress=='dock_timeout':
                    skipped=self.goal
                    self.arrivals+=1;self.command=[0.,0.];self.last_sent_command=[0.,0.];self.command_at=now
                    self.plan_generation+=1;self.plan_result=None;self.follow_result=None;self.follow_track=[]
                    self.follow_arrived=False;self.waypoint_progress.reset()
                    self.record_test_event('docking_search_timeout',goal_index=skipped,timeout_s=DOCK_SEARCH_SECONDS,next_index=skipped+1 if skipped+1<len(self.route['points']) else None)
                    self.point_follow_status={'status':f'{DOCK_SEARCH_SECONDS:g}초 미포착 · 도킹 건너뜀','tracking':'dock_timeout'}
                    if skipped+1<len(self.route['points']):
                        self.goal+=1;self.error='';self.send(self.command,now)
                    else:self.stop_trial()
                    return
                if progress=='dock_wait':
                    elapsed=now-self.waypoint_progress.dock_started
                    scan=self.scan if self.scan is not None and now-self.scan_at<config['scan_max_age_s'] else None
                    clear=scan is not None and (self.buoy_course or not maneuver_space(scan,config,math.radians(15))['rotation_blocked'])
                    self.command=docking_search_command(state,self.follow_percent,elapsed,clear,
                        getattr(self,'last_sent_command',[0.,0.]),config.get('command_slew_per_s',.35))
                    self.command_at=now;self.follow_arrived=True;self.follow_result=None;self.follow_track=[]
                    self.avoidance_info=None;self.error=''
                    label='저장 지도 탐색 · 저속 제자리 회전' if clear else '저장 지도 탐색 · 라이다/회전 공간 대기'
                    self.point_follow_status=dict(status=label,control=label,maneuver='도킹 탐색',tracking='dock_search',
                        distance_m=distance,search_remaining_s=max(0.,DOCK_SEARCH_SECONDS-elapsed))
                    self.send(self.command,now);return
                if progress in ('hold','final_hold','complete'):
                    first_arrival=not getattr(self,'follow_arrived',False)
                    if progress=='final_hold' and not getattr(self,'follow_arrived',False):self.arrivals+=1
                    self.follow_arrived=True;self.command=[0.,0.];self.command_at=now;self.error=''
                    label={'hold':'위치유지 중',
                           'final_hold':'최종 지점 도착 · 위치유지 중','complete':'미션 완료'}[progress]
                    self.avoidance_info={'available':False,'reason':label}
                    self.point_follow_status={'status':label,'distance_m':distance}
                    if progress=='complete':
                        self.arrivals+=1;self.stop_trial()
                    elif first_arrival and self.brake_ready(now):
                        self.stop_brake.begin(now,duration=.8 if kind=='hold' else .5);self.tick_stop_brake(now)
                    else:self.send(self.command,now)
                    return
                self.arrivals+=1
                self.goal+=1
                self.follow_percent=self.follow_output_limit()
                self.follow_arrived=False;self.waypoint_progress.reset()
                self.plan_generation+=1;self.follow_result=None;self.follow_track=[]
                if not getattr(self,'direct_follow',False):
                    self.command=[0.,0.];self.command_at=now
                    self.send(self.command,now);return
            state=dict(state,maneuver_reverse=getattr(self,'follow_reverse',False),arrival_goal=goal.tolist())
            approach_distance=config['arrival_radius_m']+config['length_m']
            final_approach=distance<=approach_distance
            if state.get('velocity') is None and 'yaw_rate_rad_s' not in state:
                heading=self.state.heading(now)
                state['yaw_rate_rad_s']=heading.get('yaw_rate_rad_s',0.) if heading else 0.
            control_goal=goal;track_status={}
            state=self.waypoint_line.motion(state)
            previous=self.follow_previous(origin)
            key=(self.docking_search_id,self.goal,tuple(goal),tuple(previous) if previous is not None else None)
            line_goal,cross_track=self.waypoint_line.target(state,goal,key,previous)
            state['gps_line_target_m']=line_goal.tolist()
            if kind in ('point','hold','dock'):
                # GPS point/hold approaches bypass the path and corridor planners.
                control_goal=line_goal
                command,_,error=point_follow_command(dict(state,final_approach=final_approach),control_goal,self.follow_percent,
                    getattr(self,'last_sent_command',[0.,0.]))
                self.follow_result=None;self.corridor_scan=None;self.follow_reverse=False
                self.follow_track=[latlon(*self.waypoint_line.anchor,origin),list(self.route['points'][self.goal])]
                self.avoidance_info=None;self.error='';self.command=command;self.command_at=now
                self.point_follow_status=dict(status='위치유지 지점 접근 중' if kind=='hold' else '위치추종 중',distance_m=distance,heading_error_deg=error,
                    tracking='point',control='GPS 고정 직선 추종',maneuver='전진/방향 보정',
                    cross_track_error_m=cross_track,course_correction_deg=math.degrees(state['course_correction_rad']),
                    control_target_m=control_goal.tolist(),controller='gps_line_course_v1')
                self.send(command,now);return
            self.buoy_vision.context=self.vision_nav()
            scan_pose=self.state.pose_at(self.scan['stamp']) if self.scan is not None else None
            corridor=None
            if self.route and len(self.route['points'])>=3:
                corridor=self.buoy_vision.route(state,local_xy(*self.route['points'][1],origin),
                    local_xy(*self.route['points'][2],origin),self.scan,scan_pose)
            self.corridor_scan=None
            if corridor is not None:
                center,fences=corridor
                try:
                    control_goal,cross_track,lookahead=path_follow_target(state,center,[*self.map_layers.hits,*fences.tolist()],config)
                    self.follow_track=[latlon(*p,origin) for p in center]
                    self.corridor_scan=self.buoy_vision.corridor.fenced_scan(self.scan,scan_pose,config,fences) if scan_pose is not None else None
                    track_status=dict(cross_track_error_m=cross_track,lookahead_m=lookahead,tracking='lidar_corridor')
                except ValueError as exc:
                    self.recover_follow(state,goal,now,str(exc));return
            elif getattr(self,'follow_path',False):
                frame=('virtual' if virtual else 'gps',tuple(origin))
                expected=(frame,getattr(self,'display_revision',0),tuple(goal))
                result=getattr(self,'display_result',None)
                try:
                    if (self.scan is None or now-self.scan_at>=config['scan_max_age_s'] or
                        getattr(self,'display_key',None)!=expected or not result or
                        not 0<=self.ros_now()-result.get('plan_stamp',0.)<=1.5):
                        raise ValueError('라이다·목표 경로 갱신 대기')
                    control_goal,cross_track,lookahead=path_follow_target(state,result['smooth'],self.map_layers.hits,config)
                    self.follow_track=[latlon(*p,origin) for p in result['smooth']]
                    track_status={'cross_track_error_m':cross_track,'lookahead_m':lookahead,'tracking':'path'}
                except ValueError as exc:
                    self.follow_track=[];self.follow_result=None
                    self.recover_follow(state,goal,now,str(exc));return
            # Keep following the checked curve through the final approach;
            # arrival itself is checked against the actual destination above.
            scan=(self.corridor_scan or self.scan) if self.scan is not None and now-self.scan_at<config['scan_max_age_s'] else None
            self.publish_navigation_scan(scan)
            command,error,maneuver=self.path_controller.command(state,control_goal,scan,config,self.follow_percent,now,
                (self.docking_search_id,self.goal,self.actuator.get('mode_name')),
                buoy_contact=self.buoy_course)
            self.follow_reverse=maneuver['maneuver_phase']=='reverse';self.follow_result=None;self.avoidance_info=None
            self.error='';self.command=command;self.command_at=now
            self.point_follow_status=dict(status=maneuver['maneuver'],distance_m=distance,heading_error_deg=error,
                control='GPS 회피 경로 추종',**maneuver,
                course_correction_deg=math.degrees(state['course_correction_rad']),
                control_target_m=np.asarray(control_goal).tolist(),controller='gps_path_maneuver_v3',**track_status)
            self.send(command,now)

        def tick(self):
            now = time.monotonic()
            self.refresh_drive_limits(now)
            self.refresh_arrival_radius(now)
            self.refresh_waypoint_timeouts(now)
            self.manage_docking_slam(now)
            self.buoy_vision.manage(self.vision_nav())
            if now-self.last_disk > 5:
                self.last_disk = now
                try: self.manage_bag(now)
                except OSError as exc: self.bag_status = '기록 재시도: '+str(exc)
            if self.fit_job and self.fit_job.done():
                try:
                    data = self.fit_job.result()
                    if self.fit_job_generation == self.fit_generation and self.learning:
                        self.model = ResponseModel(data); self.store.save_model(data)
                        self.save_selection()
                        self.fit_status = '분리 검증 통과' if data['qualified'] else ' / '.join(data['reasons'])
                except (ValueError, np.linalg.LinAlgError) as exc: self.fit_status = str(exc)
                finally: self.fit_job = None
            if self.learning and self.fit_job is None and now-self.fit_at > config['fit_interval_s']:
                self.fit_at = now; self.fit_job_generation = self.fit_generation
                training_rows=self.store.training_rows()
                self.fit_job = self.fit_pool.submit(fit_model,training_rows,config,
                    deepcopy(self.store.model_display(self.model.data)) if self.model else None)
            if self.plan_job and self.plan_job.done():
                try:
                    result = self.plan_job.result()
                    if self.plan_job_generation == self.plan_generation and self.execute and now-self.plan_started <= config['planner_deadline_s']:
                        self.plan_result = result; self.command = result['command']; self.command_at = now; self.error = ''
                        if getattr(self,'test_course',None) and self.test_course['running'] and self.actuator.get('mode_name')=='AUTO':
                            self.test_comparison.plan(result['plan_stamp'],result['path'],result['dt_s'])
                            if self.test_comparison.last_plan==result['plan_stamp']:
                                self.record_test_event('model_test_plan',test_id=self.test_course['id'],stamp=result['plan_stamp'],path=result['path'],dt_s=result['dt_s'])
                    elif now-self.plan_started > config['planner_deadline_s']: self.error = '경로 계산 지연 · 회피 재탐색'
                except (ValueError, RuntimeError, np.linalg.LinAlgError) as exc:
                    self.command_at = 0.; self.error = str(exc)
                finally: self.plan_job = None
            if self.tick_point_restart(now):return
            if self.tick_stop_brake(now):return
            if getattr(self, 'stopping', False):
                self.send([0., 0.], now, disarm=True)
                if now-self.command_at > 1: self.stopping = False
            if not self.execute:
                self.waypoint_timeouts.reset();return
            if self.tick_waypoint_timeout(now):return
            if self.tick_priority_route(now):return
            if getattr(self,'direct_follow',False):
                self.tick_point_follow(now);return
            if waypoint_kind(self.route,self.goal) in ('point','hold','dock','orbit'):
                self.follow_position_source='gps';self.follow_percent=self.planner.cap*100
                self.tick_point_follow(now);return
            state = self.state.snapshot(now)
            scan_pose = self.state.pose_at(self.scan['stamp']) if self.scan is not None else None
            ready = (state is not None and self.ready_since is not None and now-self.ready_since >= 2 and
                     self.scan is not None and now-self.scan_at < config['scan_max_age_s'] and
                     now-self.actuator_at < .3 and applied_command(self.actuator, config) is not None and self.disk_ok and
                     state['position_sigma_m'] <= config.get('navigation_position_sigma_limit_m',3.) and scan_pose is not None)
            if not ready:
                if getattr(self,'test_comparison',None):self.test_comparison.plans.clear()
                self.publish_navigation_scan(None)
                self.command_at = 0.; self.error = 'GPS·IMU·라이다·추진 상태 수신 대기'
                self.send([0., 0.], now); return
            goal = local_xy(*self.route['points'][self.goal], self.state.origin)
            points=self.route['points']
            arrival=config['arrival_radius_m']
            distance=float(np.linalg.norm(goal-state['position']))
            if distance>arrival+.3:self.arrival_threshold=arrival
            if distance <= getattr(self,'arrival_threshold',arrival):
                self.goal=(self.goal+1)%len(points);self.arrivals+=1
                goal=local_xy(*points[self.goal],self.state.origin)
                # Overlapping arrival circles must not alternate goals while
                # the boat is stationary. Require 1 m progress into an already
                # occupied circle, or a fresh entry after leaving it.
                next_distance=float(np.linalg.norm(goal-state['position']))
                self.arrival_threshold=(min(arrival,max(.3,next_distance-1.))
                                        if next_distance<=arrival else arrival)
                self.plan_generation+=1
                # Keep only the still-fresh, obstacle-checked command during
                # replanning; changing a waypoint is not a stop request.
            if waypoint_kind(self.route,self.goal) in ('point','hold','dock'):
                self.follow_position_source='gps';self.follow_percent=self.planner.cap*100
                self.tick_point_follow(now);return
            self.publish_navigation_scan(self.scan)
            state['goal']=goal.tolist()
            state['previous_command']=list(getattr(self,'last_sent_command',[0.,0.]))
            if self.plan_job is None and now-self.plan_at >= self.model.dt:
                self.plan_at = now; self.plan_started = now; self.plan_job_generation = self.plan_generation
                self.plan_job = self.plan_pool.submit(self.planner.plan, state, dict(self.scan, pose=scan_pose), list(self.commands), self.ros_now())
            command = self.command if now-self.command_at < .4 else [0., 0.]
            if getattr(self,'recovery_requested',False) or np.max(np.abs(command))<1e-9:
                try:
                    command,_=recovery_command(state,goal,self.scan,config,self.planner.cap*100,
                        getattr(self,'last_sent_command',[0.,0.]))
                    self.command=command;self.command_at=now
                    self.recovery_requested=False
                except ValueError as exc:self.error=str(exc);command=[0.,0.]
            command=self.anticipate_command(command,state,goal,self.planner.cap*100,now,self.model)
            if getattr(self,'avoidance_info',None) and self.avoidance_info.get('intervened'):
                self.command=command;self.command_at=now
            self.send(command, now)

        def display_layers(self, now, real_state):
            pose=real_state;origin=self.state.origin;source='gps'
            fix=self.virtual_fix
            if fix and 0<=now-fix['received']<=config['gps_max_age_s']:
                heading=self.state.heading(now)
                if heading is not None:
                    source='virtual';origin=self.follow_origin or fix['point']
                    pose={'position':local_xy(*fix['point'],origin).tolist(),
                          'yaw':math.radians(90-heading['heading_deg']),
                          'velocity':real_state.get('velocity') if real_state else None}
            result={'map_origin':origin,'display_guidance':[],'display_guidance_raw':[],
                    'feasible_path':[],'feasible_status':'실행 가능 궤적 · 위치·IMU 수신 대기',
                    'display_planning':None,'preview':[],'preview_status':'위치·IMU 수신 대기'}
            if pose is None or pose.get('yaw') is None or origin is None:return result
            if self.execute and self.gps_docking is not None:
                docking=self.gps_docking
                start=docking.targets[docking.index-1] if docking.index else docking.anchor
                line=[latlon(*start,docking.origin),latlon(*docking.targets[docking.index],docking.origin)]
                result.update(display_guidance=line,display_guidance_raw=line,
                    feasible_status='GPS 도킹 · IMU 방향 추종',display_planning={'status':'GPS 도킹 · A/B 좌표 고정'})
                return result
            frame=(source,tuple(origin))
            jumped=(source=='virtual' and self.display_position is not None and
                    np.linalg.norm(np.asarray(pose['position'])-self.display_position)>2.)
            if self.display_frame!=frame or jumped:
                if self.display_frame is not None:self.map_layers=LidarLayers()
                self.display_revision+=1
                self.display_frame=frame;self.display_result=None;self.display_key=None
            self.display_position=np.asarray(pose['position']).copy()
            scan_fresh=self.scan is not None and now-self.scan_at<config['scan_max_age_s']
            if scan_fresh:
                # Never place a timestamped scan at the newer display pose.
                scan_pose=self.state.pose_at(self.scan['stamp']) if source=='gps' else dict(pose)
                if scan_pose is not None:self.map_layers.update(self.scan,scan_pose,config)
            if self.display_job is not None and self.display_job.done():
                try:
                    value=self.display_job.result()
                    if self.display_job_key==self.display_key:self.display_result=value
                except (ValueError,RuntimeError) as exc:self.display_result={'raw':[],'smooth':[],'status':str(exc)}
                self.display_job=None
            if self.execute and waypoint_kind(self.route,self.goal)=='orbit':
                line=list(self.follow_track)
                result.update(display_guidance=line,display_guidance_raw=line,
                    feasible_status='선회 부표 선택 주행',display_planning={'status':'GPS·IMU / 실측 부표 중심'})
                return result
            if self.route and self.route['points']:
                goal=local_xy(*self.route['points'][min(self.goal,len(self.route['points'])-1)],origin)
                if waypoint_kind(self.route,self.goal) in ('point','hold','dock'):
                    self.display_key=None;self.display_result=None
                    anchor=self.waypoint_line.anchor if self.execute and self.waypoint_line.anchor is not None else pose['position']
                    line=[latlon(*anchor,origin),latlon(*goal,origin)]
                    result.update(display_guidance=line,display_guidance_raw=line,
                        feasible_status='위치추종 · GPS 목표 직접 추종',
                        display_planning={'status':'위치추종 · GPS 목표 직접 추종'})
                    return result
                key=(frame,self.display_revision,tuple(goal))
                if key!=self.display_key:self.display_key=key;self.display_result=None
                selected=self.validation_model or self.model
                previous=self.follow_previous(origin)
                line_key=(self.docking_search_id,self.goal,tuple(goal),tuple(previous) if previous is not None else None)
                rejoin,_=self.waypoint_line.target(pose,goal,line_key,previous)
                path_config=dict(config,gps_reference_leg_m=[self.waypoint_line.anchor.tolist(),goal.tolist()],
                                 gps_reference_target_m=rejoin.tolist())
                interval=.25 if selected and pose.get('velocity') is not None else .75
                if scan_fresh and scan_pose is not None and self.display_job is None and now-self.display_at>=interval:
                    self.display_at=now;self.display_job_key=key
                    self.display_job=self.display_pool.submit(self.feasible_overlay.plan,deepcopy(pose),goal.tolist(),
                        list(self.map_layers.hits),dict(self.scan,pose=deepcopy(scan_pose)),list(self.commands),path_config,
                        deepcopy(selected.data) if selected else None,getattr(self,'follow_percent',config['percent']),self.ros_now())
                if self.display_result and scan_fresh:
                    result['display_guidance_raw']=[latlon(*p,origin) for p in self.display_result['raw']]
                    result['display_guidance']=[latlon(*p,origin) for p in self.display_result['smooth']]
                    result['display_planning']={k:v for k,v in self.display_result.items() if k not in ('raw','smooth','feasible')}
                    result['feasible_status']=self.display_result.get('feasible_status','실행 가능 궤적 계산 대기')
                    recent=0<=self.ros_now()-self.display_result.get('plan_stamp',0.)<=1.25
                    angle=abs(math.atan2(math.sin(pose['yaw']-self.display_result.get('initial_yaw',pose['yaw'])),math.cos(pose['yaw']-self.display_result.get('initial_yaw',pose['yaw']))))
                    moved=np.linalg.norm(np.asarray(pose['position'])-self.display_result.get('initial_position',pose['position']))
                    if recent and angle<=math.radians(15) and moved<=.75:
                        result['feasible_path']=[latlon(*p,origin) for p in self.display_result.get('feasible',[])]
                    elif self.display_result.get('feasible'):
                        result['feasible_status']='현재 자세·속도로 궤적 재계산 중'
                elif not scan_fresh:result['display_planning']={'status':'라이다 수신 대기'}
            try:
                selected=self.validation_model or self.model
                path,status=measured_rollout(pose,selected.for_navigation(config) if selected else None,list(self.commands),self.ros_now())
                result['preview']=[latlon(*p,origin) for p in path];result['preview_status']=status
                result['preview_model_id']=selected.data['id'] if selected else None
            except ValueError as exc:result['preview_status']=str(exc)
            return result

        def report(self):
            now=time.monotonic()
            if self.disk_ok and now-self.store.last_commit>=1.:
                try:self.store.commit()
                except (OSError,sqlite3.Error) as exc:
                    self.disk_ok=False;self.error='GPS 기록 저장 오류: '+str(exc)
            real_state=self.state.snapshot(now) or self.state.display_pose(now)
            display=self.display_layers(now,real_state)
            if getattr(self,'test_course',None) and self.test_course['running'] and time.monotonic()-getattr(self,'map_save_at',0.)>5:
                try:self.save_test_map();self.map_save_at=time.monotonic()
                except OSError as exc:self.error='관측 지도 저장 오류: '+str(exc)
            state = self.state.snapshot(time.monotonic()) or self.state.display_pose(time.monotonic())
            if state:
                state = {k: v for k, v in state.items() if k != 'received'}
                state['lat'], state['lon'] = latlon(*state['position'], self.state.origin)
                state['heading_deg'] = (90-math.degrees(state['yaw'])) % 360 if state['yaw'] is not None else None
            paths = []
            tracks = []; guidance = []; raw_guidance=[]; planning = None
            active_plan=self.plan_result;plan_origin=self.state.origin
            if self.execute and self.direct_follow:
                active_plan=self.follow_result if self.follow_result and 0<=self.ros_now()-self.follow_result['plan_stamp']<.4 else None
                plan_origin=self.follow_origin if self.follow_position_source=='virtual' else self.state.origin
            if active_plan and plan_origin:
                paths = [latlon(*xy, plan_origin) for xy in active_plan['path']]
                guidance = [latlon(*xy,plan_origin) for xy in active_plan.get('route',[])]
                raw_guidance = [latlon(*xy,plan_origin) for xy in active_plan.get('raw_route',[])]
                planning = {k:active_plan.get(k) for k in ('planning_ms','recovery','scan_range_m','obstacle_discs')}
                for track in (active_plan.get('obstacles',[]) if self.execute and time.monotonic()-self.command_at<.5 else []):
                    xy=np.asarray(track['position']); velocity=np.asarray(track['velocity'])
                    tracks.append(dict(track, path=[latlon(*(xy+velocity*t),plan_origin) for t in (0.,1.,2.,3.)]))
            preview = display['preview']
            model = self.store.model_display(self.model.data) if self.model else None
            value = {'state': state, 'state_reason': self.state.reason, 'mode': self.actuator.get('mode_name'),
                **display,
                'learning': self.learning, 'frozen': self.frozen, 'fit_running': self.fit_job is not None,
                'learning_branch':self.lineage,
                'model_test':dict(self.test_course,metrics=self.test_comparison.summary()) if self.test_course else None,
                'regions':getattr(self,'regions',[]),
                'fit_status': self.fit_status, 'model': model, 'execute': self.execute,
                'navigation_mode':'point_follow' if getattr(self,'direct_follow',False) else 'model',
                'point_follow_tracking':'path' if getattr(self,'follow_path',False) else 'point',
                'point_follow_path':getattr(self,'follow_track',[]) if self.execute and getattr(self,'follow_path',False) else [],
                'point_follow':getattr(self,'point_follow_status',None),'planner_compute':gpu_status(),
                'anticipation':getattr(self,'avoidance_info',None),
                'marine_lidar':self.marine_lidar.status(),
                'point_follow_limit_percent':getattr(self,'follow_percent',None),
                'arrival_radius_m':config['arrival_radius_m'],
                'drive_limits':dict(self.drive_limits),
                'buoy_course':self.buoy_course,
                'point_follow_position_source':getattr(self,'follow_position_source','gps'),
                'goal': self.goal, 'arrivals': self.arrivals, 'route': self.route, 'prediction': paths, 'preview': preview,
                'docking_search_id':self.docking_search_id,
                'docking_start':self.docking_start_status(),
                'waypoint_restart_schema':1,'run_start_index':self.run_start_index,
                'live_route_schema':1,'route_update':getattr(self,'route_update',None),
                'waypoint_skip_schema':1,'waypoint_skip':getattr(self,'waypoint_skip_status',None),
                'waypoint_timeout_schema':1,'waypoint_timeouts':self.waypoint_timeouts.snapshot(now),
                'waypoint_timeout_last':self.waypoint_timeout_last,
                'waypoint_restart':dict(state='neutralizing',start_index=self.restart_request['prepared']['start_index']) if self.restart_request else None,
                'docking_run_route':getattr(self,'docking_run_route',None),
                'waypoint_schema':1,'waypoint_output_schema':1,'registered_gps_docking_schema':1,'waypoint_kinds':list(KINDS),'vision':self.buoy_vision.last_status,
                'orbit_abort_schema':1,'orbit_abort':self.orbit_abort_status,
                'buoy_orbit_schema':1,'buoy_orbit':self.orbit_status if self.execute and waypoint_kind(self.route,self.goal)=='orbit' else None,
                'buoy_route':self.buoy_route_display(),
                'waypoint_mission':self.waypoint_progress.snapshot(self.route,self.goal,now) if self.route else None,
                'docking_phase':self.docking_phase(),'gps_docking':self.gps_docking_report(now),
                'docking_observation':self.docking_observation,
                'priority_route':{'state':self.priority_stage,'complete':self.docking_complete,'aborted':self.docking_aborted,
                    'slam_auto_off':self.docking_slam_auto_off,'slam_transition_pending':self.docking_slam_target is not None,
                    'slam_transition_error':self.docking_slam_error,
                    'gps_skipped_index':self.priority_mission.skipped_index if self.priority_mission else None,
                    'gps_resume_index':self.priority_mission.resume_index if self.priority_mission else None,
                    'docking_goal':self.tank_status_data.get('goal'),'reason':self.tank_status_data.get('reason','')},
                'obstacle_tracks':tracks, 'guidance':guidance, 'planning':planning,
                'guidance_raw':raw_guidance,'map_layers':self.map_layers.snapshot(self.ros_now()),
                'error': self.error, 'ack': self.ack, 'bag': self.bag_status, 'disk_ok': self.disk_ok,
                'segment': self.store.segment, 'origin': self.state.origin, 'history': list(self.chart),
                'trail': list(self.trail)[::max(1, len(self.trail)//2500)] if self.gps_recording.session() else [],
                'validation_model':self.store.model_display(self.validation_model.data) if self.validation_model else None,
                'recording_summary':self.store.recording_summary(),
                'native_frames': len(self.rows), 'valid_frames': sum(r['valid'] for r in self.rows),
                'storage': str(root), 'signature': fingerprint(config), 'stamp': self.ros_now()}
            self.status_pub.publish(String(data=json.dumps(value, allow_nan=False)))

    node = Outdoor()
    try:
        while rclpy.ok() and not exits.requested: rclpy.spin_once(node, timeout_sec=.02)
    finally:
        node.stop_trial()
        node.buoy_vision.close()
        node.marine_lidar.close()
        if node.docking_audit is not None:node.docking_audit.close()
        if getattr(node, 'stopping', False):
            until = time.monotonic()+.8
            while any(not f.done() for f in node.pending) and time.monotonic() < until: rclpy.spin_once(node, timeout_sec=.01)
            neutralize(node)
        if node.bag and node.bag.poll() is None:
            os.killpg(node.bag.pid, signal.SIGINT)
            try: node.bag.wait(timeout=20)
            except subprocess.TimeoutExpired: node.bag.terminate()
        node.fit_generation += 1
        node.fit_pool.shutdown(wait=True, cancel_futures=True); node.plan_pool.shutdown(wait=True, cancel_futures=True)
        node.display_pool.shutdown(wait=True,cancel_futures=True)
        node.store.close(); node.destroy_node(); rclpy.shutdown(); exits.restore(); lock.close()


if __name__ == '__main__': main()
