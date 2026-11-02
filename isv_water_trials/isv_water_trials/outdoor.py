"""Outdoor RC recorder/learner and explicitly started GPS A/B verification."""
import argparse
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
import time
import uuid
import numpy as np
from .outdoor_model import ResponseModel, LiveComparison, applied_command, fit_model, fingerprint, latlon, local_xy, logical_pwm
from .outdoor_state import OutdoorState
from .outdoor_store import OutdoorStore
from .outdoor_planner import OutdoorPlanner
from .outdoor_config import load_config
from .outdoor_layers import LidarLayers
from .tank_storage import atomic_json


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
            self.rows = deque(self.store.rows(), maxlen=30000); self.commands = deque(maxlen=600)
            self.chart = deque(maxlen=300); self.trail = deque(maxlen=20000)
            try: trail_after = json.loads((root/'display.json').read_text())['trail_after_t']
            except (FileNotFoundError, ValueError, KeyError): trail_after = 0.
            self.trail.extend(self.store.trail(trail_after))
            self.last_fix_t = None
            self.comparison = LiveComparison(); self.model = None; self.validation_model = None; self.frozen = False; self.learning = True
            self.lineage=None; self.baseline_model=None
            self.fit_job = None; self.fit_generation = 0; self.fit_at = time.monotonic()
            self.fit_pool = ProcessPoolExecutor(max_workers=1, mp_context=multiprocessing.get_context('spawn'))
            self.plan_pool = ThreadPoolExecutor(max_workers=1)
            self.plan_job = None; self.plan_generation = 0; self.plan_started = 0.; self.plan_at = 0.
            self.execute = False; self.planner = None; self.plan_result = None; self.goal = 0; self.direction = 1; self.arrivals = 0
            self.actuator = {}; self.actuator_at = 0.; self.scan = None; self.scan_at = 0.
            self.command = [0., 0.]; self.command_at = 0.; self.pending = []; self.pending_at = 0.
            self.error = ''; self.fit_status = 'RC 기록 수집 중'; self.ack = None; self.ready_since = None
            self.last_frame_t = None; self.last_valid = False; self.route = None; self.bag = None; self.bag_at = 0.
            self.bag_log = None; self.bag_status = '대기'; self.last_disk = 0.; self.disk_ok = True
            try:
                current,validation,self.learning=self.store.live_selection()
                self.model=ResponseModel(current) if current else None
                self.validation_model=ResponseModel(validation) if validation else None
            except (ValueError, KeyError, OSError) as exc:self.error='저장 설정 확인: '+str(exc)
            try: self.route = self.validate_route(json.loads((root/'route.json').read_text(encoding='utf-8')))
            except (FileNotFoundError, ValueError, KeyError): pass
            self.status_pub = self.create_publisher(String, '/outdoor/status', 5)
            self.command_pub = self.create_publisher(String, '/outdoor/command', 10)
            self.arm = self.create_client(SetBool, '/actuators/arm')
            self.throttles = [self.create_client(ThrottlePulseWidth, '/actuators/throttle_'+side+'/set_pulse_width') for side in ('left', 'right')]
            for topic, kind, handler in [('/gps/fix', NavSatFix, self.on_fix),
                    ('/gps/fix_velocity', TwistWithCovarianceStamped, self.on_velocity),
                    ('/vectornav/raw/common', CommonGroup, self.on_imu),
                    ('/scan', LaserScan, self.on_scan), ('/actuators/status', String, self.on_actuator)]:
                self.create_subscription(kind, topic, handler, qos_profile_sensor_data)
            self.create_subscription(String, '/outdoor/request', self.request, 10)
            self.create_subscription(Empty, '/water_trial/stop', lambda _: self.stop_trial(), 10)
            self.create_timer(.1, self.tick); self.create_timer(.5, self.report)
            self.store.event('session_started', learning=self.learning, model=self.model.data['id'] if self.model else None)

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
                if self.state.fixes and stamp != self.last_fix_t:
                    if self.last_fix_t is not None and (stamp-self.last_fix_t > 1.6 or stamp < self.last_fix_t): self.store.new_segment('gps_gap')
                    self.last_fix_t = stamp
                    self.trail.append([msg.latitude, msg.longitude, self.store.segment, stamp])
                    if self.disk_ok: self.store.position(stamp, msg.latitude, msg.longitude)
            except ValueError as exc: self.error = str(exc)
            except (OSError, sqlite3.Error) as exc: self.disk_ok = False; self.error = '위치 기록 저장 오류: '+str(exc)

        def on_imu(self, msg):
            if msg.group_fields & 0x0008 and msg.group_fields & 0x0020:
                try: self.state.imu(self.stamp(msg, config['imu_max_age_s']), time.monotonic(), msg.yawpitchroll.x, msg.angularrate.z)
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
                         'range_max': msg.range_max, 'stamp': stamp}
            self.scan_at = time.monotonic()

        def on_velocity(self, msg):
            now = time.monotonic()
            try:
                frame = self.state.velocity(self.stamp(msg), now, msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.covariance)
            except ValueError as exc: self.error = str(exc); return
            if frame is None: self.last_valid = False; return
            if self.last_frame_t is not None and (frame['t']-self.last_frame_t > 1.6 or frame['t'] <= self.last_frame_t):
                self.store.new_segment('sensor_gap'); self.comparison = LiveComparison(); self.ready_since = None
            self.last_frame_t = frame['t']
            valid = now-self.actuator_at < .3 and applied_command(self.actuator, config) is not None
            row = {'t': frame['t'], 'velocity': frame['velocity'], 'position': frame['position'], 'yaw': frame['yaw'],
                   'latlon': latlon(*frame['position'], self.state.origin),
                   'mode': self.actuator.get('mode_name'), 'valid': valid and self.learning,
                   'learning': self.learning, 'actuator': self.actuator,
                   'branch_id':self.lineage['id'] if self.lineage else None,
                   'commands': [c for c in self.commands if c[0] >= frame['t']-3],
                   'gps_sigma_m': frame['position_sigma_m'], 'gps_velocity_sigma_m_s': frame['velocity_sigma_m_s'],
                   'imu_skew_s': frame['imu_skew_s']}
            if self.disk_ok:
                try: row = self.store.add(row); self.rows.append(row)
                except (OSError, sqlite3.Error) as exc: self.disk_ok = False; self.error = '기록 저장 오류: '+str(exc)
            comparison = self.comparison.update(self.model, frame['t'], frame['velocity'], list(self.commands)) if self.commands else None
            applied = applied_command(self.actuator, config)
            self.chart.append({'t': frame['t'], 'command': applied, 'measured': frame['velocity'],
                               'predicted': comparison['predicted'] if comparison else None,
                               'horizon_s': comparison['horizon_s'] if comparison else None})
            if self.ready_since is None: self.ready_since = now
            self.last_valid = valid

        def validate_route(self, data):
            points = data['points']
            if not 2 <= len(points) <= 100 or any(len(p) != 2 or not all(math.isfinite(v) for v in p) or not -85 < p[0] < 85 or not -180 <= p[1] <= 180 for p in points):
                raise ValueError('위도·경도로 2~100개의 지점을 지정하세요')
            if any(not 5 <= np.linalg.norm(local_xy(*b, a)) <= 200 for a, b in zip(points, points[1:])):
                raise ValueError('이웃한 지점의 거리는 5~200m로 지정하세요')
            return {'points': points}

        def load_model(self, identity):
            if len(identity) != 16 or any(c not in '0123456789abcdef' for c in identity): raise ValueError('모델 ID 오류')
            # Compatibility for older CLI scripts: choose validation only.
            self.validation_model=ResponseModel(self.store.load_compatible_model(identity))

        def save_selection(self):
            self.store.save_live_selection(self.model.data if self.model else None,
                self.validation_model.data if self.validation_model else None,self.learning)

        def stop_trial(self):
            was_running = self.execute
            self.execute = False; self.command = [0., 0.]; self.command_at = time.monotonic()
            self.plan_generation += 1; self.plan_result = None
            if was_running: self.stopping = True

        def request(self, msg):
            data = {}; ok = False
            try:
                data = json.loads(msg.data); action = data['action']
                if action in ('pause', 'resume'):
                    self.fit_generation += 1; self.learning = action == 'resume'
                    self.store.new_segment(action); self.save_selection()
                    self.fit_status='학습 일시정지 · 원본 기록 유지' if not self.learning else '누적 RC 기록 비교 중'
                elif action in ('apply_model','freeze'):
                    if self.model is None: raise ValueError('먼저 RC 기록으로 모델을 학습하세요')
                    if not self.model.data.get('qualified'):raise ValueError('검증 기준을 통과한 학습 모델이 필요합니다')
                    if self.execute:raise ValueError('주행 정지 후 새 검증 모델을 적용하세요')
                    self.store.finalize_model(self.model.data)
                    self.validation_model=ResponseModel(deepcopy(self.model.data))
                    self.save_selection()
                elif action == 'load':
                    self.stop_trial(); self.load_model(data['id']); self.save_selection()
                elif action == 'route':
                    self.stop_trial(); self.route = self.validate_route(data); self.goal = 0; self.direction = 1; self.arrivals = 0
                    atomic_json(root/'route.json', self.route)
                elif action == 'start':
                    if not self.validation_model: raise ValueError('현재 모델을 검증에 적용한 뒤 시작하세요')
                    if not self.route: raise ValueError('GPS 지도에서 A/B 지점을 저장하세요')
                    conflicts = {'rc_auto_mode_test','tank_round_trip','water_trial_runner','water_trial_avoid_preview'} & set(self.get_node_names())
                    if conflicts: raise ValueError('기존 추진 노드를 먼저 종료하세요: '+', '.join(sorted(conflicts)))
                    self.planner = OutdoorPlanner(self.validation_model.data, config, float(data.get('percent', config['percent'])))
                    self.execute = True; self.stopping = False; self.plan_generation += 1
                elif action == 'stop': self.stop_trial()
                elif action == 'clear_trail':
                    self.trail.clear()
                    atomic_json(root/'display.json', {'trail_after_t': self.ros_now()})
                else: raise ValueError('알 수 없는 요청')
                self.store.event(action, model=self.model.data['id'] if self.model else None); self.error = ''; ok = True
            except (ValueError, TypeError, KeyError, OSError) as exc: self.error = str(exc)
            self.ack = {'id': data.get('request_id'), 'ok': ok, 'message': self.error if not ok else '적용됨'}

        def manage_bag(self, now):
            if args.no_bag: self.bag_status = 'fixture: bag disabled'; return
            free = shutil.disk_usage(root).free
            self.disk_ok = free > 256*1024**2
            if self.bag and self.bag.poll() is None:
                self.bag_status = 'recording'
                if free < 1024**3: os.killpg(self.bag.pid, signal.SIGINT); self.bag_status = '원본 기록: 디스크 여유 대기'
                return
            if now-self.bag_at < 10 or free < 2*1024**3: return
            self.bag_at = now
            folder = root/'bags'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ'); folder.parent.mkdir(exist_ok=True)
            if self.bag_log: self.bag_log.close()
            self.bag_log = (root/'bag.log').open('a', encoding='utf-8')
            topics = ['/gps/fix', '/gps/fix_velocity', '/vectornav/imu', '/vectornav/raw/common', '/scan',
                      '/velodyne_points', '/actuators/status', '/outdoor/command', '/tf', '/tf_static']
            self.bag = subprocess.Popen(['ros2', 'bag', 'record', '-o', str(folder), '--max-bag-duration', '60', *topics],
                                        stdin=subprocess.DEVNULL, stdout=self.bag_log, stderr=subprocess.STDOUT, start_new_session=True)

        def send(self, command, now, disarm=False):
            if self.pending and not all(f.done() for f in self.pending):
                if now-self.pending_at <= .6: return
                self.error = '추진 응답 재연결 대기'
                for future in self.pending:
                    if not future.done(): future.cancel()
                self.pending = []; self.command_at = 0.
                command = [0., 0.]
            if not all(c.service_is_ready() for c in [self.arm, *self.throttles]): return
            self.pending_at = now
            self.pending = [self.arm.call_async(SetBool.Request(data=not disarm))]
            self.pending.extend(c.call_async(ThrottlePulseWidth.Request(pulse_width=p)) for c, p in zip(self.throttles, logical_pwm(command)))
            self.command_pub.publish(String(data=json.dumps({'t': self.ros_now(), 'command': command, 'pwm': logical_pwm(command), 'disarm': disarm})))

        def tick(self):
            now = time.monotonic()
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
                    elif now-self.plan_started > config['planner_deadline_s']: self.error = '경로 계산 시간 초과 · 중립 대기'
                except (ValueError, RuntimeError, np.linalg.LinAlgError) as exc:
                    self.command_at = 0.; self.error = str(exc)
                finally: self.plan_job = None
            if getattr(self, 'stopping', False):
                self.send([0., 0.], now, disarm=True)
                if now-self.command_at > 1: self.stopping = False
            if not self.execute: return
            state = self.state.snapshot(now)
            scan_pose = self.state.pose_at(self.scan['stamp']) if self.scan is not None else None
            ready = (state is not None and self.ready_since is not None and now-self.ready_since >= 2 and
                     self.scan is not None and now-self.scan_at < config['scan_max_age_s'] and
                     now-self.actuator_at < .3 and applied_command(self.actuator, config) is not None and self.disk_ok and
                     state['position_sigma_m'] <= config['arrival_radius_m'] and scan_pose is not None)
            if not ready:
                self.command_at = 0.; self.error = 'GPS·IMU·라이다·추진 상태 수신 대기'
                self.send([0., 0.], now); return
            goal = local_xy(*self.route['points'][self.goal], self.state.origin)
            if np.linalg.norm(goal-state['position']) <= config['arrival_radius_m']:
                self.goal = (self.goal+1) % len(self.route['points']); self.arrivals += 1
                goal = local_xy(*self.route['points'][self.goal], self.state.origin)
                self.plan_generation += 1; self.command_at = 0.
            state['goal'] = goal.tolist()
            if self.plan_job is None and now-self.plan_at >= self.model.dt:
                self.plan_at = now; self.plan_started = now; self.plan_job_generation = self.plan_generation
                self.plan_job = self.plan_pool.submit(self.planner.plan, state, dict(self.scan, pose=scan_pose), list(self.commands), self.ros_now())
            command = self.command if now-self.command_at < .4 else [0., 0.]
            self.send(command, now)

        def report(self):
            if self.scan is not None and time.monotonic()-self.scan_at<config['scan_max_age_s']:
                pose=self.state.pose_at(self.scan['stamp'])
                if pose is not None:self.map_layers.update(self.scan,pose,config)
            state = self.state.snapshot(time.monotonic()) or self.state.display_pose(time.monotonic())
            if state:
                state = {k: v for k, v in state.items() if k != 'received'}
                state['lat'], state['lon'] = latlon(*state['position'], self.state.origin)
                state['heading_deg'] = (90-math.degrees(state['yaw'])) % 360 if state['yaw'] is not None else None
            paths = []
            tracks = []; guidance = []; raw_guidance=[]; planning = None
            if self.plan_result and self.state.origin:
                paths = [latlon(*xy, self.state.origin) for xy in self.plan_result['path']]
                guidance = [latlon(*xy,self.state.origin) for xy in self.plan_result.get('route',[])]
                raw_guidance = [latlon(*xy,self.state.origin) for xy in self.plan_result.get('raw_route',[])]
                planning = {k:self.plan_result.get(k) for k in ('planning_ms','recovery','scan_range_m','obstacle_discs')}
                for track in (self.plan_result.get('obstacles',[]) if self.execute and time.monotonic()-self.command_at<.5 else []):
                    xy=np.asarray(track['position']); velocity=np.asarray(track['velocity'])
                    tracks.append(dict(track, path=[latlon(*(xy+velocity*t),self.state.origin) for t in (0.,1.,2.,3.)]))
            preview = []
            if state and state.get('velocity') is not None and self.model and self.commands:
                try:
                    history = np.array(self.commands); times = self.ros_now()+np.arange(-self.model.delay, 0)*self.model.dt
                    ids = np.searchsorted(history[:, 0], times, side='right')-1
                    if np.any(ids < 0): raise ValueError('history')
                    if len(ids) and np.any(times-history[ids, 0] > .35): raise ValueError('stale history')
                    predicted = self.model.predict(state['velocity'], np.tile(history[-1, 1:], (round(3/self.model.dt), 1)), history[ids, 1:])
                    xy = np.array(state['position']); yaw = state['yaw']
                    for u, v, r in predicted:
                        yaw += r*self.model.dt
                        xy += self.model.dt*np.array([math.cos(yaw)*u-math.sin(yaw)*v, math.sin(yaw)*u+math.cos(yaw)*v])
                        preview.append(latlon(*xy, self.state.origin))
                except ValueError: pass
            model = self.store.model_display(self.model.data) if self.model else None
            value = {'state': state, 'state_reason': self.state.reason, 'mode': self.actuator.get('mode_name'),
                'learning': self.learning, 'frozen': self.frozen, 'fit_running': self.fit_job is not None,
                'learning_branch':self.lineage,
                'fit_status': self.fit_status, 'model': model, 'execute': self.execute,
                'goal': self.goal, 'arrivals': self.arrivals, 'route': self.route, 'prediction': paths, 'preview': preview,
                'obstacle_tracks':tracks, 'guidance':guidance, 'planning':planning,
                'guidance_raw':raw_guidance,'map_layers':self.map_layers.snapshot(self.ros_now()),
                'error': self.error, 'ack': self.ack, 'bag': self.bag_status, 'disk_ok': self.disk_ok,
                'segment': self.store.segment, 'origin': self.state.origin, 'history': list(self.chart),
                'trail': list(self.trail)[::max(1, len(self.trail)//2500)],
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
        node.store.close(); node.destroy_node(); rclpy.shutdown(); exits.restore(); lock.close()


if __name__ == '__main__': main()
