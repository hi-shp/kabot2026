"""Continuous GPS-free A/B trial. Preview by default; --execute enables PWM."""
import argparse
from collections import deque
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import zlib

from .tank_core import Grid, guidance, projected_clearance, pwm, wrap, yaw
from .common import clean_json
from .tank_safety import select_command
from .tank_storage import Journal, atomic_json


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--route', default='~/water_trials/tank/route.json')
    parser.add_argument('--percent', type=float, default=20.)
    parser.add_argument('--clearance', type=float, default=.75, help='LiDAR-reference footprint clearance, metres')
    parser.add_argument('--arrival', type=float, default=.35)
    parser.add_argument('--dwell', type=float, default=2.)
    parser.add_argument('--forward-axis-deg', type=float, default=0.,
                        help='Boat forward in ROS velodyne frame; VLP16 cable aft: +X=0 (default), +Y=90')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--no-excitation', action='store_true', help='Only track A/B; disable open-water input schedule')
    parser.add_argument('--model', help='Optional qualified response_model.json for displayed predictions')
    parser.add_argument('--analysis-interval', type=float, default=60.)
    parser.add_argument('--output', default='~/water_trials/indoor')
    args = parser.parse_args(argv)
    if not 0 < args.percent <= 20 or not .75 <= args.clearance <= 3 or not .1 <= args.arrival <= 1 or not 0 <= args.dwell <= 60:
        parser.error('percent: 0..20, clearance: .75..3m, arrival: .1..1m, dwell: 0..60s')
    if not math.isfinite(args.forward_axis_deg): parser.error('Invalid forward axis')
    if not math.isfinite(args.analysis_interval) or args.analysis_interval<30: parser.error('analysis-interval >=30s')
    model = json.loads(Path(args.model).expanduser().read_text(encoding='utf-8')) if args.model else None
    forward = math.radians(args.forward_axis_deg or 0.)
    route = json.loads(Path(args.route).expanduser().read_text(encoding='utf-8'))
    points = [tuple(float(route[k][v]) for v in ('x','y')) for k in ('a','b')]
    if not all(math.isfinite(v) for point in points for v in point) or math.dist(*points)<1.:
        parser.error('A and B must be finite and at least 1 metre apart')
    if route.get('frame_id') != 'tank_map': parser.error('Route must use tank_map')
    import fcntl
    runtime_lock = open('/tmp/kabot_tank_trial_'+os.environ.get('ROS_DOMAIN_ID','0')+'.lock','w')
    try: fcntl.flock(runtime_lock,fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError: parser.error('A tank trial is already running in this ROS domain')
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from nav_msgs.msg import OccupancyGrid, Odometry
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from sensor_msgs.msg import LaserScan, Imu
    from std_msgs.msg import String
    from std_srvs.srv import SetBool
    from mechaship_interfaces.srv import ThrottlePulseWidth
    from .shutdown import ExitSignals, neutralize
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    exits = ExitSignals(ctrl_c_only=True); node = Node('tank_round_trip')
    latest = {}; grid = None; map_error = None; pending_map = None; map_signature = None
    def receive(key):
        def cb(msg): latest[key] = (time.monotonic(), msg)
        return cb
    def receive_map(msg):
        nonlocal pending_map,map_signature
        signature=(msg.header.frame_id,str(msg.info),zlib.crc32(msg.data.tobytes()))
        if signature!=map_signature:
            pending_map = msg;map_signature=signature
    map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid, '/map', receive_map, map_qos)
    for cls, topic, key in [(PoseWithCovarianceStamped,'/tank/pose','pose'), (Odometry,'/tank/odom','odom'),
                            (LaserScan,'/scan','scan'), (Imu,'/vectornav/imu','imu'), (String,'/actuators/status','mcu'),
                            (String,'/tank/session','session')]:
        node.create_subscription(cls, topic, receive(key), qos_profile_sensor_data)
    status_pub = node.create_publisher(String, '/tank/status', 10)
    prediction_pub = node.create_publisher(String, '/tank/prediction', 10)
    command_pub = node.create_publisher(String, '/water_trial/command', 10)
    clients = [node.create_client(ThrottlePulseWidth, '/actuators/throttle_'+side+'/set_pulse_width') for side in ('left','right')] if args.execute else []
    arm = node.create_client(SetBool, '/actuators/arm') if args.execute else None
    directory = Path(args.output).expanduser()/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    directory.mkdir(parents=True)
    if shutil.disk_usage(directory).free < 300*1024**2: raise RuntimeError('Less than 300 MiB free for trial logs')
    atomic_json(directory/'metadata.json', {'args':vars(args),'route':route,'mass_kg':25.,
        'length_m':1.1,'width_m':.5,'thruster_spacing_m':.32,'pose_reference':'velodyne',
        'notes':['LiDAR pose covariance is uncalibrated; validate drift against pool dimensions.',
                 'Percent is PWM command fraction, not measured thrust or speed.',
                 'Collision rollout is provisional; stopping response requires tank validation.']})
    log = Journal(directory)
    print(('EXECUTE' if args.execute else 'PREVIEW (no actuator services)')+' LOG='+str(directory),flush=True)
    started = time.monotonic(); goal = 0; legs = 0; dwell_until = 0.; path = []
    previous_state = None; previous_pose = None; settle_until = started+1.
    futures = []; arm_future = None; last_plan = 0.; need_arm = True
    acknowledgement = None; disk_check = 0.; disk_ok = True
    motion_history = deque(maxlen=12); chart_history = deque(maxlen=300)
    velocity_history = deque(maxlen=12); experiment_time = 0.; last_tick = started; io_error = None
    analysis = None; analysis_started = started; analysis_stream = (directory/'analysis.log').open('a',encoding='utf-8')
    analysis_env = dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
    try:
        while not exits.requested and rclpy.ok():
            tick = time.monotonic()+.1
            rclpy.spin_once(node,timeout_sec=.02)
            now = time.monotonic()
            if analysis and analysis.poll() is None and now-analysis_started>120:
                analysis.kill(); analysis.wait()
            if now-analysis_started>=args.analysis_interval and (analysis is None or analysis.poll() is not None) and not io_error:
                analysis = subprocess.Popen(['nice','-n','10',sys.executable,'-m','isv_water_trials.indoor_analyze',str(directory)],
                    stdout=analysis_stream,stderr=subprocess.STDOUT,env=analysis_env)
                analysis_started = now
            if pending_map is not None:
                try: grid = Grid(pending_map,args.clearance); map_error = None; path = []
                except (ValueError,TypeError) as exc: map_error = str(exc); grid = None
                pending_map = None
            if now-disk_check>5:
                disk_check = now; disk_ok = not io_error and shutil.disk_usage(directory).free > 100*1024**2
            reason = ''; state = 'WAITING'; output = (1500,1500); position = None; heading = None
            mcu = None
            velocity = None; odom_row = None; prediction_data = {}; phase = 'hold'
            if 'odom' in latest:
                o = latest['odom'][1]; p = o.pose.pose
                stamp = o.header.stamp.sec+o.header.stamp.nanosec/1e9
                odom_row = {'x':p.position.x,'y':p.position.y,'yaw_rad':yaw(p.orientation),
                            'stamp_s':stamp,'age_s':now-latest['odom'][0]}
                sample = (stamp,p.position.x,p.position.y,yaw(p.orientation)+forward)
                if all(math.isfinite(v) for v in sample):
                    if velocity_history and (stamp<velocity_history[-1][0] or stamp-velocity_history[-1][0]>.5): velocity_history.clear()
                    if not velocity_history or stamp>velocity_history[-1][0]: velocity_history.append(sample)
                    while velocity_history and stamp-velocity_history[0][0]>.6: velocity_history.popleft()
                    if len(velocity_history)>1 and stamp-velocity_history[0][0]>.15 and odom_row['age_s']<.4:
                        a,b = velocity_history[0],velocity_history[-1]; dt = b[0]-a[0]
                        vx,vy = (b[1]-a[1])/dt,(b[2]-a[2])/dt; co,si = math.cos(b[3]),math.sin(b[3])
                        velocity = (vx*co+vy*si,-vx*si+vy*co,wrap(b[3]-a[3])/dt)
            if 'mcu' in latest and now-latest['mcu'][0] < .7:
                try:
                    mcu = json.loads(latest['mcu'][1].data)
                    if not isinstance(mcu,dict): mcu = None
                except (ValueError,TypeError): pass
            try: session = json.loads(latest['session'][1].data) if 'session' in latest else {}
            except (ValueError,TypeError): session = {}
            if futures and all(f.done() for f in futures):
                acknowledgement = all(not f.cancelled() and f.exception() is None and f.result() and f.result().status for f in futures)
                need_arm = not acknowledgement; futures = []
            if futures and now-futures_started>.25:
                for f in futures: f.cancel()
                futures = []; need_arm = True; acknowledgement = False
            if arm_future is not None and arm_future.done():
                arm_future = None
            if arm_future is not None and now-arm_started>.3:
                arm_future.cancel(); arm_future = None; need_arm = True
            ready = all(k in latest and now-latest[k][0] < .5 for k in ('pose','odom','scan','imu'))
            # A publisher can repeat an old scan; receipt age alone is insufficient.
            if ready:
                for key in ('odom','scan','imu'):
                    h = latest[key][1].header
                    age = (node.get_clock().now().nanoseconds-(h.stamp.sec*10**9+h.stamp.nanosec))/1e9
                    if not -.1<=age<.6: ready = False
                if latest['scan'][1].header.frame_id != 'velodyne': ready = False
            if not disk_ok: reason = '기록 공간 부족'
            elif grid is None: reason = map_error or '지도 대기'
            elif not ready: reason = '위치·오도메트리·스캔 데이터 대기'
            elif args.execute and (not mcu or not mcu.get('receiving')):
                reason = '제어보드 응답 대기'
            elif ('session' not in latest or now-latest['session'][0]>3
                  or not isinstance(session,dict) or session.get('session_id')!=route.get('session_id')):
                reason = '현재 지도에서 A·B를 다시 선택하고 노드를 재실행해 주세요.'
            elif session.get('fixed_map') and not session.get('localization_ready'):
                reason = '고정 지도에서 현재 위치·방향을 지정하고 라이다 정합을 기다려 주세요.'
            else:
                msg = latest['pose'][1]; p = msg.pose.pose
                position = (p.position.x,p.position.y); heading = wrap(yaw(p.orientation)+forward)
                stamp_age = (node.get_clock().now().nanoseconds-(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))/1e9
                if msg.header.frame_id != grid.frame or grid.frame != 'tank_map': reason = '지도 좌표계 불일치'
                elif not all(math.isfinite(v) for v in (*position,heading)) or not -.1 <= stamp_age < .6: reason = '위치 시각·값 확인 필요'
                elif previous_pose and (math.dist(position,previous_pose[1])>max(.4,2*(now-previous_pose[0])) or abs(wrap(heading-previous_pose[2]))>1.):
                    settle_until = now+2.; path = []; reason = '위치 보정 후 안정화 대기'
                previous_pose = (now,position,heading)
                if not reason and now<settle_until: reason = '위치 안정화 대기'
                if not reason and args.execute and mcu and mcu.get('mode') == 1:
                    state = 'RC'; reason = '수동 조종 중'; path = []; need_arm = True
                if not reason and now<dwell_until:
                    state = 'DWELL'; reason = '도착·중립 유지'
                if not reason:
                    try:
                        if not path or now-last_plan>2:
                            # Both goals must remain usable, including the return leg.
                            grid.path(points[0],points[1])
                            path = grid.path(position,points[goal]); last_plan = now
                        if math.dist(position,points[goal]) <= args.arrival:
                            legs += 1; goal = 1-goal; path = []; dwell_until = now+args.dwell
                            state = 'DWELL'; reason = '도착·중립 유지'
                        else:
                            while len(path)>1 and math.dist(position,path[0]) < .5: path.pop(0)
                            target = path[0] if math.dist(position,path[0]) >= .5 else path[-1]
                            (left,right),prediction_data = select_command(grid,latest['scan'][1],position,heading,
                                velocity,target,points[goal],forward,experiment_time,args.percent,args.clearance,
                                excite=not args.no_excitation)
                            phase = prediction_data['phase']
                            if prediction_data['reason']:
                                state = 'BLOCKED'; reason = prediction_data['reason']
                            elif not grid.line_clear(position,target):
                                path = []; state = 'BLOCKED'; reason = '경로 재탐색'
                            else:
                                state = 'RUNNING' if args.execute else 'PREVIEW'; output = (pwm(left),pwm(right))
                    except ValueError as exc:
                        state = 'BLOCKED'; reason = str(exc); path = []
            if reason and state == 'WAITING': need_arm = True
            if state=='RUNNING' and phase not in ('hold','HOLD','avoidance','transit'):
                experiment_time += min(.2,now-last_tick)
            last_tick = now
            if args.execute:
                try:
                    services_ready = all(c.service_is_ready() for c in [arm,*clients])
                    # Fresh command frames are still required by the MCU watchdog.
                    if services_ready and not futures:
                        if output != (1500,1500) and need_arm and arm_future is None:
                            arm_future = arm.call_async(SetBool.Request(data=True))
                            arm_started = now
                        futures = [c.call_async(ThrottlePulseWidth.Request(pulse_width=value)) for c,value in zip(clients,output)]
                        futures_started = now
                    if not services_ready:
                        state = 'WAITING'; reason = '추진기 서비스 연결 대기'; output = (1500,1500); need_arm = True
                except (RuntimeError,OSError):
                    state = 'WAITING'; reason = '추진기 통신 복구 대기'; output = (1500,1500); need_arm = True
            row = {'t':now-started,'ros_ns':node.get_clock().now().nanoseconds,'state':state,'reason':reason,
                   'goal':'AB'[goal],'legs':legs,'map_position_m':position,'heading_rad':heading,
                   'phase':phase,'velocity_body':velocity,'experiment_s':experiment_time,
                   'left':output[0],'right':output[1],'execute':args.execute,'ack':acknowledgement,
                   'mcu':mcu,'pose_age_s':now-latest['pose'][0] if 'pose' in latest else None}
            if odom_row: row['odom'] = odom_row
            if 'imu' in latest:
                row['imu'] = {'gyro_sensor_z_rad_s':latest['imu'][1].angular_velocity.z,
                              'age_s':now-latest['imu'][0]}
                imu = latest['imu'][1]
                row['imu'].update(quaternion=[getattr(imu.orientation,k) for k in ('x','y','z','w')],
                    gyro_rad_s=[getattr(imu.angular_velocity,k) for k in ('x','y','z')],
                    accel_m_s2=[getattr(imu.linear_acceleration,k) for k in ('x','y','z')],
                    frame=imu.header.frame_id,stamp_s=imu.header.stamp.sec+imu.header.stamp.nanosec/1e9)
            try: log.write(clean_json(row))
            except OSError as exc:
                io_error = str(exc); disk_ok = False
            if velocity:
                chart_history.append([round(now-started,2),*velocity,(output[0]-1500)/4,(output[1]-1500)/4,
                    row.get('imu',{}).get('gyro_sensor_z_rad_s')])
            status_pub.publish(String(data=json.dumps(clean_json({'state':state,'reason':reason,'goal':'AB'[goal],
                'legs':legs,'path_m':path,'execute':args.execute,'phase':phase,'output_limit_percent':args.percent,
                'log_directory':str(directory),'history':list(chart_history)[::3],
                'storage_error':io_error,'velocity_body':velocity}),ensure_ascii=False,allow_nan=False)))
            command_pub.publish(String(data=json.dumps({'source':'indoor_trial','left':output[0],'right':output[1],
                                                        'execute':args.execute,'state':state})))
            if prediction_data.get('points_m'):
                if model and velocity and position is not None:
                    from .tank_model import predict
                    learned = predict(model,position,heading,velocity,(output[0]-1500)/4,(output[1]-1500)/4)
                    if learned: prediction_data = learned
                prediction_pub.publish(String(data=json.dumps(prediction_data)))
            elif position is not None and all(math.isfinite(v) for v in (*position,heading)) and 'odom' in row and row['odom']['age_s']<.5:
                # A constant-velocity preview is explicit; learned dynamics are fitted offline.
                o = row['odom']; sample = (o['stamp_s'],o['x'],o['y'])
                if all(math.isfinite(v) for v in sample) and (not motion_history or sample[0]>motion_history[-1][0]):
                    if motion_history and sample[0]-motion_history[-1][0]>.5: motion_history.clear()
                    motion_history.append(sample)
                while motion_history and o['stamp_s']-motion_history[0][0]>.6: motion_history.popleft()
                if len(motion_history)>=2 and motion_history[-1][0]-motion_history[0][0]>.15:
                    first,last = motion_history[0],motion_history[-1]; dt = last[0]-first[0]
                    vx,vy = (last[1]-first[1])/dt,(last[2]-first[2])/dt
                    angle = heading-forward-o['yaw_rad']; co,si = math.cos(angle),math.sin(angle)
                    vx,vy = co*vx-si*vy,si*vx+co*vy
                    prediction = [[position[0]+s*vx,position[1]+s*vy] for s in (0,.5,1,1.5,2)]
                    if math.hypot(vx,vy)<2 and all(math.isfinite(v) for p in prediction for v in p):
                        prediction_pub.publish(String(data=json.dumps({'kind':'constant_velocity','seconds':2,'points_m':prediction})))
            if (state,reason)!=previous_state:
                print(state+(': '+reason if reason else '')+' → '+'AB'[goal],flush=True); previous_state = (state,reason)
            while not exits.requested and time.monotonic()<tick: rclpy.spin_once(node,timeout_sec=.01)
    finally:
        try:
            if args.execute:
                until = time.monotonic()+.7
                while time.monotonic()<until and (any(not f.done() for f in futures) or (arm_future and not arm_future.done())):
                    rclpy.spin_once(node,timeout_sec=.01)
                print('NEUTRAL_ACK' if neutralize(node) else 'STOP_UNCONFIRMED: use physical EMO',flush=True)
        finally:
            try: log.close()
            except OSError: pass
            if analysis and analysis.poll() is None:
                analysis.terminate()
                try: analysis.wait(timeout=5)
                except subprocess.TimeoutExpired: analysis.kill(); analysis.wait()
            analysis_stream.close()
            node.destroy_node()
            if rclpy.ok(): rclpy.shutdown()
            exits.restore()


if __name__ == '__main__': main()
