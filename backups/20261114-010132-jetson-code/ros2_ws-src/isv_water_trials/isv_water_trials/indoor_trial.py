"""Continuous GPS-free A/B trial. Preview by default; --execute enables PWM."""
import argparse
from collections import deque
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import zlib

from .tank_core import Grid, guidance, projected_clearance, pwm, wrap, yaw
from .common import clean_json
from .tank_safety import select_command, DepartureRest, MotionVelocity
from .tank_storage import OptionalTrialJournal, atomic_json
from .tank_waypoints import mission_points
from .outdoor_config import read_drive_limits
from .outdoor_follow import limit_output


def main(argv=None):
    from .tank_mission import arrive_once
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--route', default='~/water_trials/tank/route.json')
    parser.add_argument('--percent', type=float, default=20.)
    parser.add_argument('--clearance', type=float, default=.75, help='LiDAR-reference footprint clearance, metres')
    parser.add_argument('--arrival', type=float, default=.5)
    parser.add_argument('--dwell', type=float, default=0., help='0: continuous turnaround; >0: settled departure measurement')
    parser.add_argument('--forward-axis-deg', type=float, default=0.,
                        help='Boat forward in ROS velodyne frame; VLP16 cable aft: +X=0 (default), +Y=90')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--once',action='store_true',help='Visit each saved point once, then neutralize and exit')
    parser.add_argument('--reverse-b',action='store_true',help='Use reverse-only thrust on the B leg')
    parser.add_argument('--open-water',action='store_true',help='Accept positive-infinite no-return rays as observed open water')
    parser.add_argument('--buoy-course',action='store_true',help='Keep goal pursuit on the contact-permitted buoy course')
    parser.add_argument('--mission-id',default='')
    parser.add_argument('--completion-file')
    parser.add_argument('--map-topic',default='/map')
    excitation_args=parser.add_mutually_exclusive_group()
    excitation_args.add_argument('--no-excitation', action='store_true', default=True, help='Continuous A/B tracking (default)')
    excitation_args.add_argument('--excitation', dest='no_excitation', action='store_false', help='Enable response experiments, including neutral coast phases')
    parser.add_argument('--model', help='Optional qualified response_model.json for displayed predictions')
    parser.add_argument('--analysis-interval', type=float, default=60.)
    parser.add_argument('--output', default='~/water_trials/indoor')
    args = parser.parse_args(argv)
    if args.buoy_course and not args.open_water:parser.error('--buoy-course requires --open-water')
    if not 0 <= args.percent <= (100 if args.open_water else 20) or not .75 <= args.clearance <= 3 or not .1 <= args.arrival <= 1 or not 0 <= args.dwell <= 60:
        parser.error('percent: 0..100 for open water, 0..20 indoors; clearance: .75..3m, arrival: .1..1m, dwell: 0..60s')
    if not math.isfinite(args.forward_axis_deg): parser.error('Invalid forward axis')
    if not math.isfinite(args.analysis_interval) or args.analysis_interval<30: parser.error('analysis-interval >=30s')
    model = json.loads(Path(args.model).expanduser().read_text(encoding='utf-8')) if args.model else None
    forward = math.radians(args.forward_axis_deg or 0.)
    route = json.loads(Path(args.route).expanduser().read_text(encoding='utf-8'))
    try:
        mission = mission_points(route)
    except (ValueError, TypeError, KeyError, AttributeError) as exc:
        parser.error(str(exc))
    goal_names = [name for name, _ in mission]
    points = [point for _, point in mission]
    if route.get('frame_id') != 'tank_map': parser.error('Route must use tank_map')
    import fcntl
    runtime_lock = open('/tmp/kabot_tank_trial_'+os.environ.get('ROS_DOMAIN_ID','0')+'.lock','w')
    try: fcntl.flock(runtime_lock,fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError: parser.error('A tank trial is already running in this ROS domain')
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
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
    velocity_estimator=MotionVelocity()
    odom_source=None
    def receive(key):
        def cb(msg):
            nonlocal odom_source
            received=time.monotonic()
            if key in ('odom','fused_odom'):
                if msg.header.frame_id!='tank_odom' or msg.child_frame_id!='velodyne':return
                latest[key+'_input']=(received,msg)
                if key=='odom' and 'fused_odom_input' in latest and received-latest['fused_odom_input'][0]<.5:return
                source=key
                if source!=odom_source:
                    velocity_estimator.samples.clear()
                    odom_source=source
                key_used='odom'
            else:key_used=key
            latest[key_used]=(received,msg)
            if key_used=='odom':
                stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9;p=msg.pose.pose
                velocity_estimator.add(stamp,p.position.x,p.position.y,yaw(p.orientation)+forward,received)
        return cb
    def receive_map(msg):
        nonlocal pending_map,map_signature
        signature=(msg.header.frame_id,str(msg.info),zlib.crc32(msg.data.tobytes()))
        if signature!=map_signature:
            pending_map = msg;map_signature=signature
    map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid, args.map_topic, receive_map, map_qos)
    live_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    for cls, topic, key in [(PoseWithCovarianceStamped,'/tank/pose','pose'), (Odometry,'/tank/odom','odom'),
                            (Odometry,'/tank/fused_odom','fused_odom'),
                            (LaserScan,'/scan','scan'), (Imu,'/vectornav/imu','imu'), (String,'/actuators/status','mcu'),
                            (String,'/tank/session','session')]:
        node.create_subscription(cls, topic, receive(key), live_qos)
    status_pub = node.create_publisher(String, '/tank/status', 10)
    prediction_pub = node.create_publisher(String, '/tank/prediction', 10)
    command_pub = node.create_publisher(String, '/water_trial/command', 10)
    clients = [node.create_client(ThrottlePulseWidth, '/actuators/throttle_'+side+'/set_pulse_width') for side in ('left','right')] if args.execute else []
    arm = node.create_client(SetBool, '/actuators/arm') if args.execute else None
    directory = Path(args.output).expanduser()/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    log = OptionalTrialJournal(directory, {'args':vars(args),'route':route,'mass_kg':25.,
        'length_m':1.1,'width_m':.5,'thruster_spacing_m':.32,'pose_reference':'velodyne',
        'notes':['LiDAR pose covariance is uncalibrated; validate drift against pool dimensions.',
                 'Percent is PWM command fraction, not measured thrust or speed.',
                 'Collision rollout is provisional; stopping response requires tank validation.']})
    print(('EXECUTE' if args.execute else 'PREVIEW (no actuator services)')+' LOG='+str(directory),flush=True)
    started = time.monotonic(); goal = 0; legs = 0; path = []
    departure = DepartureRest(minimum=args.dwell,enabled=args.dwell>0); departure_reference=None; departure_id=0
    previous_state = None; previous_pose = None; settle_until = started
    futures = []; arm_future = None; last_plan = 0.; need_arm = True
    mission_complete=False
    acknowledgement = None
    motion_history = deque(maxlen=12); chart_history = deque(maxlen=300)
    experiment_time = 0.; last_tick = started; io_error = log.error
    output_settings_at=-float('inf')
    analysis = None; analysis_started = started; analysis_stream = None
    if log.journal is not None:
        try: analysis_stream = (directory/'analysis.log').open('a',encoding='utf-8')
        except OSError as exc: io_error = str(exc)
    analysis_env = dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
    try:
        while not exits.requested and rclpy.ok():
            tick = time.monotonic()+.1
            rclpy.spin_once(node,timeout_sec=.02)
            # Drain ready callbacks even when planning overran the previous tick.
            # A bounded batch prevents IMU traffic from starving command output.
            drain_until=time.monotonic()+.015
            for _ in range(32):
                if time.monotonic()>=drain_until:break
                rclpy.spin_once(node,timeout_sec=0.)
            now = time.monotonic()
            if now-output_settings_at>=.2:
                output_settings_at=now
                try:
                    limits=read_drive_limits(Path.home()/'water_trials/outdoor/drive_limits.json',args.percent)
                    if args.mission_id:args.percent=limits['dock']
                except (OSError,ValueError,KeyError,TypeError):pass
            if analysis and analysis.poll() is None and now-analysis_started>120:
                analysis.kill(); analysis.wait()
            if analysis_stream is not None and now-analysis_started>=args.analysis_interval and (analysis is None or analysis.poll() is not None) and not io_error:
                try:
                    analysis = subprocess.Popen(['nice','-n','10',sys.executable,'-m','isv_water_trials.indoor_analyze',str(directory)],
                        stdout=analysis_stream,stderr=subprocess.STDOUT,env=analysis_env)
                    analysis_started = now
                except OSError as exc: io_error = str(exc)
            if pending_map is not None:
                try: grid = Grid(pending_map,args.clearance); map_error = None; path = []
                except (ValueError,TypeError) as exc: map_error = str(exc); grid = None
                pending_map = None
            reason = ''; state = 'WAITING'; output = (1500,1500); position = None; heading = None
            mcu = None
            velocity = None; odom_row = None; prediction_data = {}; phase = 'hold'
            departure_event=None
            if 'odom' in latest:
                o = latest['odom'][1]; p = o.pose.pose
                stamp = o.header.stamp.sec+o.header.stamp.nanosec/1e9
                odom_row = {'x':p.position.x,'y':p.position.y,'yaw_rad':yaw(p.orientation),
                            'stamp_s':stamp,'age_s':now-latest['odom'][0]}
                velocity=velocity_estimator.estimate(now,node.get_clock().now().nanoseconds/1e9)
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
            if args.execute and mcu and mcu.get('receiving') and mcu.get('mode')==1:
                state='RC'; reason='수동 조종 중 · 위치 추적·재지정 가능'
                path=[];need_arm=True
                if previous_state is None or previous_state[0]!='RC':departure.begin()
            elif grid is None: reason = map_error or '지도 대기'
            elif not ready:
                missing=[k for k in ('pose','odom','scan','imu') if k not in latest or now-latest[k][0]>=.5]
                reason=('위치 정합 대기: '+session.get('localization_reason','현재 위치 미확정')) if session.get('fixed_map') and not session.get('localization_ready') else '센서 수신·시각 확인: '+', '.join(missing or ['odom','scan','imu'])
            elif args.execute and (not mcu or not mcu.get('receiving')):
                reason = '제어보드 응답 대기'
            elif ('session' not in latest or now-latest['session'][0]>3
                  or not isinstance(session,dict)
                  or (session.get('session_id')!=route.get('session_id')
                      and not (route.get('mission_waypoints') and route.get('frame_id')=='tank_map'))):
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
                    settle_until = now+.3; path = []; reason = '큰 위치 변화 재확인'
                previous_pose = (now,position,heading)
                if not reason and now<settle_until: reason = '큰 위치 변화 재확인'
                if not reason and departure.active:
                    imu=latest['imu'][1]
                    imu_stamp=imu.header.stamp.sec+imu.header.stamp.nanosec/1e9
                    neutral=not args.execute or bool(mcu and mcu.get('mode')==2 and mcu.get('receiving') and mcu.get('left')==1500 and mcu.get('right')==1500)
                    reference=departure.update(now,(odom_row['stamp_s'],imu_stamp),velocity,
                        tuple(getattr(imu.angular_velocity,k) for k in ('x','y','z')),
                        tuple(getattr(imu.orientation,k) for k in ('x','y','z','w')),neutral)
                    state='DWELL'; phase='departure_rest'
                    reason=f'출발 준비 · 중립 {departure.minimum:g}초 이상, 이동 {departure.speed:g}m/s·회전 {math.degrees(departure.rate):g}°/s 이하 {departure.stable:g}초 확인'
                    if reference is not None:
                        departure_id+=1
                        departure_reference=dict(reference,departure_id=departure_id,goal=goal_names[goal],
                            map_position_m=position,heading_rad=heading,imu_frame=imu.header.frame_id,
                            ros_ns=node.get_clock().now().nanoseconds)
                        departure_event=departure_reference; reason='출발 기준 자세·잔여 움직임 저장 완료'
                if not reason:
                    try:
                        if math.dist(position,points[goal]) <= args.arrival:
                            goal,legs,mission_complete=arrive_once(goal,legs,len(points),args.once);path=[]
                            if mission_complete:
                                state='COMPLETE';phase='hold';reason='도킹 A/B 도달 완료'
                            elif len(points) == 1:
                                state = 'ARRIVED'; phase = 'hold'; reason = goal_names[goal]+' 도착 · 단일 지점 대기'
                            else:
                                departure.begin()
                                if departure.active:
                                    state = 'DWELL'; phase='departure_rest'; reason = '도착 · 중립 유지 후 정지 확인'
                        if not reason and args.percent==0:
                            state='PAUSED';phase='hold';reason='도킹 출력 0%'
                        if not reason:
                            if not path or now-last_plan>2:
                                try:
                                    planning_grid=(grid.with_live_scan(position,points[goal],latest['scan'][1],heading,forward)
                                                   if args.open_water else grid)
                                    control_grid,path,approach,margin_recovery=planning_grid.trial_path(position,points[goal],args.arrival)
                                except ValueError:
                                    if not args.buoy_course:raise
                                    control_grid=grid;path=[tuple(position),points[goal]]
                                    approach=points[goal];margin_recovery=False
                                last_plan = now
                            while len(path)>1 and math.dist(position,path[0]) < .5: path.pop(0)
                            target = path[0] if math.dist(position,path[0]) >= .5 else path[-1]
                            (left,right),prediction_data = select_command(control_grid,latest['scan'][1],position,heading,
                                velocity,target,approach,forward,experiment_time,args.percent,control_grid.clearance,
                                excite=not args.no_excitation,margin_recovery=margin_recovery,
                                reverse_only=args.reverse_b and goal_names[goal]=='B',open_water=args.open_water,
                                buoy_course=args.buoy_course)
                            prediction_data.update(requested_goal_m=points[goal],approach_goal_m=approach,
                                                   clearance_m=control_grid.clearance)
                            phase = prediction_data['phase']
                            if prediction_data['reason']:
                                state = 'BLOCKED'; reason = prediction_data['reason']
                            elif not args.buoy_course and not control_grid.line_clear(position,target):
                                path = []; state = 'BLOCKED'; reason = '경로 재탐색'
                            else:
                                state = 'RUNNING' if args.execute else 'PREVIEW'; output = (pwm(left),pwm(right))
                    except ValueError as exc:
                        state = 'BLOCKED'; reason = str(exc); path = []
            if reason and state == 'WAITING':
                need_arm = True
                if mcu and mcu.get('mode')==1:reason='RC 모드 · '+reason
            if state=='RUNNING' and phase not in ('hold','HOLD','avoidance','transit','clearance_recovery'):
                experiment_time += min(.2,now-last_tick)
            last_tick = now
            if args.execute:
                normalized=[(value-1500)/400. for value in output]
                normalized=limit_output(normalized,args.percent)
                output=tuple(1500+round(400*value) for value in normalized)
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
            if departure.active and state!='DWELL': departure.interrupt()
            row = {'t':now-started,'ros_ns':node.get_clock().now().nanoseconds,'state':state,'reason':reason,
                   'goal':goal_names[goal],'legs':legs,'map_position_m':position,'heading_rad':heading,
                   'phase':phase,'velocity_body':velocity,'experiment_s':experiment_time,
                   'left':output[0],'right':output[1],'execute':args.execute,'ack':acknowledgement,
                   'mcu':mcu,'pose_age_s':now-latest['pose'][0] if 'pose' in latest else None,
                    'departure_id':departure_id,'odom_source':odom_source}
            if departure_event is not None: row['departure_reference']=departure_event
            if odom_row: row['odom'] = odom_row
            if 'imu' in latest:
                row['imu'] = {'gyro_sensor_z_rad_s':latest['imu'][1].angular_velocity.z,
                              'age_s':now-latest['imu'][0]}
                imu = latest['imu'][1]
                row['imu'].update(quaternion=[getattr(imu.orientation,k) for k in ('x','y','z','w')],
                    gyro_rad_s=[getattr(imu.angular_velocity,k) for k in ('x','y','z')],
                    accel_m_s2=[getattr(imu.linear_acceleration,k) for k in ('x','y','z')],
                    frame=imu.header.frame_id,stamp_s=imu.header.stamp.sec+imu.header.stamp.nanosec/1e9)
                if departure_reference and departure_reference['imu_frame']==imu.header.frame_id:
                    q=departure_reference['quaternion_xyzw'];reference_yaw=math.atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]*q[1]+q[2]*q[2]))
                    row['imu']['yaw_from_departure_rad']=wrap(yaw(imu.orientation)-reference_yaw)
            log.write(clean_json(row))
            if log.error: io_error = log.error
            if velocity:
                chart_history.append([round(now-started,2),*velocity,(output[0]-1500)/4,(output[1]-1500)/4,
                    row.get('imu',{}).get('gyro_sensor_z_rad_s')])
            status_pub.publish(String(data=json.dumps(clean_json({'state':state,'reason':reason,'goal':goal_names[goal],
                'mission_id':args.mission_id,
                'goal_point_m':points[goal],'mission_waypoints_m':points,
                'map_position_m':position,'heading_rad':heading,
                'legs':legs,'path_m':path,'execute':args.execute,'phase':phase,'output_limit_percent':args.percent,
                'log_directory':str(directory),'history':list(chart_history)[::3],
                'storage_error':io_error,'recording_enabled':log.journal is not None,'buoy_course':args.buoy_course,
                'velocity_body':velocity,'departure_reference':departure_reference,'odom_source':odom_source}),ensure_ascii=False,allow_nan=False)))
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
                print(state+(': '+reason if reason else '')+' → '+goal_names[goal],flush=True); previous_state = (state,reason)
            if mission_complete:break
            while not exits.requested and time.monotonic()<tick: rclpy.spin_once(node,timeout_sec=.01)
    finally:
        try:
            neutral_confirmed=not args.execute
            if args.execute:
                until = time.monotonic()+.7
                while time.monotonic()<until and (any(not f.done() for f in futures) or (arm_future and not arm_future.done())):
                    rclpy.spin_once(node,timeout_sec=.01)
                neutral_confirmed=neutralize(node)
                print('NEUTRAL_ACK' if neutral_confirmed else 'STOP_UNCONFIRMED: use physical EMO',flush=True)
            if args.completion_file:
                atomic_json(Path(args.completion_file).expanduser(),dict(mission_id=args.mission_id,
                    complete=mission_complete,neutral_confirmed=neutral_confirmed,legs=legs))
        finally:
            try: log.close()
            except OSError: pass
            if analysis and analysis.poll() is None:
                analysis.terminate()
                try: analysis.wait(timeout=5)
                except subprocess.TimeoutExpired: analysis.kill(); analysis.wait()
            if analysis_stream is not None:
                try: analysis_stream.close()
                except OSError: pass
            node.destroy_node()
            if rclpy.ok(): rclpy.shutdown()
            exits.restore()


if __name__ == '__main__': main()
