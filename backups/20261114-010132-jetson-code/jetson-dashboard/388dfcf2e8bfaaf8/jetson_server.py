#!/usr/bin/env python3
"""ROS subscriptions and a loopback-only HTTP feed for the PC sensor viewer."""
import argparse
from collections import defaultdict, deque
import gzip
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import uuid
from urllib.parse import parse_qs, urlsplit

from telemetry_core import (POINT_HEADER, GpsTrack, bounded_preview, cloud_value,
                            finite, imu_value, quaternion_angles, scan_value,
                            vector, vectornav_value, vectornav_heading)

VERSION = 'kabot-sensor-viewer-v1'


def display_points(rows, limit):
    if not isinstance(rows, list) or len(rows) <= limit:
        return rows
    return [rows[round(i*(len(rows)-1)/(limit-1))] for i in range(limit)]


def display_outdoor(data):
    """Bound dashboard copies only; the vessel retains its full navigation data."""
    value = {k:v for k,v in data.items() if k not in ('history','model','validation_model')}
    for key in ('model','validation_model'):
        model = data.get(key)
        value[key] = {'available':True} if model else None
    value['trail'] = display_points(data.get('trail', []), 300)
    for key in ('guidance','guidance_raw','display_guidance','display_guidance_raw',
                'point_follow_path','prediction','feasible_path','preview'):
        if key in value: value[key] = display_points(value[key],120)
    layers = dict(data.get('map_layers') or {})
    for key, limit in (('hits',180),('accumulated',240),('survey',240)):
        if key in layers: layers[key] = display_points(layers[key],limit)
    value['map_layers'] = layers
    return value


VIEW_VALUES = {
    'outdoor': {'outdoor','gps','heading','heading_raw','virtual_gps','saved_route','map_reference','runtime'},
    'sensors': {'imu','vectornav','heading','heading_raw','scan','gps','gps_velocity','gps_corrections',
                'cloud','actuators','hull_mask','runtime'},
    'slam': {'map','tank_pose','tank_session','tank_status','tank_route','tank_localization_candidates',
             'tank_map_reset','runtime'},
}


def dashboard_request():
    try:
        path=Path.home()/'water_trials/runtime/dashboard_wake.json'
        if time.time()-path.stat().st_mtime>30:return None
        value=json.loads(path.read_text())
        return str(value['id'])[:64]
    except (OSError,ValueError,KeyError,TypeError):return None


def runtime_status(proc_root=Path('/proc')):
    """Read local process lifetimes; stale ROS topics do not imply a live session."""
    targets = {
        'sensors': {'all_sensors.launch.py', 'mechaship_bringup.launch.py',
                    'sensor_status.py', 'velodyne_driver_node', 'vectornav', 'ublox_gps_node'},
        'mapping': {'tank_mapping.launch.py', 'tank_create_map.launch.py', 'tank_pose'},
        'trial': {'indoor_trial', 'isv_water_trials.indoor_trial'},
    }
    result = dict.fromkeys(targets, 'stopped')
    try: entries = list(proc_root.iterdir())
    except OSError: return dict.fromkeys(targets, 'unknown')
    for entry in entries:
        if not entry.name.isdigit(): continue
        try:
            if entry.joinpath('stat').read_text().rsplit(')', 1)[1].split()[0] == 'Z': continue
            args = entry.joinpath('cmdline').read_bytes().decode(errors='replace').strip('\0').split('\0')
        except (OSError, IndexError): continue
        tokens = {arg.rsplit('/', 1)[-1] for arg in args}
        for name, names in targets.items():
            if tokens & names: result[name] = 'running'
        if 'isv_water_trials.tank_session' in args and '_worker' in args:
            index = args.index('_worker') + 1
            if index < len(args) and args[index] in result and result[args[index]] != 'running':
                result[args[index]] = 'restarting'
    return result


class Store:
    def __init__(self, args):
        self.args = args
        self.lock = threading.RLock()
        self.graph = {'nodes':[], 'topics':[], 'updated_s':None}
        self.arrivals = defaultdict(lambda:deque(maxlen=600))
        self.values = {}
        self.preview = None
        self.selected = '/actuators/status'
        self.cloud_bytes = b''
        self.cloud_sequence = 0
        self.errors = {}
        self.last_client = 0.
        self.instance_id=uuid.uuid4().hex
        self.sensor_restart = {'busy':False, 'error':None}
        self.node_stop = {'busy':False,'error':None,'completed':False,'result':None}
        self.watertrial_control = {'busy':False,'action':None,'error':None,'result':None}
        self.esc_initialize = {'busy':False, 'error':None, 'acknowledged':False}

    def request_watertrial_control(self, action):
        if action not in ('stop_other','stop_watertrial'):
            return 400, {'error':'Invalid WaterTrial action'}
        with self.lock:
            if self.watertrial_control['busy'] or self.sensor_restart['busy'] or self.esc_initialize['busy'] or self.node_stop['busy']:
                return 409, {'error':'Another bringup operation is in progress'}
            self.watertrial_control={'busy':True,'action':action,'error':None,'result':None}
        threading.Thread(target=self._run_watertrial_control,args=(action,),daemon=True).start()
        return 202, {'ok':True,'action':action}

    def _run_watertrial_control(self, action):
        code = {
            'stop_other': "from isv_water_trials.stop_nodes import stop; print(json.dumps(stop(disarm_first=False)))",
            'stop_watertrial': "from isv_water_trials.tank_session import main,running,NAMES; from isv_water_trials.stop_nodes import stop_watertrial; [main(['stop',name],_internal=True) for name in NAMES]; result=stop_watertrial(); result['ok']=result['ok'] and not any(running(name) for name in NAMES); print(json.dumps(result))",
        }[action]
        error=None; result=None
        try:
            process=subprocess.run([sys.executable,'-c','import json; '+code],timeout=150,
                stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
                start_new_session=True)
            lines=process.stdout.decode(errors='replace').strip().splitlines()
            result=json.loads(lines[-1]) if lines else None
            if process.returncode or not result or not result.get('ok'):
                error='Operation did not complete: '+str((result or {}).get('remaining',lines[-1:] if lines else []))
        except (OSError,subprocess.SubprocessError,ValueError) as exc:
            error='WaterTrial operation failed: '+str(exc)
        finally:
            with self.lock:
                self.watertrial_control={'busy':False,'action':action,'error':error,'result':result}

    def restart_sensors(self):
        with self.lock:
            if self.sensor_restart['busy'] or self.esc_initialize['busy'] or self.node_stop['busy'] or self.watertrial_control['busy']:
                return 409, {'error':'센서 또는 ESC 처리 중입니다.'}
            self.sensor_restart = {'busy':True, 'error':None}
            self.node_stop={'busy':False,'error':None,'completed':False,'result':None}
        threading.Thread(target=self._restart_sensors,daemon=True).start()
        return 202, {'ok':True}

    def initialize_esc(self):
        with self.lock:
            if self.esc_initialize['busy'] or self.sensor_restart['busy'] or self.node_stop['busy'] or self.watertrial_control['busy']:
                return 409, {'error':'센서 또는 ESC 처리 중입니다.'}
            self.esc_initialize = {'busy':True, 'error':None, 'acknowledged':False}
        threading.Thread(target=self._initialize_esc,daemon=True).start()
        return 202, {'ok':True}

    def stop_all_nodes(self):
        with self.lock:
            if self.node_stop['busy'] or self.sensor_restart['busy'] or self.esc_initialize['busy'] or self.watertrial_control['busy']:
                return 409,{'error':'브링업·종료·ESC 처리 중입니다.'}
            self.node_stop={'busy':True,'error':None,'completed':False,'result':None}
        threading.Thread(target=self._stop_all_nodes,daemon=True).start()
        return 202,{'ok':True}

    def _stop_all_nodes(self):
        error=None;result=None
        try:
            process=subprocess.run([sys.executable,'-m','isv_water_trials.stop_nodes'],
                timeout=45,stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
                start_new_session=True)
            lines=process.stdout.decode(errors='replace').strip().splitlines()
            result=json.loads(lines[-1]) if lines else None
            if process.returncode or not result or not result.get('ok'):
                error='일부 노드 종료 미완료 · '+str((result or {}).get('remaining',[]))
        except (OSError,subprocess.SubprocessError,ValueError):error='전체 노드 종료 요청 실패'
        finally:
            with self.lock:self.node_stop={'busy':False,'error':error,'completed':error is None,'result':result}

    def _initialize_esc(self):
        error = None
        try:
            subprocess.run([sys.executable,'-m','isv_water_trials.reinitialize_esc'],
                           check=True,timeout=15,stdin=subprocess.DEVNULL,
                           stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,
                           start_new_session=True)
            # The existing sender uses a seven-second neutral initialization interval.
            time.sleep(7)
        except subprocess.CalledProcessError as exc:
            detail=(exc.stderr or exc.stdout or '').strip().splitlines()
            error='ESC 초기화 요청 실패'+(f' · {detail[-1][-240:]}' if detail else ' · 요청 프로세스가 실패했습니다.')
        except subprocess.TimeoutExpired:
            error='ESC 초기화 요청 실패 · 응답 시간이 초과됐습니다.'
        except OSError as exc:
            error=f'ESC 초기화 요청 실패 · 실행 오류: {exc}'
        finally:
            with self.lock:
                self.esc_initialize = {'busy':False, 'error':error, 'acknowledged':error is None}

    def _restart_sensors(self):
        code = '''from isv_water_trials.tank_session import main,running
previous=running('sensors')
extra=previous.get('args',[]) if previous else []
main(['stop','sensors'])
if running('sensors'):raise RuntimeError('Sensor shutdown timed out')
main(['start','sensors','--',*extra])
if not running('sensors'):raise RuntimeError('Sensor startup failed')
'''
        error = None
        try:
            subprocess.run([sys.executable,'-c',code],check=True,timeout=45,
                           stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL,start_new_session=True)
        except (OSError,subprocess.SubprocessError):
            error = '센서 재시작 실패 · sensors.log를 확인해 주세요.'
        finally:
            with self.lock:
                self.sensor_restart = {'busy':False, 'error':error}

    def touch(self):
        with self.lock:
            self.last_client = time.monotonic()

    def note(self, topic):
        with self.lock:
            self.arrivals[topic].append(time.monotonic())

    def value(self, key, data):
        with self.lock:
            self.values[key] = dict(data, received_monotonic=time.monotonic(), stamp_s=time.time())

    def accepts_sensor(self, key, topic, preferred):
        """Keep the preferred live source; follow a renamed topic after a gap."""
        with self.lock:
            current = self.values.get(key)
            if not current or current.get('topic') == topic:
                return True
            rank = lambda name: preferred.index(name) if name in preferred else len(preferred)
            return (rank(topic) < rank(current.get('topic')) or
                    time.monotonic()-current['received_monotonic'] > 2.)

    def sensors_running(self):
        names={'sensor_status','velodyne_driver_node','vectornav','ublox_gps_node','arduino_actuator_bridge'}
        with self.lock:
            return any(name.rsplit('/',1)[-1] in names for name in self.graph['nodes']) or any(
                time.monotonic()-self.values[k]['received_monotonic']<3 for k in ('scan','imu','gps','cloud') if k in self.values)

    def snapshot(self, view='full'):
        from isv_water_trials.tank_record import recording_status
        recording=recording_status()
        now = time.monotonic()
        with self.lock:
            topics = []
            for row in self.graph['topics']:
                arrivals = self.arrivals.get(row['name'], ())
                recent = [x for x in arrivals if now-x < 5]
                hz = (len(recent)-1)/(recent[-1]-recent[0]) if len(recent)>1 and recent[-1]>recent[0] else 0. if arrivals else None
                topics.append(dict(row, hz=round(hz,2) if hz is not None else None,
                                   age_s=round(now-arrivals[-1],3) if arrivals else None))
            values = {k:dict((key,value) for key,value in v.items() if key!='received_monotonic')
                      | {'age_s':round(now-v['received_monotonic'],3)} for k,v in self.values.items()
                      if view not in VIEW_VALUES or k in VIEW_VALUES[view]}
            if view in VIEW_VALUES:
                if 'outdoor' in values: values['outdoor'] = display_outdoor(values['outdoor'])
                if 'tank_status' in values:
                    values['tank_status'] = {k:v for k,v in values['tank_status'].items() if k!='history'}
                if 'gps' in values:
                    values['gps']['track_enu_m'] = display_points(values['gps'].get('track_enu_m',[]),200)
                if 'scan' in values:
                    values['scan']['polar_deg_m'] = display_points(values['scan'].get('polar_deg_m',[]),360)
            heading = values.get('heading',{})
            raw_heading = values.get('heading_raw',{})
            if (heading.get('age_s',math.inf)>1 or finite(heading.get('heading_deg')) is None or
                raw_heading.get('imu_heading_mode')=='absolute_magnetic_raw'):
                if raw_heading.get('age_s',math.inf)<1 and finite(raw_heading.get('heading_deg')) is not None:
                    values['heading'] = raw_heading
            return {'version':VERSION,'instance_id':self.instance_id,'server_time_s':time.time(),
                    'nodes':self.graph['nodes'] if view not in ('outdoor','slam') else [],
                    'topics':topics if view not in ('outdoor','slam') else [],'graph_updated_s':self.graph['updated_s'],
                    'values':values,'preview':self.preview if view=='full' else None,'selected_topic':self.selected,
                    'errors':dict(self.errors),'sensor_restart':dict(self.sensor_restart),
                    'node_stop':dict(self.node_stop),
                    'watertrial_control':dict(self.watertrial_control),
                    'esc_initialize':dict(self.esc_initialize),'onboard_recording':recording,
                    'limits':{'max_points':self.args.max_points,'cloud_hz':self.args.cloud_hz,
                              'max_range_m':self.args.max_range,'ros_domain_id':os.environ.get('ROS_DOMAIN_ID','0')}}


def make_node(store):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from rosidl_runtime_py.utilities import get_message

    class Monitor(Node):
        def __init__(self):
            super().__init__('kabot_sensor_monitor')
            self.subscriptions_by_topic = {}
            self.latest_cloud = None
            self.cloud_topic = None
            self.cloud_seen = 0
            self.cloud_done = 0
            self.preview_times = {}
            self.summary_times = {}
            self.summary_values = {}
            self.gps_track = GpsTrack()
            self.heading_reference = {}
            try:
                from ament_index_python.packages import get_package_share_directory
                from isv_water_trials.outdoor_config import load_config
                self.heading_reference = load_config(Path(get_package_share_directory('isv_water_trials'))/'config/outdoor.json')
            except (ImportError,LookupError,OSError,ValueError,KeyError):
                pass
            try:
                route = json.loads((Path.home()/'water_trials/outdoor/route.json').read_text())
                point = route['points'][0]
                if len(point)==2 and all(finite(v) is not None for v in point) and -90<=point[0]<=90 and -180<=point[1]<=180:
                    store.value('map_reference',{'point':point,'source':'saved_route'})
            except (OSError,ValueError,KeyError,IndexError,TypeError):
                pass
            from tank_feed import TankFeed
            store.tank = self.tank = TankFeed(self,store)
            from outdoor_feed import OutdoorFeed
            store.outdoor = OutdoorFeed(self, store)
            self.create_timer(3.,self.refresh)
            self.create_timer(1./store.args.cloud_hz,self.encode_cloud)
            self.refresh()

        def refresh(self):
            store.value('runtime', {'services': runtime_status()})
            topics = self.get_topic_names_and_types()
            cloud_names = [name for name,types in topics if 'sensor_msgs/msg/PointCloud2' in types and self.count_publishers(name)>0]
            with store.lock:
                selected = store.selected
            cloud_topic = selected if selected in cloud_names else next((n for n in ('/velodyne_points','/ouster/points','/points') if n in cloud_names),cloud_names[0] if cloud_names else None)
            if cloud_topic != self.cloud_topic:
                self.cloud_topic = cloud_topic
                self.latest_cloud = None
                self.cloud_done = self.cloud_seen
                with store.lock:
                    store.cloud_bytes = b''
                    store.values.pop('cloud',None)
            active_names = set()
            graph = []
            for name,types in topics[:500]:
                if not types:
                    continue
                typename = types[0]
                publishers = self.get_publishers_info_by_topic(name)
                is_large = typename in ('sensor_msgs/msg/PointCloud2','sensor_msgs/msg/Image','sensor_msgs/msg/CompressedImage','nav_msgs/msg/OccupancyGrid')
                noisy = name in ('/rosout','/parameter_events') or '/_action/' in name
                observe = bool(publishers) and (not is_large or name==cloud_topic or name==selected or name=='/map') and (not noisy or name==selected)
                if observe:
                    active_names.add(name)
                    old = self.subscriptions_by_topic.get(name)
                    if old and old[0] != typename:
                        self.destroy_subscription(old[1]);del self.subscriptions_by_topic[name]
                    if name not in self.subscriptions_by_topic:
                        try:
                            qos = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL) if name=='/map' else qos_profile_sensor_data
                            sub = self.create_subscription(get_message(typename),name,self.callback(name,typename),qos)
                            self.subscriptions_by_topic[name] = (typename,sub)
                            with store.lock:
                                store.errors.pop(name,None)
                        except (ImportError,AttributeError,ValueError,RuntimeError) as exc:
                            with store.lock:
                                store.errors[name] = type(exc).__name__+': '+str(exc)[:180]
                graph.append({'name':name,'types':types,'publishers':len(publishers),
                              'subscribers':self.count_subscribers(name),'observed':observe and name in self.subscriptions_by_topic,
                              'publisher_nodes':[('/'+p.node_namespace.strip('/')+'/'+p.node_name).replace('//','/') for p in publishers]})
            for name in list(self.subscriptions_by_topic):
                if name not in active_names:
                    self.destroy_subscription(self.subscriptions_by_topic.pop(name)[1])
            nodes = sorted(('/'+namespace.strip('/')+'/'+name).replace('//','/') for name,namespace in self.get_node_names_and_namespaces())
            with store.lock:
                store.graph = {'nodes':nodes,'topics':sorted(graph,key=lambda x:x['name']),'updated_s':time.time()}

        def callback(self, topic, typename):
            def receive(message):
                store.note(topic)
                now = time.monotonic()
                if typename == 'sensor_msgs/msg/PointCloud2' and topic == self.cloud_topic:
                    self.latest_cloud = message
                    self.cloud_seen += 1
                if typename == 'nav_msgs/msg/OccupancyGrid' and topic == '/map':
                    self.tank.map(message)
                data = None
                if now-self.summary_times.get(topic,0) >= (.2 if topic=='/outdoor/status' else .1):
                    self.summary_times[topic] = now
                    try:
                        if typename == 'sensor_msgs/msg/Imu':
                            data = imu_value(message)
                            if store.accepts_sensor('imu',topic,('/vectornav/imu','/imu','/imu/data')):
                                store.value('imu',dict(data,topic=topic))
                        elif typename == 'vectornav_msgs/msg/CommonGroup':
                            data = vectornav_value(message);store.value('vectornav',dict(data,topic=topic))
                            store.value('heading_raw',vectornav_heading(dict(data,topic=topic),self.heading_reference))
                        elif typename == 'sensor_msgs/msg/LaserScan':
                            data = scan_value(message)
                            if store.accepts_sensor('scan',topic,('/scan','/scan_preprocessed')):
                                store.value('scan',dict(data,topic=topic))
                        elif typename == 'sensor_msgs/msg/NavSatFix':
                            if topic == '/gps/virtual/fix':
                                data = {'latitude':finite(message.latitude),'longitude':finite(message.longitude),'virtual':True}
                                store.value('virtual_gps_fix',dict(data,topic=topic))
                            elif store.accepts_sensor('gps',topic,('/gps/fix','/fix','/gps/filtered')):
                                data = self.gps_track.update(message)
                                stamp = message.header.stamp.sec+message.header.stamp.nanosec/1e9
                                data.update(measurement_stamp_s=stamp,
                                            measurement_age_s=self.get_clock().now().nanoseconds/1e9-stamp)
                                store.value('gps',dict(data,topic=topic))
                        elif typename == 'std_msgs/msg/String' and topic == '/navigation/heading':
                            data = json.loads(message.data);store.value('heading',data)
                        elif typename == 'std_msgs/msg/String' and topic == '/gps/corrections':
                            data = json.loads(message.data)
                            store.value('gps_corrections',dict(data,topic=topic))
                        elif typename in ('geometry_msgs/msg/TwistWithCovarianceStamped','geometry_msgs/msg/TwistStamped','geometry_msgs/msg/Twist'):
                            twist = message.twist if hasattr(message,'twist') else message
                            twist = twist.twist if hasattr(twist,'twist') else twist
                            data = {'linear_velocity_m_s':vector(twist.linear),'angular_velocity_deg_s':vector(twist.angular,180/math.pi)}
                            if topic == '/gps/fix_velocity':
                                speed = math.hypot(twist.linear.x,twist.linear.y)
                                stamp = message.header.stamp.sec+message.header.stamp.nanosec/1e9
                                covariance = message.twist.covariance
                                data.update(east_m_s=finite(twist.linear.x), north_m_s=finite(twist.linear.y),
                                            velocity_sigma_m_s=math.sqrt(max(0.,covariance[0],covariance[7])),
                                            measurement_stamp_s=stamp,
                                            measurement_age_s=self.get_clock().now().nanoseconds/1e9-stamp)
                                store.value('gps_velocity',dict(data,speed_m_s=finite(speed),topic=topic))
                        elif typename in ('geometry_msgs/msg/PoseStamped','geometry_msgs/msg/PoseWithCovarianceStamped','nav_msgs/msg/Odometry'):
                            pose = message.pose
                            pose = pose.pose if hasattr(pose,'pose') else pose
                            data = {'position_m':vector(pose.position),'orientation':quaternion_angles(pose.orientation)}
                            if topic == '/tank/pose': store.value('tank_pose',data)
                        elif typename == 'std_msgs/msg/String' and topic == '/actuators/status':
                            data = json.loads(message.data);store.value('actuators',dict(data,topic=topic))
                        elif typename == 'std_msgs/msg/String' and topic == '/outdoor/status':
                            data = json.loads(message.data);store.value('outdoor',data)
                        elif typename == 'std_msgs/msg/String' and topic == '/lidar/self_mask/status':
                            data = json.loads(message.data);store.value('hull_mask',data)
                        elif typename == 'std_msgs/msg/String' and topic in ('/tank/status','/tank/prediction','/tank/session','/tank/localization_candidates'):
                            data = json.loads(message.data);store.value(topic.replace('/tank/','tank_'),data)
                    except (ValueError,AttributeError,TypeError,OverflowError) as exc:
                        with store.lock:
                            store.errors[topic] = type(exc).__name__+': '+str(exc)[:180]
                if data is not None:
                    self.summary_values[topic] = data
                with store.lock:
                    selected = store.selected
                if topic == selected and now-self.preview_times.get(topic,0) >= .5:
                    self.preview_times[topic] = now
                    if typename == 'sensor_msgs/msg/PointCloud2':
                        raw = {'frame_id':message.header.frame_id,'width':message.width,'height':message.height,
                               'point_step_bytes':message.point_step,'data_bytes':len(message.data)}
                    elif typename in ('sensor_msgs/msg/Image','sensor_msgs/msg/CompressedImage'):
                        raw = {'data_bytes':len(message.data),'encoding':getattr(message,'encoding',getattr(message,'format','')),
                               'width':getattr(message,'width',None),'height':getattr(message,'height',None)}
                    else:
                        raw = bounded_preview(message)
                    with store.lock:
                        store.preview = {'topic':topic,'type':typename,'received_s':time.time(),
                                         'display':self.summary_values.get(topic,raw),'raw':raw,
                                         'units':'degrees / m / SI' if topic in self.summary_values else '메시지 정의 기준'}
            return receive

        def encode_cloud(self):
            if self.latest_cloud is None or self.cloud_done == self.cloud_seen or time.monotonic()-store.last_client > 5:
                return
            sequence = self.cloud_seen
            started = time.perf_counter()
            try:
                payload,metadata = cloud_value(self.latest_cloud,store.args.max_points,store.args.max_range)
                self.cloud_done = sequence
                metadata.update(decode_ms=round((time.perf_counter()-started)*1000,2),topic=self.cloud_topic)
                with store.lock:
                    store.cloud_sequence += 1
                    number = store.cloud_sequence
                    metadata['sequence'] = number
                    stamp = time.time()
                    store.cloud_bytes = POINT_HEADER.pack(b'KPC1',number,metadata['display_points'],stamp,16)+payload
                    store.errors.pop(self.cloud_topic,None)
                store.value('cloud',metadata)
            except (ValueError,TypeError,BufferError) as exc:
                with store.lock:
                    store.errors[self.cloud_topic] = 'Cloud: '+str(exc)[:180]
    return Monitor()


def make_handler(store):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def log_message(self, *_):
            pass

        def send(self, status, body=b'', content_type='application/json', compress=False):
            if compress and len(body)>1500 and 'gzip' in self.headers.get('Accept-Encoding',''):
                body = gzip.compress(body,compresslevel=1);encoding = 'gzip'
            else:
                encoding = None
            self.send_response(status);self.send_header('Content-Type',content_type)
            self.send_header('Content-Length',str(len(body)));self.send_header('Cache-Control','no-store')
            if encoding:
                self.send_header('Content-Encoding',encoding)
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError,ConnectionResetError):
                pass

        def do_GET(self):
            store.last_http = time.monotonic()
            path = urlsplit(self.path)
            if path.path == '/health':
                self.send(200,json.dumps({'version':VERSION,'pid':os.getpid(),'sensors_running':store.sensors_running(),'dashboard_request':dashboard_request()}).encode());return
            if path.path == '/api/state':
                query=parse_qs(path.query)
                view=query.get('view',['sensors' if query.get('sensors')==['1'] else 'full'])[0]
                snapshot=store.snapshot(view)
                self.send(200,json.dumps(snapshot,ensure_ascii=False,allow_nan=False,separators=(',',':')).encode(),compress=True)
            elif path.path == '/api/cloud':
                store.touch()
                try:
                    after = int(parse_qs(path.query).get('after',['-1'])[0])
                except ValueError:
                    self.send(400);return
                with store.lock:
                    body = store.cloud_bytes
                    unchanged = store.cloud_sequence == after
                self.send(204 if not body or unchanged else 200,b'' if not body or unchanged else body,'application/octet-stream')
            elif path.path == '/api/map':
                try: after = int(parse_qs(path.query).get('after',['-1'])[0])
                except ValueError: self.send(400); return
                body = store.tank.payload(after)
                self.send(200 if body else 204,body,'application/octet-stream',compress=True)
            elif path.path == '/api/recordings':
                from isv_water_trials.tank_record import recordings
                self.send(200,json.dumps(recordings(),ensure_ascii=False).encode(),compress=True)
            elif path.path == '/api/maps':
                status,response=store.tank.maps(query=parse_qs(path.query))
                self.send(status,json.dumps(response,ensure_ascii=False).encode())
            else:
                self.send(404)

        def do_POST(self):
            store.last_http = time.monotonic()
            path = urlsplit(self.path).path
            if path not in ('/api/select','/api/route','/api/initial_pose','/api/localize','/api/outdoor','/api/sensors','/api/maps'):
                self.send(404);return
            try:
                size = int(self.headers.get('Content-Length','0'))
                if not 0 < size <= (16384 if path == '/api/outdoor' else 1024):
                    raise ValueError('Bad body size')
                data = json.loads(self.rfile.read(size))
                if path == '/api/sensors':
                    if data == {'action':'stop_all'}:
                        status,response=store.request_watertrial_control('stop_other')
                        self.send(status,json.dumps(response,ensure_ascii=False).encode());return
                    if data.get('action') in ('stop_other','start_sensors','stop_watertrial'):
                        status,response=store.request_watertrial_control(data['action'])
                        self.send(status,json.dumps(response,ensure_ascii=False).encode());return
                    if data == {'action':'stop_all'}:
                        status,response=store.stop_all_nodes()
                        self.send(status,json.dumps(response,ensure_ascii=False).encode());return
                    if data == {'action':'initialize_esc'}:
                        status,response = store.initialize_esc()
                        self.send(status,json.dumps(response,ensure_ascii=False).encode());return
                    if data != {'action':'restart'}:raise ValueError('Invalid sensor action')
                    status,response = store.restart_sensors()
                    self.send(status,json.dumps(response,ensure_ascii=False).encode());return
                if path == '/api/outdoor':
                    status,response = store.outdoor.request(data)
                    self.send(status,json.dumps(response,ensure_ascii=False).encode());return
                if path == '/api/maps':
                    status,response = store.tank.maps(data=data)
                    self.send(status,json.dumps(response,ensure_ascii=False).encode());return
                if path != '/api/select':
                    status,response = store.tank.request(path,data)
                    self.send(status,json.dumps(response,ensure_ascii=False).encode());return
                topic = data['topic']
                with store.lock:
                    if topic not in {row['name'] for row in store.graph['topics']}:
                        raise ValueError('Unknown topic')
                    store.selected = topic
                    store.preview = None
                self.send(200,b'{"ok":true}')
            except (ValueError,TypeError,KeyError,json.JSONDecodeError):
                self.send(400,b'{"error":"invalid topic"}')
    return Handler


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port',type=int,default=0)
    parser.add_argument('--max-points',type=int,default=1800)
    parser.add_argument('--cloud-hz',type=float,default=3.)
    parser.add_argument('--max-range',type=float,default=100.)
    args = parser.parse_args()
    if not 500 <= args.max_points <= 30000 or not 1 <= args.cloud_hz <= 10 or not 0 < args.max_range <= 1000:
        parser.error('Invalid display limits')
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stop = threading.Event()
    signal.signal(signal.SIGINT,lambda *_:stop.set())
    if hasattr(signal,'SIGHUP'):
        signal.signal(signal.SIGHUP,signal.SIG_IGN)
    signal.signal(signal.SIGTERM,signal.SIG_IGN)
    store = Store(args);store.last_http=time.monotonic();node = make_node(store)
    server = ThreadingHTTPServer(('127.0.0.1',args.port),make_handler(store))
    threading.Thread(target=server.serve_forever,daemon=True).start()
    print(json.dumps({'ready_port':server.server_port,'pid':os.getpid(),'version':VERSION}),flush=True)
    try:
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node,timeout_sec=.05)
            if time.monotonic()-store.last_http>90:break
    finally:
        server.shutdown();server.server_close();node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
