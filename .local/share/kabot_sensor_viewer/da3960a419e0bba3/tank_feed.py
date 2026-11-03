"""Map display and route/initial-pose editing. This module has no actuator API."""
from concurrent.futures import Future, TimeoutError
import json
import math
from pathlib import Path
from queue import SimpleQueue, Empty
import struct
import threading
import time
import uuid


class TankFeed:
    def __init__(self, node, store):
        from geometry_msgs.msg import PoseWithCovarianceStamped
        self.node, self.store = node, store
        self.pose_type = PoseWithCovarianceStamped
        self.publisher = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        self.pending = SimpleQueue(); self.lock = threading.RLock()
        self.grid = None; self.body = b''; self.sequence = 0
        self.stream_id = uuid.uuid4().hex
        self.last_geometry = None; self.last_cells = None
        self.route_checked = 0.; self.route_mtime = None
        node.create_timer(.1, self.work)

    def map(self, message):
        from telemetry_core import quaternion_angles
        info = message.info; count = info.width*info.height
        if not 0<count<=8_000_000 or len(message.data)!=count: return
        if not .005<=info.resolution<=1: return
        angle = quaternion_angles(info.origin.orientation)
        metadata = {'width':info.width,'height':info.height,'resolution_m':info.resolution,
                    'origin_x_m':info.origin.position.x,'origin_y_m':info.origin.position.y,
                    'origin_yaw_deg':angle['deg_signed']['yaw'] if angle else 0.,
                    'frame_id':message.header.frame_id}
        with self.lock:
            cells=message.data.tobytes()
            if metadata!=self.last_geometry or cells!=self.last_cells:
                self.sequence += 1
                self.body = struct.pack('<4sIII',b'KMAP',self.sequence,info.width,info.height)+cells
                self.last_geometry=dict(metadata);self.last_cells=cells
            self.grid = message
            metadata['sequence'] = self.sequence
            metadata['stream_id'] = self.stream_id
        self.store.value('map',metadata)

    def payload(self, after):
        with self.lock:
            return b'' if after==self.sequence else self.body

    def request(self, path, data):
        future = Future(); self.pending.put((path,data,future))
        try: return future.result(timeout=2.)
        except TimeoutError:
            future.cancel(); return 503,{'error':'ROS 응답 대기 시간이 지났습니다.'}

    def work(self):
        if time.monotonic()-getattr(self,'route_checked',0)>1:
            self.route_checked = time.monotonic()
            target = Path.home()/'water_trials/tank/route.json'
            try:
                changed = target.stat().st_mtime
                if changed!=getattr(self,'route_mtime',None):
                    route = json.loads(target.read_text(encoding='utf-8'))
                    self.store.value('tank_route',route); self.route_mtime = changed
            except (OSError,ValueError): pass
        for _ in range(4):
            try: path,data,future = self.pending.get_nowait()
            except Empty: return
            if future.cancelled(): continue
            try:
                point = lambda key: tuple(float(data[key][v]) for v in ('x','y'))
                if path=='/api/route':
                    from isv_water_trials.tank_core import Grid
                    if self.grid is None: raise ValueError('지도를 먼저 만들어 주세요.')
                    grid = Grid(self.grid,.75); a,b = point('a'),point('b')
                    if math.dist(a,b)<1: raise ValueError('A와 B는 1m 이상 떨어뜨려 주세요.')
                    grid.path(a,b)
                    target = Path.home()/'water_trials/tank/route.json'; target.parent.mkdir(parents=True,exist_ok=True)
                    with self.store.lock: session = dict(self.store.values.get('tank_session',{}))
                    if not session.get('session_id') or time.monotonic()-session.get('received_monotonic',0)>3:
                        raise ValueError('지도 세션 수신 대기 중입니다.')
                    route = {'frame_id':grid.frame,'session_id':session['session_id'],'a':dict(zip(('x','y'),a)),'b':dict(zip(('x','y'),b)),
                             'clearance_m':.75,'map_resolution_m':grid.resolution}
                    from isv_water_trials.tank_storage import atomic_json
                    atomic_json(target,route)
                    self.store.value('tank_route',route)
                    response = {'ok':True,'message':'A·B 저장 완료. 왕복 노드를 실행하면 이 경로를 읽습니다.'}
                elif path=='/api/initial_pose':
                    if self.grid is None: raise ValueError('지도 수신 대기 중입니다.')
                    with self.store.lock: status = dict(self.store.values.get('tank_status',{}))
                    if status.get('execute') and time.monotonic()-status.get('received_monotonic',0)<2:
                        raise ValueError('왕복 노드를 Ctrl+C로 종료한 뒤 현재 위치를 설정해 주세요.')
                    consumers = self.node.get_subscriptions_info_by_topic('/initialpose')
                    if not any(c.node_name=='slam_toolbox' for c in consumers):
                        raise ValueError('저장 지도의 localization 모드에서 현재 위치를 설정해 주세요.')
                    x,y,angle = (float(data[k]) for k in ('x','y','yaw_deg'))
                    if not all(math.isfinite(v) for v in (x,y,angle)): raise ValueError('위치·방향 값을 확인해 주세요.')
                    msg = self.pose_type(); msg.header.frame_id = self.grid.header.frame_id
                    msg.header.stamp = self.node.get_clock().now().to_msg()
                    msg.pose.pose.position.x = x; msg.pose.pose.position.y = y
                    msg.pose.pose.orientation.z = math.sin(math.radians(angle)/2)
                    msg.pose.pose.orientation.w = math.cos(math.radians(angle)/2)
                    msg.pose.covariance[0] = msg.pose.covariance[7] = .25
                    msg.pose.covariance[35] = math.radians(15)**2
                    self.publisher.publish(msg)
                    response = {'ok':True,'message':'초기 위치·방향 발행 완료. 지도 위 실제 위치와 비교해 주세요.'}
                else: raise ValueError('Unknown map operation')
                if not future.cancelled(): future.set_result((200,response))
            except (ValueError,KeyError,TypeError,ImportError,OSError) as exc:
                if not future.cancelled(): future.set_result((400,{'error':str(exc)}))
