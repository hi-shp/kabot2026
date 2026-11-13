"""Bounded browser requests -> ROS owner; acknowledgements come from the owner."""
from concurrent.futures import Future, TimeoutError
from queue import Queue, Empty, Full
import json
import math
import time
import uuid
import os
from pathlib import Path


def validated_route(data):
    points=data.get('points')
    if (not isinstance(points,list) or not 1<=len(points)<=100 or
        any(not isinstance(p,list) or len(p)!=2 or
            any(type(v) not in (int,float) or not math.isfinite(v) for v in p) or
            not -85<p[0]<85 or not -180<=p[1]<=180 for p in points)):
        raise ValueError('위도·경도로 1~100개의 지점을 지정하세요')
    for a,b in zip(points,points[1:]):
        east=math.radians(b[1]-a[1])*6371000*math.cos(math.radians(a[0]))
        north=math.radians(b[0]-a[0])*6371000
        if not 1<=math.hypot(east,north)<=200:
            raise ValueError('이웃한 지점의 거리는 1~200m로 지정하세요')
    return {'points':[list(p) for p in points]}


class OutdoorFeed:
    def __init__(self, node, store, route_path=None):
        from std_msgs.msg import String
        self.kind = String; self.node = node; self.store = store
        self.route_path=Path(route_path) if route_path is not None else Path.home()/'water_trials/outdoor/route.json'
        self.publisher = node.create_publisher(String, '/outdoor/request', 10)
        self.hull_publisher = node.create_publisher(String, '/lidar/self_mask/request', 10)
        from sensor_msgs.msg import NavSatFix
        self.gps_kind = NavSatFix
        self.virtual_publisher = node.create_publisher(NavSatFix, '/gps/virtual/fix', 10)
        self.virtual_point = None
        self.virtual_sequence = 0
        node.create_timer(.2, self.publish_virtual_gps)
        self.virtual_status()
        self.route_status()
        node.create_timer(.5,self.route_status)
        self.queue = Queue(maxsize=16); node.create_timer(.1, self.work)

    def route_status(self):
        route=None;error=None
        try:route=validated_route(json.loads(self.route_path.read_text(encoding='utf-8')))
        except FileNotFoundError:pass
        except (OSError,ValueError,TypeError,AttributeError) as exc:error=str(exc)
        self.store.value('saved_route',dict(route=route,error=error,available=True,path=str(self.route_path)))

    def save_without_owner(self,data):
        """Persist the same route file while no navigation owner holds its lock."""
        import fcntl
        try:
            route=validated_route(data)
            self.route_path.parent.mkdir(parents=True,exist_ok=True)
            with (self.route_path.parent/'owner.lock').open('a') as owner_lock:
                try:fcntl.flock(owner_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
                except BlockingIOError:return 503,{'error':'주행 노드 전환 중입니다. 잠시 후 다시 저장하세요'}
                if self.publisher.get_subscription_count():return 503,{'error':'주행 노드 연결 중입니다. 잠시 후 다시 저장하세요'}
                temporary=self.route_path.with_name('.route-'+uuid.uuid4().hex+'.incoming')
                try:
                    with temporary.open('w',encoding='utf-8') as stream:
                        json.dump(route,stream,ensure_ascii=False,allow_nan=False);stream.flush();os.fsync(stream.fileno())
                    os.replace(temporary,self.route_path)
                finally:
                    if temporary.exists():temporary.unlink()
            self.route_status()
            return 200,dict(saved=True,route=route,path=str(self.route_path),navigation_running=False)
        except (ValueError,TypeError,AttributeError) as exc:return 400,{'error':str(exc)}
        except OSError as exc:return 500,{'error':'경로 파일 저장 실패: '+str(exc)}

    def virtual_status(self):
        self.store.value('virtual_gps',dict(active=self.virtual_point is not None,
                         point=self.virtual_point,topic='/gps/virtual/fix',rate_hz=5,
                         published=self.virtual_sequence))

    def publish_virtual_gps(self):
        if self.virtual_point is not None:
            msg = self.gps_kind()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = 'virtual_gps'
            msg.latitude, msg.longitude = self.virtual_point
            msg.altitude = 0.
            msg.status.status = 0  # NavSatStatus.STATUS_FIX; synthetic source is explicit in topic/frame.
            msg.status.service = 1
            msg.position_covariance = [.25,0.,0.,0.,.25,0.,0.,0.,1.]
            msg.position_covariance_type = 2
            self.virtual_publisher.publish(msg)
            self.virtual_sequence += 1
        self.virtual_status()

    def request(self, data):
        virtual = data.get('action') in ('virtual_gps_start','virtual_gps_stop')
        if data.get('action') in ('start','start_model_test') and self.virtual_point is not None:
            return 409, {'error':'가상 GPS를 종료하고 실제 배 위치로 주행을 시작하세요'}
        if data.get('action')=='virtual_gps_start':
            with self.store.lock:
                status=self.store.values.get('outdoor',{})
                virtual_follow=(status.get('navigation_mode')=='point_follow' and status.get('point_follow_position_source')=='virtual')
                if status.get('execute') and not virtual_follow and time.monotonic()-status.get('received_monotonic',0)<3:
                    return 409, {'error':'주행 정지 후 가상 GPS를 사용할 수 있습니다'}
        if virtual and data['action']=='virtual_gps_start':
            point=data.get('point')
            if (not isinstance(point,list) or len(point)!=2 or
                any(type(v) not in (int,float) or not math.isfinite(v) for v in point) or
                not -90<=point[0]<=90 or not -180<=point[1]<=180):
                return 400, {'error':'지도에서 가상 GPS 위치를 선택하세요'}
        if not virtual and data.get('action') not in ('pause','resume','apply_model','freeze','load','route','regions','start','stop','clear_trail','start_model_test','start_point_follow','point_follow_limit','hull_start','hull_stop','hull_save','hull_disable'):
            return 400, {'error': '알 수 없는 요청'}
        if not virtual:
            with self.store.lock:
                status = self.store.values.get('hull_mask' if data['action'].startswith('hull_') else 'outdoor', {})
                stale=time.monotonic()-status.get('received_monotonic', 0)>3
            if stale:
                if data['action']=='route' and self.publisher.get_subscription_count()==0:
                    return self.save_without_owner(data)
                return 503, {'error': '브링업의 야외 기록 노드 연결 대기'}
        request_id = uuid.uuid4().hex
        future = Future()
        try: self.queue.put_nowait((dict(data, request_id=request_id), future))
        except Full: return 429, {'error': '요청 처리 대기 중'}
        try: return future.result(timeout=2)
        except TimeoutError:
            future.cancel(); return 503, {'error': '요청 전송 시간 초과'}

    def work(self):
        try: data, future = self.queue.get_nowait()
        except Empty: return
        if not future.set_running_or_notify_cancel(): return
        if data['action'] in ('virtual_gps_start','virtual_gps_stop'):
            self.virtual_point = list(data['point']) if data['action']=='virtual_gps_start' else None
            if self.virtual_point is None:
                msg=self.gps_kind();msg.header.stamp=self.node.get_clock().now().to_msg()
                msg.header.frame_id='virtual_gps';msg.status.status=-1
                self.virtual_publisher.publish(msg)
            self.publish_virtual_gps()
            future.set_result((200,{'request_id':data['request_id'],'simulation':True,
                                   'active':self.virtual_point is not None,'topic':'/gps/virtual/fix'}))
            return
        publisher=self.hull_publisher if data['action'].startswith('hull_') else self.publisher
        if data['action']=='start_point_follow':
            data['position_source']='virtual' if self.virtual_point is not None else 'gps'
        if publisher.get_subscription_count() < 1:
            future.set_result((503, {'error': '야외 노드 연결 대기'})); return
        publisher.publish(self.kind(data=json.dumps(data, allow_nan=False)))
        future.set_result((202, {'request_id': data['request_id'], 'pending': True}))
