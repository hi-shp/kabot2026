"""Bounded browser requests -> ROS owner; acknowledgements come from the owner."""
from concurrent.futures import Future, TimeoutError
from queue import Queue, Empty, Full
import json
import math
import time
import uuid


class OutdoorFeed:
    def __init__(self, node, store):
        from std_msgs.msg import String
        self.kind = String; self.node = node; self.store = store
        self.publisher = node.create_publisher(String, '/outdoor/request', 10)
        self.hull_publisher = node.create_publisher(String, '/lidar/self_mask/request', 10)
        from sensor_msgs.msg import NavSatFix
        self.gps_kind = NavSatFix
        self.virtual_publisher = node.create_publisher(NavSatFix, '/gps/virtual/fix', 10)
        self.virtual_point = None
        self.virtual_sequence = 0
        node.create_timer(.2, self.publish_virtual_gps)
        self.virtual_status()
        self.queue = Queue(maxsize=16); node.create_timer(.1, self.work)

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
        if virtual and data['action']=='virtual_gps_start':
            point=data.get('point')
            if (not isinstance(point,list) or len(point)!=2 or
                any(type(v) not in (int,float) or not math.isfinite(v) for v in point) or
                not -90<=point[0]<=90 or not -180<=point[1]<=180):
                return 400, {'error':'지도에서 가상 GPS 위치를 선택하세요'}
        if not virtual and data.get('action') not in ('pause','resume','apply_model','freeze','load','route','regions','start','stop','clear_trail','start_model_test','hull_start','hull_stop','hull_save','hull_disable'):
            return 400, {'error': '알 수 없는 요청'}
        if not virtual:
            with self.store.lock:
                status = self.store.values.get('hull_mask' if data['action'].startswith('hull_') else 'outdoor', {})
                if time.monotonic()-status.get('received_monotonic', 0) > 3:
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
            self.publish_virtual_gps()
            future.set_result((200,{'request_id':data['request_id'],'simulation':True,
                                   'active':self.virtual_point is not None,'topic':'/gps/virtual/fix'}))
            return
        publisher=self.hull_publisher if data['action'].startswith('hull_') else self.publisher
        if publisher.get_subscription_count() < 1:
            future.set_result((503, {'error': '야외 노드 연결 대기'})); return
        publisher.publish(self.kind(data=json.dumps(data, allow_nan=False)))
        future.set_result((202, {'request_id': data['request_id'], 'pending': True}))
