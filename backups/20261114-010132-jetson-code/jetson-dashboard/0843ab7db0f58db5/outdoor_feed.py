"""Bounded browser requests -> ROS owner; acknowledgements come from the owner."""
from concurrent.futures import Future, TimeoutError
from queue import Queue, Empty, Full
import json
import time
import uuid


class OutdoorFeed:
    def __init__(self, node, store):
        from std_msgs.msg import String
        self.kind = String; self.node = node; self.store = store
        self.publisher = node.create_publisher(String, '/outdoor/request', 10)
        self.queue = Queue(maxsize=16); node.create_timer(.1, self.work)

    def request(self, data):
        if data.get('action') not in ('pause','resume','apply_model','freeze','load','route','regions','start','stop','clear_trail','start_model_test'):
            return 400, {'error': '알 수 없는 요청'}
        with self.store.lock:
            status = self.store.values.get('outdoor', {})
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
        if self.publisher.get_subscription_count() < 1:
            future.set_result((503, {'error': '야외 노드 연결 대기'})); return
        self.publisher.publish(self.kind(data=json.dumps(data, allow_nan=False)))
        future.set_result((202, {'request_id': data['request_id'], 'pending': True}))
