"""Existing Windows GUI bridge for map data and mapping-only ROS commands.

Only stdlib is imported until attached to a ROS node; pyproj stays in mapper venv.
"""
from copy import deepcopy
import json
import math
import threading
import time
import uuid
from urllib.parse import urlsplit

ACTIONS={'map_output_prepare','map_drive_start','map_dry_run_start','map_dry_run_stop','set_gps_target','save_gps_map','load_gps_map','start','pause','resume','stop','save','load','reset','select_roi',
         'set_hopping','delete_hopping','save_hopping','auto_start','auto_stop','auto_emergency'}


class MappingBridge:
    def __init__(self,node=None,controls_enabled=False,transport=None):
        self.controls_enabled=controls_enabled
        self.lock=threading.Lock();self.gate=threading.Lock()
        self.value=None;self.received=-math.inf
        self.transport=transport
        self.client=None
        if node is not None:
            from std_msgs.msg import String
            from rcl_interfaces.srv import SetParametersAtomically
            self.subscription=node.create_subscription(String,'/docking/map_preview',lambda m:self.update(m.data),1)
            if controls_enabled:
                self.client=node.create_client(SetParametersAtomically,'/docking/mapping/gui_command')
                self.transport=self.ros_send

    def update(self,raw):
        try:
            if len(raw)>1500000:return
            value=json.loads(raw)
            if not isinstance(value,dict) or value.get('preview_schema')!=1 or not isinstance(value.get('status'),dict):return
            for key,limit in [('map_points',6000),('current_scan',1000),('rc_path',1000)]:
                points=value.get(key)
                if not isinstance(points,list) or len(points)>limit:return
                if any(not isinstance(p,list) or len(p)!=2 or any(isinstance(x,bool) or not isinstance(x,(int,float))
                    or not math.isfinite(x) for x in p) or abs(p[0])>90 or abs(p[1])>180 for p in points):return
            for key,limit in [('map_points_xyz',6000),('current_scan_xyz',1000),('rc_path_xyz',1000)]:
                points=value.get(key,[])
                if not isinstance(points,list) or len(points)>limit:return
                if any(not isinstance(p,list) or len(p)!=3 or any(isinstance(x,bool) or not isinstance(x,(int,float)) or not math.isfinite(x) for x in p) for p in points):return
            with self.lock:self.value=value;self.received=time.monotonic()
        except (ValueError,TypeError):return

    def snapshot(self):
        with self.lock:
            value=deepcopy(self.value) if self.value is not None else dict(status=dict(state='WAIT_MAPPING_NODE'),
                map_points=[],current_scan=[],rc_path=[],origin_lla=None,hopping=None)
            age=time.monotonic()-self.received
        connected=math.isfinite(age) and age<3.
        value.update(connected=connected,receive_age_s=age if math.isfinite(age) else None,
                     controls_enabled=bool(connected and self.controls_enabled and self.transport is not None))
        return value

    def ros_send(self,payload):
        from rcl_interfaces.srv import SetParametersAtomically
        from rcl_interfaces.msg import Parameter,ParameterValue
        if self.client is None or not self.client.service_is_ready():raise RuntimeError('매핑 서비스 수신 대기')
        request=SetParametersAtomically.Request(parameters=[Parameter(name='ui_request',value=ParameterValue(
            type=4,string_value=json.dumps(payload,allow_nan=False)))])
        future=self.client.call_async(request);done=threading.Event()
        future.add_done_callback(lambda _:done.set())
        if not done.wait(5):
            future.cancel();raise TimeoutError('매핑 접수 응답 지연: 상태를 확인하세요. 자동 재전송하지 않습니다.')
        result=future.result().result
        return dict(success=result.successful,message=result.reason)

    def send(self,payload):
        if not isinstance(payload,dict) or payload.get('action') not in ACTIONS:raise ValueError('지원하지 않는 매핑 명령')
        if not self.snapshot()['controls_enabled']:raise PermissionError('매핑 연결 또는 편집 권한을 확인하세요.')
        acquired=self.gate.acquire(blocking=False)
        if not acquired and payload['action'] not in ('auto_stop','auto_emergency','map_dry_run_stop'):
            raise RuntimeError('다른 매핑 요청 처리 중')
        try:
            request=dict(payload,request_id=uuid.uuid4().hex)
            result=self.transport(request)
            return dict(result,request_id=request['request_id'])
        finally:
            if acquired:self.gate.release()

    def http_command(self,handler,port):
        host=handler.headers.get('Host','')
        try:
            authority=urlsplit('http://'+host)
            loopback=(authority.hostname in ('127.0.0.1','localhost') and authority.port is not None
                      and 0<authority.port<=65535 and not authority.username and not authority.password
                      and not authority.path and not authority.query and not authority.fragment)
        except ValueError:loopback=False
        # SSH may forward local :18871 to server :8771. Keep exact same-origin
        # matching while accepting loopback authorities at the forwarded port.
        if (not loopback or handler.headers.get('Origin')!='http://'+host
                or handler.headers.get('Content-Type','').split(';')[0]!='application/json'):
            handler.send(403,b'{"error":"same-origin JSON required"}');return
        try:
            size=int(handler.headers.get('Content-Length','0'))
            if not 0<size<=8192:raise ValueError('Invalid mapping request size')
            result=self.send(json.loads(handler.rfile.read(size)))
            handler.send(200,json.dumps(result,allow_nan=False).encode())
        except PermissionError as exc:handler.send(403,json.dumps({'error':str(exc)}).encode())
        except (ValueError,TypeError) as exc:handler.send(400,json.dumps({'error':str(exc)}).encode())
        except (RuntimeError,TimeoutError) as exc:handler.send(503,json.dumps({'error':str(exc)}).encode())
