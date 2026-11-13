"""Windows mission dashboard over an SSH localhost tunnel.

Calls only the mission's guarded GUI service, never direct actuator services.
The native mission GUI can continue running independently on the Jetson.
"""
import argparse
import json
from pathlib import Path
import time
import threading
import os
from viewer_pointcloud import PointCloudBuffer

SETTINGS_ACTIONS = ('set_route', 'capture_wp', 'set_target')
GUI_ACTIONS = SETTINGS_ACTIONS + ('start', 'stop', 'reset')
VIEWER_PROTOCOL = 3


def reuse_existing(port, read_only, watch=True):
    """Keep the new SSH tunnel alive while reusing this exact viewer service."""
    import urllib.request
    import urllib.error
    url=f'http://127.0.0.1:{port}/api/viewer'
    try:
        with urllib.request.urlopen(url,timeout=2) as response:info=json.load(response)
    except urllib.error.HTTPError as error:
        raise RuntimeError(f'포트 {port}의 기존 프로그램이 이전 버전이거나 다른 서버입니다. 기존 보기 창을 종료하세요.') from error
    except TimeoutError as error:
        raise RuntimeError(f'포트 {port} 응답 시간 초과. SSH 포트 연결 명령은 젯슨 터미널이 아니라 Windows PowerShell에서 실행하세요.') from error
    except urllib.error.URLError:
        return False
    if info.get('app')!='isv_windows_dashboard' or info.get('protocol')!=VIEWER_PROTOCOL:
        raise RuntimeError(f'포트 {port}을 다른 프로그램이 사용 중입니다. 해당 프로그램을 자동 종료하지 않습니다.')
    if info.get('read_only')!=read_only:
        raise RuntimeError('기존 보기 서버의 조작/읽기 전용 모드가 다릅니다. 기존 보기 창을 먼저 종료하세요.')
    print(f'WINDOWS GUI READY (existing PID {info["pid"]}): http://127.0.0.1:{port}',flush=True)
    if not watch:return True
    try:
        while True:
            time.sleep(2)
            try:
                with urllib.request.urlopen(url,timeout=2) as response:current=json.load(response)
                if current.get('pid')!=info['pid']:break
            except (urllib.error.URLError,ValueError,TimeoutError):break
    except KeyboardInterrupt:pass
    return True


def create_dashboard(port, apply_settings=None):
    from isv_shape_navigation.mission_dashboard import Dashboard
    dashboard=Dashboard(Path(__file__).resolve().parent/'windows_view_assets',port)
    dashboard.pointcloud=PointCloudBuffer()
    original_get=dashboard.server.RequestHandlerClass.do_GET
    def viewer_get(handler):
        if handler.path=='/api/viewer':
            handler.send(200,json.dumps(dict(app='isv_windows_dashboard',protocol=VIEWER_PROTOCOL,
                pid=os.getpid(),read_only=apply_settings is None)).encode())
        elif handler.path=='/api/pointcloud':
            handler.send(200,json.dumps(dashboard.pointcloud.snapshot(),allow_nan=False).encode())
        elif handler.path=='/lidar3d.js':
            handler.send(200,(Path(__file__).resolve().parent/'windows_view_assets/lidar3d.js').read_bytes(),
                'application/javascript; charset=utf-8')
        else:original_get(handler)
    dashboard.server.RequestHandlerClass.do_GET=viewer_get
    gate = threading.Lock()
    def settings_command(handler):
        host=handler.headers.get('Host','')
        if (apply_settings is None or handler.path!='/api/command'
                or host not in {f'127.0.0.1:{dashboard.port}',f'localhost:{dashboard.port}'}
                or handler.headers.get('Origin')!='http://'+host
                or handler.headers.get('Content-Type','').split(';')[0]!='application/json'):
            handler.send(403,b'{"error":"Only same-origin mission settings are accepted"}');return
        try:
            size=int(handler.headers.get('Content-Length','0'))
            if not 0<size<=8192:raise ValueError('Invalid request size')
            command=json.loads(handler.rfile.read(size))
            if not isinstance(command,dict) or command.get('action') not in GUI_ACTIONS:
                handler.send(403,b'{"error":"Direct actuator commands are not available here"}');return
        except (ValueError,TypeError):
            handler.send(400,b'{"error":"Invalid settings request"}');return
        with dashboard.lock:
            stale=not hasattr(dashboard,'updated') or time.monotonic()-dashboard.updated>1.5
            busy=dashboard.report.get('output',{}).get('active',False)
        if command['action']!='stop' and (stale or (busy and command['action']!='stop')):
            handler.send(409,json.dumps({'error':'최신 임무 상태와 정지 확인이 필요합니다.'}).encode());return
        if not gate.acquire(blocking=False):
            handler.send(409,b'{"error":"Another settings request is in progress"}');return
        try:
            result=apply_settings(command)
            with dashboard.lock:dashboard.last_command=dict(action=command['action'],**result)
            handler.send(200,json.dumps(result,allow_nan=False).encode())
        except (RuntimeError,ValueError,TimeoutError) as error:
            handler.send(503,json.dumps({'error':str(error)}).encode())
        finally:gate.release()
    dashboard.server.RequestHandlerClass.do_POST=settings_command
    return dashboard


def self_test():
    import urllib.request
    import urllib.error
    dashboard=create_dashboard(0)
    url='http://127.0.0.1:'+str(dashboard.port)
    try:
        from unittest.mock import patch
        with patch('urllib.request.urlopen',side_effect=TimeoutError('test')):
            try:reuse_existing(12345,False,watch=False);raise AssertionError('A timeout must provide a clear error')
            except RuntimeError as error:assert 'Windows PowerShell' in str(error)
        assert dashboard.server.server_address[0]=='127.0.0.1'
        assert reuse_existing(dashboard.port,True,watch=False)
        assert 'Windows' in urllib.request.urlopen(url).read().decode()
        assert b'SatelliteMap' in urllib.request.urlopen(url+'/app.js').read()
        assert json.loads(urllib.request.urlopen(url+'/api/state').read())['state']=='WAIT_NODE'
        req=urllib.request.Request(url+'/api/command',data=b'{"action":"start"}',
            headers={'Content-Type':'application/json','Origin':url})
        try:urllib.request.urlopen(req);raise AssertionError('Control POST must be rejected')
        except urllib.error.HTTPError as exc:assert exc.code==403
        assert dashboard.commands.empty()
        assert urllib.request.urlopen(url+'/camera.jpg').status==204
        print('READ_ONLY_MODE_OK: localhost only; GET works; POST rejected; no command queued')
    finally:dashboard.close()
    calls=[]
    dashboard=create_dashboard(0,lambda cmd:(calls.append(cmd) or {'success':True,'message':'test accepted'}))
    url='http://127.0.0.1:'+str(dashboard.port)
    def post(data,origin=None):
        req=urllib.request.Request(url+'/api/command',data=json.dumps(data).encode(),
            headers={'Content-Type':'application/json','Origin':origin or url})
        try:return urllib.request.urlopen(req).status
        except urllib.error.HTTPError as error:return error.code
    try:
        assert post({'action':'set_target','target_label':'red triangle'})==409  # No fresh controller.
        dashboard.publish({'stamp':0,'state':'WAIT_GPS','output':{'active':False},'intended_pwm':[1500,1500]})
        assert post({'action':'set_target','target_label':'red triangle'})==200
        assert len(calls)==1
        for action in ('arm','pwm','throttle'):
            assert post({'action':action})==403
        assert post({'action':'set_target'},'https://untrusted.example')==403
        dashboard.report['output']['active']=True
        assert post({'action':'set_target'})==409
        assert post({'action':'stop'})==200
        assert post({'action':'start'})==409
        assert len(calls)==2 and dashboard.commands.empty()
        print('GUI_HTTP_OK: stopped settings required; stop remains available; direct PWM/cross-origin rejected')
    finally:dashboard.close()


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--port',type=int,default=8771)
    parser.add_argument('--self-test',action='store_true')
    parser.add_argument('--read-only',action='store_true',help='Disable all setting changes too')
    parser.add_argument('--reuse-existing',action='store_true',help='Reuse an already running viewer and keep this SSH tunnel alive')
    args=parser.parse_args()
    if args.self_test:self_test();return
    if args.reuse_existing:
        try:
            if reuse_existing(args.port,args.read_only):return
        except (RuntimeError,ValueError) as error:
            parser.exit(2,'GUI 연결 확인 실패: '+str(error)+'\n')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
    from sensor_msgs.msg import Image, PointCloud2
    from cv_bridge import CvBridge
    import cv2
    from rcl_interfaces.srv import SetParametersAtomically
    from rcl_interfaces.msg import Parameter, ParameterValue
    rclpy.init();node=rclpy.create_node('isv_windows_settings_viewer')
    client=None if args.read_only else node.create_client(SetParametersAtomically,'/course/gui_command')
    def apply_settings(command):
        if client is None or not client.service_is_ready():
            raise RuntimeError('설정 서비스가 없습니다. 업데이트된 임무 프로그램을 다시 실행하세요.')
        request=SetParametersAtomically.Request(parameters=[Parameter(name='ui_request',
            value=ParameterValue(type=4,string_value=json.dumps(command,allow_nan=False)))])
        future=client.call_async(request);done=threading.Event()
        future.add_done_callback(lambda _:done.set())
        if not done.wait(5):
            future.cancel()
            raise TimeoutError('적용 응답이 지연됐습니다. 현재 설정을 확인하세요. 자동 재전송하지 않습니다.')
        result=future.result().result
        return {'success':result.successful,'message':result.reason}
    dashboard=create_dashboard(args.port,None if args.read_only else apply_settings)
    bridge=CvBridge();last_frame=-float('inf')
    def status(message):
        try:
            report=json.loads(message.data)
            report['viewer_only']=args.read_only
            report['settings_available']=bool(client and client.service_is_ready())
            dashboard.publish(report)
        except (ValueError,KeyError,TypeError):pass
    def camera(message):
        nonlocal last_frame
        now=time.monotonic()
        if now-last_frame<.35:return
        try:
            image=bridge.imgmsg_to_cv2(message,desired_encoding='bgr8')
            height,width=image.shape[:2]
            image=cv2.resize(image,(480,round(height*480/width)))
            ok,jpeg=cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,70])
            if ok:dashboard.camera(jpeg.tobytes());last_frame=now
        except (ValueError,RuntimeError,cv2.error):pass
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    cloud_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE)
    subscriptions=[node.create_subscription(PointCloud2,'/velodyne_points',dashboard.pointcloud.update,cloud_qos),
                   node.create_subscription(String,'/course/status',status,10),
                   node.create_subscription(Image,'/camera/image_raw',camera,qos_profile_sensor_data)]
    print('WINDOWS SETTINGS VIEW READY: http://127.0.0.1:'+str(dashboard.port),flush=True)
    print('Use an SSH local-port tunnel. Mission GUI commands only; no direct actuator commands.',flush=True)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        dashboard.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
