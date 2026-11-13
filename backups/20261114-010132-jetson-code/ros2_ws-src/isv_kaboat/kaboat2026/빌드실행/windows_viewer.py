"""Windows mission dashboard over an SSH localhost tunnel.

Calls only the mission's guarded GUI service, never direct actuator services.
The native mission GUI can continue running independently on the Jetson.
"""
import argparse
import json
import math
from pathlib import Path
import time
import threading
import os
from viewer_pointcloud import PointCloudBuffer, attach_navigation_alignment
from viewer_heading import HeadingPreview

SETTINGS_ACTIONS = ('set_route', 'capture_wp', 'set_target', 'set_speed')
GUI_ACTIONS = SETTINGS_ACTIONS + ('start', 'stop', 'reset')
VIEWER_PROTOCOL = 4


def reuse_existing(port, read_only, watch=True,mapping_controls=False):
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
    if info.get('mapping_controls',False)!=mapping_controls:
        raise RuntimeError('기존 GUI의 도킹 맵 편집 설정이 다릅니다. 기존 보기 프로세스를 재시작하세요.')
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


def create_dashboard(port, apply_settings=None,docking=None,bringup=None,runtime=None):
    from isv_shape_navigation.mission_dashboard import Dashboard
    dashboard=Dashboard(Path(__file__).resolve().parent/'windows_view_assets',port)
    dashboard.pointcloud=PointCloudBuffer()
    dashboard.mapping_pointcloud=PointCloudBuffer(max_range=2.0,range_mode="horizontal",front_only=True)
    dashboard.gap_observer=None
    dashboard.docking=docking
    original_get=dashboard.server.RequestHandlerClass.do_GET
    def viewer_get(handler):
        asset_path=handler.path.split('?',1)[0]
        if asset_path in ('/','/app.js','/style.css','/lidar3d.js','/docking_map.js','/docking_map.css','/bringup_controls.js','/runtime_controls.js'):
            handler.path=asset_path
        if handler.path=='/api/viewer':
            handler.send(200,json.dumps(dict(app='isv_windows_dashboard',protocol=VIEWER_PROTOCOL,
                pid=os.getpid(),read_only=apply_settings is None,
                mapping_controls=bool(docking and docking.controls_enabled),bringup_controls=bringup is not None)).encode())
        elif handler.path=='/api/runtime':
            handler.send(200,json.dumps(runtime.snapshot() if runtime else {},allow_nan=False).encode())
        elif handler.path=='/api/bringup':
            value=bringup.snapshot() if bringup else dict(state='DISABLED',can_start=False,can_stop=False)
            handler.send(200,json.dumps(value,allow_nan=False).encode())
        elif handler.path=='/api/docking':
            value=docking.snapshot() if docking else dict(connected=False,controls_enabled=False,status=dict(state='WAIT_MAPPING_NODE'))
            handler.send(200,json.dumps(value,allow_nan=False).encode())
        elif handler.path=='/api/state':
            with dashboard.lock:
                value=dict(dashboard.report,history=list(dashboard.history),events=list(dashboard.events),
                    last_command=dashboard.last_command,
                    report_age=time.monotonic()-dashboard.updated if hasattr(dashboard,'updated') else None,
                    camera_age=time.monotonic()-dashboard.jpeg_at if dashboard.jpeg else None)
            if dashboard.gap_observer is not None and (value['report_age'] is None or value['report_age']>1.5):
                value['gap_preview']=dashboard.gap_observer.snapshot()
            handler.send(200,json.dumps(value,allow_nan=False).encode())
        elif handler.path=='/api/mapping_pointcloud':
            handler.send(200,json.dumps(dashboard.mapping_pointcloud.snapshot(),allow_nan=False).encode())
        elif handler.path=='/api/pointcloud':
            value=dashboard.pointcloud.snapshot()
            if dashboard.gap_observer is not None and not value['body_from_cloud']['valid']:
                value['body_from_cloud']=dashboard.gap_observer.alignment()
            handler.send(200,json.dumps(value,allow_nan=False).encode())
        elif handler.path in ('/lidar3d.js','/docking_map.js','/docking_map.css','/bringup_controls.js','/runtime_controls.js'):
            kind='text/css' if handler.path.endswith('.css') else 'application/javascript'
            handler.send(200,(Path(__file__).resolve().parent/'windows_view_assets'/handler.path[1:]).read_bytes(),kind+'; charset=utf-8')
        else:original_get(handler)
    dashboard.server.RequestHandlerClass.do_GET=viewer_get
    gate = threading.Lock()
    def settings_command(handler):
        if handler.path=='/api/runtime/command':
            if runtime is None:handler.send(403,b'{"error":"Runtime controls disabled"}')
            else:runtime.http_command(handler)
            return
        if handler.path=='/api/bringup/command':
            if bringup is None:handler.send(403,b'{"error":"Bringup controls disabled"}')
            else:bringup.http_command(handler)
            return
        if handler.path=='/api/docking/command':
            if docking is None:handler.send(403,b'{"error":"Mapping controls disabled"}')
            else:docking.http_command(handler,dashboard.port)
            return
        host=handler.headers.get('Host','')
        from urllib.parse import urlsplit
        try:
            a=urlsplit('http://'+host)
            loopback=a.hostname in ('127.0.0.1','localhost') and a.port is not None and not a.username and not a.password and not a.path and not a.query and not a.fragment
        except ValueError:loopback=False
        if (apply_settings is None or handler.path!='/api/command'
                or not loopback
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
        if command['action']!='stop' and (stale or (busy and command['action'] not in ('stop','set_speed'))):
            handler.send(409,json.dumps({'error':'최신 임무 상태와 정지 확인이 필요합니다.'}).encode());return
        if not gate.acquire(blocking=False):
            handler.send(409,b'{"error":"Another settings request is in progress"}');return
        try:
            result=apply_settings(command)
            if runtime is not None and result.get('success') and command['action'] in SETTINGS_ACTIONS and command['action']!='capture_wp':runtime.persist(command)
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
        assert b'SatelliteMap' in urllib.request.urlopen(url+'/app.js?v=gap-visibility').read()
        assert 'Windows' in urllib.request.urlopen(url+'/?view=gap-visibility').read().decode()
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
        assert post({'action':'set_speed','percent':80})==200
        assert post({'action':'stop'})==200
        assert post({'action':'start'})==409
        assert len(calls)==3 and dashboard.commands.empty()
        print('GUI_HTTP_OK: live speed change allowed; other settings stopped-only; direct PWM/cross-origin rejected')
    finally:dashboard.close()


def main():
    parser = argparse.ArgumentParser(description='Final mission status 0~7 read-only GUI')
    parser.add_argument('--port', type=int, default=8771)
    parser.add_argument('--gps-topic', default='/gps/fix')
    parser.add_argument('--image-topic', default='/yolo/dbg_image/compressed')
    parser.add_argument('--read-only', action='store_true', help='Final GUI is always read-only')
    args = parser.parse_args()
    from final_mission_viewer import run
    run(args.port, args.gps_topic, args.image_topic)


if __name__ == '__main__':
    main()
