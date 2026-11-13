#!/usr/bin/env python3
"""Local dashboard over SSH with explicit navigation controls."""
import argparse
import getpass
import hashlib
import http.client
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import shlex
import signal
import socket
import threading
import time
from urllib.parse import urlsplit
import webbrowser
from dashboard_window import DashboardWindow

ROOT = Path(__file__).resolve().parent


def ensure_pc_fast_feed(args, dashboard):
    """Start the PC companion once; never deploy or restart any vessel code."""
    import subprocess
    import sys
    import urllib.request
    try:
        with urllib.request.urlopen('http://127.0.0.1:18766/state', timeout=.8) as response:
            if json.load(response).get('version')=='kabot-pc-fast-feed-v1':
                return
    except (OSError, ValueError):
        pass
    command=[sys.executable,str(ROOT/'pc_fast_feed.py'),'--host',args.host,
             '--dashboard',dashboard,'--domain',str(args.domain)]
    if args.user:command+=['--user',args.user]
    if args.ssh_port:command+=['--ssh-port',str(args.ssh_port)]
    subprocess.Popen(command,stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,
                     stderr=subprocess.DEVNULL,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))


class BrowserWake:
    """Focus once per bringup request, including an already-running session."""
    def __init__(self,url,enabled=True,opener=None,clock=None):
        self.url=url;self.enabled=enabled;self.opener=opener or DashboardWindow();self.clock=clock or time.monotonic
        self.last_view=float('-inf');self.last_open=float('-inf');self.active=False;self.lock=threading.Lock()
        self.request_id=None
    def viewed(self):
        with self.lock:self.last_view=self.clock()
    def open(self,force=False):
        if not self.enabled and not force:return
        with self.lock:self.last_open=self.clock()
        self.opener(self.url)
    def observe(self,active,request_id=None):
        with self.lock:
            now=self.clock();explicit=bool(request_id and request_id!=self.request_id)
            should_open=self.enabled and (explicit or (active and not self.active and now-self.last_open>3))
            if request_id:self.request_id=request_id
            self.active=active
        if should_open:self.open()


def ssh_settings(args,known_hosts):
    import paramiko
    config=paramiko.SSHConfig();path=Path.home()/'.ssh/config'
    if path.exists():
        with path.open(encoding='utf-8') as stream:config.parse(stream)
    value=config.lookup(args.host)
    host=value.get('hostname',args.host);alias=value.get('hostkeyalias',host)
    # Reuse the verified alias key even when DHCP changes the target address.
    keys=known_hosts.lookup(alias)
    if keys:
        for kind,key in keys.items():known_hosts.add(host,kind,key)
    return dict(hostname=host,port=args.ssh_port or int(value.get('port',22)),
                username=args.user or value.get('user','isv'),
                key_filename=value.get('identityfile'))


def bind_dashboard(port):
    """Hyper-V can reserve the usual port; keep the actual bound socket."""
    error=None
    for candidate in dict.fromkeys((port,18765,0)):
        try:return ThreadingHTTPServer(('127.0.0.1',candidate),BaseHTTPRequestHandler)
        except OSError as exc:error=exc
    raise error


class ChannelHTTP(http.client.HTTPConnection):
    def __init__(self, channel):
        super().__init__('127.0.0.1',timeout=5)
        self.channel = channel

    def connect(self):
        self.sock = self.channel


class RemoteBridge:
    def __init__(self, args, password):
        self.args = args
        self.password = password
        self.client = None
        self.remote_port = None
        self.process = None
        self.owned_processes = []
        self.lock = threading.RLock()
        self.stop = threading.Event()
        self.status = '젯슨 연결 중'
        self.last_error = None
        self.received_bytes = 0
        self.started = time.monotonic()
        self.last_state_bytes = 0
        self.sensors_running=False
        self.network_info={}
        self.last_health=0.
        self.browser=BrowserWake('http://127.0.0.1:'+str(args.port),enabled=not getattr(args,'no_browser',False))

    def request(self, method, path, body=None):
        with self.lock:
            client,port = self.client,self.remote_port
        if client is None or port is None or not client.get_transport() or not client.get_transport().is_active():
            raise ConnectionError('젯슨 연결 대기')
        channel = client.get_transport().open_channel('direct-tcpip',('127.0.0.1',port),('127.0.0.1',0),timeout=5)
        channel.settimeout(60 if method=='POST' and path.startswith('/api/gps-paths') else 5)
        connection = ChannelHTTP(channel)
        try:
            headers = {'Connection':'close','Accept-Encoding':'gzip'}
            if body is not None:
                headers['Content-Type'] = 'application/json'
            connection.request(method,path,body=body,headers=headers)
            response = connection.getresponse()
            budget=8*1024*1024 if path.startswith('/api/map?') else 2*1024*1024
            if int(response.getheader('Content-Length','0')) > budget:
                raise ValueError('Viewer response exceeds budget')
            payload = response.read(budget+1)
            if len(payload)>budget:
                raise ValueError('Viewer response exceeds budget')
            with self.lock:
                self.received_bytes += len(payload)
            return response.status,dict(response.getheaders()),payload
        finally:
            connection.close()

    def deploy(self, client):
        files = {name:(ROOT/name).read_bytes().replace(b'\r\n',b'\n') for name in ('jetson_server.py','telemetry_core.py','tank_feed.py','outdoor_feed.py','gps_tracks.py','network_info.py','waypoint_route.py','map_checkpoint.py','docking_routes.py')}
        fingerprint = hashlib.sha256(b''.join(files.values())).hexdigest()[:16]
        sftp = client.open_sftp()
        home = sftp.normalize('.')
        directory = home+'/.local/share/kabot_sensor_viewer/'+fingerprint
        def mkdir(path):
            try:
                sftp.stat(path)
            except FileNotFoundError:
                mkdir(path.rsplit('/',1)[0]);sftp.mkdir(path)
        mkdir(directory)
        for name,data in files.items():
            path = directory+'/'+name
            try:
                with sftp.open(path,'rb') as source:
                    matches = source.read() == data
            except FileNotFoundError:
                matches = False
            if not matches:
                with sftp.open(path+'.incoming','wb') as target:
                    target.write(data)
                sftp.posix_rename(path+'.incoming',path)
        sftp.close()
        command = ('source /opt/ros/humble/setup.bash && source "$HOME/ros2_ws/install/setup.bash" && '
                   'export ROS_DOMAIN_ID='+str(self.args.domain)+' ROS_LOCALHOST_ONLY=0 && '
                   'exec python3 -u '+shlex.quote(directory+'/jetson_server.py')+
                   ' --port 0 --max-points '+str(self.args.max_points)+' --cloud-hz '+str(self.args.hz))
        stdin,stdout,stderr = client.exec_command('bash -lc '+shlex.quote(command),timeout=35)
        stdin.channel.shutdown_write()
        stdout.channel.settimeout(30)
        ready = None
        for _ in range(30):
            line = stdout.readline()
            if not line:
                break
            try:
                parsed = json.loads(line)
                if 'ready_port' in parsed:
                    ready = parsed;break
            except json.JSONDecodeError:
                pass
        if ready is None:
            detail = stderr.read(2000).decode(errors='replace') if stdout.channel.exit_status_ready() else 'ROS monitor startup timeout'
            raise RuntimeError(detail)
        self.remote_port = int(ready['ready_port'])
        self.process = {'pid':int(ready['pid']),'path':directory+'/jetson_server.py','channel':stdout.channel}
        self.owned_processes.append(self.process)

    def work(self):
        import paramiko
        delay = 1.
        while not self.stop.is_set():
            try:
                active = self.client and self.client.get_transport() and self.client.get_transport().is_active()
                if not active:
                    self.status = '젯슨 연결 중'
                    candidate = paramiko.SSHClient()
                    known_hosts = Path.home()/'.ssh/known_hosts'
                    if known_hosts.exists():
                        candidate.load_host_keys(str(known_hosts))
                    candidate.set_missing_host_key_policy(paramiko.RejectPolicy())
                    candidate.connect(**ssh_settings(self.args,candidate.get_host_keys()),password=self.password,
                                      look_for_keys=self.password is None,allow_agent=self.password is None,
                                      timeout=8,banner_timeout=8,auth_timeout=8)
                    candidate.get_transport().set_keepalive(15)
                    with self.lock:
                        self.client = candidate
                alive = False
                data={}
                if self.remote_port:
                    try:
                        status,_,body = self.request('GET','/health')
                        data = json.loads(body)
                        alive = status==200 and data.get('version')=='kabot-sensor-viewer-v1' and self.process and data.get('pid')==self.process['pid']
                    except (OSError,EOFError,ValueError,http.client.HTTPException,paramiko.SSHException):
                        alive = False
                if not alive:
                    self.remote_port = None
                    self.status = '센서 모니터 준비 중'
                    self.deploy(self.client)
                    status,_,body=self.request('GET','/health');data=json.loads(body)
                self.sensors_running=bool(data.get('sensors_running'))
                self.network_info=data.get('network') or {}
                self.last_health=time.monotonic()
                self.browser.observe(self.sensors_running,data.get('dashboard_request'))
                self.status = '연결됨'
                self.last_error = None
                delay = 1.
                self.stop.wait(3)
            except Exception as exc:
                self.sensors_running=False;self.browser.observe(False)
                message = type(exc).__name__+': '+str(exc)[:220]
                self.status = '연결 재시도 중'
                if message != self.last_error:
                    print(message,flush=True)
                self.last_error = message
                self.stop.wait(delay);delay = min(10,delay*2)

    def metadata(self):
        with self.lock:
            connected=bool(self.client and self.client.get_transport() and self.client.get_transport().is_active()
                           and self.last_error is None and time.monotonic()-self.last_health<10.)
            source_ip=None
            try:
                if connected:source_ip=self.client.get_transport().sock.getsockname()[0]
            except (AttributeError,OSError):
                pass
            target_ip=None
            try:
                if connected:target_ip=self.client.get_transport().sock.getpeername()[0]
            except Exception:
                pass
            network=self.network_info
            verified=connected and network.get('server_ip')==target_ip
            connection_path=network.get('connection_path') if verified else None
            return {'version':'kabot-pc-viewer-v1','status':self.status,'error':self.last_error,'host':self.args.host,
                    'connection_path':connection_path,'target_ip':target_ip,'source_ip':source_ip,'connected':connected,
                    'connection_evidence':network.get('evidence') if verified else '실제 SSH 수신 인터페이스 확인 중',
                    'endpoints':network.get('endpoints',{}),'remote_user':network.get('remote_user','isv'),
                    'received_bytes':self.received_bytes,'uptime_s':time.monotonic()-self.started,'sensors_running':self.sensors_running}

    def close(self):
        self.stop.set()
        client = self.client
        if client and client.get_transport() and client.get_transport().is_active():
            code = ('import os,sys,signal;from pathlib import Path;'
                    'p=int(sys.argv[1]);name=sys.argv[2];f=Path("/proc")/str(p)/"cmdline";'
                    'data=f.read_bytes().split(bytes([0])) if f.exists() else [];'
                    'os.kill(p,signal.SIGINT) if os.fsencode(name) in data else None')
            for process in self.owned_processes:
                try:
                    command = 'python3 -c '+shlex.quote(code)+' '+str(process['pid'])+' '+shlex.quote(process['path'])
                    _,out,_ = client.exec_command(command,timeout=3)
                    out.channel.recv_exit_status()
                except Exception:
                    pass
            client.close()


def make_handler(bridge, args):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def log_message(self, *_):
            pass

        def handle(self):
            try:
                super().handle()
            except (ConnectionResetError,ConnectionAbortedError,BrokenPipeError):
                # Closing/reloading a browser may reset an idle keep-alive socket.
                pass

        def send(self, status, payload, content_type='application/json', encoding=None):
            self.send_response(status)
            self.send_header('Content-Type',content_type)
            self.send_header('Content-Length',str(len(payload)))
            self.send_header('Cache-Control','no-store')
            self.send_header('X-Content-Type-Options','nosniff')
            if encoding:
                self.send_header('Content-Encoding',encoding)
            self.end_headers()
            try:
                self.wfile.write(payload)
            except (BrokenPipeError,ConnectionResetError,ConnectionAbortedError):
                pass

        def do_GET(self):
            path = urlsplit(self.path).path
            if path == '/api/capture':
                capture=getattr(bridge,'capture',None)
                self.send(200,json.dumps(capture.status() if capture else {'phase':'unavailable'},ensure_ascii=False).encode());return
            if path == '/api/connection':
                self.send(200,json.dumps(bridge.metadata(),ensure_ascii=False).encode());return
            if path in ('/api/state','/api/cloud','/api/map','/api/maps','/api/docking-routes','/api/recordings','/api/gps-paths'):
                if path=='/api/state' and 'capture=1' not in urlsplit(self.path).query:bridge.browser.viewed()
                self.proxy('GET');return
            if path == '/outdoor':
                self.send_response(302);self.send_header('Location','/');self.send_header('Content-Length','0');self.end_headers();return
            files = {'/':('outdoor.html','text/html; charset=utf-8'),
                     '/sensors':('index.html','text/html; charset=utf-8'),
                     '/outdoor.js':('outdoor.js','text/javascript; charset=utf-8'),
                     '/outdoor.css':('outdoor.css','text/css; charset=utf-8'),
                     '/viewer.js':('viewer.js','text/javascript; charset=utf-8'),
                     '/viewer.css':('viewer.css','text/css; charset=utf-8'),
                     '/theme.css':('theme.css','text/css; charset=utf-8')}
            if path in files:
                name,content_type = files[path]
                self.send(200,(ROOT/'web'/name).read_bytes(),content_type)
            else:
                self.send(404,b'{}')

        def do_POST(self):
            origin = self.headers.get('Origin')
            allowed = ('http://127.0.0.1:'+str(args.port),'http://localhost:'+str(args.port))
            if origin not in allowed or urlsplit(self.path).path not in ('/api/select','/api/route','/api/initial_pose','/api/localize','/api/outdoor','/api/capture','/api/show','/api/sensors','/api/maps','/api/docking-routes','/api/gps-paths'):
                self.send(403,b'{}');return
            if urlsplit(self.path).path=='/api/show':
                bridge.browser.open(force=True);self.send(200,b'{"shown":true}');return
            try:size = int(self.headers.get('Content-Length','0'))
            except ValueError:self.send(400,b'{}');return
            if not 0 < size <= (32768 if urlsplit(self.path).path in ('/api/outdoor','/api/capture') else 4096 if urlsplit(self.path).path=='/api/maps' else 1024):
                self.send(400,b'{}');return
            body=self.rfile.read(size)
            if urlsplit(self.path).path=='/api/capture':
                try:data=json.loads(body)
                except (ValueError,UnicodeError):self.send(400,b'{}');return
                capture=getattr(bridge,'capture',None)
                status,payload=capture.request(data) if capture else (503,{'error':'촬영 기능 준비 필요'})
                self.send(status,json.dumps(payload,ensure_ascii=False).encode());return
            self.proxy('POST',body)

        def proxy(self, method, body=None):
            try:
                status,headers,payload = bridge.request(method,self.path,body)
                self.send(status,payload,headers.get('Content-Type','application/json'),headers.get('Content-Encoding'))
            except Exception as exc:
                payload = {'error':'센서 연결 대기','detail':type(exc).__name__}
                self.send(503,json.dumps(payload,ensure_ascii=False).encode())
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host',default='jetson')
    parser.add_argument('--user',default=None)
    parser.add_argument('--ssh-port',type=int,default=None)
    parser.add_argument('--port',type=int,default=8765)
    parser.add_argument('--domain',type=int,default=50)
    parser.add_argument('--max-points',type=int,default=1800)
    parser.add_argument('--hz',type=float,default=3.)
    parser.add_argument('--key',action='store_true',help='Use SSH key or agent instead of password')
    parser.add_argument('--no-browser',action='store_true')
    parser.add_argument('--wait-for-sensors',action='store_true',help='Open only when bringup starts')
    parser.add_argument('--log-file',type=Path)
    parser.add_argument('--capture-dir',type=Path,help='PC screenshot/video output folder')
    args = parser.parse_args()
    if args.log_file:
        import sys
        args.log_file.parent.mkdir(parents=True,exist_ok=True)
        # Bound unattended logs between launches.
        if args.log_file.exists() and args.log_file.stat().st_size>2*1024*1024:
            args.log_file.replace(args.log_file.with_suffix('.previous.log'))
        sys.stdout=sys.stderr=args.log_file.open('a',encoding='utf-8',buffering=1)
    if not 1 <= args.port <= 65535 or not 0 <= args.domain <= 232 or not 500 <= args.max_points <= 30000 or not 1 <= args.hz <= 10:
        parser.error('Invalid port, domain or display limits')
    try:
        import paramiko
    except ImportError:
        parser.error('Run start_viewer.ps1 to prepare the SSH dependency')
    # Manual relaunch raises the existing dashboard instead of binding a second server.
    import urllib.request
    instance=Path(os.environ.get('LOCALAPPDATA',str(Path.home()/'.cache')))/'KabotSensorViewer/instance.json'
    ports=[args.port]
    try:ports.append(int(json.loads(instance.read_text())['port']))
    except (OSError,ValueError,KeyError,TypeError):pass
    for port in dict.fromkeys(ports):
        if not 1<=port<=65535:continue
        url='http://127.0.0.1:'+str(port)
        try:
            with urllib.request.urlopen(url+'/api/connection',timeout=1) as response:
                existing=json.load(response)
            if existing.get('version')=='kabot-pc-viewer-v1':
                ensure_pc_fast_feed(args,url)
                if not args.wait_for_sensors and not args.no_browser:
                    request=urllib.request.Request(url+'/api/show',method='POST',headers={'Origin':url})
                    urllib.request.urlopen(request,timeout=3).close()
                return
        except (OSError,ValueError):pass
    password = None if args.key else getpass.getpass('Jetson SSH password: ')
    try:server=bind_dashboard(args.port)
    except OSError as exc:parser.error('Local port unavailable: '+str(exc))
    args.port=server.server_port
    bridge = RemoteBridge(args,password)
    from dashboard_capture import CaptureService,DEFAULT_ROOT
    bridge.capture=CaptureService('http://127.0.0.1:'+str(args.port),args.capture_dir or DEFAULT_ROOT)
    server.RequestHandlerClass=make_handler(bridge,args)
    instance.parent.mkdir(parents=True,exist_ok=True)
    instance.write_text(json.dumps({'pid':os.getpid(),'port':args.port}),encoding='utf-8')
    threading.Thread(target=server.serve_forever,daemon=True).start()
    worker = threading.Thread(target=bridge.work,daemon=True);worker.start()
    url = 'http://127.0.0.1:'+str(args.port)
    ensure_pc_fast_feed(args,url)
    print('Live sensor viewer: '+url,flush=True)
    print('Ctrl+C closes this viewer and its own monitor.',flush=True)
    if not args.no_browser and not args.wait_for_sensors:
        bridge.browser.open()
    if hasattr(signal,'SIGHUP'):
        signal.signal(signal.SIGHUP,signal.SIG_IGN)
    try:
        while not bridge.stop.wait(.2):
            pass
    except KeyboardInterrupt:
        pass
    finally:
        bridge.capture.close();bridge.close();server.shutdown();server.server_close()


if __name__ == '__main__':
    main()
