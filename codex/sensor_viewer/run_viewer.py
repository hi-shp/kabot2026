#!/usr/bin/env python3
"""Start the local browser viewer and a read-only ROS monitor over SSH."""
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

ROOT = Path(__file__).resolve().parent


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

    def request(self, method, path, body=None):
        with self.lock:
            client,port = self.client,self.remote_port
        if client is None or port is None or not client.get_transport() or not client.get_transport().is_active():
            raise ConnectionError('젯슨 연결 대기')
        channel = client.get_transport().open_channel('direct-tcpip',('127.0.0.1',port),('127.0.0.1',0),timeout=5)
        channel.settimeout(5)
        connection = ChannelHTTP(channel)
        try:
            headers = {'Connection':'close','Accept-Encoding':'gzip'}
            if body is not None:
                headers['Content-Type'] = 'application/json'
            connection.request(method,path,body=body,headers=headers)
            response = connection.getresponse()
            if int(response.getheader('Content-Length','0')) > 2*1024*1024:
                raise ValueError('Viewer response exceeds budget')
            payload = response.read(2*1024*1024+1)
            if len(payload)>2*1024*1024:
                raise ValueError('Viewer response exceeds budget')
            with self.lock:
                self.received_bytes += len(payload)
            return response.status,dict(response.getheaders()),payload
        finally:
            connection.close()

    def deploy(self, client):
        files = {name:(ROOT/name).read_bytes().replace(b'\r\n',b'\n') for name in ('jetson_server.py','telemetry_core.py','tank_feed.py')}
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
                        candidate.load_system_host_keys(str(known_hosts))
                    candidate.set_missing_host_key_policy(paramiko.RejectPolicy())
                    candidate.connect(self.args.host,port=self.args.ssh_port,username=self.args.user,password=self.password,
                                      look_for_keys=self.password is None,allow_agent=self.password is None,
                                      timeout=8,banner_timeout=8,auth_timeout=8)
                    candidate.get_transport().set_keepalive(15)
                    with self.lock:
                        self.client = candidate
                alive = False
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
                self.status = '연결됨'
                self.last_error = None
                delay = 1.
                self.stop.wait(3)
            except Exception as exc:
                message = type(exc).__name__+': '+str(exc)[:220]
                self.status = '연결 재시도 중'
                if message != self.last_error:
                    print(message,flush=True)
                self.last_error = message
                self.stop.wait(delay);delay = min(10,delay*2)

    def metadata(self):
        with self.lock:
            return {'status':self.status,'error':self.last_error,'host':self.args.host,
                    'received_bytes':self.received_bytes,'uptime_s':time.monotonic()-self.started}

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
            except (BrokenPipeError,ConnectionResetError):
                pass

        def do_GET(self):
            path = urlsplit(self.path).path
            if path == '/api/connection':
                self.send(200,json.dumps(bridge.metadata(),ensure_ascii=False).encode());return
            if path in ('/api/state','/api/cloud','/api/map'):
                self.proxy('GET');return
            files = {'/':('index.html','text/html; charset=utf-8'),
                     '/viewer.js':('viewer.js','text/javascript; charset=utf-8'),
                     '/viewer.css':('viewer.css','text/css; charset=utf-8')}
            if path in files:
                name,content_type = files[path]
                self.send(200,(ROOT/'web'/name).read_bytes(),content_type)
            else:
                self.send(404,b'{}')

        def do_POST(self):
            origin = self.headers.get('Origin')
            allowed = ('http://127.0.0.1:'+str(args.port),'http://localhost:'+str(args.port))
            if origin not in allowed or urlsplit(self.path).path not in ('/api/select','/api/route','/api/initial_pose'):
                self.send(403,b'{}');return
            size = int(self.headers.get('Content-Length','0'))
            if not 0 < size <= 1024:
                self.send(400,b'{}');return
            self.proxy('POST',self.rfile.read(size))

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
    parser.add_argument('--host',default='10.182.77.20')
    parser.add_argument('--user',default='isv')
    parser.add_argument('--ssh-port',type=int,default=22)
    parser.add_argument('--port',type=int,default=8765)
    parser.add_argument('--domain',type=int,default=50)
    parser.add_argument('--max-points',type=int,default=5000)
    parser.add_argument('--hz',type=float,default=5.)
    parser.add_argument('--key',action='store_true',help='Use SSH key or agent instead of password')
    parser.add_argument('--no-browser',action='store_true')
    args = parser.parse_args()
    if not 1 <= args.port <= 65535 or not 0 <= args.domain <= 232 or not 500 <= args.max_points <= 30000 or not 1 <= args.hz <= 10:
        parser.error('Invalid port, domain or display limits')
    try:
        import paramiko
    except ImportError:
        parser.error('Run start_viewer.ps1 to prepare the SSH dependency')
    password = None if args.key else getpass.getpass('Jetson SSH password: ')
    bridge = RemoteBridge(args,password)
    try:
        server = ThreadingHTTPServer(('127.0.0.1',args.port),make_handler(bridge,args))
    except OSError as exc:
        parser.error('Local port unavailable: '+str(exc))
    threading.Thread(target=server.serve_forever,daemon=True).start()
    worker = threading.Thread(target=bridge.work,daemon=True);worker.start()
    url = 'http://127.0.0.1:'+str(args.port)
    print('Live sensor viewer: '+url,flush=True)
    print('Ctrl+C closes this viewer and its own monitor.',flush=True)
    if not args.no_browser:
        webbrowser.open(url)
    if hasattr(signal,'SIGHUP'):
        signal.signal(signal.SIGHUP,signal.SIG_IGN)
    try:
        while not bridge.stop.wait(.2):
            pass
    except KeyboardInterrupt:
        pass
    finally:
        bridge.close();server.shutdown();server.server_close()


if __name__ == '__main__':
    main()
