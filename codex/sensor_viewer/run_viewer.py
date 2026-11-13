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
from ssh_discovery import adapter_networks, discover_candidates, update_verified_alias, tcp_ready
from connection_controls import ConnectionRequests, RemoteCodeWindow, HOSTS
from starlink_link import StarlinkLink, overlay_address
from connection_network import WifiRoutes

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
    command=[sys.executable,str(ROOT/'pc_fast_feed.py'),'--host',args.host,'--follow-dashboard',
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
    if args.host=='jetson-starlink' and value.get('hostname',args.host)==args.host:
        # The first overlay connection reuses the already pinned Jetson identity.
        value=config.lookup('jetson-hotspot')
        pinned=value.get('hostkeyalias')
        if not pinned or not known_hosts.lookup(pinned):
            raise ConnectionError('Verified Jetson hotspot HostKeyAlias is required for first Starlink connection')
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
    def __init__(self, args, password, wifi=None):
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
        self.active_host=None
        self.next_starlink_check=0.
        self.starlink_successes=0
        self.next_discovery={}
        self.connection_wake=threading.Event()
        self.connections=ConnectionRequests(lock=self.lock)
        self.remote_window=RemoteCodeWindow()
        self.manual_host=None
        self.starlink=StarlinkLink()
        self.wifi=wifi or WifiRoutes()
        self.link_signature=None
        self.browser=BrowserWake('http://127.0.0.1:'+str(args.port),enabled=not getattr(args,'no_browser',False))

    def request(self, method, path, body=None):
        with self.lock:
            client,port = self.client,self.remote_port
        if client is None or port is None or not client.get_transport() or not client.get_transport().is_active():
            raise ConnectionError('젯슨 연결 대기')
        health=path=='/health'
        channel = client.get_transport().open_channel('direct-tcpip',('127.0.0.1',port),('127.0.0.1',0),timeout=1.5 if health else 5)
        channel.settimeout(60 if method=='POST' and path.startswith('/api/gps-paths') else 1.5 if health else 5)
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
        files = {name:(ROOT/name).read_bytes().replace(b'\r\n',b'\n') for name in ('jetson_server.py','telemetry_core.py','tank_feed.py','outdoor_feed.py','gps_tracks.py','network_info.py','waypoint_route.py','buoy_orbit.py','map_checkpoint.py','docking_routes.py','vision_feed.py')}
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
        previous=self.process
        self.remote_port = int(ready['ready_port'])
        self.process = {'pid':int(ready['pid']),'path':directory+'/jetson_server.py','channel':stdout.channel}
        self.owned_processes.append(self.process)
        if previous and previous['pid']!=self.process['pid']:
            self.retire_monitor(client,previous)

    def retire_monitor(self,client,process):
        code=('import os,sys,signal;from pathlib import Path;'
              'p=int(sys.argv[1]);name=sys.argv[2];f=Path("/proc")/str(p)/"cmdline";'
              'data=f.read_bytes().split(bytes([0])) if f.exists() else [];'
              'os.kill(p,signal.SIGINT) if os.fsencode(name) in data else None')
        try:
            _,out,_=client.exec_command('python3 -c '+shlex.quote(code)+' '+str(process['pid'])+' '+shlex.quote(process['path']),timeout=2)
            deadline=time.monotonic()+2.
            while not out.channel.exit_status_ready() and time.monotonic()<deadline:time.sleep(.05)
            out.channel.close()
        except Exception:pass

    def connect_host(self, host, hostname=None):
        route=next((kind for kind,alias in HOSTS.items() if alias==host),None)
        if route:self.wifi.require(route)
        import paramiko
        from types import SimpleNamespace
        candidate = paramiko.SSHClient()
        known_hosts = Path.home()/'.ssh/known_hosts'
        if known_hosts.exists():
            candidate.load_host_keys(str(known_hosts))
        candidate.set_missing_host_key_policy(paramiko.RejectPolicy())
        settings_args = SimpleNamespace(host=host, ssh_port=self.args.ssh_port, user=self.args.user)
        try:
            if host=='jetson-starlink':
                addresses=self.starlink.candidates()
                if not addresses:raise ConnectionError(self.starlink.status()['message'])
                hostname=hostname or addresses[0]
                if hostname not in addresses:raise ConnectionError('등록된 스타링크 젯슨 주소가 아닙니다')
            if os.name == 'nt' and host == 'jetson-hotspot':
                route = 'hotspot'
                if not adapter_networks(route):
                    raise ConnectionError(route+' network adapter is not connected')
            settings = ssh_settings(settings_args,candidate.get_host_keys())
            if hostname is not None:
                for kind,key in (candidate.get_host_keys().lookup(settings['hostname']) or {}).items():
                    candidate.get_host_keys().add(hostname,kind,key)
                settings['hostname'] = hostname
            if host=='jetson-starlink' and not self.starlink.reachable(settings['hostname'],settings['port']):
                raise ConnectionError('스타링크 젯슨 SSH 응답 대기 · 인터넷과 젯슨 전원을 확인하세요')
            if os.name=='nt' and host=='jetson-hotspot':
                if not tcp_ready(route,settings['hostname'],settings['port']):
                    raise ConnectionError('Selected-network SSH port is not reachable yet')
            candidate.connect(**settings,password=self.password,
                              look_for_keys=self.password is None,allow_agent=self.password is None,
                              timeout=4,banner_timeout=4,auth_timeout=4)
            if self.args.host == 'auto' or host in HOSTS.values():
                expected = 'starlink' if host == 'jetson-starlink' else 'hotspot'
                code = '''import getpass,json,os,subprocess,sys
from pathlib import Path
expected=sys.argv[1]
ip=(os.environ.get("SSH_CONNECTION","").split()+[None]*4)[2]
rows=json.loads(subprocess.check_output(["ip","-j","-4","address","show"],timeout=2))
found=False
for row in rows:
    name=row["ifname"]; root=Path("/sys/class/net")/name
    addresses=[a.get("local") for a in row.get("addr_info",[]) if a.get("family")=="inet"]
    if ip not in addresses: continue
    mac=(root/"address").read_text().strip().lower()
    kind="starlink" if name=="tailscale0" else "hotspot" if (root/"wireless").is_dir() or (root/"phy80211").exists() else None
    found=kind==expected
ok=found and getpass.getuser()=="isv" and Path("/home/isv/ros2_ws/src/isv_water_trials").is_dir()
print(expected if ok else "mismatch")
sys.exit(0 if ok else 1)'''
                command = 'python3 -c '+shlex.quote(code)+' '+shlex.quote(expected)
                _,out,_ = candidate.exec_command(command,timeout=4)
                if out.read().strip() != expected.encode() or out.channel.recv_exit_status() != 0:
                    raise ConnectionError('Jetson workspace identity check failed')
            candidate.get_transport().set_keepalive(3)
            if route:self.wifi.require(route)
            if hostname is not None:
                update_verified_alias(host,hostname)
            return candidate
        except Exception:
            candidate.close()
            raise

    def discover_host(self, host):
        if host not in HOSTS.values():
            return None
        policy=self.wifi.snapshot()
        if not any(policy['allowed'][route] and alias==host for route,alias in HOSTS.items()):return None
        now = time.monotonic()
        if now < self.next_discovery.get(host,0):
            return None
        self.next_discovery[host] = now+(2 if host=='jetson-starlink' else 10)
        addresses=self.starlink.candidates() if host=='jetson-starlink' else discover_candidates('hotspot',getattr(self.args,'ssh_port',None) or 22)
        for address in addresses:
            try:
                return self.connect_host(host,hostname=address)
            except Exception:
                continue
        return None

    def connect_preferred(self):
        policy=self.wifi.snapshot()
        if self.manual_host in HOSTS.values() and not any(policy['allowed'][route] and alias==self.manual_host for route,alias in HOSTS.items()):
            self.manual_host=None
        if self.manual_host=='jetson-starlink':self.starlink.prepare_login()
        hosts = (self.manual_host,) if self.manual_host else tuple(alias for route,alias in HOSTS.items() if policy['allowed'][route]) if self.args.host == 'auto' else (self.args.host,)
        if not hosts:raise ConnectionError('Kk 또는 welcome_CU Wi-Fi 연결을 기다립니다')
        errors = []
        for host in hosts:
            try:
                return host,self.connect_host(host)
            except Exception as exc:
                errors.append(host+': '+type(exc).__name__)
                candidate = self.discover_host(host)
                if candidate is not None:
                    return host,candidate
        raise ConnectionError('; '.join(errors))

    def maybe_prefer_starlink(self):
        if not self.wifi.snapshot()['allowed']['starlink']:return
        if self.manual_host or self.args.host != 'auto' or self.active_host != 'jetson-hotspot' or time.monotonic() < self.next_starlink_check:
            return
        self.next_starlink_check = time.monotonic()+3
        try:
            candidate = self.connect_host('jetson-starlink')
        except Exception:
            candidate = self.discover_host('jetson-starlink')
            if candidate is None:
                self.starlink_successes = 0
                return
        self.starlink_successes += 1
        if self.starlink_successes < 2:
            candidate.close()
            return
        with self.lock:
            previous,self.client = self.client,candidate
            self.active_host = 'jetson-starlink'
            self.last_health=0.
            self.starlink_successes = 0
        if previous:
            previous.close()

    def request_connection(self,route,client_id=None,sequence=None):
        self.wifi.require(route)
        with self.lock:
            transport=self.client.get_transport() if self.client else None
            active=bool(transport and transport.is_active() and self.active_host==HOSTS.get(route) and
                        self.last_error is None and time.monotonic()-self.last_health<10.)
            job=self.connections.request(route,already_connected=active,client_id=client_id,sequence=sequence)
        self.connection_wake.set()
        return job

    def connect_requested(self,job):
        """Run only on the bridge worker; HTTP threads never swap clients."""
        host=job['host'];candidate=None
        try:
            self.wifi.require(job['route'])
            if job.get('attempts',1)==1:
                self.next_discovery[host]=0.
            if host=='jetson-starlink':self.starlink.prepare_login()
            try:candidate=self.connect_host(host)
            except Exception as exc:
                candidate=self.discover_host(host)
                if candidate is None:
                    if host=='jetson-starlink':raise ConnectionError(str(exc)) from exc
                    raise ConnectionError('같은 핫스팟에서 젯슨을 찾지 못했습니다. 서로 다른 핫스팟은 스타링크 연결을 사용하세요.') from exc
            with self.lock:
                if not self.connections.current(job):
                    candidate.close();return False
                self.wifi.require(job['route'])
                previous=self.client;self.client=candidate;self.active_host=host;self.manual_host=host
                self.last_health=0.;self.last_error=None
                # Reuse the same onboard monitor through the new SSH transport.
                # Health checking below replaces it only when it has exited.
            if previous:
                try:previous.close()
                except Exception:pass
            return True
        except Exception as exc:
            if candidate:candidate.close()
            self.connections.retry(job,error=str(exc)[:240])
            return False

    def finish_connection(self,job):
        with self.lock:
            if not self.connections.current(job):return
            ip=self.client.get_transport().sock.getpeername()[0]
            try:window=self.remote_window.open(job['host'])
            except OSError as exc:window='error: '+str(exc)
            self.connections.finish(job,ip=ip,vscode=window)

    def check_links(self):
        policy=self.wifi.snapshot()
        networks={'hotspot':adapter_networks('hotspot')}
        signature=(policy['ssid'],tuple((route,tuple(sorted(ip for ip,_ in values))) for route,values in networks.items()))
        close=None
        with self.lock:
            if signature!=self.link_signature:
                self.link_signature=signature;self.next_starlink_check=0.;self.next_discovery.clear()
                self.starlink_successes=0;self.connection_wake.set()
            allowed_hosts={alias for route,alias in HOSTS.items() if policy['allowed'][route]}
            if self.manual_host in HOSTS.values() and self.manual_host not in allowed_hosts:self.manual_host=None
            job=self.connections.snapshot()
            if job and not policy['allowed'].get(job['route'],False):
                self.connections.finish(job,error=policy['reasons'].get(job['route'],'Wi-Fi 연결 대기'))
            if self.client and self.active_host in HOSTS.values() and self.active_host not in allowed_hosts:
                close=self.client;self.last_error='현재 Wi-Fi에 맞는 연결로 전환 중';self.connection_wake.set()
            client=self.client;route='hotspot' if self.active_host=='jetson-hotspot' else None
            if route and client:
                try:source=client.get_transport().sock.getsockname()[0]
                except (AttributeError,OSError):source=None
                if source and source not in {ip for ip,_ in networks[route]}:
                    close=client;self.connection_wake.set()
        if close:close.close()

    def watch_links(self):
        while not self.stop.is_set():
            try:self.check_links()
            except (OSError,ValueError):pass
            self.stop.wait(.5)

    def work(self):
        import paramiko
        delay = 1.
        while not self.stop.is_set():
            if getattr(self.args,'wait_for_connection',False) and self.connections.snapshot() is None:
                self.status='스타링크·핫스팟 연결 버튼 대기'
                self.connection_wake.wait(1);self.connection_wake.clear();continue
            job=self.connections.claim()
            try:
                requested=self.connect_requested(job) if job else False
                if job and not self.connections.current(job):continue
                active = self.client and self.client.get_transport() and self.client.get_transport().is_active()
                if not active:
                    current=self.connections.snapshot()
                    if current and current['busy']:
                        self.connection_wake.wait(1);self.connection_wake.clear();continue
                    self.status = '젯슨 연결 중'
                    host,candidate = self.connect_preferred()
                    with self.lock:
                        self.client = candidate
                        self.active_host = host
                        self.next_starlink_check = time.monotonic()+3
                        self.starlink_successes = 0
                elif not job and not (self.connections.snapshot() or {}).get('busy'):
                    self.maybe_prefer_starlink()
                alive = False
                data={}
                if self.remote_port:
                    try:
                        status,_,body = self.request('GET','/health')
                        data = json.loads(body)
                        alive = status==200 and data.get('version')=='kabot-sensor-viewer-v1' and self.process and data.get('pid')==self.process['pid']
                    except paramiko.ChannelException as exc:
                        if exc.code!=2:raise
                        alive=False
                    except (OSError,EOFError,ValueError,http.client.HTTPException,paramiko.SSHException) as exc:
                        raise ConnectionError('SSH health check interrupted') from exc
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
                if job and requested:self.finish_connection(job)
                delay = 1.
                self.connection_wake.wait(1);self.connection_wake.clear()
            except Exception as exc:
                if job:self.connections.retry(job,error=type(exc).__name__+': '+str(exc)[:200])
                self.sensors_running=False;self.browser.observe(False)
                message = type(exc).__name__+': '+str(exc)[:220]
                self.status = '연결 재시도 중'
                if message != self.last_error:
                    print(message,flush=True)
                self.last_error = message
                with self.lock:
                    failed,self.client = self.client,None
                    self.active_host = None
                if failed:
                    failed.close()
                self.connection_wake.wait(1);self.connection_wake.clear()

    def metadata(self):
        wifi=self.wifi.snapshot()
        starlink=self.starlink.status()
        starlink=dict(starlink,vpn_ready=bool(starlink.get('ready')),wifi_allowed=wifi['allowed']['starlink'],current_ssid=wifi['ssid'])
        if not wifi['allowed']['starlink']:
            starlink.update(state='wifi_blocked',ready=False,message=wifi['reasons']['starlink'])
        with self.lock:
            connected=bool(self.client and self.client.get_transport() and self.client.get_transport().is_active()
                           and self.last_error is None and time.monotonic()-self.last_health<10.)
            if self.active_host in HOSTS.values() and not any(wifi['allowed'][route] and alias==self.active_host for route,alias in HOSTS.items()):connected=False
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
            matches=[(kind,endpoint) for kind,endpoint in network.get('endpoints',{}).items()
                     if kind in HOSTS and target_ip in endpoint.get('addresses',[])] if connected else []
            # A reused monitor retains its original SSH_CONNECTION environment.
            # Identify the current socket using live interface/MAC addresses.
            connection_path=matches[0][0] if len(matches)==1 else None
            evidence=('SSH 수신 인터페이스 '+matches[0][1]['interface']+' · MAC '+matches[0][1]['mac']) if connection_path else '실제 SSH 수신 인터페이스 확인 중'
            endpoints=dict(network.get('endpoints',{}))
            if connected and self.active_host=='jetson-starlink' and overlay_address(target_ip):
                connection_path='starlink'
                evidence='젯슨 SSH 키·사용자·작업경로 검증 완료 · 인터넷 VPN '+target_ip
                endpoints['starlink']=dict(interface='tailscale0',mac='',addresses=[target_ip],link_up=True)
            return {'version':'kabot-pc-viewer-v1','status':self.status,'error':self.last_error,'host':self.args.host,'active_host':self.active_host if connected else None,
                    'connection_path':connection_path,'target_ip':target_ip,'source_ip':source_ip,'connected':connected,
                    'connection_evidence':evidence,
                    'endpoints':endpoints,'remote_user':network.get('remote_user','isv'),
                    'connection_request':self.connections.snapshot(),'selected_host':self.manual_host or self.args.host,
                    'starlink':starlink,'wifi':wifi,
                    'received_bytes':self.received_bytes,'uptime_s':time.monotonic()-self.started,'sensors_running':self.sensors_running}

    def close(self):
        self.stop.set()
        self.connection_wake.set()
        self.starlink.close()
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
            if path in ('/api/state','/api/cloud','/api/map','/api/maps','/api/docking-routes','/api/localization-prior','/api/vision','/api/camera.jpg','/api/recordings','/api/gps-paths'):
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
            if origin not in allowed or urlsplit(self.path).path not in ('/api/connection','/api/select','/api/route','/api/initial_pose','/api/localize','/api/outdoor','/api/capture','/api/show','/api/sensors','/api/maps','/api/docking-routes','/api/localization-prior','/api/vision','/api/gps-paths'):
                self.send(403,b'{}');return
            if urlsplit(self.path).path=='/api/show':
                bridge.browser.open(force=True);self.send(200,b'{"shown":true}');return
            try:size = int(self.headers.get('Content-Length','0'))
            except ValueError:self.send(400,b'{}');return
            if not 0 < size <= (65536 if urlsplit(self.path).path=='/api/maps' else 32768 if urlsplit(self.path).path in ('/api/outdoor','/api/capture') else 1024):
                self.send(400,b'{}');return
            body=self.rfile.read(size)
            if urlsplit(self.path).path=='/api/connection':
                try:
                    data=json.loads(body)
                    if not isinstance(data,dict) or 'route' not in data or set(data)-{'route','client_id','sequence'}:raise ValueError('연결 종류만 지정하세요')
                    job=bridge.request_connection(data['route'],client_id=data.get('client_id'),sequence=data.get('sequence'))
                    self.send(202 if job['busy'] else 200,json.dumps(job,ensure_ascii=False).encode())
                except (ValueError,TypeError) as exc:self.send(400,json.dumps({'error':str(exc)},ensure_ascii=False).encode())
                return
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
    parser.add_argument('--host',default='auto')
    parser.add_argument('--user',default=None)
    parser.add_argument('--ssh-port',type=int,default=None)
    parser.add_argument('--port',type=int,default=8765)
    parser.add_argument('--domain',type=int,default=50)
    parser.add_argument('--max-points',type=int,default=1800)
    parser.add_argument('--hz',type=float,default=3.)
    parser.add_argument('--key',action='store_true',help='Use SSH key or agent instead of password')
    parser.add_argument('--no-browser',action='store_true')
    parser.add_argument('--wait-for-sensors',action='store_true',help='Open only when bringup starts')
    parser.add_argument('--wait-for-connection',action='store_true',help='Make no SSH connection until a dashboard connection button is clicked')
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
    threading.Thread(target=bridge.watch_links,daemon=True).start()
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
