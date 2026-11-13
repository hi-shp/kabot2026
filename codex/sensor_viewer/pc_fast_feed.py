"""PC-only feed served from passive ROS subscriptions streamed over SSH.

Never deploy a monitor, write a remote file, publish ROS, or restart a vessel node.
"""
import argparse
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import shlex
import socket
import threading
import time
import urllib.request
import uuid

import paramiko


class Feed:
    def __init__(self, args):
        self.args = args
        self.condition = threading.Condition()
        self.sequence = 0
        self.packet = {}
        self.error = 'Connecting to existing sensor feed'
        self.updated = 0.
        self.arrivals = deque(maxlen=200)
        self.client = None
        self.clock_offset_s = 0.
        self.transit_age_s = 0.
        self.connected_host=None
        self.instance_id = uuid.uuid4().hex
        self.received_bytes = 0

    def dashboard_host(self):
        if not self.args.follow_dashboard:
            return self.args.host
        with urllib.request.urlopen(self.args.dashboard+'/api/connection', timeout=1) as response:
            state = json.load(response)
        if not state.get('connected') or not state.get('active_host'):
            raise ConnectionError('Dashboard SSH connection is not ready')
        return state['active_host']

    def connect(self, host=None):
        config = paramiko.SSHConfig()
        with (Path.home()/'.ssh/config').open(encoding='utf-8') as stream:
            config.parse(stream)
        host = host or self.args.host
        settings = config.lookup(host)
        client = paramiko.SSHClient()
        client.load_system_host_keys()
        known_hosts = Path.home()/'.ssh/known_hosts'
        if known_hosts.exists():
            client.load_host_keys(str(known_hosts))
        hostname = settings.get('hostname', host)
        for kind, key in (client.get_host_keys().lookup(settings.get('hostkeyalias', hostname)) or {}).items():
            client.get_host_keys().add(hostname, kind, key)
        try:
            client.connect(hostname, port=self.args.ssh_port or int(settings.get('port', 22)), username=self.args.user or settings.get('user'),
                           key_filename=settings.get('identityfile'), timeout=3, banner_timeout=3, auth_timeout=3)
        except Exception:
            client.close();raise
        with self.condition:self.client=client;self.connected_host=host
        client.get_transport().set_keepalive(3)
        # The PC and Jetson wall clocks need not agree. Estimate their offset
        # before measuring SSH backlog, otherwise queued data looks freshly read.
        began = time.time()
        _, clock, _ = client.exec_command("python3 -c 'import time; print(time.time())'", timeout=6)
        remote_time = float(clock.read().strip())
        self.clock_offset_s = (began+time.time())/2-remote_time
        # Read-only subscriptions, passed in memory. Existing files/nodes are untouched.
        source = (Path(__file__).parent/'pc_sensor_reader.py').read_text(encoding='utf-8')
        command = ('. /opt/ros/humble/setup.sh && . /home/isv/ros2_ws/install/setup.sh && '
                   'exec env ROS_DOMAIN_ID='+str(self.args.domain)+' ROS_LOCALHOST_ONLY=0 python3 -u -c ' + shlex.quote(source))
        _, stdout, stderr = client.exec_command('bash -lc ' + shlex.quote(command), timeout=6)
        stdout.channel.settimeout(2.5)
        return stdout, stderr

    def check_dashboard_route(self):
        with self.condition:client=self.client;host=self.connected_host
        if client is None or not self.args.follow_dashboard:return
        try:
            with urllib.request.urlopen(self.args.dashboard+'/api/connection',timeout=.8) as response:state=json.load(response)
            peer=client.get_transport().sock.getpeername()[0]
            matches=state.get('connected') and state.get('active_host')==host and state.get('target_ip')==peer
        except (OSError,ValueError,AttributeError):matches=False
        if not matches:
            with self.condition:
                if self.client is not client:return
            client.close()

    def watch_dashboard_route(self):
        while True:
            try:self.check_dashboard_route()
            except Exception:pass
            time.sleep(.5)

    def run(self):
        if self.args.follow_dashboard:threading.Thread(target=self.watch_dashboard_route,daemon=True).start()
        while True:
            try:
                host = self.dashboard_host()
                stream, errors = self.connect(host)
                next_host_check = time.monotonic()+2
                for line in stream:
                    if self.args.follow_dashboard and time.monotonic() >= next_host_check:
                        next_host_check = time.monotonic()+2
                        if self.dashboard_host() != host:
                            raise ConnectionError('Dashboard connection path changed')
                    if not line.startswith('{'):
                        continue
                    state = json.loads(line)
                    transit = time.time()-state['server_time_s']-self.clock_offset_s
                    if transit < -.5 or transit > 2:
                        raise OSError('Sensor stream clock changed or SSH backlog exceeded 2 s')
                    now = time.monotonic()
                    self.arrivals.append(now)
                    span = self.arrivals[-1]-self.arrivals[0] if len(self.arrivals)>1 else 0
                    with self.condition:
                        self.received_bytes += len(line.encode('utf-8'))
                        self.sequence += 1
                        self.error = None
                        self.updated = now
                        self.transit_age_s = max(0., transit)
                        self.packet = dict(state, sequence=self.sequence, instance_id='pc-readonly-stream',
                                           request_ms=0., feed_hz=round((len(self.arrivals)-1)/span, 1) if span else 0)
                        self.condition.notify_all()
                raise OSError(errors.read(300).decode(errors='replace') or 'Read-only sensor stream closed')
            except Exception as exc:
                with self.condition:
                    self.error = type(exc).__name__ + ': ' + str(exc)[:240]
                    self.arrivals.clear()
                    self.condition.notify_all()
                with self.condition:
                    failed,self.client=self.client,None;self.connected_host=None
                if failed:failed.close()
                time.sleep(1)


def make_handler(feed, args):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def log_message(self, *_):
            pass

        def do_GET(self):
            # Local dashboard only. This server has no mutation/command endpoints.
            origin = self.headers.get('Origin')
            if origin and origin not in (args.dashboard, 'http://127.0.0.1:18765'):
                self.send_error(403)
                return
            if self.path == '/events':
                self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.connection.settimeout(3)
                self.send_response(200)
                self.send_header('Content-Type', 'text/event-stream')
                self.send_header('Access-Control-Allow-Origin', origin or args.dashboard)
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Connection', 'close')
                self.end_headers()
                self.close_connection = True
                previous = -1
                try:
                    self.wfile.write(b'retry: 500\n\n')
                    self.wfile.flush()
                    while True:
                        with feed.condition:
                            feed.condition.wait_for(lambda: feed.sequence != previous, timeout=1)
                            previous = feed.sequence
                            packet = dict(feed.packet, version='kabot-pc-fast-feed-v1', error=feed.error, pc_sent_at_s=time.time(),
                                          age_s=round(time.monotonic()-feed.updated+feed.transit_age_s, 4) if feed.updated else None)
                        payload = json.dumps(packet, separators=(',', ':'), allow_nan=False).encode()
                        self.wfile.write(b'data: '+payload+b'\n\n')
                        self.wfile.flush()
                except (OSError, TimeoutError):
                    return
            if self.path not in ('/state', '/metrics'):
                self.send_error(404)
                return
            with feed.condition:
                packet = dict(feed.packet, version='kabot-pc-fast-feed-v1', error=feed.error, pc_sent_at_s=time.time(), age_s=round(time.monotonic()-feed.updated+feed.transit_age_s, 3) if feed.updated else None)
                if self.path == '/metrics':
                    packet = dict(version='kabot-pc-feed-metrics-v1',
                                  instance_id=feed.instance_id, received_bytes=feed.received_bytes,
                                  connected_host=feed.connected_host, error=feed.error,
                                  age_s=packet['age_s'], feed_hz=feed.packet.get('feed_hz'))
            body = json.dumps(packet, separators=(',', ':'), allow_nan=False).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', origin or args.dashboard)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

    return Handler


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='jetson')
    parser.add_argument('--follow-dashboard', action='store_true')
    parser.add_argument('--user')
    parser.add_argument('--ssh-port', type=int)
    parser.add_argument('--domain', type=int, default=50)
    parser.add_argument('--dashboard', default='http://127.0.0.1:8765')
    parser.add_argument('--port', type=int, default=18766)
    args = parser.parse_args()
    feed = Feed(args)

    server = ThreadingHTTPServer(('127.0.0.1', args.port), make_handler(feed, args))
    threading.Thread(target=feed.run, daemon=True).start()
    print('PC read-only fast feed http://127.0.0.1:'+str(args.port), flush=True)
    server.serve_forever()


if __name__ == '__main__':
    main()
