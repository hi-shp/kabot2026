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

    def connect(self):
        config = paramiko.SSHConfig()
        with (Path.home()/'.ssh/config').open(encoding='utf-8') as stream:
            config.parse(stream)
        settings = config.lookup(self.args.host)
        client = paramiko.SSHClient()
        client.load_system_host_keys()
        hostname = settings.get('hostname', self.args.host)
        for kind, key in (client.get_host_keys().lookup(settings.get('hostkeyalias', hostname)) or {}).items():
            client.get_host_keys().add(hostname, kind, key)
        client.connect(hostname, port=self.args.ssh_port or int(settings.get('port', 22)), username=self.args.user or settings.get('user'),
                       key_filename=settings.get('identityfile'), timeout=6, auth_timeout=6)
        self.client = client
        client.get_transport().set_keepalive(15)
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
        return stdout, stderr

    def run(self):
        while True:
            try:
                stream, errors = self.connect()
                for line in stream:
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
                if self.client:
                    self.client.close()
                time.sleep(2)


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
            if self.path != '/state':
                self.send_error(404)
                return
            with feed.condition:
                packet = dict(feed.packet, version='kabot-pc-fast-feed-v1', error=feed.error, pc_sent_at_s=time.time(), age_s=round(time.monotonic()-feed.updated+feed.transit_age_s, 3) if feed.updated else None)
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
