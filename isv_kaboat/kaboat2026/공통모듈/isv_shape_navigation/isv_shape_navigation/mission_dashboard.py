"""Local-only dashboard bridge; ROS commands are consumed by the ROS timer.

No arbitrary shell commands, training or remote filesystem access. State and
camera are read-only HTTP resources. POSTs require same-origin browser access.
"""
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
from queue import Queue, Full
import threading
import time
import mimetypes


class Dashboard:
    def __init__(self, web_root, port=8770):
        self.web_root=Path(web_root).resolve()
        self.lock=threading.Lock()
        self.commands=Queue(maxsize=16)
        self.report={'state':'WAIT_NODE','output':{'enabled':False}}
        self.jpeg=None
        self.jpeg_at=0.
        self.history=deque(maxlen=600)
        self.events=deque(maxlen=30)
        self.previous_state=None
        self.last_command=None
        bridge=self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self,*_):pass
            def send(self,code,payload,kind='application/json; charset=utf-8'):
                self.send_response(code)
                self.send_header('Content-Type',kind)
                self.send_header('Content-Length',str(len(payload)))
                self.send_header('Cache-Control','no-store')
                self.send_header('X-Content-Type-Options','nosniff')
                self.end_headers()
                try:self.wfile.write(payload)
                except (BrokenPipeError,ConnectionResetError):pass
            def do_GET(self):
                if self.path=='/api/state':
                    with bridge.lock:
                        value=dict(bridge.report,history=list(bridge.history),events=list(bridge.events),
                                   last_command=bridge.last_command,
                                   report_age=time.monotonic()-bridge.updated if hasattr(bridge,'updated') else None,
                                   camera_age=time.monotonic()-bridge.jpeg_at if bridge.jpeg else None)
                    self.send(200,json.dumps(value,allow_nan=False).encode())
                elif self.path.split('?')[0]=='/camera.jpg':
                    with bridge.lock:jpeg=bridge.jpeg
                    self.send(200 if jpeg else 204,jpeg or b'','image/jpeg')
                else:
                    names={'/':'index.html','/app.js':'app.js','/style.css':'style.css'}
                    name=names.get(self.path)
                    path=bridge.web_root/name if name else None
                    if path and path.is_file():
                        kind={'html':'text/html','js':'application/javascript','css':'text/css'}[path.suffix[1:]]
                        self.send(200,path.read_bytes(),kind+'; charset=utf-8')
                    else:self.send(404,b'{}')
            def do_POST(self):
                host=self.headers.get('Host','')
                allowed={f'127.0.0.1:{bridge.port}',f'localhost:{bridge.port}'}
                origin=self.headers.get('Origin')
                if (host not in allowed or origin!='http://'+host
                        or self.headers.get('Content-Type','').split(';')[0]!='application/json'):
                    self.send(403,b'{"error":"same-origin JSON required"}');return
                try:
                    size=int(self.headers.get('Content-Length','0'))
                    if not 0<size<=8192:raise ValueError('Invalid request size')
                    request=json.loads(self.rfile.read(size))
                    if self.path!='/api/command' or not isinstance(request,dict):raise ValueError('Invalid request')
                    if request.get('action') not in ('start','stop','reset','set_route','capture_wp','set_target'):
                        raise ValueError('Unknown action')
                    bridge.commands.put_nowait(request)
                    self.send(202,b'{"queued":true}')
                except (ValueError,TypeError,Full):self.send(400,b'{"error":"Invalid command or full queue"}')
        self.server=ThreadingHTTPServer(('127.0.0.1',port),Handler)
        self.port=self.server.server_address[1]
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True)
        self.thread.start()

    def publish(self,report):
        with self.lock:
            self.report=report
            self.updated=time.monotonic()
            self.history.append({'stamp':report['stamp'],'pwm':report['intended_pwm'],
                                 'feedback':report.get('feedback_pwm')})
            state=report['state']
            if state!=self.previous_state:
                self.events.appendleft({'time':time.strftime('%H:%M:%S'),'state':state})
                self.previous_state=state

    def camera(self,jpeg):
        with self.lock:self.jpeg,self.jpeg_at=jpeg,time.monotonic()

    def close(self):
        self.server.shutdown();self.server.server_close();self.thread.join(timeout=2)
