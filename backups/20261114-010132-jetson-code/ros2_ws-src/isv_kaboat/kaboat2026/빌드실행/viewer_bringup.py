"""Only start and stop this project's sensor bringup."""
from pathlib import Path
import json
import os
import shlex
import signal
import subprocess
import threading
from urllib.parse import urlsplit


class BringupManager:
    def __init__(self,root):
        self.root=Path(root).resolve()
        self.launch=str(self.root/'빌드실행/all_in_one.launch.py')
        self.lock=threading.RLock()
        self.child=None
        self.stopping_pid=None

    def matches(self,pid):
        try:
            args=(Path('/proc')/str(pid)/'cmdline').read_bytes().decode().split('\0')
            env=dict(x.split(b'=',1) for x in (Path('/proc')/str(pid)/'environ').read_bytes().split(bytes([0])) if b'=' in x)
            same_domain=os.fsdecode(env.get(b'ROS_DOMAIN_ID',b'0'))==os.environ.get('ROS_DOMAIN_ID','0')
            return same_domain and self.launch in args and 'components:=bringup' in args
        except OSError:
            return False

    def pid(self):
        if self.child is not None and self.child.poll() is None:
            return self.child.pid
        for path in Path('/proc').iterdir():
            if path.name.isdigit() and self.matches(path.name):
                return int(path.name)
        return None

    def snapshot(self):
        with self.lock:
            pid=self.pid()
            stopping=pid is not None and pid==self.stopping_pid
            return dict(state='STOPPING' if stopping else 'RUNNING' if pid else 'STOPPED',
                        running=pid is not None,pid=pid,can_start=pid is None,
                        can_stop=pid is not None and not stopping,
                        message='종료 중' if stopping else '브링업 실행 중' if pid else '브링업 꺼짐')

    def command(self,action):
        if action not in ('start','stop'):
            raise ValueError('Unsupported bringup action')
        with self.lock:
            pid=self.pid()
            if action=='start' and pid is None:
                script='\n'.join(['set -e','source /opt/ros/humble/setup.bash',
                    'source /home/isv/ros2_ws/install/setup.bash',
                    'source '+shlex.quote(str(self.root/'.runtime/install/setup.bash')),
                    'exec ros2 launch '+shlex.quote(self.launch)+
                    ' mode:=preview components:=bringup gui:=none'])
                self.child=subprocess.Popen(['/bin/bash','-c',script],cwd=self.root,
                    stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,
                    stderr=subprocess.STDOUT,start_new_session=True)
                self.stopping_pid=None
            elif action=='stop' and pid is not None:
                if self.matches(pid) or self.child is not None and self.child.pid==pid and self.child.poll() is None:
                    os.kill(pid,signal.SIGINT)
                    self.stopping_pid=pid
            return dict(self.snapshot(),success=True)
    def http_command(self,handler):
        host=handler.headers.get('Host','')
        try:
            url=urlsplit('http://'+host)
            valid=(url.hostname in ('localhost','127.0.0.1') and url.port is not None
                   and not url.username and not url.password and not url.path and not url.query and not url.fragment)
        except ValueError:valid=False
        if (not valid or handler.headers.get('Origin')!='http://'+host
                or handler.headers.get('Content-Type','').split(';')[0]!='application/json'):
            handler.send(403,b'{"error":"same-origin JSON required"}');return
        try:
            size=int(handler.headers.get('Content-Length','0'))
            if not 0<size<=1024:raise ValueError('Invalid request size')
            data=json.loads(handler.rfile.read(size))
            if not isinstance(data,dict):raise ValueError('Invalid bringup request')
            result=self.command(data.get('action'))
            handler.send(200,json.dumps(result,ensure_ascii=False).encode())
        except (ValueError,TypeError) as exc:handler.send(400,json.dumps(dict(error=str(exc))).encode())
        except (RuntimeError,OSError) as exc:handler.send(409,json.dumps(dict(error=str(exc))).encode())
