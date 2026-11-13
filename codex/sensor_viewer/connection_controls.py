"""PC-only connection requests; no ROS or vessel control commands."""
from copy import deepcopy
import os
from pathlib import Path
import shutil
import subprocess
import threading
import time
import uuid

HOSTS={'hotspot':'jetson-hotspot','starlink':'jetson-starlink'}


class ConnectionRequests:
    """One worker consumes the latest request; repeated clicks coalesce."""
    def __init__(self,lock=None,clock=None):
        self.lock=lock or threading.RLock();self.clock=clock or time.monotonic;self.job=None;self.client_sequences={}

    def request(self,route,already_connected=False,client_id=None,sequence=None):
        if route not in HOSTS:raise ValueError('핫스팟 또는 스타링크 연결을 선택하세요')
        if client_id is not None or sequence is not None:
            if not isinstance(client_id,str) or not 1<=len(client_id)<=80 or type(sequence) is not int or not 0<=sequence<=2**53:
                raise ValueError('연결 요청 번호 오류')
        with self.lock:
            if client_id is not None:
                if sequence<=self.client_sequences.get(client_id,-1):return self.snapshot()
                self.client_sequences[client_id]=sequence
                if len(self.client_sequences)>64:self.client_sequences.pop(next(iter(self.client_sequences)))
            now=self.clock();previous=self.job
            if previous and previous['route']==route:
                if previous['state'] in ('queued','connecting','waiting') or (already_connected and previous['state']=='connected'):
                    return self.snapshot()
            self.job=dict(request_id=uuid.uuid4().hex,route=route,host=HOSTS[route],state='queued',
                          error=None,ip=None,vscode=None,updated_at=now,attempts=0,next_attempt_at=now)
            return self.snapshot()

    def claim(self):
        with self.lock:
            if not self.job or self.job['state'] not in ('queued','waiting') or self.clock()<self.job['next_attempt_at']:return None
            self.job['state']='connecting';self.job['attempts']+=1;return deepcopy(self.job)

    def current(self,job):
        with self.lock:return bool(self.job and self.job['request_id']==job['request_id'])

    def finish(self,job,error=None,ip=None,vscode=None):
        with self.lock:
            if not self.current(job):return False
            self.job.update(state='error' if error else 'connected',error=error,ip=ip,vscode=vscode,updated_at=self.clock())
            return True

    def retry(self,job,error):
        with self.lock:
            if not self.current(job):return False
            self.job.update(state='waiting',error=error,updated_at=self.clock(),next_attempt_at=self.clock()+1.)
            return True

    def snapshot(self):
        with self.lock:
            if not self.job:return None
            return dict(deepcopy(self.job),busy=self.job['state'] in ('queued','connecting','waiting'))


class RemoteCodeWindow:
    """Use VS Code's matching-folder reuse, without forcing extra windows."""
    def __init__(self):self.lock=threading.Lock();self.launches={}

    def open(self,host):
        if host not in HOSTS.values():raise ValueError('Unsupported SSH host')
        with self.lock:
            previous=self.launches.get(host)
            if previous and previous.poll() is None:return 'opening'
            cli=shutil.which('code')
            candidates=[Path(cli).parent.parent/'Code.exe'] if cli else []
            candidates.extend([Path(os.environ.get('LOCALAPPDATA',''))/'Programs/Microsoft VS Code/Code.exe',
                               Path(os.environ.get('PROGRAMFILES','C:/Program Files'))/'Microsoft VS Code/Code.exe'])
            resolved=None
            for candidate in candidates:
                if not candidate.is_file():continue
                # Windows updates can keep application resources in a version directory.
                scripts=[candidate.parent/'resources/app/out/cli.js',*candidate.parent.glob('*/resources/app/out/cli.js')]
                scripts=[path for path in scripts if path.is_file()]
                if scripts:
                    resolved=(candidate,max(scripts,key=lambda path:path.stat().st_mtime));break
            if resolved is None:raise OSError('VS Code 실행 파일을 찾지 못했습니다')
            executable,script=resolved
            env=dict(os.environ,ELECTRON_RUN_AS_NODE='1',VSCODE_CLI='1')
            uri='vscode-remote://ssh-remote+'+host+'/home/isv/ros2_ws'
            self.launches[host]=subprocess.Popen([str(executable),str(script),'--folder-uri',uri],
                env=env,stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,
                creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
            return 'opening'
