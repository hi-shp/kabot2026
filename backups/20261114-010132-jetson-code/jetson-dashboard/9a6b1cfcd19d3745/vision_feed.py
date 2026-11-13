"""Passive HTTP view of the vessel-owned camera and persistent switches."""
import json
from pathlib import Path
import threading
import time
import subprocess
import sys


class VisionFeed:
    def __init__(self, root=None):
        self.root=Path(root) if root else Path.home()/'water_trials/vision'
        self.lock=threading.RLock()
        self.last_worker_attempt=0.

    def ensure_worker(self,config):
        from isv_water_trials.vision_settings import navigation_camera_owner
        if not config.get('camera') or navigation_camera_owner(self.root):return
        if time.monotonic()-self.last_worker_attempt<3:return
        self.last_worker_attempt=time.monotonic();self.root.mkdir(parents=True,exist_ok=True)
        with (self.root/'worker.log').open('a') as log:
            subprocess.Popen([sys.executable,'-m','isv_water_trials.buoy_vision','_camera_worker'],
                stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)

    def snapshot(self):
        from isv_water_trials.vision_settings import read_json,settings
        with self.lock:
            value=read_json(self.root/'status.json',{}) or {}
            value['settings']=settings(self.root)
            value['camera_selection_schema']=1
            value['camera_fps_schema']=1
        age=time.time()-value.get('written_unix',0)
        value['runtime_fresh']=0<=age<2.
        if not value['runtime_fresh']:
            value.update(camera_running=False,corridor_active=False,error='선체 카메라 처리 노드 대기')
            self.ensure_worker(value['settings'])
        if value.get('camera_backend')!=value['settings']['camera_backend']:
            value.update(camera_running=False,error='카메라 전환 중' if value['settings']['camera'] else '')
        return value

    def request(self, data):
        from isv_water_trials.vision_settings import set_switch,select_camera,set_camera_fps
        try:
            with self.lock:
                if data['key']=='fps':
                    config=set_camera_fps(data['value'],self.root)
                elif data['key']=='camera' and 'backend' in data:
                    config=select_camera(data['backend'],data['enabled'],self.root)
                else:config=set_switch(data['key'],data['enabled'],self.root)
                self.ensure_worker(config)
            return 200,self.snapshot()
        except (KeyError,ValueError,TypeError) as exc:return 400,{'error':str(exc)}
        except OSError as exc:return 503,{'error':'Camera settings write failed: '+str(exc)}

    def jpeg(self):
        if not self.snapshot().get('camera_running'):return b''
        try:return (self.root/'camera.jpg').read_bytes()
        except OSError:return b''
