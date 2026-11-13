"""Passive HTTP view of the vessel-owned camera and persistent switches."""
import json
from pathlib import Path
import threading
import time


class VisionFeed:
    def __init__(self, root=None):
        self.root=Path(root) if root else Path.home()/'water_trials/vision'
        self.lock=threading.RLock()

    def snapshot(self):
        from isv_water_trials.vision_settings import read_json,settings
        with self.lock:
            value=read_json(self.root/'status.json',{}) or {}
            value['settings']=settings(self.root)
        age=time.time()-value.get('written_unix',0)
        value['runtime_fresh']=0<=age<2.
        if not value['runtime_fresh']:
            value.update(camera_running=False,corridor_active=False,error='선체 카메라 처리 노드 대기')
        return value

    def request(self, data):
        from isv_water_trials.vision_settings import set_switch
        try:
            with self.lock:set_switch(data['key'],data['enabled'],self.root)
            return 200,self.snapshot()
        except (KeyError,ValueError,TypeError) as exc:return 400,{'error':str(exc)}
        except OSError as exc:return 503,{'error':'Camera settings write failed: '+str(exc)}

    def jpeg(self):
        if not self.snapshot().get('camera_running'):return b''
        try:return (self.root/'camera.jpg').read_bytes()
        except OSError:return b''
