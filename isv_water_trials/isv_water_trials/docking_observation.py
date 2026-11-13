"""Read-only docking references and bounded, durable onboard comparison logs."""
from copy import deepcopy
from datetime import datetime,timezone
import json
import math
from pathlib import Path
from queue import Queue,Empty,Full
import re
import sqlite3
import threading
import time
import zlib
from .gps_docking import GpsDocking,rotate


def compare_targets(route,map_pose,gps_pose,origin,registered_a,arm=(0.,0.),lidar_yaw=0.):
    conversion=GpsDocking(route,map_pose,gps_pose,origin,arm,lidar_yaw)
    offset=rotate(arm,gps_pose['yaw'])
    targets={name:[p[0]-offset[0],p[1]-offset[1]] for name,p in zip(conversion.names,conversion.targets)}
    expected=targets.get('A')
    delta=[expected[i]-registered_a[i] for i in range(2)] if expected else None
    relative=rotate((registered_a[0]+offset[0]-conversion.anchor[0],
                     registered_a[1]+offset[1]-conversion.anchor[1]),-conversion.rotation)
    return dict(estimated_targets_enu_m=targets,registered_a_enu_m=list(registered_a),
        a_gps_error_enu_m=delta,a_gps_error_m=math.hypot(*delta) if delta else None,
        registered_a_map_m=[conversion.map_anchor[0]+relative[0],conversion.map_anchor[1]+relative[1]],
        transform=conversion.snapshot())


def read_profile(home,map_id,version):
    choices=[]
    for path in (Path(home)/'water_trials/docking_routes').glob('*.dockroute.json'):
        try:
            profile=json.loads(path.read_text(encoding='utf-8'))
            if profile.get('map_id')!=map_id or profile.get('version')!=version:continue
            route=profile['route']
            if route.get('session_id')!=map_id:continue
            if not all(math.isfinite(float(route[name][axis])) for name in ('a','b') for axis in ('x','y')):continue
            choices.append((float(profile.get('saved_unix',0)),str(path),profile))
        except (OSError,ValueError,KeyError,TypeError):continue
    if not choices:raise ValueError('현재 지도의 '+version+' A/B 도킹 경로를 저장하세요')
    _,path,profile=max(choices,key=lambda item:item[0])
    return dict(profile['route'],profile_file=path,profile_version=version)


def clean(value):
    if isinstance(value,dict):return {str(k):clean(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [clean(v) for v in value]
    if isinstance(value,float):return value if math.isfinite(value) else None
    if hasattr(value,'tolist'):return clean(value.tolist())
    return value


class DockingAudit:
    """One low-priority writer; no filesystem operation can block propulsion."""
    def __init__(self,root):
        self.root=Path(root);self.queue=Queue(maxsize=256);self.dropped=0;self.error='';self.path=None
        self.thread=threading.Thread(target=self.work,name='docking-audit',daemon=True);self.thread.start()

    def append(self,run_id,row):
        try:self.queue.put_nowait((run_id,deepcopy(row)))
        except Full:self.dropped+=1

    def status(self):
        return dict(path=str(self.path) if self.path else None,queued=self.queue.qsize(),dropped=self.dropped,error=self.error)

    def work(self):
        db=None;current=None;committed=time.monotonic()
        try:
            while True:
                try:item=self.queue.get(timeout=.5)
                except Empty:
                    if db is not None and time.monotonic()-committed>=1:db.commit();committed=time.monotonic()
                    continue
                if item is None:break
                run_id,row=item
                try:
                    if run_id!=current or db is None:
                        if db is not None:db.commit();db.close();db=None
                        self.root.mkdir(parents=True,exist_ok=True)
                        token=re.sub('[^a-zA-Z0-9_-]','_',str(run_id))[:80]
                        self.path=self.root/('dock_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+token+'.sqlite3')
                        db=sqlite3.connect(self.path);db.execute('PRAGMA journal_mode=WAL');db.execute('PRAGMA synchronous=FULL')
                        db.execute('CREATE TABLE IF NOT EXISTS observations(id INTEGER PRIMARY KEY, stamp REAL, kind TEXT, payload_zlib BLOB)')
                        current=run_id
                    data=clean(row);payload=zlib.compress(json.dumps(data,ensure_ascii=False,allow_nan=False,separators=(',',':')).encode(),1)
                    db.execute('INSERT INTO observations(stamp,kind,payload_zlib) VALUES(?,?,?)',(data.get('stamp_s',time.time()),data.get('kind','sample'),payload))
                    if time.monotonic()-committed>=1:db.commit();committed=time.monotonic()
                    self.error=''
                except (OSError,sqlite3.Error,ValueError,TypeError) as exc:
                    self.error=str(exc)
                    if db is not None:
                        try:db.close()
                        except sqlite3.Error:pass
                        db=None
        except (OSError,sqlite3.Error) as exc:self.error=str(exc)
        finally:
            if db is not None:
                try:db.commit();db.close()
                except sqlite3.Error as exc:self.error=str(exc)

    def close(self):
        try:self.queue.put(None,timeout=2.)
        except Full:return
        self.thread.join(timeout=3.)
