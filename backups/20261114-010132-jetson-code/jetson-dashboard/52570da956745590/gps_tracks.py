"""Saved GPS reference paths backed by the vessel's durable position journal."""
from datetime import datetime, timedelta, timezone
import json
import math
import os
from pathlib import Path
import re
import sqlite3
import tempfile
import threading
import time
import uuid


KST = timezone(timedelta(hours=9))
SCHEMA = 'isv-gps-path-v1'


def atomic_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent, delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(value, stream, ensure_ascii=False, allow_nan=False, separators=(',', ':'))
            stream.flush(); os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary is not None: temporary.unlink(missing_ok=True)


def read_json(path, default=None):
    try: return json.loads(path.read_text(encoding='utf-8'))
    except (OSError, ValueError): return default


def preview(rows, limit=300):
    if len(rows)>limit:
        rows = [rows[round(i*(len(rows)-1)/(limit-1))] for i in range(limit)]
    segments = {}
    return [[p[0], p[1], segments.setdefault(p[2], len(segments)), p[3]] for p in rows]


class GpsPaths:
    def __init__(self, root=None, journal=None, clock=time.time):
        self.root = Path(root) if root else Path.home()/'water_trials/gps_paths'
        self.journal = Path(journal) if journal else Path.home()/'water_trials/outdoor/observations.sqlite3'
        self.clock = clock
        self.lock = threading.RLock()
        self.updated = -math.inf
        self.cached = {}
        self.reference_key = None
        self.reference = None
        self.stopped_key = None
        self.stopped_preview = None

    def _connect(self):
        if not self.journal.is_file(): return None
        connection = sqlite3.connect(self.journal.resolve().as_uri()+'?mode=ro', uri=True, timeout=.5)
        connection.execute('PRAGMA query_only=ON')
        return connection

    def _rows(self, active, end=None, sampled=False):
        connection = self._connect()
        if connection is None: return [], 0
        try:
            where = 'id>? AND stamp>=?'
            params = [active['after_id'], active['started_stamp']]
            if end is not None: where += ' AND stamp<=?'; params.append(end)
            try:
                count, first, last = connection.execute(
                    'SELECT COUNT(*),MIN(id),MAX(id) FROM positions WHERE '+where, params).fetchone()
            except sqlite3.OperationalError as exc:
                if 'no such table' in str(exc): return [], 0
                raise
            if not count: return [], 0
            sample = ''; sample_params = []
            if sampled and count>300:
                stride = max(1, math.ceil((last-first+1)/298))
                sample = ' AND (id=? OR id=? OR (id-?) % ?=0)'
                sample_params = [first, last, first, stride]
            rows = connection.execute('SELECT lat,lon,segment,stamp FROM positions WHERE '+where+sample+
                                      ' ORDER BY id', params+sample_params).fetchall()
            return rows, count
        finally: connection.close()

    def start(self, stamp=None):
        with self.lock:
            stamp = self.clock() if stamp is None else stamp
            connection = self._connect()
            after_id = 0
            if connection is not None:
                try:
                    try: after_id = connection.execute('SELECT COALESCE(MAX(id),0) FROM positions').fetchone()[0]
                    except sqlite3.OperationalError as exc:
                        if 'no such table' not in str(exc): raise
                finally: connection.close()
            active = {'id':uuid.uuid4().hex, 'started_stamp':stamp, 'after_id':after_id,'recording':True,
                      'started_kst':datetime.fromtimestamp(stamp,KST).isoformat(timespec='seconds')}
            atomic_json(self.root/'.active.json', active)
            self.updated = -math.inf
            return active

    def stop(self):
        with self.lock:
            active=read_json(self.root/'.active.json')
            if active and (active.get('recording') is not False or 'ended_stamp' not in active):
                active=dict(active,recording=False,ended_stamp=self.clock())
                atomic_json(self.root/'.active.json',active)
                self.updated=-math.inf;self.stopped_key=None
            return active

    def save(self):
        with self.lock:
            active = read_json(self.root/'.active.json')
            if not active: raise ValueError('Create a new GPS path first.')
            active=self.stop()
            end = active['ended_stamp']
        # Allow the onboard writer's periodic commit to include the save boundary.
        time.sleep(1.6)
        rows, count = self._rows(active, end=end)
        saved = datetime.fromtimestamp(end,KST)
        name = 'gps_path_'+saved.strftime('%Y%m%d_%H%M%S_%f')+'_'+uuid.uuid4().hex[:6]+'.gpspath.json'
        metadata = dict(schema=SCHEMA, name=name, session_id=active['id'],
                        started_kst=active['started_kst'], started_stamp=active['started_stamp'],
                        saved_kst=saved.isoformat(timespec='seconds'), ended_stamp=end, point_count=count)
        atomic_json(self.root/name, dict(metadata, points=rows))
        atomic_json(self.root/(name+'.meta.json'), metadata)
        self.updated=-math.inf;self.stopped_key=None
        return dict(metadata, path=str(self.root/name))

    def files(self):
        result = []
        for path in sorted(self.root.glob('gps_path_*.gpspath.json'), reverse=True):
            meta = read_json(self.root/(path.name+'.meta.json'))
            if not meta:
                data = read_json(path)
                if isinstance(data,dict) and data.get('schema')==SCHEMA:
                    meta = {k:v for k,v in data.items() if k!='points'}
            if meta: result.append(meta)
        return result

    def _path(self, name):
        if not isinstance(name,str) or not re.fullmatch(r'gps_path_[0-9_]+[0-9a-f]{6}\.gpspath\.json', name):
            raise ValueError('Invalid GPS path filename.')
        path = self.root/name
        if path.is_symlink() or not path.is_file(): raise ValueError('GPS path file is missing.')
        return path

    def _load_reference(self, name):
        data = read_json(self._path(name))
        if not isinstance(data,dict) or data.get('schema')!=SCHEMA: raise ValueError('Invalid GPS path file.')
        points = data.get('points')
        if not isinstance(points,list): raise ValueError('Invalid GPS path points.')
        for p in points:
            if (not isinstance(p,list) or len(p)!=4 or
                not isinstance(p[2],(str,int)) or
                not all(isinstance(p[i],(int,float)) and math.isfinite(p[i]) for i in (0,1,3)) or
                not -90<=p[0]<=90 or not -180<=p[1]<=180): raise ValueError('Invalid GPS coordinates.')
        return dict(name=name, point_count=len(points), points=preview(points),
                    started_kst=data.get('started_kst'), saved_kst=data.get('saved_kst'))

    def load(self, name):
        with self.lock:
            reference = self._load_reference(name)
            atomic_json(self.root/'.reference.json', {'name':name})
            self.reference_key = name; self.reference = reference
            self.updated = -math.inf
            return reference

    def hide(self):
        with self.lock:
            atomic_json(self.root/'.reference.json', {'name':None})
            self.reference_key = None; self.reference = None; self.updated = -math.inf

    def marks(self):
        with self.lock:
            value=read_json(self.root/'marks.json',{'schema':'isv-gps-marks-v1','points':[]})
            if not isinstance(value,dict) or not isinstance(value.get('points'),list):
                raise ValueError('Invalid GPS marks file.')
            return value

    def mark(self, latitude, longitude, request_id=None, source='gps_space_dashboard'):
        latitude=float(latitude);longitude=float(longitude)
        if not math.isfinite(latitude) or not math.isfinite(longitude) or not -90<=latitude<=90 or not -180<=longitude<=180:
            raise ValueError('Invalid GPS coordinates.')
        with self.lock:
            value=self.marks()
            if request_id and any(p.get('request_id')==request_id for p in value['points']):return value
            stamp=self.clock()
            number=max(value.get('next_number',1),max((p['number'] for p in value['points']),default=0)+1)
            value['next_number']=number+1
            value['points'].append(dict(id=uuid.uuid4().hex,number=number,lat=latitude,lon=longitude,
                stamp=stamp,recorded_kst=datetime.fromtimestamp(stamp,KST).isoformat(timespec='seconds'),
                request_id=request_id,source=source))
            atomic_json(self.root/'marks.json',value)
            return value

    def delete_marks(self,ids):
        if not isinstance(ids,list) or not all(isinstance(identity,str) for identity in ids):
            raise ValueError('삭제할 기록점을 선택하세요.')
        with self.lock:
            value=self.marks();selected=set(ids)
            removed=[p for p in value['points'] if p['id'] in selected]
            if not removed:return value
            value['next_number']=max(value.get('next_number',1),max((p['number'] for p in value['points']),default=0)+1)
            with (self.root/'deleted_marks.jsonl').open('a',encoding='utf-8') as stream:
                stream.write(json.dumps(dict(deleted_stamp=self.clock(),points=removed),ensure_ascii=False,allow_nan=False)+'\n')
            value['points']=[p for p in value['points'] if p['id'] not in selected]
            atomic_json(self.root/'marks.json',value)
            return value

    def snapshot(self):
        with self.lock:
            if time.monotonic()-self.updated<1.: return self.cached
            active = read_json(self.root/'.active.json')
            if active and 'recording' not in active:active=self.stop()
            error = None
            if active:
                try:
                    key=(active['id'],active.get('ended_stamp'))
                    stopped=active.get('recording') is not True
                    if stopped and key==self.stopped_key:
                        rows,count=self.stopped_preview
                    else:
                        rows, count = self._rows(active,end=active.get('ended_stamp') if stopped else None,sampled=True)
                        # Re-read once after the writer's final periodic commit.
                        if stopped and self.clock()-active['ended_stamp']>2.:
                            self.stopped_key=key;self.stopped_preview=(rows,count)
                    active = dict(active, point_count=count, points=preview(rows))
                except (sqlite3.Error,OSError,KeyError) as exc:
                    active = dict(active, point_count=0, points=[]); error = str(exc)
            name = (read_json(self.root/'.reference.json',{}) or {}).get('name')
            if name!=self.reference_key:
                try: self.reference = self._load_reference(name) if name else None
                except (ValueError,OSError) as exc: self.reference = None; error = str(exc)
                self.reference_key = name
            self.cached = {'active':active, 'reference':self.reference, 'error':error}
            self.updated = time.monotonic()
            return self.cached
