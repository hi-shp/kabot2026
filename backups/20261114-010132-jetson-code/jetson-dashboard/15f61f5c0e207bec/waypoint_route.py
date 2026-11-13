"""GPS route metadata and progress; coordinates stay in the existing format."""
import math
from datetime import datetime,timezone,timedelta
import hashlib
import json
from pathlib import Path
import re
import time
import uuid

KINDS=('path','point','hold','dock')
HOLD_SECONDS=5.


class WaypointRoutes:
    def __init__(self,route_path=None):
        self.route_path=Path(route_path) if route_path else Path.home()/'water_trials/outdoor/route.json'
        self.root=self.route_path.parent.parent/'waypoint_routes'

    def read(self,name):
        if not isinstance(name,str) or not re.fullmatch(r'route_[0-9_]+[0-9a-f]{12}\.waypoints\.json',name):
            raise ValueError('저장된 경로를 선택하세요')
        path=self.root/name
        if path.is_symlink():raise ValueError('Invalid route file')
        value=json.loads(path.read_text(encoding='utf-8'))
        value['route']=normalize_route(value['route'],allow_draft=True)
        return value

    def files(self):
        rows=[]
        for path in sorted(self.root.glob('route_*.waypoints.json'),reverse=True):
            try:rows.append({k:v for k,v in self.read(path.name).items() if k!='route'})
            except (OSError,ValueError,KeyError,TypeError):continue
        return rows

    def selected(self):
        try:return self.read(json.loads((self.root/'.active.json').read_text())['name'])
        except (OSError,ValueError,KeyError,TypeError):return None

    def select(self,profile):
        target=self.root/'.active.json';temporary=target.with_name('.active.'+uuid.uuid4().hex+'.tmp')
        try:
            temporary.write_text(json.dumps({'name':profile['name']}),encoding='utf-8');temporary.replace(target)
        finally:temporary.unlink(missing_ok=True)

    def save(self,data,label=None,source=None,stamp=None,name=None):
        route=normalize_route(data,allow_draft=True)
        existing=self.read(name) if name else None
        identity=hashlib.sha256(json.dumps(route,sort_keys=True,separators=(',',':')).encode()).hexdigest()[:12]
        if not existing:
            for row in self.files():
                value=self.read(row['name'])
                if value['route']==route:return value
        stamp=time.time() if stamp is None else stamp
        saved=datetime.fromtimestamp(stamp,timezone(timedelta(hours=9)))
        name=existing['name'] if existing else 'route_'+saved.strftime('%Y%m%d_%H%M%S_')+identity+'.waypoints.json'
        value=dict(existing or {},schema='isv-waypoint-route-v1',name=name,label=label or (existing or {}).get('label') or '다중 웨이포인트 경로',
            saved_kst=saved.isoformat(timespec='seconds'),point_count=len(route['points']),route=route,
            source=source or (existing or {}).get('source'),revision=uuid.uuid4().hex)
        value['created_kst']=(existing or {}).get('created_kst',(existing or {}).get('saved_kst',value['saved_kst']))
        value['draft']=any(p is None for p in route['points'])
        self.root.mkdir(parents=True,exist_ok=True)
        target=self.root/name;temporary=target.with_name(target.name+'.'+uuid.uuid4().hex+'.tmp')
        try:
            temporary.write_text(json.dumps(value,ensure_ascii=False,allow_nan=False),encoding='utf-8')
            temporary.replace(target)
        finally:temporary.unlink(missing_ok=True)
        return value


def normalize_route(data,allow_draft=False):
    points=data.get('points') if isinstance(data,dict) else None
    if (not isinstance(points,list) or not 1<=len(points)<=100 or
        any((p is None and not allow_draft) or (p is not None and (not isinstance(p,(list,tuple)) or len(p)!=2 or
            any(type(v) not in (int,float) or not math.isfinite(v) for v in p) or
            not -85<p[0]<85 or not -180<=p[1]<=180)) for p in points)):
        raise ValueError('위도·경도로 1~100개의 지점을 지정하세요')
    for a,b in zip(points,points[1:]):
        if a is None or b is None:continue
        east=math.radians(b[1]-a[1])*6371000*math.cos(math.radians(a[0]))
        north=math.radians(b[0]-a[0])*6371000
        if not 1<=math.hypot(east,north)<=200:
            raise ValueError('이웃한 지점의 거리는 1~200m로 지정하세요')
    kinds=data.get('waypoint_types',['path']*len(points))
    if not isinstance(kinds,list) or len(kinds)!=len(points) or any(k not in KINDS for k in kinds):
        raise ValueError('각 지점의 종류를 경로추종·위치추종·위치유지·도킹 중에서 지정하세요')
    seconds=[HOLD_SECONDS if kind=='hold' else 0. for kind in kinds]
    result={'points':[list(p) if p is not None else None for p in points],'waypoint_types':list(kinds),'hold_seconds':list(seconds)}
    if 'waypoint_outputs' in data:
        outputs=data['waypoint_outputs']
        if (not isinstance(outputs,list) or len(outputs)!=len(points) or
            any(v is not None and (type(v) not in (int,float) or not math.isfinite(v) or not 0<=v<=100) for v in outputs)):
            raise ValueError('지점별 출력은 0~100% 또는 빈칸으로 입력하세요')
        result['waypoint_outputs']=list(outputs)
    if data.get('mission')=='tournament':result['mission']='tournament'
    return result


def waypoint_output(route,index,default):
    outputs=route.get('waypoint_outputs',[]) if route else []
    value=outputs[index] if 0<=index<len(outputs) else None
    return float(default if value is None else value)


def waypoint_kind(route,index):
    kinds=route.get('waypoint_types',[]) if route else []
    return kinds[index] if 0<=index<len(kinds) else 'path'


class WaypointProgress:
    def __init__(self):self.reset()

    def reset(self):
        self.index=None;self.entered=None;self.reached=False;self.inside=False

    def update(self,route,index,inside,now,arrival_allowed=True):
        if self.index!=index:
            self.reset();self.index=index
        kind=waypoint_kind(route,index)
        self.inside=inside
        if not inside and not (kind=='hold' and self.reached):
            self.entered=None
            return 'approach'
        if not self.reached and not arrival_allowed:return 'approach'
        self.reached=True
        if kind=='dock':return 'dock_wait'
        if kind=='hold':
            if self.entered is None:self.entered=now
            if now-self.entered<HOLD_SECONDS:return 'hold'
            return 'advance' if index+1<len(route['points']) else 'complete'
        return 'advance' if index+1<len(route['points']) else 'final_hold'

    def snapshot(self,route,index,now):
        kind=waypoint_kind(route,index)
        seconds=HOLD_SECONDS if kind=='hold' else 0.
        holding=self.index==index and self.entered is not None and kind=='hold'
        return {'kind':kind,'index':index,'holding':holding,
                'returning_to_hold':False,
                'waiting_for_map':self.index==index and self.reached and self.inside and kind=='dock',
                'hold_seconds':seconds,'remaining_s':max(0.,seconds-(now-self.entered)) if seconds and self.entered is not None else None}
