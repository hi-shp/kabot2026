"""GPS route metadata and progress; coordinates stay in the existing format."""
import math

KINDS=('path','hold','dock')
HOLD_SECONDS=3.


def normalize_route(data):
    points=data.get('points') if isinstance(data,dict) else None
    if (not isinstance(points,list) or not 1<=len(points)<=100 or
        any(not isinstance(p,(list,tuple)) or len(p)!=2 or
            any(type(v) not in (int,float) or not math.isfinite(v) for v in p) or
            not -85<p[0]<85 or not -180<=p[1]<=180 for p in points)):
        raise ValueError('위도·경도로 1~100개의 지점을 지정하세요')
    for a,b in zip(points,points[1:]):
        east=math.radians(b[1]-a[1])*6371000*math.cos(math.radians(a[0]))
        north=math.radians(b[0]-a[0])*6371000
        if not 1<=math.hypot(east,north)<=200:
            raise ValueError('이웃한 지점의 거리는 1~200m로 지정하세요')
    kinds=data.get('waypoint_types',['path']*len(points))
    if not isinstance(kinds,list) or len(kinds)!=len(points) or any(k not in KINDS for k in kinds):
        raise ValueError('각 지점의 종류를 경로추종·위치유지·도킹 중에서 지정하세요')
    seconds=[HOLD_SECONDS if kind=='hold' else 0. for kind in kinds]
    return {'points':[list(p) for p in points],'waypoint_types':list(kinds),'hold_seconds':list(seconds)}


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
        if not inside:
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
                'returning_to_hold':self.index==index and self.reached and not self.inside and kind=='hold',
                'waiting_for_map':self.index==index and self.reached and self.inside and kind=='dock',
                'hold_seconds':seconds,'remaining_s':max(0.,seconds-(now-self.entered)) if seconds and self.entered is not None else None}
