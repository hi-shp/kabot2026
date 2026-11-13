"""GPS route metadata and progress; coordinates stay in the existing format."""
import math
from .buoy_orbit import normalize_orbit

KINDS=('path','point','hold','dock','dock_left','dock_middle','dock_right','orbit')
GPS_DOCK_KINDS={'dock_left':'left','dock_middle':'middle','dock_right':'right'}
HOLD_SECONDS=5.
DOCK_SEARCH_SECONDS=120.


class ReverseStop:
    """Time the normal-stop pulse from its first dispatched throttle command."""
    def __init__(self):self.reset()

    def reset(self):self.requested=None;self.started=None;self.duration=.5

    @property
    def active(self):return self.requested is not None

    def begin(self,now,duration=.5):self.requested=now;self.started=None;self.duration=duration

    def sent(self,now):
        if self.started is None:self.started=now

    def finished(self,now):
        return (now-self.started>=self.duration if self.started is not None
                else now-self.requested>=.6)


def docking_search_command(state,percent,elapsed,clear,previous=(0.,0.),slew=.35):
    """Pulse a low-speed pivot with zero surge between fresh scan matches."""
    if not clear or elapsed<.5 or (elapsed-.5)%1.5>=1.:
        return [0.,0.]
    cap=min(.08,max(0.,float(percent)/100.))
    velocity=state.get('velocity')
    rate=float(velocity[2]) if velocity is not None else float(state.get('yaw_rate_rad_s',0.))
    target=max(0.,min(cap,cap*(1.-rate/math.radians(12.))))
    old=max(-cap,min(cap,(previous[1]-previous[0])/2.))
    turn=max(0.,old+max(-slew*.1,min(slew*.1,target-old)))
    return [-turn,turn]


def normalize_route(data,allow_draft=False):
    points=data.get('points') if isinstance(data,dict) else None
    if (not isinstance(points,list) or not 1<=len(points)<=100 or
        any((p is None and not allow_draft) or (p is not None and (not isinstance(p,(list,tuple)) or len(p)!=2 or
            any(type(v) not in (int,float) or not math.isfinite(v) for v in p) or
            not -85<p[0]<85 or not -180<=p[1]<=180)) for p in points)):
        raise ValueError('위도·경도로 1~100개의 지점을 지정하세요')
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
    if 'waypoint_orbits' in data or 'orbit' in kinds:
        orbits=data.get('waypoint_orbits',[None]*len(points))
        if not isinstance(orbits,list) or len(orbits)!=len(points):raise ValueError('각 지점의 선회 설정 개수를 확인하세요')
        result['waypoint_orbits']=[]
        for i,value in enumerate(orbits):
            cfg=normalize_orbit(value,points[i],allow_draft=allow_draft) if value is not None or kinds[i]=='orbit' else None
            result['waypoint_orbits'].append(cfg)
            if kinds[i]=='orbit' and cfg['selected'] is not None:
                chosen=next(p['point'] for p in cfg['options'] if p['id']==cfg['selected'])
                result['points'][i]=list(chosen) if chosen is not None else None
    for a,b in zip(result['points'],result['points'][1:]):
        if a is None or b is None:continue
        east=math.radians(b[1]-a[1])*6371000*math.cos(math.radians(a[0]))
        north=math.radians(b[0]-a[0])*6371000
        if not 1<=math.hypot(east,north)<=200:
            raise ValueError('이웃한 지점의 거리는 1~200m로 지정하세요')
    if data.get('mission')=='tournament':result['mission']='tournament'
    return result


def waypoint_output(route,index,default):
    outputs=route.get('waypoint_outputs',[]) if route else []
    value=outputs[index] if 0<=index<len(outputs) else None
    return float(default if value is None else value)


def waypoint_kind(route,index):
    kinds=route.get('waypoint_types',[]) if route else []
    kind=kinds[index] if 0<=index<len(kinds) else 'path'
    return 'dock' if kind in GPS_DOCK_KINDS else kind


def gps_docking_version(route,index):
    kinds=route.get('waypoint_types',[]) if route else []
    return GPS_DOCK_KINDS.get(kinds[index]) if 0<=index<len(kinds) else None


class WaypointProgress:
    def __init__(self):self.reset()

    def reset(self):
        self.index=None;self.entered=None;self.reached=False;self.inside=False
        self.dock_started=None

    def update(self,route,index,inside,now,arrival_allowed=True):
        if self.index!=index:
            self.reset();self.index=index
        kind=waypoint_kind(route,index)
        if kind=='orbit':return 'approach'
        self.inside=inside
        if kind=='dock' and self.dock_started is not None and now-self.dock_started>=DOCK_SEARCH_SECONDS:
            return 'dock_timeout'
        if kind=='dock' and self.dock_started is not None:
            return 'dock_wait'
        if not inside and not (kind=='hold' and self.reached):
            self.entered=None
            return 'approach'
        if not self.reached and not arrival_allowed:return 'approach'
        self.reached=True
        if kind=='dock':
            if self.dock_started is None:self.dock_started=now
            return 'dock_wait'
        if kind=='hold':
            if self.entered is None:self.entered=now
            if now-self.entered<HOLD_SECONDS:return 'hold'
            return 'advance' if index+1<len(route['points']) else 'complete'
        return 'advance' if index+1<len(route['points']) else 'final_hold'

    def snapshot(self,route,index,now):
        kind=waypoint_kind(route,index)
        seconds=HOLD_SECONDS if kind=='hold' else 0.
        holding=self.index==index and self.entered is not None and kind=='hold'
        searching=self.index==index and self.dock_started is not None and kind=='dock'
        return {'kind':route.get('waypoint_types',[])[index] if route and 0<=index<len(route.get('waypoint_types',[])) else kind,
                'docking_version':gps_docking_version(route,index),'index':index,'holding':holding,
                'returning_to_hold':False,
                'waiting_for_map':self.index==index and self.reached and self.inside and kind=='dock',
                'searching_for_map':searching,'search_timeout_s':DOCK_SEARCH_SECONDS,
                'search_remaining_s':max(0.,DOCK_SEARCH_SECONDS-(now-self.dock_started)) if searching else None,
                'hold_seconds':seconds,'remaining_s':max(0.,seconds-(now-self.entered)) if seconds and self.entered is not None else None}
