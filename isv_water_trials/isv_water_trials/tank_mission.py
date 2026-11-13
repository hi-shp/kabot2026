"""A saved-map docking pass interrupts exactly one GPS target."""
from dataclasses import dataclass
import math


def arrive_once(goal, legs, count, once):
    legs+=1
    complete=bool(once and legs>=count)
    return (goal if complete or count==1 else (goal+1)%count),legs,complete


def docking_points(route, map_id):
    if not isinstance(route,dict) or route.get('frame_id')!='tank_map' or route.get('session_id')!=map_id:
        raise ValueError('현재 저장 지도에 연결된 A/B 경로를 지정하세요.')
    points=[]
    for key in ('a','b'):
        point=route.get(key)
        if point is None:continue
        xy=tuple(float(point[axis]) for axis in ('x','y'))
        if not all(math.isfinite(v) for v in xy):raise ValueError('A/B 좌표를 확인하세요.')
        points.append(xy)
    if not points:raise ValueError('도킹 지점을 먼저 지정하세요.')
    return tuple(points)


@dataclass
class DockingMission:
    map_id: str
    skipped_index: int
    resume_index: object
    mission_id: str

    @classmethod
    def begin(cls,map_id,index,count,mission_id,loop=False):
        if not 0<=index<count:raise ValueError('GPS 목표 지점 범위를 확인하세요.')
        next_index=index+1
        if next_index==count:next_index=0 if loop else None
        return cls(map_id,index,next_index,mission_id)
