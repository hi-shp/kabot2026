"""Bounded reverse/turn search while a route is blocked. No ROS/hardware IO."""
import math
import numpy as np


def recovery_command(state,goal,scan,config,percent,previous=(0.,0.),dt=.1):
    if scan is None:raise ValueError('회피 탐색 · 라이다 수신 대기')
    angles=np.asarray(scan.get('angles',[]),dtype=float)
    ranges=np.asarray(scan.get('ranges',[]),dtype=float)
    if len(angles)<180 or len(angles)!=len(ranges):raise ValueError('회피 탐색 · 360도 라이다 수신 대기')
    valid=(ranges>max(0.,scan.get('range_min',0.)))&(np.isfinite(ranges)|np.isposinf(ranges))
    if valid.mean()<.6:raise ValueError('회피 탐색 · 라이다 관측 누락')
    percent=float(percent)
    if not math.isfinite(percent) or not 0<percent<=100:raise ValueError('출력 상한 오류')
    user_cap=percent/100
    cap=min(user_cap,float(config.get('recovery_percent',10.))/100)
    delta=np.asarray(goal,dtype=float)-np.asarray(state['position'],dtype=float)
    yaw=float(state['yaw'])
    if not np.isfinite(delta).all() or not math.isfinite(yaw):raise ValueError('회피 탐색 · 위치/방향 수신 대기')
    error=math.atan2(math.sin(math.atan2(delta[1],delta[0])-yaw),math.cos(math.atan2(delta[1],delta[0])-yaw))
    preferred=1. if error>=0 else -1.
    hit=valid&np.isfinite(ranges)&(ranges<12.)
    bearing=angles[hit]+math.radians(config.get('lidar_yaw_deg',0.))
    points=ranges[hit,None]*np.c_[np.cos(bearing),np.sin(bearing)]
    points+=np.asarray(config.get('lidar_lever_arm_m',[0.,0.]))
    half=np.array([config['length_m']/2,config['width_m']/2])
    points=points[~np.all(abs(points)<half,axis=1)]
    candidates=[]
    for reversing in (True,False):
        for direction in (preferred,-preferred):
            minimum=5.
            for t in np.linspace(0,1,9):
                turn=direction*math.radians(20)*t
                pos=np.array([-.3*t,0.]) if reversing else np.zeros(2)
                if len(points):
                    delta_body=points-pos;c,s=math.cos(turn),math.sin(turn)
                    body=np.c_[delta_body[:,0]*c+delta_body[:,1]*s,-delta_body[:,0]*s+delta_body[:,1]*c]
                    outside=abs(body)-half
                    signed=np.linalg.norm(np.maximum(outside,0),axis=1)+np.minimum(np.max(outside,axis=1),0)
                    minimum=min(minimum,float(signed.min()))
            # Pick the least obstructed short retreat/turn, with a small stable
            # preference for reversing and turning toward the next waypoint.
            score=minimum+(.015 if reversing else 0)+(.005 if direction==preferred else 0)
            surge=-cap*.45 if reversing else 0.
            turn=direction*cap*(.25 if reversing else .5)
            candidates.append((score,np.array([surge-turn,surge+turn]),minimum,reversing,direction))
    _,target,clearance,reversing,direction=max(candidates,key=lambda x:x[0])
    previous=np.clip(np.asarray(previous,dtype=float),-user_cap,user_cap)
    slew=float(config.get('command_slew_per_s',.35))*dt
    command=np.clip(previous+np.clip(target-previous,-slew,slew),-user_cap,user_cap)
    if np.max(abs(command))<1e-9:
        basis=previous if np.max(abs(previous))>1e-9 else target
        command=basis/max(float(np.max(abs(basis))),1e-9)*min(.005,cap*.1)
    return command.tolist(),{'recovery':True,'recovery_mode':'후진·선회 재탐색' if reversing else '선회 재탐색',
                            'recovery_clearance_m':clearance,'recovery_turn':direction,
                            'recovery_limit_percent':cap*100}
