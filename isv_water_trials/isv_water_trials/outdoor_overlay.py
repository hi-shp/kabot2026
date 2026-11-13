"""Bounded display-only A* guidance and measured-model rollout, in ENU metres."""
import math
import time
import numpy as np
from scipy.spatial import cKDTree
from .codex_nav.route_planner import a_star
from .codex_nav.control_path import smooth_path
from .codex_nav.gpu_clearance import cuda_clearance,gpu_status
from .path_avoidance import StableAvoidancePath


class FeasibleOverlay:
    """Display trajectories from the same learned planner used for execution."""
    def __init__(self):
        self.planner=None;self.key=None
        self.stable_path = StableAvoidancePath()

    def plan(self,state,goal,hits,scan,commands,config,model_data,percent,now):
        result=display_guidance(state['position'],goal,hits,config,state['yaw'])
        result=self.stable_path.select(result,state['position'],goal,hits,config,state['yaw'],now)
        result.update(feasible=[],feasible_status='실행 가능 궤적 계산 대기',plan_stamp=now,
                      initial_position=list(state['position']),initial_yaw=state['yaw'])
        if model_data is None:
            result['feasible_status']='실행 가능 궤적 · 적용 모델 없음';return result
        if state.get('velocity') is None:
            result['feasible_status']='실행 가능 궤적 · GPS 실측 속도 수신 대기';return result
        if not result['smooth']:
            result['feasible_status']='실행 가능 궤적 · 기준 경로 수신 대기';return result
        if not commands or now-commands[-1][0]>.35:
            result['feasible_status']='실행 가능 궤적 · 추진 명령 이력 수신 대기';return result
        try:
            from .outdoor_planner import OutdoorPlanner
            key=(model_data['id'],float(percent))
            if self.key!=key:
                self.planner=OutdoorPlanner(model_data,dict(config,forward_arrival_required=True),percent);self.key=key
            path=np.asarray(result['smooth'])
            arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(path,axis=0),axis=1))]
            lookahead=float(np.clip(abs(state['velocity'][0])*3+2,2.,8.))
            target=path[min(len(path)-1,int(np.searchsorted(arc,lookahead)))]
            initial=dict(state,goal=target.tolist(),arrival_goal=list(goal),previous_command=list(commands[-1][1:]))
            plan=self.planner.plan(initial,scan,commands,now)
            result.update(feasible=[list(state['position']),*plan['path']],
                feasible_status='현재 자세·속도·지연 반영 · MPC 후보 궤적',
                feasible_model_id=model_data['id'],feasible_command=plan['command'],
                feasible_clearance_m=plan['clearance_m'],feasible_planning_ms=plan['planning_ms'])
        except (ValueError,RuntimeError) as exc:
            result['feasible_status']='실행 가능 궤적 · '+str(exc)
        return result


def display_guidance(position, goal, hits, config, yaw=0.):
    started=time.perf_counter()
    exact=config.get('exact_passage',False)
    start=np.asarray(position,dtype=float);goal=np.asarray(goal,dtype=float)
    result={'raw':[],'smooth':[],'status':'경로 계산 중','visualization_only':True}
    if not np.isfinite([start,goal]).all() or np.linalg.norm(goal-start)>300:
        return dict(result,status='표시 범위 300m 초과')
    if np.linalg.norm(goal-start)<.05:
        return dict(result,status='목표 도착',planning_ms=0.)
    padding=6.
    low=np.minimum(start,goal)-padding;high=np.maximum(start,goal)+padding
    resolution=max(.25,float(np.max(high-low))/180)
    xs=np.arange(low[0],high[0]+resolution/2,resolution)
    ys=np.arange(low[1],high[1]+resolution/2,resolution)
    if exact:
        xs+=start[0]-xs[np.argmin(abs(xs-start[0]))]
        ys+=start[1]-ys[np.argmin(abs(ys-start[1]))]
    gx,gy=np.meshgrid(xs,ys)
    points=np.asarray(hits,dtype=float).reshape(-1,2)
    points=points[np.isfinite(points).all(axis=1)]
    # Returns inside the known physical hull cannot be external obstacles.
    delta=points-start;c,s=math.cos(yaw),math.sin(yaw)
    body_x=delta[:,0]*c+delta[:,1]*s;body_y=-delta[:,0]*s+delta[:,1]*c
    own_hull=(abs(body_x)<config['length_m']/2)&(abs(body_y)<config['width_m']/2)
    points=points[~own_hull]
    result['self_hull_returns']=int(own_hull.sum())
    tree=cKDTree(points) if len(points) else None
    # This layer is a geometric centerline corridor, not a swept-hull rollout.
    extra=0. if exact else float(config.get('obstacle_margin_m',.1))
    radius=config['width_m']/2+extra
    result['corridor_half_width_m']=radius
    def clearance(p):
        p=np.asarray(p).reshape(-1,2)
        return tree.query(p)[0] if tree is not None else np.full(len(p),np.inf)
    grid=np.c_[gx.ravel(),gy.ravel()]
    accelerated=cuda_clearance(grid,np.c_[points,np.zeros(len(points))],0.)
    distance=(accelerated if accelerated is not None else clearance(grid)).reshape(gx.shape)
    result['compute_backend']=dict(gpu_status(),used_gpu=accelerated is not None)
    free=distance+1e-9>=radius+(0. if exact else resolution/math.sqrt(2))
    def index(p):return (int(np.argmin(abs(ys-p[1]))),int(np.argmin(abs(xs-p[0]))))
    si,gi=index(start),index(goal)
    start_clear=bool(clearance(start)[0]+1e-9>=radius+(0. if exact else .02))
    if not start_clear or not free[gi]:
        return dict(result,status='출발점 주변 장애물' if not start_clear else '목표점 주변 장애물',resolution_m=resolution)
    # Preserve the actual safe start instead of applying the grid's reserve
    # twice to its snapped cell. The connector is checked by smooth_path.
    free[si]=True
    weights=1.+.25/np.maximum(distance-radius,.1)
    reference=config.get('gps_reference_leg_m')
    rejoin=config.get('gps_reference_target_m')
    if reference is not None:
        a,b=np.asarray(reference,dtype=float)
        leg=b-a;length2=float(leg@leg)
        if length2>.0025:
            fraction=np.clip(((grid-a)@leg)/length2,0.,1.)
            deviation=np.linalg.norm(grid-(a+fraction[:,None]*leg),axis=1).reshape(gx.shape)
            weights=weights+.35*deviation
    raw=a_star(free.copy(),weights,xs,ys,si,gi,resolution)
    if raw is None:return dict(result,status='통과 가능한 A* 경로 없음',resolution_m=resolution)
    raw[0]=start;raw[-1]=goal
    def safe(p):return bool(np.all(clearance(p)+1e-9>=radius+(0. if exact else .02)))
    pivot=int(np.argmin(np.linalg.norm(raw-np.asarray(rejoin),axis=1))) if rejoin is not None else 0
    if 0<pivot<len(raw)-1:
        first=smooth_path(raw[:pivot+1],safe);second=smooth_path(raw[pivot:],safe)
        rounded=np.vstack([first[:-1],second]) if first is not None and second is not None else None
    else:rounded=smooth_path(raw,safe)
    def compact(path):
        if path is None:return []
        if len(path)>800:path=path[np.unique(np.r_[np.arange(0,len(path),math.ceil(len(path)/800)),len(path)-1])]
        return path.tolist()
    return dict(result,raw=compact(raw),smooth=compact(rounded),
                status='표시용 A*·곡선 경로' if rounded is not None else 'A* 표시 · 곡선 정합 대기',
                resolution_m=resolution,planning_ms=round((time.perf_counter()-started)*1000,1))


def measured_rollout(state, model, commands, now, seconds=3.):
    """Use actual command history and velocity; never synthesize missing inputs."""
    if model is None:return [],'적용 모델 없음'
    if state is None or state.get('velocity') is None:return [],'GPS 실측 속도 수신 대기'
    history=np.asarray(commands,dtype=float).reshape(-1,3)
    if not len(history) or now-history[-1,0]>.35:return [],'추진 명령 이력 수신 대기'
    times=now+np.arange(-model.delay,0)*model.dt
    ids=np.searchsorted(history[:,0],times,side='right')-1
    if np.any(ids<0) or (len(ids) and np.any(times-history[ids,0]>.35)):
        return [],'명령 지연 이력 수집 중'
    predicted=model.predict(state['velocity'],np.tile(history[-1,1:],(max(1,round(seconds/model.dt)),1)),history[ids,1:])
    xy=np.asarray(state['position'],dtype=float).copy();yaw=state['yaw'];path=[xy.tolist()]
    for u,v,r in predicted:
        mid=yaw+r*model.dt/2
        xy+=model.dt*np.array([math.cos(mid)*u-math.sin(mid)*v,math.sin(mid)*u+math.cos(mid)*v])
        yaw+=r*model.dt;path.append(xy.tolist())
    return path,'현재 추진 명령 유지 · 3초 모델 예측'
