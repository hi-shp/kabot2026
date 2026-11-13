"""Reverse-capable heading fallback and learned-model waypoint maneuvers."""
import math
import numpy as np


def reverse_maneuver(error_deg,previous=False):
    return abs(error_deg)>(65. if previous else 100.)


def forward_arrival(state, goal, previous=(0.,0.)):
    """An arrival must face the destination and must not be travelling astern."""
    delta=np.asarray(goal)-state['position'];distance=float(np.linalg.norm(delta))
    alignment=1. if distance<.05 else float(delta@np.array([math.cos(state['yaw']),math.sin(state['yaw'])])/distance)
    velocity=state.get('velocity')
    return (alignment>=math.cos(math.radians(25)) and
            (velocity is None or velocity[0]>=-.02) and sum(previous)>=-.02)


def forward_approach_metrics(initial, states, goal, radius):
    """Return invalid arrivals and reverse closing distance for sampled rollouts."""
    delta=np.asarray(goal)-states[:,:,:2];distance=np.linalg.norm(delta,axis=2)
    previous=np.concatenate([np.repeat(np.asarray(initial)[None,None,:2],len(states),axis=0),states[:,:-1,:2]],axis=1)
    closing=np.maximum(np.linalg.norm(np.asarray(goal)-previous,axis=2)-distance,0.)
    alignment=(delta[:,:,0]*np.cos(states[:,:,2])+delta[:,:,1]*np.sin(states[:,:,2]))/np.maximum(distance,.001)
    alignment=np.where(distance<.05,1.,alignment)
    backing=states[:,:,3]<-.02
    invalid=(distance<=radius+.3)&(closing>.001)&(backing|(alignment<math.cos(math.radians(25))))
    return invalid.any(axis=1),(closing*backing).sum(axis=1),distance,alignment


def maneuver_space(scan, config, error):
    """Check the measured swept rectangle for a pivot and a short reverse escape."""
    result={'rotation_blocked':False,'reverse_clear':False}
    if scan is None:return result
    ranges=np.asarray(scan.get('ranges',[]));angles=np.asarray(scan.get('angles',[]))
    if len(ranges)<180 or len(ranges)!=len(angles):return result
    valid=(ranges>scan.get('range_min',.1))&(np.isfinite(ranges)|np.isposinf(ranges))
    if np.mean(valid)<.8:return result
    angle=angles+math.radians(config.get('lidar_yaw_deg',0.))
    hit=valid&np.isfinite(ranges)
    points=np.c_[ranges[hit]*np.cos(angle[hit]),ranges[hit]*np.sin(angle[hit])]
    points+=np.asarray(config.get('lidar_lever_arm_m',[0.,0.]))
    half=np.array([config['length_m']/2,config['width_m']/2])
    points=points[~np.all(abs(points)<half,axis=1)]
    def free(position,yaw):
        delta=points-position;c,s=math.cos(yaw),math.sin(yaw)
        body=np.c_[delta[:,0]*c+delta[:,1]*s,-delta[:,0]*s+delta[:,1]*c]
        return not np.any(np.all(abs(body)<half+float(config.get('obstacle_margin_m',0.)),axis=1))
    result['rotation_blocked']=not all(free(np.zeros(2),yaw) for yaw in np.linspace(0,error,max(2,int(abs(error)/math.radians(5))+1)))
    # Check the curved escape swept area, including the differential-thrust turn.
    sign=1. if error>=0 else -1.
    result['reverse_clear']=all(free(np.array([-d,0.]),sign*math.radians(25)*d/.8) for d in np.linspace(0,.8,20))
    rear=abs(np.arctan2(np.sin(angle-math.pi),np.cos(angle-math.pi)))<math.radians(40)
    result['reverse_clear']=result['reverse_clear'] and bool(np.any(rear) and np.all(valid[rear]))
    return result


class LearnedFollowPlanner:
    def __init__(self):self.planner=None;self.key=None

    def plan(self,model_data,config,percent,state,scan,history,now):
        from .outdoor_planner import OutdoorPlanner
        key=(model_data['id'],float(percent))
        if key!=self.key:
            self.planner=OutdoorPlanner(model_data,dict(config,allow_reverse_maneuvers=True,forward_arrival_required=True),percent)
            self.key=key
        return self.planner.plan(state,scan,history,now)


def path_follow_target(state, path, hits, config, buoy_contact=False):
    """Project onto the displayed polyline, then choose a visible arc lookahead."""
    from scipy.spatial import cKDTree
    path=np.asarray(path,dtype=float)
    if path.ndim!=2 or path.shape[1]!=2 or len(path)<2 or not np.isfinite(path).all():
        raise ValueError('목표 경로 계산 대기')
    position=np.asarray(state['position'],dtype=float)
    segments=np.diff(path,axis=0);lengths=np.linalg.norm(segments,axis=1)
    fractions=np.clip(np.sum((position-path[:-1])*segments,axis=1)/np.maximum(lengths**2,1e-12),0.,1.)
    projections=path[:-1]+fractions[:,None]*segments
    index=int(np.argmin(np.linalg.norm(projections-position,axis=1)))
    cross_track=float(np.linalg.norm(projections[index]-position))
    arc=np.r_[0.,np.cumsum(lengths)]
    progress=arc[index]+fractions[index]*lengths[index]
    velocity=state.get('velocity')
    speed=math.hypot(*velocity[:2]) if velocity is not None else 0.
    lookahead=float(np.clip(3.*speed,3.,6.))
    points=np.asarray(hits,dtype=float).reshape(-1,2)
    points=points[np.isfinite(points).all(axis=1)]
    delta=points-position;c,s=math.cos(state['yaw']),math.sin(state['yaw'])
    inside=(abs(delta[:,0]*c+delta[:,1]*s)<config['length_m']/2)&(abs(-delta[:,0]*s+delta[:,1]*c)<config['width_m']/2)
    points=points[~inside];tree=cKDTree(points) if len(points) else None
    radius=config['width_m']/2+float(config.get('obstacle_margin_m',0.))
    # Shorten at corners instead of cutting across the inside obstacle.
    for ahead in np.linspace(lookahead,min(.15,lookahead),12):
        target_arc=min(arc[-1],progress+ahead)
        j=min(len(segments)-1,int(np.searchsorted(arc,target_arc,side='right')-1))
        target=path[j]+segments[j]*((target_arc-arc[j])/max(lengths[j],1e-12))
        stop=min(len(path)-1,int(np.searchsorted(arc,target_arc,side='left')))
        prefix=np.vstack([position,projections[index],path[index+1:stop],target])
        samples=[]
        for a,b in zip(prefix[:-1],prefix[1:]):
            samples.extend(np.linspace(a,b,max(2,int(np.linalg.norm(b-a)/.05)+1)))
        samples.extend(np.linspace(position,target,max(2,int(np.linalg.norm(target-position)/.05)+1)))
        if buoy_contact or tree is None or np.all(tree.query(samples)[0]+1e-9>=radius):
            return target,cross_track,float(target_arc-progress)
    raise ValueError('목표 경로 주변 장애물 · 경로 갱신 대기')


def limit_output(command,percent):
    cap=max(0.,min(1.,percent/100.))
    turn=float(np.clip((command[1]-command[0])/2,-cap,cap))
    surge=float(np.clip(sum(command)/2,-cap+abs(turn),cap-abs(turn)))
    return [surge-turn,surge+turn]


class WaypointLineGuide:
    """Fixed GPS leg with a measured course correction, without changing IMU calibration."""
    def __init__(self):
        self.key=None;self.anchor=None;self.offset=0.;self.stamp=None

    def motion(self,state):
        velocity=state.get('velocity')
        if velocity is None:return dict(state,course_correction_rad=0.)
        u,v,rate=map(float,velocity);speed=math.hypot(u,v)
        stamp=state.get('t')
        if stamp!=self.stamp and u>.3 and speed>.45 and abs(rate)<math.radians(15):
            measured=float(np.clip(math.atan2(v,u),-math.pi/3,math.pi/3))
            self.offset+=.25*math.atan2(math.sin(measured-self.offset),math.cos(measured-self.offset))
            self.stamp=stamp
        weight=float(np.clip((speed-.15)/.3,0.,1.))
        return dict(state,course_correction_rad=self.offset*weight)

    def target(self,state,goal,key,previous=None):
        position=np.asarray(state['position'],dtype=float);goal=np.asarray(goal,dtype=float)
        if key!=self.key:
            self.key=key;self.anchor=np.asarray(previous if previous is not None else position,dtype=float).copy()
        leg=goal-self.anchor;length=float(np.linalg.norm(leg))
        if length<.05:return goal,0.
        direction=leg/length;along=float((position-self.anchor)@direction)
        cross=float(np.linalg.norm(position-(self.anchor+along*direction)))
        velocity=state.get('velocity');speed=math.hypot(*velocity[:2]) if velocity is not None else 0.
        ahead=max(3.,3.*speed)
        target=self.anchor+direction*min(length,max(0.,along)+ahead)
        return target,cross


def point_follow_command(state, goal, percent, previous=(0.,0.), dt=.1, slew=.35, inertia_config=None):
    cap=float(percent)/100.
    if not math.isfinite(cap) or not 0<=cap<=1:
        raise ValueError('출력 상한은 0~100%로 지정하세요')
    delta=np.asarray(goal,dtype=float)-np.asarray(state['position'],dtype=float)
    yaw=float(state['yaw'])
    if delta.shape!=(2,) or not np.isfinite(delta).all() or not math.isfinite(yaw):
        raise ValueError('GPS 위치·IMU 방향 수신 대기')
    distance=float(np.linalg.norm(delta))
    course=yaw+float(state.get('course_correction_rad',0.))
    error=math.atan2(math.sin(math.atan2(delta[1],delta[0])-course),
                     math.cos(math.atan2(delta[1],delta[0])-course))
    velocity=state.get('velocity')
    rate=float(velocity[2]) if velocity is not None else float(state.get('yaw_rate_rad_s',0.))
    if cap==0:return [0.,0.],distance,math.degrees(error)
    turn=cap*float(np.clip((error-.9*rate)/math.radians(90),-1,1))
    surge=cap*max(0.,math.cos(error))
    if abs(turn)>1e-9 and state.get('rotation_blocked',False):
        return [0.,0.],distance,math.degrees(error)
    # Scale both motors together instead of cutting forward thrust to preserve a large turn.
    command=np.array([surge-turn,surge+turn])
    command=(command*min(1.,cap/max(float(np.max(abs(command))),1e-12))).tolist()
    return command,distance,math.degrees(error)


def buoy_goal_command(state, goal, percent, reverse_only=False):
    """Direct goal control for the explicitly selected contact-permitted buoy course."""
    command,distance,error=point_follow_command(dict(state,rotation_blocked=False),goal,percent)
    if reverse_only and percent:
        delta=np.asarray(goal)-state['position']
        angle=math.atan2(delta[1],delta[0])-state['yaw']-math.pi
        angle=math.atan2(math.sin(angle),math.cos(angle))
        cap=percent/100.
        turn=cap*max(-1.,min(1.,angle/.8))
        surge=-cap*max(0.,math.cos(angle))
        command=[max(-cap,min(0.,surge-turn)),max(-cap,min(0.,surge+turn))]
    return command,distance,error
