"""Explicitly bounded real-boat tests. Virtual discs add to measured obstacles."""
from collections import deque
import math
import numpy as np


def polygon(points):
    p=np.asarray(points,dtype=float)
    if p.ndim!=2 or p.shape[1]!=2 or not 3<=len(p)<=12 or not np.isfinite(p).all():
        raise ValueError('시험구역은 가장자리를 따라 3~12개 점으로 지정하세요')
    e=np.roll(p,-1,axis=0)-p
    if np.any(np.linalg.norm(e,axis=1)<.5) or np.max(np.ptp(p,axis=0))>60:
        raise ValueError('시험구역 점 간격은 0.5m 이상, 전체 폭은 60m 이내로 지정하세요')
    cross=e[:,0]*np.roll(e,-1,axis=0)[:,1]-e[:,1]*np.roll(e,-1,axis=0)[:,0]
    area=np.sum(p[:,0]*np.roll(p,-1,axis=0)[:,1]-p[:,1]*np.roll(p,-1,axis=0)[:,0])/2
    if area<0:p=p[::-1];area=-area
    if area<30 or not (np.all(cross>1e-6) or np.all(cross<-1e-6)):
        raise ValueError('시험구역은 교차·들어간 모서리 없이 넓은 볼록 다각형으로 지정하세요')
    # All vertices must lie on the same inside side of every edge (reject stars).
    if np.min(point_clearance(p,p))<-.001:raise ValueError('시험구역 가장자리가 교차합니다')
    return p


def point_clearance(points,area):
    p=np.atleast_2d(points);a=np.asarray(area)
    e=np.roll(a,-1,axis=0)-a
    n=np.c_[-e[:,1],e[:,0]]/np.linalg.norm(e,axis=1)[:,None]
    return np.min(np.sum((p[:,None,:]-a[None,:,:])*n[None,:,:],axis=2),axis=1)


def hull_clearance(states,area,length,width):
    z=np.atleast_2d(states);a=np.asarray(area);e=np.roll(a,-1,axis=0)-a
    n=np.c_[-e[:,1],e[:,0]]/np.linalg.norm(e,axis=1)[:,None]
    c,s=np.cos(z[:,2]),np.sin(z[:,2])
    support=np.abs(c[:,None]*n[:,0]+s[:,None]*n[:,1])*length/2
    support+=np.abs(-s[:,None]*n[:,0]+c[:,None]*n[:,1])*width/2
    return np.min(np.sum((z[:,None,:2]-a)*n,axis=2)-support,axis=1)


def fence(area,step=.35):
    points=[]
    for a,b in zip(area,np.roll(area,-1,axis=0)):
        points.extend(np.linspace(a,b,max(2,math.ceil(np.linalg.norm(b-a)/step)+1)))
    return np.c_[points,np.full(len(points),step/2+.02)]


def create_course(area,state,scan,config,regions=None):
    from .outdoor_regions import OperatingRegion
    region=OperatingRegion(regions) if regions is not None else None
    area=np.asarray(regions[0]['points']) if region else polygon(area)
    position=np.asarray(state['position'])
    clearance=region.clearance if region else lambda p:point_clearance(p,area)
    sigma=float(state['position_sigma_m'])
    margin=config['obstacle_margin_m']+2*sigma
    radius=math.hypot(config['length_m'],config['width_m'])/2
    if (region.hull_clearance([*position,state['yaw']],config['length_m'],config['width_m'])[0] if region else hull_clearance([*position,state['yaw']],area,config['length_m'],config['width_m'])[0])<margin:
        raise ValueError('현재 배를 시험구역 안쪽으로 두고 충분한 경계 여유를 확보하세요')
    pose=scan.get('pose',state);yaw=pose['yaw'];lx,ly=config['lidar_lever_arm_m']
    origin=np.array(pose['position'])+[math.cos(yaw)*lx-math.sin(yaw)*ly,math.sin(yaw)*lx+math.cos(yaw)*ly]
    ranges=np.asarray(scan['ranges'],dtype=float);angles=np.asarray(scan['angles'])
    if len(ranges)<180 or len(ranges)!=len(angles):raise ValueError('시험구역 라이다 관측 대기')
    maximum=float(scan.get('range_max',20.))
    if config['scan_range_m']>0:maximum=min(maximum,config['scan_range_m'])
    if not math.isfinite(maximum) or maximum<=0:raise ValueError('라이다 최대거리 오류')
    observed=np.where(np.isposinf(ranges),maximum,ranges)
    observed=np.where(np.isfinite(observed)&(observed>scan['range_min']),np.minimum(observed,maximum),0.)
    heading=yaw+math.radians(config['lidar_yaw_deg'])
    def visible(points):
        delta=points-origin;radial=np.linalg.norm(delta,axis=1)
        bearing=(np.arctan2(delta[:,1],delta[:,0])-heading-angles[0])%(2*math.pi)
        ids=np.rint(bearing/(2*math.pi/len(angles))).astype(int)%len(angles)
        return radial+radius+margin<observed[ids]
    lo,hi=region.bounds if region else (area.min(axis=0),area.max(axis=0))
    lo=np.maximum(lo,position-25);hi=np.minimum(hi,position+25)
    x,y=np.meshgrid(np.linspace(lo[0],hi[0],25),np.linspace(lo[1],hi[1],25));candidates=np.c_[x.ravel(),y.ravel()]
    keep=(clearance(candidates)>radius+margin+1)&visible(candidates)
    if region:keep &= region.reachable(candidates,position,radius+margin)
    candidates=candidates[keep]
    if len(candidates)<8:raise ValueError('관측된 시험 수역이 좁습니다. 구역과 라이다를 확인하세요')
    # Seek a repeatable two-ended course that actually needs a virtual detour.
    _,_,axes=np.linalg.svd(candidates-candidates.mean(axis=0),full_matrices=False)
    axis=axes[0];projected=candidates@axis
    left=candidates[np.argsort(projected)[:12]];right=candidates[np.argsort(projected)[-12:]]
    for a in left:
        for b in right[::-1]:
            middle=(a+b)/2;spacing=np.linalg.norm(b-a);disc_radius=.45
            if spacing<max(7,2*config['arrival_radius_m']+2):continue
            if np.linalg.norm(middle-position)<radius+margin+disc_radius+1:continue
            if clearance([middle])[0]<radius+margin+disc_radius+1:continue
            if not visible(np.linspace(a,b,20)).all():continue
            goals=[a.tolist(),b.tolist()]
            if np.linalg.norm(position-b)<np.linalg.norm(position-a):goals.reverse()
            if region:
                # Spread automatic targets through the connected water area.
                # A* chooses how to reach them around the excluded polygons.
                pool=candidates[np.linalg.norm(candidates-middle,axis=1)>radius+margin+disc_radius+1]
                while len(goals)<8 and len(pool):
                    distance=np.min(np.linalg.norm(pool[:,None,:]-np.asarray(goals)[None,:,:],axis=2),axis=1)
                    best=int(np.argmax(distance))
                    if distance[best]<max(4,2*config['arrival_radius_m']+1):break
                    goals.append(pool[best].tolist())
            return {'area':area.tolist(),'regions':region.regions if region else None,'goals':goals,'obstacles':[[*middle.tolist(),disc_radius]],
                    'position_sigma_m':sigma,'margin_m':margin}
    raise ValueError('현재 관측 안에서 충분한 우회 공간을 찾지 못했습니다')


def free_target(state, scan, config, visited=(), sequence=0):
    """Choose a local goal with a scan-observed corridor, without a geofence."""
    angles=np.asarray(scan['angles'],dtype=float);ranges=np.asarray(scan['ranges'],dtype=float)
    if len(angles)<180 or angles.shape!=ranges.shape or not np.isfinite(angles).all():
        raise ValueError('360도 라이다 수신 대기')
    step=2*math.pi/len(angles)
    if not np.allclose(np.diff(angles),step,rtol=.02,atol=1e-5):
        raise ValueError('360도 라이다 각도 확인 필요')
    maximum=float(scan.get('range_max',20.))
    if config['scan_range_m']>0:maximum=min(maximum,config['scan_range_m'])
    if not math.isfinite(maximum) or maximum<=0:raise ValueError('라이다 최대거리 오류')
    observed=np.where(np.isposinf(ranges),maximum,ranges)
    observed=np.where(np.isfinite(observed)&(observed>scan['range_min']),np.minimum(observed,maximum),0.)
    if np.count_nonzero(observed)>0 and np.count_nonzero(observed)<.8*len(observed):
        raise ValueError('라이다 유효 관측 범위 부족')
    position=np.asarray(state['position'],dtype=float);yaw=float(state['yaw'])
    pose=scan.get('pose',state);a=float(pose['yaw']);lx,ly=config['lidar_lever_arm_m']
    origin=np.asarray(pose['position'])+[math.cos(a)*lx-math.sin(a)*ly,math.sin(a)*lx+math.cos(a)*ly]
    heading=a+math.radians(config['lidar_yaw_deg'])
    radius=math.hypot(config['length_m'],config['width_m'])/2+config['obstacle_margin_m']
    minimum=max(2.,float(config['arrival_radius_m'])+1.)
    far=min(8.,maximum-radius-.2)
    if far<minimum:raise ValueError('주변 라이다에서 이동 공간을 찾지 못했습니다')
    bearings=yaw+np.linspace(-math.pi,math.pi,72,endpoint=False)
    lengths=np.linspace(minimum,far,3)
    delta=(np.c_[np.cos(bearings),np.sin(bearings)][:,None,:]*lengths[None,:,None]).reshape(-1,2)
    # Pick guidance outside the current footprint; MPC checks the immediate
    # manoeuvre using its oriented hull. The already occupied footprint must
    # not require every rear/occluded scan sector to report free space.
    samples=max(8,math.ceil(far/.4)+1)
    travel_all=np.linalg.norm(delta,axis=1)
    start=np.minimum(.9,1.1*radius/travel_all)
    fractions=start[:,None]+(1-start[:,None])*np.linspace(0,1,samples)[None,:]
    centers=position+delta[:,None,:]*fractions[:,:,None]
    relative=centers-origin;distance=np.linalg.norm(relative,axis=-1)
    bearing=np.arctan2(relative[...,1],relative[...,0])-heading-angles[0]
    bins=np.rint(bearing/step).astype(int)%len(angles)
    half=np.arcsin(np.minimum(1.,radius/np.maximum(distance,1e-9)))
    near=distance<=radius
    valid=np.ones(distance.shape,dtype=bool);covered_count=np.zeros(distance.shape);known_count=np.zeros(distance.shape)
    # A circular envelope is deliberately conservative; real-time MPC still
    # rechecks its complete predicted hull against each new scan.
    for offset in range(-len(angles)//2,len(angles)//2+1):
        covered=near|(abs(offset)*step<=half+step)
        ray=observed[(bins+offset)%len(angles)];known=ray>0
        clearance=ray>distance+radius
        valid &= ~covered|~known|clearance
        covered_count+=covered;known_count+=covered&known
    valid &= known_count>=.9*np.maximum(covered_count,1)
    good=np.all(valid,axis=1)
    if not np.any(good):raise ValueError('현재 라이다에서 선체가 통과할 이동 공간이 없습니다')
    goals=position+delta[good];vectors=delta[good];travel=np.linalg.norm(vectors,axis=1)
    preferred=yaw+(0.,.55,-.55,1.,-1.)[sequence%5]
    turn=np.arctan2(vectors[:,1],vectors[:,0])-preferred
    score=travel+.9*np.cos(turn)
    history=np.asarray(visited[-20:],dtype=float)
    if history.size:
        nearest=np.linalg.norm(goals[:,None,:]-history[None,:,:],axis=2).min(axis=1)
        score+=.4*np.minimum(nearest,5.)
    return goals[int(np.argmax(score))].tolist()


def virtual_detour(state, scan, config, goal):
    """One virtual disc only when both sides have observed room for a detour."""
    position=np.asarray(state['position']);goal=np.asarray(goal);length=np.linalg.norm(goal-position)
    radius=.4;hull=math.hypot(config['length_m'],config['width_m'])/2+config['obstacle_margin_m']
    if length<5:return []
    middle=(position+goal)/2
    pose=scan.get('pose',state);a=pose['yaw'];lx,ly=config['lidar_lever_arm_m']
    origin=np.asarray(pose['position'])+[math.cos(a)*lx-math.sin(a)*ly,math.sin(a)*lx+math.cos(a)*ly]
    # Check a ring around the disc, including the hull footprint on a detour.
    theta=np.linspace(-math.pi,math.pi,48,endpoint=False)
    probes=middle+(radius+2*hull+.35)*np.c_[np.cos(theta),np.sin(theta)]
    ranges=np.asarray(scan['ranges']);angles=np.asarray(scan['angles']);maximum=float(scan['range_max'])
    delta=probes-origin;d=np.linalg.norm(delta,axis=1)
    bearings=np.arctan2(delta[:,1],delta[:,0])-a-math.radians(config['lidar_yaw_deg'])-angles[0]
    ids=np.rint(bearings/(2*math.pi/len(ranges))).astype(int)%len(ranges)
    observed=np.where(np.isposinf(ranges),maximum,ranges)
    if not np.all(np.isfinite(observed[ids])&(observed[ids]>d+.2)):return []
    return [[float(middle[0]),float(middle[1]),radius]]


def create_free_course(state,scan,config):
    goal=free_target(state,scan,config)
    return {'navigation':'free','area':None,'regions':None,'obstacles':virtual_detour(state,scan,config,goal),
            'goals':[goal],'visited':[list(state['position'])],
            'goal_sequence':0}


def latest_path_blocked(scan, pose, path, config):
    """Fast current-return check while a new MPC job is running (first second)."""
    trajectory=np.asarray(path,dtype=float)
    if trajectory.size==0:return False
    trajectory=trajectory.reshape(-1,3)
    ranges=np.asarray(scan['ranges']);angles=np.asarray(scan['angles'])
    valid=np.isfinite(ranges)&(ranges>scan['range_min'])&(ranges<scan['range_max'])
    if not np.any(valid):return False
    a=pose['yaw'];lx,ly=config['lidar_lever_arm_m']
    origin=np.asarray(pose['position'])+[math.cos(a)*lx-math.sin(a)*ly,math.sin(a)*lx+math.cos(a)*ly]
    b=angles[valid]+a+math.radians(config['lidar_yaw_deg'])
    points=origin+ranges[valid,None]*np.c_[np.cos(b),np.sin(b)]
    delta=points[:,None,:]-trajectory[None,:,:2];c=np.cos(trajectory[:,2]);s=np.sin(trajectory[:,2])
    x=delta[:,:,0]*c+delta[:,:,1]*s;y=-delta[:,:,0]*s+delta[:,:,1]*c
    margin=max(.1,config['obstacle_margin_m'])
    return bool(np.any((np.abs(x)<config['length_m']/2+margin)&(np.abs(y)<config['width_m']/2+margin)))


class PlanComparison:
    """Compare frozen plans to later measured positions; this includes replanning effects."""
    def __init__(self):
        self.plans=deque(maxlen=8);self.errors=deque(maxlen=300);self.last_plan=-math.inf;self.last_measurement=-math.inf
        self.total_count=0;self.total_squared=0.;self.max_error=0.

    def plan(self,stamp,path,dt):
        if stamp-self.last_plan<1:return
        if len(path)<2:return
        self.last_plan=stamp;self.plans.append((stamp,np.asarray(path),dt,set()))

    def observe(self,stamp,position):
        if stamp<=self.last_measurement:return []
        self.last_measurement=stamp;events=[]
        for start,path,dt,used in self.plans:
            elapsed=stamp-start
            if not dt<=elapsed<=len(path)*dt:continue
            horizon=int(round(elapsed))
            if horizon not in (1,2,3) or horizon in used or abs(elapsed-horizon)>.25:continue
            times=np.arange(1,len(path)+1)*dt
            expected=np.array([np.interp(elapsed,times,path[:,i]) for i in (0,1)])
            error=float(np.linalg.norm(expected-position));used.add(horizon)
            self.total_count+=1;self.total_squared+=error**2;self.max_error=max(self.max_error,error)
            event={'t':stamp,'horizon_s':elapsed,'planned':expected.tolist(),'measured':list(position),'error_m':error}
            self.errors.append(event);events.append(event)
        return events

    def summary(self):
        return {'samples':self.total_count,'rmse_m':math.sqrt(self.total_squared/self.total_count) if self.total_count else None,
                'max_error_m':self.max_error if self.total_count else None,'recent':list(self.errors)}
