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


def create_course(area,state,scan,config):
    area=polygon(area);position=np.asarray(state['position'])
    sigma=float(state['position_sigma_m'])
    margin=config['obstacle_margin_m']+2*sigma
    radius=math.hypot(config['length_m'],config['width_m'])/2
    if hull_clearance([*position,state['yaw']],area,config['length_m'],config['width_m'])[0]<margin:
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
    lo,hi=area.min(axis=0),area.max(axis=0)
    x,y=np.meshgrid(np.linspace(lo[0],hi[0],25),np.linspace(lo[1],hi[1],25));candidates=np.c_[x.ravel(),y.ravel()]
    keep=(point_clearance(candidates,area)>radius+margin+1)&visible(candidates)
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
            if point_clearance([middle],area)[0]<radius+margin+disc_radius+1:continue
            if not visible(np.linspace(a,b,20)).all():continue
            goals=[a.tolist(),b.tolist()]
            if np.linalg.norm(position-b)<np.linalg.norm(position-a):goals.reverse()
            return {'area':area.tolist(),'goals':goals,'obstacles':[[*middle.tolist(),disc_radius]],
                    'position_sigma_m':sigma,'margin_m':margin}
    raise ValueError('현재 관측 안에서 충분한 우회 공간을 찾지 못했습니다')


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
