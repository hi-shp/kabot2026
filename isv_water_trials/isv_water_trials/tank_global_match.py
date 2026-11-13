"""Bounded, deterministic SE(2) scan-to-fixed-map search with ambiguity rejection."""
import math
import time
import uuid
from collections import deque
import numpy as np
from scipy.ndimage import distance_transform_edt
from .tank_localization_prior import pose_bonus
from .tank_localization_region import region_allows,region_mask_xy


def wrap(a):
    return (a+np.pi)%(2*np.pi)-np.pi


def handoff_ready(result,min_score=.60,min_margin=.0375):
    """Accept a strong, separated hypothesis; a high-scoring tie stays unresolved."""
    if result.get('display_fallback'):return False
    candidates=result.get('candidates',[])
    if len(candidates)<2:return False
    best,second=candidates[:2]
    if best.get('path_valid') is False:return False
    fields=('score','inliers','known','free','x','y','yaw_deg')
    if not all(math.isfinite(best.get(k,float('nan'))) for k in fields):return False
    if not math.isfinite(second.get('score',float('nan'))):return False
    return (best['score']>=min_score and best['score']-second['score']>=min_margin-1e-12
            and best['inliers']>=.4875 and best['known']>=.6 and best['free']>=.6375)


def partial_handoff_ready(result):
    """A small current overlap can acquire the user-selected docking map."""
    candidates=result.get('candidates',[])
    if result.get('display_fallback') or not candidates:return False
    best=candidates[0]
    if not all(math.isfinite(best.get(k,float('nan'))) for k in ('x','y','yaw_deg','score','inliers','known','free')):return False
    count=result.get('point_count',0)
    return (count>=9 and best['inliers']*count>=5 and best['inliers']>=.045 and
            best['known']>=.045 and best['free']>=.45 and best['score']>=.225)


def retain_display_candidate(result,previous=(),region=None):
    """Keep finite evidence visible without authorizing a localization handoff."""
    result=dict(result)
    fields=('x','y','yaw_deg','score','inliers','known','free')
    candidates=[c for c in result.get('candidates',[]) if all(math.isfinite(c.get(k,float('nan'))) for k in fields) and region_allows(region,c)]
    if not candidates:
        candidates=[dict(c,score_stale=True) for c in previous if all(math.isfinite(c.get(k,float('nan'))) for k in fields) and region_allows(region,c)]
        if candidates:
            candidates=[max(candidates,key=lambda c:c['score'])]
            result['display_fallback']=True
    result['candidates']=candidates
    if candidates and (result.get('display_fallback') or candidates[0].get('path_valid') is False):
        result.update(display_fallback=True,state='UNCONFIRMED',margin=0.,reason='최고 점수 후보 유지 · 위치 재검토 중')
    return result


def identify_candidates(candidates,previous):
    """Keep selection identities through score reorderings and small corrections."""
    unused=list(previous);out=[]
    for candidate in candidates:
        nearby=[p for p in unused if math.hypot(candidate['x']-p['x'],candidate['y']-p['y'])<1.
                and abs(wrap(math.radians(candidate['yaw_deg']-p['yaw_deg'])))<math.radians(25)]
        old=min(nearby,key=lambda p:math.hypot(candidate['x']-p['x'],candidate['y']-p['y'])+
                abs(wrap(math.radians(candidate['yaw_deg']-p['yaw_deg'])))) if nearby else None
        identity=old.get('candidate_id') if old else None
        if old is not None:unused.remove(old)
        out.append(dict(candidate,candidate_id=identity or uuid.uuid4().hex))
    return out


class DisplayTrack:
    """Score-qualified display estimate. It never marks navigation localization ready."""
    def __init__(self):self.anchor=None;self.conflict=False;self.revision=0;self.fallback=False;self.metrics={}

    @staticmethod
    def propagate(point,reference,current):
        a=math.radians(point['yaw_deg'])-reference[2];co=math.cos(a);si=math.sin(a)
        dx=current[0]-reference[0];dy=current[1]-reference[1]
        return dict(x=point['x']+co*dx-si*dy,y=point['y']+si*dx+co*dy,
                    yaw_deg=math.degrees(wrap(current[2]+a))%360)

    def adopt(self,result,reference,current,stamp,min_score=.6):
        candidates=result.get('candidates',[])
        if not candidates or reference is None or current is None:return False
        best=candidates[0]
        if not all(math.isfinite(best.get(k,float('nan'))) for k in ('x','y','yaw_deg','score','inliers','known','free')):return False
        fallback=bool(result.get('display_fallback'))
        if not fallback and (best['score']<min_score or best['inliers']<.4875 or best['known']<.6 or best['free']<.6375):return False
        proposed=self.propagate(best,reference,current)
        if self.anchor is not None:
            prior=self.pose(current)
            da=wrap(math.radians(proposed['yaw_deg']-prior['yaw_deg']))
            if math.hypot(proposed['x']-prior['x'],proposed['y']-prior['y'])>1. or abs(da)>math.radians(20):
                # Compare on the SAME scan/history, never against an old score.
                incumbent=result.get('incumbent_score')
                if not (fallback or self.fallback or result.get('incumbent_path_valid') is False or
                        incumbent is not None and best['score']>=incumbent+.015):
                    self.conflict=True;return False
            elif not result.get('path_checked'):
                # Preserve smoothing for callers without trajectory evidence.
                proposed=dict(x=prior['x']+.2*(proposed['x']-prior['x']),
                              y=prior['y']+.2*(proposed['y']-prior['y']),
                              yaw_deg=(prior['yaw_deg']+math.degrees(da)*.2)%360)
            # A path-checked correction is applied whole: interpolating two valid
            # hypotheses can place their midpoint inside a wall.
        if best.get('score_stale'):stamp=best.get('matched_stamp_s',stamp)
        self.anchor=(proposed,tuple(current),stamp,best['score']);self.conflict=False;self.fallback=fallback
        self.metrics={k:best[k] for k in ('inliers','known','free')}
        self.revision+=1
        return True

    def pose(self,current):
        if self.anchor is None:return None
        point,reference,stamp,score=self.anchor
        return dict(self.propagate(point,reference,current),score=score,matched_stamp_s=stamp,provisional=True,conflict=self.conflict,revision=self.revision,display_fallback=self.fallback,**self.metrics)

    def trail(self,history,limit=1200):
        if self.anchor is None or not history:return []
        point,reference,_,_=self.anchor
        indices=np.linspace(0,len(history)-1,min(limit,len(history)),dtype=int)
        return [[round(p['x'],3),round(p['y'],3)] for p in
                (self.propagate(point,reference,history[i]) for i in indices)]


class MotionHistory:
    """Bounded odometry path. Reset together with the odometry reference frame."""
    def __init__(self):self.poses=deque(maxlen=4096);self.revision=0
    def clear(self):self.poses.clear();self.revision+=1
    def add(self,pose):
        if not self.poses or math.hypot(pose[0]-self.poses[-1][0],pose[1]-self.poses[-1][1])>=.04 or abs(wrap(pose[2]-self.poses[-1][2]))>=math.radians(3):
            self.poses.append(tuple(pose));self.revision+=1
    def snapshot(self,current=None):
        points=list(self.poses)
        if current is not None and (not points or tuple(current)!=points[-1]):points.append(tuple(current))
        return points


def scan_points(ranges,angle_min,angle_increment,range_min=0.,range_max=25.,limit=160,partial=False):
    values=np.asarray(ranges,dtype=float);angles=angle_min+np.arange(len(values))*angle_increment
    valid=np.isfinite(values)&(values>=max(.8,range_min))&(values<min(25.,range_max))
    indices=np.flatnonzero(valid)
    sectors=len(np.unique(np.floor((angles[valid]%(2*np.pi))/(2*np.pi/12)).astype(int)))
    if len(indices)<(9 if partial else 36) or sectors<(1 if partial else 6):raise ValueError('라이다 유효 거리·방향 관측 부족')
    if len(indices)>limit:indices=indices[np.linspace(0,len(indices)-1,limit,dtype=int)]
    return np.column_stack((values[indices]*np.cos(angles[indices]),values[indices]*np.sin(angles[indices])))


def scan_stable(a,b):
    a=np.asarray(a);b=np.asarray(b)
    if a.shape!=b.shape:return False
    valid=np.isfinite(a)&np.isfinite(b)&(a>.8)&(b>.8)&(a<25)&(b<25)
    return np.count_nonzero(valid)>=36 and np.mean(np.abs(a[valid]-b[valid])<.2)>=.6375


class ScanEvidence:
    """Short odometry-aligned keyframes, not repeated votes from a stationary scan."""
    def __init__(self):
        self.frames=deque(maxlen=16);self.last=None

    def clear(self):
        self.frames.clear();self.last=None

    def add(self,stamp,pose,points):
        pose=np.asarray(pose,dtype=float)
        if not np.isfinite(pose).all():return
        if self.last is not None:
            dt=stamp-self.last[0];d=np.linalg.norm(pose[:2]-self.last[1][:2]);a=abs(wrap(pose[2]-self.last[1][2]))
            if dt<=0:return
            # A gap/reset/jump invalidates the accumulated relative geometry.
            if dt>1. or d>max(.2,dt*1.5) or a>max(.2,dt*2.):self.clear()
        self.last=(stamp,pose)
        while self.frames and stamp-self.frames[0][0]>20:self.frames.popleft()
        if not self.frames or np.linalg.norm(pose[:2]-self.frames[-1][1][:2])>=.15 or abs(wrap(pose[2]-self.frames[-1][1][2]))>=math.radians(5):
            self.frames.append((stamp,pose,np.asarray(points)))

    def collect(self,stamp,pose,points,limit=480):
        pose=np.asarray(pose,dtype=float)
        frames=[f for f in self.frames if 0<=stamp-f[0]<=20 and
                (np.linalg.norm(f[1][:2]-pose[:2])>=.15 or abs(wrap(f[1][2]-pose[2]))>=math.radians(5))]
        frames.append((stamp,pose,np.asarray(points)))
        if len(frames)>8:frames=[frames[i] for i in np.linspace(0,len(frames)-1,8,dtype=int)]
        per=max(48,limit//len(frames));out=[];co=math.cos(pose[2]);si=math.sin(pose[2])
        for _,p,cloud in frames:
            cloud=cloud[np.linspace(0,len(cloud)-1,min(per,len(cloud)),dtype=int)]
            delta=p[:2]-pose[:2];origin=np.array([co*delta[0]+si*delta[1],-si*delta[0]+co*delta[1]])
            a=p[2]-pose[2];rot=np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
            end=cloud@rot.T+origin
            out.append(np.column_stack((end,np.tile(origin,(len(cloud),1)))))
        travel=max(np.linalg.norm(f[1][:2]-pose[:2]) for f in frames)
        return np.concatenate(out),dict(observation_count=len(frames),observation_span_s=round(stamp-frames[0][0],2),observation_baseline_m=round(float(travel),3))


class GlobalMatcher:
    def __init__(self,info,cells,partial=False,navigation_info=None,navigation_cells=None):
        self.partial=partial
        self.region=None
        self.info=info;self.resolution=float(info['resolution'])
        self.grid=np.frombuffer(bytes(cells),dtype=np.int8).reshape(info['height'],info['width'])
        self.free=(self.grid>=0)&(self.grid<35)
        if np.count_nonzero(self.grid>=65)<9:raise ValueError('지도 벽 관측 부족')
        self.distance=(distance_transform_edt(self.grid<65)*self.resolution).astype(np.float32)
        self.h,self.w=self.grid.shape
        qx,qy,qz,qw=info['orientation'];self.angle=math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
        self.co=math.cos(self.angle);self.si=math.sin(self.angle)
        self.navigation_info=navigation_info if partial else None
        self.navigation_cells=None
        if self.navigation_info is not None:
            self.navigation_cells=np.frombuffer(navigation_cells,dtype=np.int8).reshape(navigation_info['height'],navigation_info['width'])

    def region_mask(self,poses):
        poses=np.asarray(poses).reshape(-1,3)
        if not self.region or not self.region['enabled']:return np.ones(len(poses),dtype=bool)
        x=self.info['position'][0]+self.co*poses[:,0]-self.si*poses[:,1]
        y=self.info['position'][1]+self.si*poses[:,0]+self.co*poses[:,1]
        return np.isfinite(poses).all(axis=1)&region_mask_xy(self.region,x,y)

    def navigation_centers(self,x,y):
        info=self.navigation_info
        qx,qy,qz,qw=info['orientation'];a=math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
        dx=self.info['position'][0]+self.co*x-self.si*y-info['position'][0]
        dy=self.info['position'][1]+self.si*x+self.co*y-info['position'][1]
        ix=np.floor((math.cos(a)*dx+math.sin(a)*dy)/info['resolution']).astype(int)
        iy=np.floor((-math.sin(a)*dx+math.cos(a)*dy)/info['resolution']).astype(int)
        inside=(ix>=0)&(ix<info['width'])&(iy>=0)&(iy<info['height'])
        cells=self.navigation_cells[np.clip(iy,0,info['height']-1),np.clip(ix,0,info['width']-1)]
        return inside&(cells>=0)&(cells<35)

    def sample(self,x,y):
        ix=np.floor(x/self.resolution).astype(int);iy=np.floor(y/self.resolution).astype(int)
        inside=(ix>=0)&(ix<self.w)&(iy>=0)&(iy<self.h)
        ix=np.clip(ix,0,self.w-1);iy=np.clip(iy,0,self.h-1)
        return np.where(inside,self.distance[iy,ix],10.),inside,self.grid[iy,ix]

    def evaluate(self,poses,points,sigma=.22,prior=None):
        poses=np.asarray(poses).reshape(-1,3);out=[]
        for start in range(0,len(poses),512):
            p=poses[start:start+512];co=np.cos(p[:,2,None]);si=np.sin(p[:,2,None])
            dx=co*points[:,0]-si*points[:,1];dy=si*points[:,0]+co*points[:,1]
            ox=co*points[:,2]-si*points[:,3] if points.shape[1]==4 else np.zeros_like(dx)
            oy=si*points[:,2]+co*points[:,3] if points.shape[1]==4 else np.zeros_like(dy)
            distance,inside,cell=self.sample(p[:,0,None]+dx,p[:,1,None]+dy)
            known=inside&(cell>=0)
            score=np.exp(-.5*(distance/sigma)**2).mean(axis=1)
            inliers=((distance<.3)&known).mean(axis=1);coverage=known.mean(axis=1)
            _,center_inside,center_cell=self.sample(p[:,0],p[:,1])
            center=center_inside&(center_cell>=0)&(center_cell<35)
            if self.navigation_info is not None:center=self.navigation_centers(p[:,0],p[:,1])
            # With an operator prior, partial-map poses may lie outside the crop.
            if self.partial and (prior is not None or self.region and self.region['enabled']):center=np.ones(len(p),dtype=bool)
            center&=self.region_mask(p)
            traversable=np.zeros(len(p))
            known_rays=np.zeros(len(p))
            for fraction in (.15,.35,.55,.75,.9):
                _,ok,value=self.sample(p[:,0,None]+ox+(dx-ox)*fraction,p[:,1,None]+oy+(dy-oy)*fraction)
                traversable+=(ok&(value>=0)&(value<35)).mean(axis=1)/5
                known_rays+=(ok&(value>=0)).mean(axis=1)/5
            if self.partial:
                # Crop-external returns are not contradictory map evidence.
                kernel=np.exp(-.5*(distance/sigma)**2)
                supported=((distance<.3)&known).sum(axis=1)
                score=(kernel*known).sum(axis=1)/np.maximum(known.sum(axis=1),1)
                score*=np.minimum(1.,supported/6.)
                traversable=np.divide(traversable,known_rays,out=np.ones_like(traversable),where=known_rays>0)
            else:
                score=score-.3*(1-traversable)-.2*(1-coverage)
            score=np.where(center,score,-2.)
            out.append(np.column_stack((score,inliers,coverage,traversable)))
        return np.concatenate(out) if out else np.empty((0,4))

    @staticmethod
    def diverse(poses,metrics,limit=64,distance=.8,angle=math.radians(15),ranking=None):
        order=np.argsort(-(metrics[:,0] if ranking is None else ranking));ordered=poses[order]
        available=metrics[order,0]>-1;chosen=[]
        for i in range(len(order)):
            if not available[i]:continue
            chosen.append(order[i])
            if len(chosen)>=limit:break
            delta=ordered[i+1:,:2]-ordered[i,:2]
            nearby=np.sum(delta*delta,axis=1)<=distance*distance
            same_angle=np.abs(wrap(ordered[i+1:,2]-ordered[i,2]))<=angle
            available[i+1:]&=~(nearby&same_angle)
        return poses[chosen],metrics[chosen]

    def prior_scores(self,poses,metrics,prior):
        if prior is None:return metrics[:,0]
        wx=self.info['position'][0]+self.co*poses[:,0]-self.si*poses[:,1]
        wy=self.info['position'][1]+self.si*poses[:,0]+self.co*poses[:,1]
        distance=np.hypot(wx-prior['x'],wy-prior['y'])/prior['position_radius_m']
        angle=wrap(poses[:,2]+self.angle-math.radians(prior['yaw_deg']))/math.radians(prior['yaw_tolerance_deg'])
        return metrics[:,0]+2.*np.exp(-.5*(distance*distance+angle*angle))

    def preferred_diverse(self,poses,metrics,prior,limit=64,distance=.8,angle=math.radians(15)):
        if prior is None:return self.diverse(poses,metrics,limit,distance,angle)
        # Reserve half the hypotheses for raw map evidence outside the prior.
        a,b=self.diverse(poses,metrics,max(1,limit//2),distance,angle,ranking=self.prior_scores(poses,metrics,prior))
        c,d=self.diverse(poses,metrics,max(1,limit//2),distance,angle)
        p=np.concatenate((a,c));m=np.concatenate((b,d))
        return self.diverse(p,m,limit,distance,angle,ranking=self.prior_scores(p,m,prior))

    def search(self,points,max_seconds=45.,cancel=None,prior=None):
        started=time.monotonic();points=np.asarray(points,dtype=float)
        if len(points)<(9 if self.partial else 36) or not np.isfinite(points).all():raise ValueError('Insufficient finite scan points')
        step=max(1,round(.4/self.resolution))
        while np.count_nonzero(self.free[::step,::step])>30000:step+=1
        if self.navigation_info is None:
            y,x=np.nonzero(self.free[::step,::step]);positions=np.column_stack(((x*step+.5)*self.resolution,(y*step+.5)*self.resolution))
        else:
            info=self.navigation_info;stride=max(1,round(.5/info['resolution']))
            free=(self.navigation_cells>=0)&(self.navigation_cells<35)
            while np.count_nonzero(free[::stride,::stride])>30000:stride+=1
            y,x=np.nonzero(free[::stride,::stride]);x=(x*stride+.5)*info['resolution'];y=(y*stride+.5)*info['resolution']
            qx,qy,qz,qw=info['orientation'];a=math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
            dx=info['position'][0]+math.cos(a)*x-math.sin(a)*y-self.info['position'][0]
            dy=info['position'][1]+math.sin(a)*x+math.cos(a)*y-self.info['position'][1]
            positions=np.column_stack((self.co*dx+self.si*dy,-self.si*dx+self.co*dy))
            margin=float(np.linalg.norm(points[:,:2],axis=1).max())
            positions=positions[(positions[:,0]>=-margin)&(positions[:,0]<=self.w*self.resolution+margin)&
                                (positions[:,1]>=-margin)&(positions[:,1]<=self.h*self.resolution+margin)]
        if prior is not None:
            seed=self.candidate_poses([prior])[0]
            radius=3.*prior['position_radius_m']
            spacing=max(step*self.resolution,radius/40.)
            offsets=np.unique(np.r_[np.arange(-radius,radius+spacing*.1,spacing),0.])
            dx,dy=np.meshgrid(offsets,offsets)
            nearby=np.c_[seed[0]+dx.ravel(),seed[1]+dy.ravel()]
            nearby=nearby[np.linalg.norm(nearby-seed[:2],axis=1)<=radius+1e-9]
            positions=np.vstack((positions,nearby))
        if self.region and self.region['enabled']:
            # Seed the whole permitted world area, including crop-external water.
            x0,y0,x1,y1=self.region['bounds'];width=x1-x0;height=y1-y0
            spacing=max(step*self.resolution,math.sqrt(width*height/20000.),width/10000.,height/10000.)
            xs=np.linspace(x0,x1,max(2,int(width/spacing)+1))
            ys=np.linspace(y0,y1,max(2,int(height/spacing)+1))
            x,y=np.meshgrid(xs,ys);dx=x.ravel()-self.info['position'][0];dy=y.ravel()-self.info['position'][1]
            extra=np.c_[self.co*dx+self.si*dy,-self.si*dx+self.co*dy]
            positions=np.vstack((positions,extra))
            positions=positions[self.region_mask(np.c_[positions,np.zeros(len(positions))])]
            positions=np.unique(positions,axis=0)
        if not len(positions):raise ValueError('지도 내 탐색 공간 부족')
        coarse_points=points[np.linspace(0,len(points)-1,min(64,len(points)),dtype=int)]
        candidates=[];scores=[]
        def deadline():
            if cancel is not None and cancel.is_set():raise InterruptedError('Search cancelled')
            if time.monotonic()-started>max_seconds:raise TimeoutError('전역 탐색 시간 초과; 후보를 확정하지 않았습니다.')
        angles=np.arange(-np.pi,np.pi,math.radians(15))
        if prior is not None:
            half=math.radians(prior['yaw_tolerance_deg'])
            angles=np.unique(np.r_[angles,wrap(seed[2]+np.unique(np.r_[np.linspace(-half,half,max(2,math.ceil(2*half/math.radians(15))+1)),0.]))])
        for angle in angles:
            deadline();poses=np.column_stack((positions,np.full(len(positions),angle)))
            metrics=self.evaluate(poses,coarse_points,sigma=.85,prior=prior)
            keep=(np.unique(np.r_[np.argsort(-metrics[:,0])[:48],np.argsort(-self.prior_scores(poses,metrics,prior))[:48]])
                  if prior is not None else np.argsort(-metrics[:,0])[:96])
            candidates.append(poses[keep]);scores.append(metrics[keep])
        poses,metrics=self.preferred_diverse(np.concatenate(candidates),np.concatenate(scores),prior,distance=max(.8,step*self.resolution*2))
        if not len(poses):return {'state':'NO_MATCH','candidates':[],'reason':'지도와 맞는 후보 없음'}
        for spacing,angular,sigma in [(step*self.resolution,math.radians(7.5),.5),
                                      (step*self.resolution/2,math.radians(3.75),.3),
                                      (step*self.resolution/4,math.radians(1.875),.22)]:
            deadline()
            offsets=np.array([(dx,dy,da) for dx in np.linspace(-spacing,spacing,5)
                              for dy in np.linspace(-spacing,spacing,5) for da in np.linspace(-angular,angular,5)])
            expanded=(poses[:,None,:]+offsets[None,:,:]).reshape(-1,3);expanded[:,2]=wrap(expanded[:,2])
            values=self.evaluate(expanded,points,sigma,prior=prior)
            poses,metrics=self.preferred_diverse(expanded,values,prior,distance=max(.4,spacing),angle=math.radians(10))
        deadline();poses,metrics=self.preferred_diverse(poses,self.evaluate(poses,points,prior=prior),prior,limit=12,distance=.7,angle=math.radians(15))
        result=self.describe(poses,metrics,prior=prior,point_count=len(points))
        result.update(elapsed_s=round(time.monotonic()-started,3),position_step_m=step*self.resolution,
                      method='correlative_grid_search',point_count=len(points))
        return result

    def describe(self,poses,metrics,prior=None,point_count=0):
        candidates=[]
        for p,m in zip(poses,metrics):
            if m[0]<=-1 or not self.region_mask([p])[0]:continue
            x=self.info['position'][0]+self.co*p[0]-self.si*p[1]
            y=self.info['position'][1]+self.si*p[0]+self.co*p[1]
            candidates.append(dict(x=float(x),y=float(y),yaw_deg=float(math.degrees(wrap(p[2]+self.angle)))%360,
                                   score=float(m[0]),inliers=float(m[1]),known=float(m[2]),free=float(m[3])))
        if not candidates:return {'state':'NO_MATCH','candidates':[],'reason':'지도와 맞는 후보 없음'}
        if prior is not None:
            for c in candidates:
                c['prior_bonus']=pose_bonus(c,prior);c['rank_score']=c['score']+c['prior_bonus']
            def eligible(c):
                return c['inliers']>=.045 and c['known']>=.045 and c['free']>=.45 and c['score']>=.225 and c['inliers']*point_count>=5
            candidates.sort(key=lambda c:(eligible(c),c['rank_score']),reverse=True)
        best=candidates[0];margin=best['score']-candidates[1]['score'] if len(candidates)>1 else 0.
        supported=best['inliers']>=.4875 and best['known']>=.6 and best['free']>=.6375 and best['score']>=.375
        unique=supported and len(candidates)>1 and margin>=.075
        return dict(state='UNIQUE' if unique else 'AMBIGUOUS' if supported else 'NO_MATCH',
                    reason='후보 구분됨' if unique else '비슷한 위치·방향 후보가 남아 있습니다' if supported else '지도와 스캔의 일치도 부족',
                    candidates=candidates,margin=float(margin))

    def validate(self,candidates,points,prior=None):
        poses=self.candidate_poses(candidates)
        metrics=self.evaluate(poses,points,prior=prior);order=np.argsort(-metrics[:,0])
        return self.describe(poses[order],metrics[order],prior=prior,point_count=len(points))

    def track(self,seed,points,min_score=.6):
        """Bounded scan-to-map refinement of one selected hypothesis, not relocalization."""
        if not region_allows(self.region,seed):return dict(accepted=False,reason='Outside localization region')
        base=self.candidate_poses([seed])[0]
        offsets=np.array([(x,y,a) for x in np.arange(-.5,.501,.1) for y in np.arange(-.5,.501,.1)
                          for a in np.radians(np.arange(-8.,8.01,2.))])
        poses=base+offsets;metrics=self.evaluate(poses,points)
        tied=np.flatnonzero(metrics[:,0]>=np.max(metrics[:,0])-1e-7)
        i=int(tied[np.argmin(np.sum(((poses[tied]-base)/np.array([.2,.2,math.radians(3)]))**2,axis=1))]);best=poses[i]
        refined=best+np.array([(x,y,a) for x in (-.05,0.,.05) for y in (-.05,0.,.05) for a in np.radians((-1.,0.,1.))])
        fine=self.evaluate(refined,points)
        tied=np.flatnonzero(fine[:,0]>=np.max(fine[:,0])-1e-7)
        j=int(tied[np.argmin(np.sum(((refined[tied]-base)/np.array([.2,.2,math.radians(3)]))**2,axis=1))]);best=refined[j];quality=fine[j]
        if quality[0]<min_score or quality[1]<.4875 or quality[2]<.6 or quality[3]<.6375:
            return dict(accepted=False,reason='저장 지도와 현재 스캔 일치도 부족',score=float(quality[0]))
        # A flat objective cannot reliably distinguish displacement along a wall.
        rivals=(np.linalg.norm(poses[:,:2]-best[:2],axis=1)>.3)|(np.abs(poses[:,2]-best[2])>math.radians(4))
        ambiguous=bool(np.any(rivals) and float(np.max(metrics[rivals,0]))>=float(quality[0])-.015)
        if ambiguous:
            # Keep the manually chosen hypothesis in unobservable directions.
            # Among equally supported local poses prefer the smallest change.
            all_poses=np.vstack((poses,refined,base[None,:]))
            all_metrics=np.vstack((metrics,fine,self.evaluate([base],points)))
            supported=(all_metrics[:,0]>=max(min_score,float(quality[0])-.01))&(all_metrics[:,1]>=.4875)&(all_metrics[:,2]>=.6)&(all_metrics[:,3]>=.6375)
            choices=all_poses[supported]
            cost=np.sum(((choices-base)/np.array([.2,.2,math.radians(3)]))**2,axis=1)
            best=choices[int(np.argmin(cost))]
        # Resolve near-equal wall matches before judging the search boundary.
        # A weak peak at the edge must not discard a well-supported nearby pose.
        delta=best-base
        if max(abs(delta[0]),abs(delta[1]))>.49 or abs(delta[2])>math.radians(7.9):
            return dict(accepted=False,reason='주변 정합 범위 초과 · 다음 스캔으로 재확인 중')
        delta=best-base;length=float(np.linalg.norm(delta[:2]))
        delta[:2]*=min(1.,.08/max(length,1e-9));delta[2]=np.clip(delta[2],-math.radians(.5),math.radians(.5))
        applied=base+delta;applied_metrics=self.evaluate([applied],points)[0]
        segment=np.linspace(base[:2],applied[:2],max(2,math.ceil(float(np.linalg.norm(delta[:2]))/(self.resolution*.4))+1))
        _,inside,cells=self.sample(segment[:,0],segment[:,1])
        if not np.all(inside&(cells>=0)&(cells<35)):
            return dict(accepted=False,reason='보정 경로가 지도 장애물과 겹침')
        before=self.evaluate([base],points)[0]
        if applied_metrics[0]+.005<before[0]:return dict(accepted=False,reason='부분 보정 일치도 저하')
        candidate=self.describe(np.asarray([applied]),np.asarray([applied_metrics]))['candidates'][0]
        supported=bool(applied_metrics[0]>=min_score and applied_metrics[1]>=.4875 and applied_metrics[2]>=.6 and applied_metrics[3]>=.6375)
        return dict(accepted=True,candidate=candidate,supported=supported,
                    translation_m=float(np.linalg.norm(delta[:2])),rotation_deg=float(math.degrees(delta[2])),
                    reason=('선택 위치 유지 · 관측 가능한 방향의 오차 보정 중' if ambiguous else '주변 지도 정합으로 위치·방향 보정 중') if supported else '초기 위치 오차를 나누어 보정 중')

    def candidate_poses(self,candidates):
        poses=[]
        for c in candidates:
            dx=c['x']-self.info['position'][0];dy=c['y']-self.info['position'][1]
            poses.append([self.co*dx+self.si*dy,-self.si*dx+self.co*dy,math.radians(c['yaw_deg'])-self.angle])
        return np.asarray(poses).reshape(-1,3)

    def path_metrics(self,candidates,history,reference):
        """Swept centreline, half-cell sampling prevents jumping over thin walls.

        0.20 m clearance = half of 0.50 m beam minus 0.05 m map tolerance.
        This is conservative candidate evidence, not a full hull collision model.
        Unknown coverage and retracing the same route are allowed. Candidate
        retention tolerates brief map noise; navigation uses strict validity.
        """
        if not candidates:return []
        if reference is None or not history:return [dict(valid=True,plausible=True,known=0.,samples=0) for _ in candidates]
        path=np.asarray(history,dtype=float)[:,:2];pieces=[path[:1]]
        for start,end in zip(path[:-1],path[1:]):
            count=max(1,int(np.ceil(np.linalg.norm(end-start)/(self.resolution*.5))))
            pieces.append(start+(end-start)*np.arange(1,count+1)[:,None]/count)
        path=np.concatenate(pieces)-np.asarray(reference[:2])
        a=reference[2];co=math.cos(a);si=math.sin(a)
        relative=np.column_stack((co*path[:,0]+si*path[:,1],-si*path[:,0]+co*path[:,1]))
        results=[]
        for p in self.candidate_poses(candidates):
            co=math.cos(p[2]);si=math.sin(p[2])
            distance,inside,cells=self.sample(p[0]+co*relative[:,0]-si*relative[:,1],p[1]+si*relative[:,0]+co*relative[:,1])
            blocked=inside&((cells>=65)|(distance<.20-1e-9))
            hard_bad=~inside|(cells>=65)
            edges=np.diff(np.r_[False,hard_bad,False].astype(int))
            runs=np.flatnonzero(edges==-1)-np.flatnonzero(edges==1)
            longest=float(max(0,runs.max(initial=0)-1))*self.resolution*.5
            plausible=not hard_bad.any() or (hard_bad.mean()<=.02 and longest<=max(.10,self.resolution)+1e-9)
            results.append(dict(valid=bool(inside.all() and not blocked.any()),plausible=bool(plausible),
                                known=float((inside&(cells>=0)).mean()),samples=len(path)))
        return results

    def constrain_path(self,result,history,reference,points,incumbent=None):
        candidates=result.get('candidates',[])
        metrics=self.path_metrics(candidates,history,reference)
        annotated=[dict(c,path_known=m['known'],path_valid=m['valid']) for c,m in zip(candidates,metrics)]
        kept=[c for c,m in zip(annotated,metrics) if m['plausible']]
        fallback=not kept or bool(result.get('display_fallback'))
        if not kept and annotated:kept=[max(annotated,key=lambda c:c['score'])]
        kept.sort(key=lambda c:c['score'],reverse=True)
        result=dict(result,candidates=kept,path_checked=bool(history and reference is not None),
                    path_rejected=len(candidates)-len(kept),path_points=len(history),display_fallback=fallback)
        result['margin']=kept[0]['score']-kept[1]['score'] if len(kept)>1 else 0.
        if kept and (kept[0]['score']<.6 or kept[0]['inliers']<.4875 or kept[0]['known']<.6 or kept[0]['free']<.6375):
            result['display_fallback']=True
        if incumbent is not None:
            valid=self.path_metrics([incumbent],history,reference)[0]['valid']
            score=float(self.evaluate(self.candidate_poses([incumbent]),points)[0,0])
            result.update(incumbent_path_valid=valid,incumbent_score=score)
        return retain_display_candidate(result)

    def refine(self,candidates,points,prior=None):
        """Small local correction around every existing hypothesis, retaining rivals."""
        started=time.monotonic();poses=self.candidate_poses(candidates)
        if not len(poses):return dict(state='NO_MATCH',reason='후보 없음',candidates=[])
        offsets=np.array([(x,y,a) for x in (-.15,0.,.15) for y in (-.15,0.,.15)
                          for a in (-math.radians(3),0.,math.radians(3))])
        expanded=(poses[:,None,:]+offsets).reshape(-1,3)
        metrics=self.evaluate(expanded,points,prior=prior)
        values=metrics[:,0].reshape(len(poses),len(offsets))
        best=np.arange(len(poses))*len(offsets)+np.argmax(values,axis=1)
        poses,metrics=self.preferred_diverse(expanded[best],metrics[best],prior,limit=12,distance=.7,angle=math.radians(15))
        result=self.describe(poses,metrics,prior=prior,point_count=len(points))
        result.update(elapsed_s=round(time.monotonic()-started,4),method='local_candidate_review',point_count=len(points))
        return result
