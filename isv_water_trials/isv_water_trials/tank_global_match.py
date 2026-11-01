"""Bounded, deterministic SE(2) scan-to-fixed-map search with ambiguity rejection."""
import math
import time
from collections import deque
import numpy as np
from scipy.ndimage import distance_transform_edt


def wrap(a):
    return (a+np.pi)%(2*np.pi)-np.pi


def handoff_ready(result,min_score=.80,min_margin=.05):
    """Accept a strong, separated hypothesis; a high-scoring tie stays unresolved."""
    candidates=result.get('candidates',[])
    if len(candidates)<2:return False
    best,second=candidates[:2]
    fields=('score','inliers','known','free','x','y','yaw_deg')
    if not all(math.isfinite(best.get(k,float('nan'))) for k in fields):return False
    if not math.isfinite(second.get('score',float('nan'))):return False
    return (best['score']>=min_score and best['score']-second['score']>=min_margin-1e-12
            and best['inliers']>=.65 and best['known']>=.8 and best['free']>=.85)


class DisplayTrack:
    """Score-qualified display estimate. It never marks navigation localization ready."""
    def __init__(self):self.anchor=None;self.conflict=False;self.revision=0

    @staticmethod
    def propagate(point,reference,current):
        a=math.radians(point['yaw_deg'])-reference[2];co=math.cos(a);si=math.sin(a)
        dx=current[0]-reference[0];dy=current[1]-reference[1]
        return dict(x=point['x']+co*dx-si*dy,y=point['y']+si*dx+co*dy,
                    yaw_deg=math.degrees(wrap(current[2]+a))%360)

    def adopt(self,result,reference,current,stamp,min_score=.8):
        candidates=result.get('candidates',[])
        if not candidates or reference is None or current is None:return False
        best=candidates[0]
        if not all(math.isfinite(best.get(k,float('nan'))) for k in ('x','y','yaw_deg','score','inliers','known','free')):return False
        if best['score']<min_score or best['inliers']<.65 or best['known']<.8 or best['free']<.85:return False
        proposed=self.propagate(best,reference,current)
        if self.anchor is not None:
            prior=self.pose(current)
            da=wrap(math.radians(proposed['yaw_deg']-prior['yaw_deg']))
            if math.hypot(proposed['x']-prior['x'],proposed['y']-prior['y'])>1. or abs(da)>math.radians(20):
                # Compare on the SAME scan/history, never against an old score.
                incumbent=result.get('incumbent_score')
                if not (result.get('incumbent_path_valid') is False or
                        incumbent is not None and best['score']>=incumbent+.02):
                    self.conflict=True;return False
            elif not result.get('path_checked'):
                # Preserve smoothing for callers without trajectory evidence.
                proposed=dict(x=prior['x']+.2*(proposed['x']-prior['x']),
                              y=prior['y']+.2*(proposed['y']-prior['y']),
                              yaw_deg=(prior['yaw_deg']+math.degrees(da)*.2)%360)
            # A path-checked correction is applied whole: interpolating two valid
            # hypotheses can place their midpoint inside a wall.
        self.anchor=(proposed,tuple(current),stamp,best['score']);self.conflict=False
        self.revision+=1
        return True

    def pose(self,current):
        if self.anchor is None:return None
        point,reference,stamp,score=self.anchor
        return dict(self.propagate(point,reference,current),score=score,matched_stamp_s=stamp,provisional=True,conflict=self.conflict,revision=self.revision)

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


def scan_points(ranges,angle_min,angle_increment,range_min=0.,range_max=25.,limit=160):
    values=np.asarray(ranges,dtype=float);angles=angle_min+np.arange(len(values))*angle_increment
    valid=np.isfinite(values)&(values>=max(.8,range_min))&(values<min(25.,range_max))
    indices=np.flatnonzero(valid)
    sectors=len(np.unique(np.floor((angles[valid]%(2*np.pi))/(2*np.pi/12)).astype(int)))
    if len(indices)<48 or sectors<8:raise ValueError('라이다 유효 거리·방향 관측 부족')
    if len(indices)>limit:indices=indices[np.linspace(0,len(indices)-1,limit,dtype=int)]
    return np.column_stack((values[indices]*np.cos(angles[indices]),values[indices]*np.sin(angles[indices])))


def scan_stable(a,b):
    a=np.asarray(a);b=np.asarray(b)
    if a.shape!=b.shape:return False
    valid=np.isfinite(a)&np.isfinite(b)&(a>.8)&(b>.8)&(a<25)&(b<25)
    return np.count_nonzero(valid)>=48 and np.mean(np.abs(a[valid]-b[valid])<.2)>=.85


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
    def __init__(self,info,cells):
        self.info=info;self.resolution=float(info['resolution'])
        self.grid=np.frombuffer(bytes(cells),dtype=np.int8).reshape(info['height'],info['width'])
        self.free=(self.grid>=0)&(self.grid<35)
        if np.count_nonzero(self.grid>=65)<12:raise ValueError('지도 벽 관측 부족')
        self.distance=(distance_transform_edt(self.grid<65)*self.resolution).astype(np.float32)
        self.h,self.w=self.grid.shape
        qx,qy,qz,qw=info['orientation'];self.angle=math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
        self.co=math.cos(self.angle);self.si=math.sin(self.angle)

    def sample(self,x,y):
        ix=np.floor(x/self.resolution).astype(int);iy=np.floor(y/self.resolution).astype(int)
        inside=(ix>=0)&(ix<self.w)&(iy>=0)&(iy<self.h)
        ix=np.clip(ix,0,self.w-1);iy=np.clip(iy,0,self.h-1)
        return np.where(inside,self.distance[iy,ix],10.),inside,self.grid[iy,ix]

    def evaluate(self,poses,points,sigma=.22):
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
            traversable=np.zeros(len(p))
            for fraction in (.15,.35,.55,.75,.9):
                _,ok,value=self.sample(p[:,0,None]+ox+(dx-ox)*fraction,p[:,1,None]+oy+(dy-oy)*fraction)
                traversable+=(ok&(value>=0)&(value<35)).mean(axis=1)/5
            score=score-.3*(1-traversable)-.2*(1-coverage)
            score=np.where(center,score,-2.)
            out.append(np.column_stack((score,inliers,coverage,traversable)))
        return np.concatenate(out) if out else np.empty((0,4))

    @staticmethod
    def diverse(poses,metrics,limit=64,distance=.8,angle=math.radians(15)):
        order=np.argsort(-metrics[:,0]);ordered=poses[order]
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

    def search(self,points,max_seconds=45.,cancel=None):
        started=time.monotonic();points=np.asarray(points,dtype=float)
        if len(points)<48 or not np.isfinite(points).all():raise ValueError('Insufficient finite scan points')
        step=max(1,round(.4/self.resolution))
        while np.count_nonzero(self.free[::step,::step])>30000:step+=1
        y,x=np.nonzero(self.free[::step,::step]);positions=np.column_stack(((x*step+.5)*self.resolution,(y*step+.5)*self.resolution))
        if not len(positions):raise ValueError('지도 내 탐색 공간 부족')
        coarse_points=points[np.linspace(0,len(points)-1,min(64,len(points)),dtype=int)]
        candidates=[];scores=[]
        def deadline():
            if cancel is not None and cancel.is_set():raise InterruptedError('Search cancelled')
            if time.monotonic()-started>max_seconds:raise TimeoutError('전역 탐색 시간 초과; 후보를 확정하지 않았습니다.')
        for angle in np.arange(-np.pi,np.pi,math.radians(15)):
            deadline();poses=np.column_stack((positions,np.full(len(positions),angle)))
            metrics=self.evaluate(poses,coarse_points,sigma=.85)
            keep=np.argsort(-metrics[:,0])[:96];candidates.append(poses[keep]);scores.append(metrics[keep])
        poses,metrics=self.diverse(np.concatenate(candidates),np.concatenate(scores),distance=max(.8,step*self.resolution*2))
        if not len(poses):return {'state':'NO_MATCH','candidates':[],'reason':'지도와 맞는 후보 없음'}
        for spacing,angular,sigma in [(step*self.resolution,math.radians(7.5),.5),
                                      (step*self.resolution/2,math.radians(3.75),.3),
                                      (step*self.resolution/4,math.radians(1.875),.22)]:
            deadline()
            offsets=np.array([(dx,dy,da) for dx in np.linspace(-spacing,spacing,5)
                              for dy in np.linspace(-spacing,spacing,5) for da in np.linspace(-angular,angular,5)])
            expanded=(poses[:,None,:]+offsets[None,:,:]).reshape(-1,3);expanded[:,2]=wrap(expanded[:,2])
            values=self.evaluate(expanded,points,sigma)
            poses,metrics=self.diverse(expanded,values,distance=max(.4,spacing),angle=math.radians(10))
        deadline();poses,metrics=self.diverse(poses,self.evaluate(poses,points),limit=12,distance=.7,angle=math.radians(15))
        result=self.describe(poses,metrics)
        result.update(elapsed_s=round(time.monotonic()-started,3),position_step_m=step*self.resolution,
                      method='correlative_grid_search',point_count=len(points))
        return result

    def describe(self,poses,metrics):
        candidates=[]
        for p,m in zip(poses,metrics):
            x=self.info['position'][0]+self.co*p[0]-self.si*p[1]
            y=self.info['position'][1]+self.si*p[0]+self.co*p[1]
            candidates.append(dict(x=float(x),y=float(y),yaw_deg=float(math.degrees(wrap(p[2]+self.angle)))%360,
                                   score=float(m[0]),inliers=float(m[1]),known=float(m[2]),free=float(m[3])))
        if not candidates:return {'state':'NO_MATCH','candidates':[],'reason':'지도와 맞는 후보 없음'}
        best=candidates[0];margin=best['score']-candidates[1]['score'] if len(candidates)>1 else 0.
        supported=best['inliers']>=.65 and best['known']>=.8 and best['free']>=.85 and best['score']>=.5
        unique=supported and len(candidates)>1 and margin>=.10
        return dict(state='UNIQUE' if unique else 'AMBIGUOUS' if supported else 'NO_MATCH',
                    reason='후보 구분됨' if unique else '비슷한 위치·방향 후보가 남아 있습니다' if supported else '지도와 스캔의 일치도 부족',
                    candidates=candidates,margin=float(margin))

    def validate(self,candidates,points):
        poses=self.candidate_poses(candidates)
        metrics=self.evaluate(poses,points);order=np.argsort(-metrics[:,0])
        return self.describe(poses[order],metrics[order])

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
        Unknown coverage is reported separately and is not labelled a wall.
        """
        if not candidates:return []
        if reference is None or not history:return [dict(valid=True,known=0.,samples=0) for _ in candidates]
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
            results.append(dict(valid=bool(inside.all() and not blocked.any()),known=float((inside&(cells>=0)).mean()),samples=len(path)))
        return results

    def constrain_path(self,result,history,reference,points,incumbent=None):
        candidates=result.get('candidates',[])
        metrics=self.path_metrics(candidates,history,reference)
        kept=[dict(c,path_known=m['known']) for c,m in zip(candidates,metrics) if m['valid']]
        result=dict(result,candidates=kept,path_checked=bool(history and reference is not None),
                    path_rejected=len(candidates)-len(kept),path_points=len(history))
        result['margin']=kept[0]['score']-kept[1]['score'] if len(kept)>1 else 0.
        if not kept:result.update(state='NO_MATCH',reason='이동 경로가 벽 또는 지도 경계와 겹쳐 후보 재탐색')
        if incumbent is not None:
            valid=self.path_metrics([incumbent],history,reference)[0]['valid']
            score=float(self.evaluate(self.candidate_poses([incumbent]),points)[0,0])
            result.update(incumbent_path_valid=valid,incumbent_score=score)
        return result

    def refine(self,candidates,points):
        """Small local correction around every existing hypothesis, retaining rivals."""
        started=time.monotonic();poses=self.candidate_poses(candidates)
        if not len(poses):return dict(state='NO_MATCH',reason='후보 없음',candidates=[])
        offsets=np.array([(x,y,a) for x in (-.15,0.,.15) for y in (-.15,0.,.15)
                          for a in (-math.radians(3),0.,math.radians(3))])
        expanded=(poses[:,None,:]+offsets).reshape(-1,3)
        metrics=self.evaluate(expanded,points)
        values=metrics[:,0].reshape(len(poses),len(offsets))
        best=np.arange(len(poses))*len(offsets)+np.argmax(values,axis=1)
        poses,metrics=self.diverse(expanded[best],metrics[best],limit=12,distance=.7,angle=math.radians(15))
        result=self.describe(poses,metrics)
        result.update(elapsed_s=round(time.monotonic()-started,4),method='local_candidate_review',point_count=len(points))
        return result
