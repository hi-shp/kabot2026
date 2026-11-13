"""Live LiDAR dock geometry and IMU alignment. No map, odometry or SLAM.

Requires two longitudinal side walls AND a perpendicular back wall. A single
gap or camera bearing does not establish a dock. Coordinates are body x/y.
"""
from dataclasses import dataclass
import math
import numpy as np
from .core import wrap


@dataclass
class LocalDockConfig:
    dock_geometry_enabled: bool = True
    dock_calibration_verified: bool = False
    dock_min_width: float = 1.8
    dock_max_width: float = 4.0
    dock_min_depth: float = 2.4
    dock_max_depth: float = 6.0
    dock_range: float = 12.0
    dock_observation_distance: float = 8.0
    dock_line_tolerance: float = .08
    dock_align_distance: float = 1.5
    dock_point_tolerance: float = .3
    dock_yaw_tolerance_deg: float = 5.0
    dock_lateral_tolerance: float = .2
    dock_stable_scans: int = 4
    dock_forward_pwm: int = 1600
    dock_reverse_pwm: int = 1400
    dock_hold_seconds: float = 2.0
    dock_reverse_exit: bool = True

    def __post_init__(self):
        for key in ('dock_min_width','dock_max_width','dock_min_depth','dock_max_depth',
                    'dock_range','dock_observation_distance','dock_line_tolerance','dock_align_distance','dock_point_tolerance',
                    'dock_yaw_tolerance_deg','dock_lateral_tolerance','dock_hold_seconds'):
            if not math.isfinite(getattr(self,key)) or getattr(self,key)<=0:raise ValueError('Invalid '+key)
        if not 1<=self.dock_stable_scans<=30 or type(self.dock_stable_scans) is not int:raise ValueError('Invalid scan count')
        if not 1500<self.dock_forward_pwm<=1700 or not 1300<=self.dock_reverse_pwm<1500:raise ValueError('Invalid dock PWM')
        if self.dock_min_width>=self.dock_max_width or self.dock_min_depth>=self.dock_max_depth:raise ValueError('Invalid dock dimensions')
        if self.dock_yaw_tolerance_deg>15:raise ValueError('Invalid alignment tolerance')
        if self.dock_observation_distance>=self.dock_range:raise ValueError('Observation distance must be inside LiDAR dock range')


def motion_clear(scan,vector,radius,horizon=.6):
    """Observed capsule in the commanded direction; works forward or astern."""
    points=scan.points(scan.yaw,scan.x,scan.y)
    if len(points) and np.min(np.linalg.norm(points,axis=1))<radius:return False
    length=float(np.linalg.norm(vector))
    if length<1e-6:
        # A pivot sweeps the full footprint: require full-circle observations.
        direction=None
    else:direction=math.atan2(vector[1],vector[0])
    angles=scan.angle_min+np.arange(len(scan.ranges))*scan.angle_increment+scan.yaw
    valid=((np.isfinite(scan.ranges)&(scan.ranges>=scan.range_min)&(scan.ranges<=scan.range_max))|np.isposinf(scan.ranges))
    if direction is None:
        covered=np.unique(np.floor((np.mod(angles,2*math.pi))/(math.pi/36)).astype(int))
        return len(covered)>=72 and bool(np.all(valid))
    travel=min(length,horizon)
    relative=np.arctan2(np.sin(angles-direction),np.cos(angles-direction))
    half=math.atan2(radius+math.hypot(scan.x,scan.y),max(.2,travel))
    sector=np.abs(relative)<=half
    if np.count_nonzero(sector)<3 or not np.all(valid[sector]) or scan.range_max<travel+radius+math.hypot(scan.x,scan.y):return False
    samples=np.sort(relative[sector]);gap_limit=max(.04,abs(scan.angle_increment)*2.1)
    if np.max(np.diff(np.r_[-half,samples,half]))>gap_limit:return False
    c,s=math.cos(direction),math.sin(direction)
    if len(points):
        along=points[:,0]*c+points[:,1]*s;cross=-points[:,0]*s+points[:,1]*c
        if np.any(np.hypot(along-np.clip(along,0,travel),cross)<radius):return False
    return True


def extract_lines(points,tolerance):
    """Deterministic RANSAC and TLS refit, bounded cost for a 10Hz scan."""
    points=points[::max(1,len(points)//900)]
    remaining=points.copy();lines=[];rng=np.random.default_rng(7)
    for _ in range(9):
        if len(remaining)<8:break
        best=None;best_count=7
        for _ in range(110):
            pair=remaining[rng.choice(len(remaining),2,replace=False)];d=pair[1]-pair[0]
            norm=float(np.linalg.norm(d))
            if norm<.45:continue
            d=d/norm;mask=np.abs((remaining-pair[0])@np.array([-d[1],d[0]]))<tolerance
            count=int(mask.sum())
            if count>best_count:best,best_count=mask,count
        if best is None:break
        inliers=remaining[best];centre=np.mean(inliers,axis=0)
        _,_,vh=np.linalg.svd(inliers-centre,full_matrices=False);d=vh[0]
        mask=np.abs((remaining-centre)@np.array([-d[1],d[0]]))<tolerance
        inliers=remaining[mask];t=(inliers-centre)@d
        # Split disconnected pieces: do not invent a wall across an empty gap.
        order=np.argsort(t);splits=np.split(order,np.flatnonzero(np.diff(t[order])>.32)+1)
        for run in splits:
            if len(run)>=6 and np.ptp(t[run])>=.55:
                lines.append((centre+d*t[run].min(),centre+d*t[run].max(),d))
        remaining=remaining[~mask]
    return lines


def detect_dock(scan,target,cfg,radius):
    points=scan.points(scan.yaw,scan.x,scan.y)
    points=points[np.linalg.norm(points,axis=1)<=cfg.dock_range]
    if len(points)<24:return None
    lines=extract_lines(points,cfg.dock_line_tolerance);found=[]
    target=np.asarray(target,dtype=float)
    for i,(a,b,d) in enumerate(lines):
        n=d if d@target>0 else -d;t=np.array([-n[1],n[0]])
        if n@target<=0:continue
        for aa,bb,dd in lines[i+1:]:
            if abs(d@dd)<math.cos(math.radians(8)):continue
            rho1=float(((a+b)/2)@t);rho2=float(((aa+bb)/2)@t)
            lo,hi=sorted((rho1,rho2));width=hi-lo
            if not max(cfg.dock_min_width,2*radius)<width<cfg.dock_max_width:continue
            if not lo+.1<target@t<hi-.1:continue
            ends1=sorted((float(a@n),float(b@n)));ends2=sorted((float(aa@n),float(bb@n)))
            if abs(ends1[0]-ends2[0])>.4:continue
            mouth=(ends1[0]+ends2[0])/2
            # Adjacent walls only: an intervening divider is not an opening.
            if any(abs(od@n)>math.cos(math.radians(8)) and
                   lo+.15<float(((oa+ob)/2)@t)<hi-.15 and
                   max(float(oa@n),float(ob@n))>mouth+.5
                   for oa,ob,od in lines):continue
            # Both side walls must meet an observed perpendicular back wall.
            for ba,bbk,bd in lines:
                if abs(bd@n)>math.sin(math.radians(8)):continue
                back=float(((ba+bbk)/2)@n);depth=back-mouth
                if not cfg.dock_min_depth<=depth<=cfg.dock_max_depth:continue
                if max(abs(ends1[1]-back),abs(ends2[1]-back))>min(.75,depth*.25):continue
                bl,bh=sorted((float(ba@t),float(bbk@t)))
                if bl>lo+.2 or bh<hi-.2:continue
                if abs(float(target@n)-back)>.5:continue
                m=n*mouth+t*((lo+hi)/2);c=m+n*(depth/2)
                if depth/2<=radius+.2:continue
                found.append(dict(m=m,c=c,q=m-n*cfg.dock_align_distance,n=n,
                                  width=width,depth=depth,lateral=float(m@t)))
    # Deduplicate repeated line-pair fits but refuse two plausible bays.
    unique=[]
    for item in found:
        if not any(np.linalg.norm(item['m']-old['m'])<.25 for old in unique):unique.append(item)
    return unique[0] if len(unique)==1 else None


class LocalDock:
    def __init__(self):self.reset()

    def reset(self):
        self.phase='DOCK_ACQUIRE';self.stable=0;self.last_scan=-math.inf
        self.last_yaw=None;self.last_width=None;self.since=None;self.locked_yaw=None
        self.report={};self.previous_phase=None
        self.last_call=-math.inf

    def step(self,now,scan,target,heading,cfg,follow,executing,neutral_confirmed,gps_ok):
        neutral=(1500,1500);radius=follow.footprint_radius+follow.clearance
        if now-self.last_call>.25:self.since=None;self.stable=0
        self.last_call=now
        geometry=detect_dock(scan,target,cfg,radius)
        if geometry is None:
            self.stable=0;self.since=None
            self.report={'reason':'도크 측벽·뒷벽 또는 표식 대응을 확인할 수 없어 정지'}
            return 'DOCK_GEOMETRY_UNCERTAIN',neutral,False
        yaw=wrap(heading+math.atan2(geometry['n'][1],geometry['n'][0]))
        if self.locked_yaw is not None and abs(wrap(yaw-self.locked_yaw))>math.radians(10):
            self.stable=0;self.since=None;return 'DOCK_ID_CHANGED',neutral,False
        if scan.stamp>self.last_scan:
            consistent=(self.last_yaw is None or (abs(wrap(yaw-self.last_yaw))<math.radians(5)
                         and abs(geometry['width']-self.last_width)<.15))
            self.stable=self.stable+1 if consistent else 1
            self.last_yaw,self.last_width,self.last_scan=yaw,geometry['width'],scan.stamp
        self.report={key:value.tolist() if isinstance(value,np.ndarray) else value for key,value in geometry.items()}
        self.report.update(phase=self.phase,stable_scans=self.stable,axis_yaw_deg=math.degrees(yaw),
                           reason='LiDAR 입구·중심점 재관측 및 IMU 진입 축 방위 유지')
        if self.stable<cfg.dock_stable_scans:return 'DOCK_ACQUIRE',neutral,False
        if self.locked_yaw is None and executing:self.locked_yaw=yaw
        if self.phase=='DOCK_ACQUIRE' and executing:self.phase='DOCK_TO_Q'
        axis=self.locked_yaw if self.locked_yaw is not None else yaw
        error=wrap(axis-heading);m,c,q=geometry['m'],geometry['c'],geometry['q']
        lateral=geometry['lateral'];complete=False
        if self.phase in ('DOCK_ACQUIRE','DOCK_TO_Q'):
            if np.linalg.norm(q)<=cfg.dock_point_tolerance:
                if executing:self.phase='DOCK_ALIGN'
                return 'DOCK_TO_Q',neutral,False
            # Once abreast of Q, return outside rather than drive blindly inside.
            if float(q@geometry['n'])<-.3:return 'DOCK_Q_OVERSHOT',neutral,False
            command=math.atan2(q[1],q[0]);base=cfg.dock_forward_pwm
            vector=q
        elif self.phase=='DOCK_ALIGN':
            if abs(error)<=math.radians(cfg.dock_yaw_tolerance_deg) and abs(lateral)<=cfg.dock_lateral_tolerance:
                if executing:self.phase='DOCK_ENTER'
                return 'DOCK_ALIGN',neutral,False
            if abs(lateral)>cfg.dock_lateral_tolerance:
                if executing:self.phase='DOCK_TO_Q'
                return 'DOCK_ALIGN',neutral,False
            command=error;base=1500;vector=np.zeros(2)
        elif self.phase=='DOCK_ENTER':
            if np.linalg.norm(c)<=cfg.dock_point_tolerance and abs(lateral)<=cfg.dock_lateral_tolerance:
                if not gps_ok:return 'WP3_POSITION_MISMATCH',neutral,False
                if executing:self.phase='DOCK_HOLD';self.since=None
                return 'DOCK_ENTER',neutral,False
            if float(c@geometry['n'])<-.3:return 'DOCK_CENTER_OVERSHOT',neutral,False
            command=error+math.atan2(lateral,1.2);base=cfg.dock_forward_pwm
            vector=c
        elif self.phase=='DOCK_HOLD':
            if not gps_ok:return 'WP3_POSITION_MISMATCH',neutral,False
            if np.linalg.norm(c)>cfg.dock_point_tolerance+.2 or abs(error)>math.radians(cfg.dock_yaw_tolerance_deg):
                self.since=None;return 'DOCK_HOLD_DRIFT',neutral,False
            if executing and neutral_confirmed:
                if self.since is None:self.since=now
                if now-self.since>=cfg.dock_hold_seconds:
                    if cfg.dock_reverse_exit:self.phase='DOCK_REVERSE';self.since=None
                    else:complete=True
            else:self.since=None
            return 'DOCK_HOLD',neutral,complete
        else:
            # Retain the inward heading while velocity is astern. Reverse the
            # lateral correction, NOT the desired bow heading by 180 degrees.
            if float(m@geometry['n'])>=cfg.dock_align_distance and abs(lateral)<=cfg.dock_lateral_tolerance and abs(error)<=math.radians(cfg.dock_yaw_tolerance_deg):
                return 'DOCK_EXITED',neutral,bool(executing)
            command=error-math.atan2(lateral,1.2);base=cfg.dock_reverse_pwm
            vector=-geometry['n']*max(.1,cfg.dock_align_distance-float(m@geometry['n']))
        if self.phase in ('DOCK_ENTER','DOCK_REVERSE') and abs(error)>math.radians(cfg.dock_yaw_tolerance_deg):
            command=error;base=1500;vector=np.zeros(2)
        if abs(command)>math.radians(35) and base!=1500:
            # No forward/reverse motion until heading is sufficiently aligned.
            base=1500;vector=np.zeros(2)
        if not motion_clear(scan,vector,radius, min(.6,float(np.linalg.norm(vector)))):
            self.since=None;return 'DOCK_PATH_BLOCKED',neutral,False
        diff=int(np.clip(-math.degrees(command)*3,-65,65))
        if base==1500:diff=int(np.clip(diff,-50,50))
        pwm=(base+diff,base-diff)
        self.report.update(turn_command_deg=math.degrees(command),phase=self.phase)
        return self.phase,pwm,False
