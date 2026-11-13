"""One selected buoy, drift-tracked centre and measured full-lap progress."""
from collections import deque
import math


def normalize_orbit(value,point,allow_draft=False):
    if value is None:value=dict(selected='custom',radius_m=3.,options=[dict(id='custom',point=point,entry_side='left',direction='ccw')])
    if not isinstance(value,dict):raise ValueError('선회 부표 설정 오류')
    radius=value.get('radius_m',3.)
    if type(radius) not in (int,float) or not math.isfinite(radius) or not 1.5<=radius<=8:raise ValueError('선회 반경은 1.5~8m로 지정하세요')
    options=value.get('options')
    if not isinstance(options,list) or not 1<=len(options)<=16:raise ValueError('선회 후보 부표를 지정하세요')
    normalized=[];ids=set()
    for option in options:
        if not isinstance(option,dict):raise ValueError('선회 후보 설정 오류')
        identity=option.get('id');p=option.get('point')
        if not isinstance(identity,str) or not identity or len(identity)>32 or identity in ids:raise ValueError('선회 후보 이름 오류')
        if not (allow_draft and p is None) and (not isinstance(p,(list,tuple)) or len(p)!=2 or any(type(v) not in (int,float) or not math.isfinite(v) for v in p) or not -85<p[0]<85 or not -180<=p[1]<=180):raise ValueError('선회 부표 위도·경도 오류')
        side=option.get('entry_side','left');direction=option.get('direction','ccw')
        if side not in ('left','right') or direction not in ('ccw','cw'):raise ValueError('선회 진입·회전 방향 오류')
        ids.add(identity);normalized.append(dict(id=identity,point=list(p) if p is not None else None,entry_side=side,direction=direction))
    selected=value.get('selected')
    if selected is not None and selected not in ids:raise ValueError('등록된 선회 부표를 선택하세요')
    return dict(selected=selected,radius_m=float(radius),options=normalized)


def orbit_choice(route,index):
    values=route.get('waypoint_orbits',[]);value=values[index] if index<len(values) else None
    cfg=normalize_orbit(value,route['points'][index])
    choice=next((p for p in cfg['options'] if p['id']==cfg['selected']),None)
    if choice is None:raise ValueError(f'{index+1}번 선회 부표를 주행 전에 선택하세요')
    return cfg,choice


def distance(a,b):return math.hypot(a[0]-b[0],a[1]-b[1])
def wrap(angle):return math.atan2(math.sin(angle),math.cos(angle))
def bearing(point,centre):return math.atan2(point[1]-centre[1],point[0]-centre[0])


class BuoyOrbit:
    def __init__(self,centre,dock,side='left',direction='ccw',radius=3.,neighbors=(),key=None):
        self.key=key;self.seed=tuple(map(float,centre));self.centre=self.seed
        self.radius=float(radius);self.side=side;self.sign=1. if direction=='ccw' else -1.
        self.entry_angle=math.atan2(centre[1]-dock[1],centre[0]-dock[0])+(math.pi/2 if side=='left' else -math.pi/2)
        nearest=min((distance(self.seed,p) for p in neighbors if distance(self.seed,p)>.1),default=20.)
        self.acquire_gate=min(4.,max(1.2,nearest*.4));self.neighbors=list(neighbors)
        self.locked=False;self.last_seen=None;self.last_detection=None;self.confirmations=deque(maxlen=5)
        self.last_measurement=None
        self.raw_stamp=None;self.marine_stamp=None;self.last_candidate=None
        self.phase='entry';self.progress=0.;self.sectors=set();self.start_angle=None
        self.searching=False
        self.previous_position=None;self.previous_stamp=None;self.target=self.seed
        self.last_log=-float('inf');self.observations=0

    def observe(self,scan,pose,mount,marine,stamp):
        candidates=[]
        if scan is not None and pose is not None and scan['stamp']!=self.raw_stamp and 0<=stamp-scan['stamp']<=.6:
            self.raw_stamp=scan['stamp'];clusters=[];cluster=[];last=None
            yaw=float(pose['yaw']);c=math.cos(yaw);s=math.sin(yaw)
            lever=mount.get('lidar_lever_arm_m',[0.,0.]);angle_offset=yaw+math.radians(mount.get('lidar_yaw_deg',0.))
            base=(pose['position'][0]+c*lever[0]-s*lever[1],pose['position'][1]+s*lever[0]+c*lever[1])
            for angle,radius in zip(scan['angles'],scan['ranges']):
                if not math.isfinite(radius) or not .35<radius<12.:
                    if cluster:clusters.append(cluster)
                    cluster=[];last=None;continue
                point=(base[0]+radius*math.cos(angle+angle_offset),base[1]+radius*math.sin(angle+angle_offset))
                if last is not None and distance(last,point)>.5:
                    if cluster:clusters.append(cluster)
                    cluster=[]
                cluster.append(point);last=point
            if cluster:clusters.append(cluster)
            if len(clusters)>1 and distance(clusters[0][0],clusters[-1][-1])<=.5:
                clusters[0]=clusters[-1]+clusters[0];clusters.pop()
            for cluster in clusters:
                if distance(cluster[0],cluster[-1])>1.2 or len(cluster)>32:continue
                centre=(sum(p[0] for p in cluster)/len(cluster),sum(p[1] for p in cluster)/len(cluster))
                candidates.append((centre,float(scan['stamp']),'scan'))
        if marine and marine.get('stamp')!=self.marine_stamp and 0<=stamp-marine['stamp']<=.6:
            self.marine_stamp=marine['stamp']
            # Only actual recent cells, never connector interpolation or old memory.
            for point in marine.get('points',[]):
                if len(point)>=4 and 0<=stamp-point[3]<=.4 and point[2]<=.6:
                    candidates.append(((float(point[0]),float(point[1])),float(point[3]),'3d'))
        self.accept(candidates,stamp)

    def accept(self,candidates,stamp):
        reference=self.centre if self.locked else self.seed
        age=max(0.,stamp-self.last_seen) if self.last_seen is not None else 0.
        gate=min(self.acquire_gate,.8+.5*min(age,3.)) if self.locked else self.acquire_gate
        ranked=[]
        for point,measured,source in candidates:
            if distance(point,reference)>gate:continue
            if not self.locked and any(distance(point,p)+.5<distance(point,self.seed) for p in self.neighbors):continue
            ranked.append((distance(point,reference),point,measured,source))
        ranked.sort(key=lambda item:item[0])
        if not ranked:return
        best=ranked[0]
        if any(distance(best[1],other[1])>1. and other[0]-best[0]<.4 for other in ranked[1:]):return
        _,point,measured,source=best
        token=(source,measured)
        if token==self.last_detection or (self.last_measurement is not None and measured<=self.last_measurement):return
        self.last_detection=token;self.last_measurement=measured;self.last_candidate=point;self.observations+=1
        if not self.locked:
            self.confirmations.append((point,measured))
            fresh=[p for p,t in self.confirmations if measured-t<=2.]
            if len(fresh)<3:return
            mean=(sum(p[0] for p in fresh)/len(fresh),sum(p[1] for p in fresh)/len(fresh))
            if max(distance(p,mean) for p in fresh)>.6:return
            self.centre=mean;self.locked=True
        else:
            dt=max(.02,measured-self.last_seen);alpha=dt/(.7+dt)
            dx=(point[0]-self.centre[0])*alpha;dy=(point[1]-self.centre[1])*alpha
            step=math.hypot(dx,dy);scale=min(1.,.8*dt/max(step,1e-9))
            self.centre=(self.centre[0]+dx*scale,self.centre[1]+dy*scale)
        self.last_seen=measured

    def on_circle(self,angle):return (self.centre[0]+self.radius*math.cos(angle),self.centre[1]+self.radius*math.sin(angle))

    def step(self,state,stamp,allowed=True):
        position=tuple(map(float,state['position']));angle=bearing(position,self.centre);radial=distance(position,self.centre)
        entry=self.on_circle(self.entry_angle);fresh=self.locked and self.last_seen is not None and 0<=stamp-self.last_seen<=3.
        fix_stamp=float(state.get('position_stamp_s') or state.get('t') or stamp)
        if self.phase=='entry' and distance(position,entry)<=1. and not fresh:self.searching=True
        if fresh:self.searching=False
        if self.phase=='entry' and allowed and fresh and distance(position,entry)<=1.:
            self.phase='orbit';self.start_angle=angle;self.previous_position=None;self.previous_stamp=None
        if self.phase=='orbit' and fix_stamp!=self.previous_stamp:
            previous=self.previous_position;dt=fix_stamp-self.previous_stamp if self.previous_stamp is not None else 0.
            if allowed and fresh and previous is not None and 0<dt<=2. and distance(position,previous)<=max(1.5,4.*dt):
                # Same centre for both fixes: centre corrections cannot create a lap.
                previous_radius=distance(previous,self.centre)
                delta=self.sign*wrap(angle-bearing(previous,self.centre))
                if .55*self.radius<=min(radial,previous_radius) and max(radial,previous_radius)<=1.8*self.radius and abs(delta)<=math.radians(35):
                    self.progress=max(0.,self.progress+delta)
                    if delta>0:self.sectors.add(int((self.sign*(angle-self.start_angle)%(2*math.pi))/(math.pi/4))%8)
            self.previous_position=position;self.previous_stamp=fix_stamp
            if (allowed and fresh and stamp-self.last_seen<=1. and self.progress>=2*math.pi and len(self.sectors)==8 and
                abs(wrap(angle-self.start_angle))<=math.radians(20) and .65*self.radius<=radial<=1.5*self.radius):self.phase='complete'
        if self.phase=='entry':
            vx=entry[0]-position[0];vy=entry[1]-position[1];length=vx*vx+vy*vy
            fraction=max(0.,min(1.,((self.centre[0]-position[0])*vx+(self.centre[1]-position[1])*vy)/max(length,1e-9)))
            clearance=distance((position[0]+fraction*vx,position[1]+fraction*vy),self.centre)
            if radial<.75*self.radius:self.target=self.on_circle(angle)
            elif self.searching:self.target=self.on_circle(angle+self.sign*math.radians(25))
            elif clearance<.7*self.radius:self.target=self.on_circle(angle+self.sign*math.radians(25))
            elif distance(position,entry)<.9 and not fresh:self.target=self.on_circle(angle+self.sign*math.radians(25))
            else:self.target=entry
        else:
            velocity=state.get('velocity');speed=math.hypot(*velocity[:2]) if velocity is not None else 0.
            lookahead=max(.8,min(1.5,.8+.6*speed))
            self.target=self.on_circle(angle+self.sign*math.atan2(lookahead,self.radius))
        return self.snapshot(stamp)

    def snapshot(self,stamp):
        age=max(0.,stamp-self.last_seen) if self.last_seen is not None else None
        return dict(phase='search' if self.searching and self.phase=='entry' else self.phase,complete=self.phase=='complete',centre_m=list(self.centre),seed_m=list(self.seed),
            entry_m=list(self.on_circle(self.entry_angle)),target_m=list(self.target),radius_m=self.radius,
            entry_side=self.side,direction='ccw' if self.sign>0 else 'cw',progress_deg=math.degrees(self.progress),
            sectors=len(self.sectors),locked=self.locked,observation_age_s=age,observations=self.observations,
            source='lidar' if self.locked and age is not None and age<=3 else 'gps_search',
            acquire_gate_m=self.acquire_gate)

    def preview(self,state):
        angle=bearing(state['position'],self.centre)
        return [list(state['position'])]+[list(self.on_circle(angle+self.sign*i*2*math.pi/48)) for i in range(49)]
