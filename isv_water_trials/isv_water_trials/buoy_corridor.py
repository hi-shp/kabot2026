"""Buoy boundaries in GPS/IMU ENU; never estimates vessel pose."""
import math
import numpy as np


def rotation(yaw):
    c,s=math.cos(yaw),math.sin(yaw)
    return np.array([[c,-s],[s,c]])


def scan_clusters(scan, config):
    angles=np.asarray(scan['angles'], dtype=float);ranges=np.asarray(scan['ranges'], dtype=float)
    valid=np.isfinite(ranges)&(ranges>max(.4,scan.get('range_min',.1)))&(ranges<config['max_range_m'])
    points=ranges[:,None]*np.c_[np.cos(angles),np.sin(angles)]
    groups=[];group=[]
    for i in range(len(ranges)):
        if not valid[i] or (group and np.linalg.norm(points[i]-points[group[-1]])>.65):
            if group:groups.append(group);group=[]
        if valid[i]:group.append(i)
    if group:groups.append(group)
    # Join the cluster straddling the scan's angular seam.
    if len(groups)>1 and groups[0][0]==0 and groups[-1][-1]==len(ranges)-1 and np.linalg.norm(points[0]-points[-1])<.65:
        groups[0]=groups.pop()+groups[0]
    return [np.median(points[g],axis=0) for g in groups if len(g)>=2 and np.max(np.linalg.norm(points[g]-np.median(points[g],axis=0),axis=1))<.85]


def associate(detections, clusters, pose, config, vision, image_pose=None):
    lidar_r=rotation(math.radians(config.get('lidar_yaw_deg',0.)))
    lidar_offset=np.asarray(config.get('lidar_lever_arm_m',[0.,0.]))
    camera_offset=np.asarray(vision['camera_xy_m'])
    body=[lidar_offset+lidar_r@p for p in clusters]
    worlds=[np.asarray(pose['position'])+rotation(pose['yaw'])@p for p in body]
    image_pose=pose if image_pose is None else image_pose
    pairs=[]
    for i,d in enumerate(detections):
        for j,p in enumerate(worlds):
            delta=(p-image_pose['position'])@rotation(image_pose['yaw'])-camera_offset
            bearing=math.atan2(delta[1],delta[0])-math.radians(vision['camera_yaw_deg'])
            error=abs(math.atan2(math.sin(bearing-d['bearing']),math.cos(bearing-d['bearing'])))
            if delta[0]>0 and error<=max(math.radians(2.5),d['half_angle']+math.radians(1.5)):
                pairs.append((error, np.linalg.norm(delta), i,j))
    result=[];used_d=set();used_c=set()
    for _,_,i,j in sorted(pairs):
        if i in used_d or j in used_c:continue
        used_d.add(i);used_c.add(j)
        result.append((detections[i]['colour'],worlds[j]))
    return result


class Corridor:
    def __init__(self):
        self.tracks=[];self.frame=None;self.boundary=None;self.locked=False
        self.reason='colour/lidar acquisition';self.last_scan=None;self.revision=0

    def reset(self):
        self.__init__()

    def observe(self, observations, stamp, ttl=6.):
        self.tracks=[p for p in self.tracks if 0<=stamp-p['t']<=max(60.,ttl)]
        for colour,xy in observations:
            matches=[p for p in self.tracks if p['colour']==colour and np.linalg.norm(p['xy']-xy)<.8]
            if matches:
                p=min(matches,key=lambda p:np.linalg.norm(p['xy']-xy))
                p['xy']=.4*p['xy']+.6*np.asarray(xy);p['t']=stamp
            else:self.tracks.append(dict(colour=colour,xy=np.asarray(xy),t=stamp))
        self.tracks=self.tracks[-100:]

    def build(self, start, finish, position, now, width, vision):
        start=np.asarray(start,dtype=float);axis=np.asarray(finish,dtype=float)-start;length=np.linalg.norm(axis)
        self.boundary=None
        if length<.5:return None
        axis/=length;left=np.array([-axis[1],axis[0]])
        progress=float((np.asarray(position)-start)@axis)
        def row(colour):
            points=sorted(((float((p['xy']-start)@axis),float((p['xy']-start)@left)) for p in self.tracks
                           if p['colour']==colour and 0<=now-p['t'] and
                           (now-p['t']<=vision['track_ttl_s'] or
                            (now-p['t']<=60. and (p['xy']-start)@axis<progress-1.))),key=lambda x:x[0])
            # Repeated observations at the same progress must not create loops.
            result=[]
            for p in points:
                if result and p[0]-result[-1][0]<.4:result[-1]=((p[0]+result[-1][0])/2,(p[1]+result[-1][1])/2)
                else:result.append(p)
            return np.asarray(result,dtype=float).reshape(-1,2)
        green,red=row('green'),row('red')
        if not len(green) or not len(red):
            self.reason='one boundary missing';return None
        low=max(0.,max(green[0,0],red[0,0])-3.)
        high=min(length,max(min(green[-1,0],red[-1,0])+3.,progress+8.))
        if high<progress+.25 or low>progress+2.:
            self.reason='boundaries outside observed section';return None
        sample=np.arange(low,high+.01,.2)
        if len(sample)<2:return None
        gy=np.interp(sample,green[:,0],green[:,1]);ry=np.interp(sample,red[:,0],red[:,1])
        margin=width/2+vision['margin_m']
        if np.any(gy-ry<2*margin) or np.any(gy-ry>20.):
            self.reason='boundary order/width inconsistent';return None
        self.boundary=dict(start=start,axis=axis,left=left,s=sample,green=gy,red=ry,margin=margin)
        self.locked=True;self.revision+=1;self.reason='green-left / red-right corridor'
        center=start+sample[:,None]*axis+((gy+ry)/2)[:,None]*left
        fences=np.vstack([start+sample[:,None]*axis+gy[:,None]*left,start+sample[:,None]*axis+ry[:,None]*left])
        return center,fences

    def contains(self, points, margin=None):
        b=self.boundary
        if b is None:return False
        p=np.atleast_2d(points)-b['start'];s=p@b['axis'];y=p@b['left']
        margin=b['margin'] if margin is None else margin
        # Beyond the current observations, retain the last observed lateral bounds.
        return bool(np.all(y<=np.interp(s,b['s'],b['green'])-margin) and np.all(y>=np.interp(s,b['s'],b['red'])+margin))

    def fenced_scan(self, scan, pose, config, fences):
        if scan is None:return None
        angles=np.asarray(scan['angles'],dtype=float);ranges=np.asarray(scan['ranges'],dtype=float).copy()
        body=(np.asarray(fences)-pose['position'])@rotation(pose['yaw'])
        body=(body-np.asarray(config.get('lidar_lever_arm_m',[0.,0.])))@rotation(math.radians(config.get('lidar_yaw_deg',0.)))
        bearings=np.arctan2(body[:,1],body[:,0]);radial=np.linalg.norm(body,axis=1)
        # Use the original scan's exact angular bins, including a non-zero origin.
        for angle,distance in zip(bearings,radial):
            delta=np.abs(np.arctan2(np.sin(angles-angle),np.cos(angles-angle)))
            i=int(np.argmin(delta))
            if delta[i]<math.radians(2.) and distance>scan.get('range_min',.1):
                if not math.isfinite(ranges[i]) or distance<ranges[i]:ranges[i]=distance
        return dict(scan,ranges=ranges)

    def guard(self, command, state, config, model=None, obstacles=()):
        if not self.locked:return list(command)
        if self.boundary is None:return [0.,0.]
        position=np.asarray(state['position'],dtype=float).copy();yaw=float(state['yaw'])
        velocity=np.asarray(state.get('velocity') if state.get('velocity') is not None else [0.,0.,0.],dtype=float)
        points=np.asarray(obstacles,dtype=float).reshape(-1,2)
        half=np.array([config['length_m']/2,config['width_m']/2])+float(config.get('obstacle_margin_m',.2))
        # Validate the hull, rather than only its centre, against both boundaries.
        corners=np.array([[x,y] for x in (-half[0],half[0]) for y in (-half[1],half[1])])
        dt=model.dt if model is not None else .1
        for _ in range(max(1,math.ceil(1.2/dt))):
            if model is not None:
                try:velocity=np.asarray(model.step(velocity,command))
                except (ValueError,TypeError):return [0.,0.]
            else:
                velocity[0]=.8*velocity[0]+.2*float(sum(command))*1.5
                velocity[2]=.8*velocity[2]+.2*float(command[1]-command[0])*1.2
            yaw+=velocity[2]*dt;position+=rotation(yaw)@velocity[:2]*dt
            hull=position+corners@rotation(yaw).T
            if not self.contains(hull,margin=float(config.get('obstacle_margin_m',.2))):return [0.,0.]
            if len(points):
                body=(points-position)@rotation(yaw)
                if np.any(np.all(np.abs(body)<half,axis=1)):return [0.,0.]
        return list(command)


class LidarCorridor(Corridor):
    """Label compact returns by the GPS leg, without camera colour or SLAM."""
    def __init__(self):
        super().__init__()
        self.reason='lidar boundaries pending'
        self.pending=[]

    def observe(self, observations, stamp, ttl=1.5):
        ttl=min(ttl,1.5)
        self.pending=[p for p in self.pending if 0<=stamp-p['t']<=1.2]
        self.tracks=[p for p in self.tracks if 0<=stamp-p['t']<=ttl]
        used=set();confirmed=[]
        for label,xy in observations:
            xy=np.asarray(xy,dtype=float)
            matches=[(np.linalg.norm(p['xy']-xy),i) for i,p in enumerate(self.pending)
                     if i not in used and p['colour']==label and np.linalg.norm(p['xy']-xy)<=.6]
            if matches:
                _,i=min(matches);p=self.pending[i]
            else:
                i=len(self.pending);p=dict(colour=label,xy=xy,t=stamp,samples=[])
                self.pending.append(p)
            used.add(i)
            if p['samples'] and stamp<=p['samples'][-1][0]:continue
            p['samples']=[s for s in p['samples'] if 0<=stamp-s[0]<=1.2]+[(stamp,xy)]
            p['samples']=p['samples'][-16:];p['t']=stamp
            points=np.array([s[1] for s in p['samples']]);center=np.median(points,axis=0)
            p['xy']=center
            if (len(points)>=4 and stamp-p['samples'][0][0]>=.3-1e-6
                and np.max(np.linalg.norm(points-center,axis=1))<=.45):
                confirmed.append((label,center))
        # Multiple returns in one scan cannot masquerade as repeated sightings.
        super().observe(confirmed,stamp,ttl)
        self.pending=self.pending[-100:]

    @staticmethod
    def coherent_row(points):
        best=[];best_distance=float('inf')
        for i,a in enumerate(points):
            for b in points[i+1:]:
                if b[0]-a[0]<1.:continue
                slope=(b[1]-a[1])/(b[0]-a[0])
                if abs(slope)>.5:continue
                residual=np.abs(points[:,1]-(a[1]+slope*(points[:,0]-a[0])))
                row=points[residual<=.6];distance=float(np.mean(np.abs(row[:,1])))
                if len(row)>len(best) or (len(row)==len(best) and distance<best_distance):
                    best=row;best_distance=distance
        return np.asarray(best,dtype=float).reshape(-1,2)

    def observe_scan(self, scan, pose, start, finish, boat, options):
        start=np.asarray(start,dtype=float);axis=np.asarray(finish,dtype=float)-start
        length=float(np.linalg.norm(axis))
        if length<.5:return
        axis/=length;left=np.array([-axis[1],axis[0]])
        lidar_r=rotation(math.radians(boat.get('lidar_yaw_deg',0.)))
        offset=np.asarray(boat.get('lidar_lever_arm_m',[0.,0.]))
        world_r=rotation(pose['yaw']);observations=[]
        for point in scan_clusters(scan,options):
            xy=np.asarray(pose['position'])+world_r@(offset+lidar_r@point)
            s=float((xy-start)@axis);y=float((xy-start)@left)
            if -2.<=s<=length+2. and .3<=abs(y)<=10.:
                observations.append(('green' if y>0 else 'red',xy))
        # These internal labels mean GPS-left/right, not detected buoy colours.
        self.tracks=[p for p in self.tracks if 0<=scan['stamp']-p['t']<=options['track_ttl_s']]
        self.observe(observations,scan['stamp'],options['track_ttl_s'])

    def build(self, start, finish, position, now, width, vision):
        self.boundary=None;self.locked=False
        start=np.asarray(start,dtype=float);axis=np.asarray(finish,dtype=float)-start
        length=float(np.linalg.norm(axis))
        if length<.5:return None
        axis/=length;left=np.array([-axis[1],axis[0]])
        progress=float((np.asarray(position)-start)@axis)
        rows=[]
        for label in ('green','red'):
            bins={}
            for p in self.tracks:
                if p['colour']!=label or not 0<=now-p['t']<=min(vision['track_ttl_s'],1.5):continue
                s=float((p['xy']-start)@axis);y=float((p['xy']-start)@left)
                if s<progress-4. or s>progress+16.:continue
                slot=math.floor(s/2.)
                if slot not in bins or abs(y)<abs(bins[slot][1]):bins[slot]=(s,y)
            points=np.asarray(sorted(bins.values()),dtype=float).reshape(-1,2)
            rows.append(self.coherent_row(points))
        green,red=rows
        if not len(green) or not len(red):
            self.reason='stable lidar rows pending; GPS avoidance';return None
        low=max(0.,progress-1.,green[0,0]-3.,red[0,0]-3.)
        high=min(length,progress+12.,green[-1,0]+3.,red[-1,0]+3.)
        if low>progress+1. or high<progress+1.:
            self.reason='lidar boundaries outside approach; GPS avoidance';return None
        sample=np.arange(low,high+.01,.25)
        gy=np.interp(sample,green[:,0],green[:,1]);ry=np.interp(sample,red[:,0],red[:,1])
        margin=width/2+vision['margin_m']+.25
        if np.any(gy-ry<2*margin) or np.any(gy-ry>16.):
            self.reason='lidar gap unsuitable; GPS avoidance';return None
        # Keep the GPS line unless clearance requires a lateral correction.
        lateral=np.clip(np.zeros_like(sample),ry+margin,gy-margin)
        if np.any(np.abs(lateral)>3.) or np.any(np.abs(np.diff(lateral))>.25):
            self.reason='lidar correction inconsistent; GPS avoidance';return None
        self.boundary=dict(start=start,axis=axis,left=left,s=sample,green=gy,red=ry,margin=margin)
        self.locked=True;self.revision+=1;self.reason='lidar GPS-left/right corridor'
        center=start+sample[:,None]*axis+lateral[:,None]*left
        fences=np.vstack([start+sample[:,None]*axis+gy[:,None]*left,
                          start+sample[:,None]*axis+ry[:,None]*left])
        return center,fences
