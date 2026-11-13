"""Bounded, fading GPS-aligned lidar overlay; does not feed the controller."""
import math
import numpy as np


class LidarLayers:
    def __init__(self, cell=.25, ttl=20., limit=4000):
        self.cell,self.ttl,self.limit=cell,ttl,limit
        self.cells={};self.hits=[];self.stamp=None

    def update(self, scan, pose, config):
        stamp=scan['stamp']
        if self.stamp is not None:
            if stamp==self.stamp:return
            if stamp<self.stamp:self.cells.clear()
        self.stamp=stamp
        yaw=pose['yaw'];c,s=math.cos(yaw),math.sin(yaw)
        lx,ly=config['lidar_lever_arm_m']
        origin=np.asarray(pose['position'])+np.array([c*lx-s*ly,s*lx+c*ly])
        heading=yaw+math.radians(config['lidar_yaw_deg'])
        angles=np.asarray(scan['angles']);ranges=np.asarray(scan['ranges'])
        maximum=float(scan.get('range_max',20.))
        if not math.isfinite(maximum) or maximum<=0:maximum=20.
        if config['scan_range_m']>0:maximum=min(maximum,config['scan_range_m'])
        valid=np.isfinite(ranges)&(ranges>scan['range_min'])&(ranges<maximum)
        points=origin+ranges[valid,None]*np.c_[np.cos(angles[valid]+heading),np.sin(angles[valid]+heading)]
        self.hits=np.round(points,2).tolist()
        if self.cells:
            keys=list(self.cells);old=np.array([self.cells[k] for k in keys])
            delta=old[:,:2]-origin;radial=np.linalg.norm(delta,axis=1)
            bearing=(np.arctan2(delta[:,1],delta[:,0])-heading-angles[0])%(2*math.pi)
            bins=np.rint(bearing/(2*math.pi)*len(angles)).astype(int)%len(angles)
            observed=np.minimum(ranges[bins],maximum)
            free=np.isfinite(observed)&(radial+.4<observed)&(observed>scan['range_min'])
            expired=(stamp-old[:,2]>self.ttl)|(radial>maximum+5)
            for k,remove in zip(keys,free|expired):
                if remove:self.cells.pop(k,None)
        for p in points:
            key=tuple(np.floor(p/self.cell).astype(int))
            self.cells[key]=(round(float(p[0]),2),round(float(p[1]),2),stamp)
        if len(self.cells)>self.limit:
            self.cells=dict(sorted(self.cells.items(),key=lambda item:item[1][2],reverse=True)[:self.limit])

    def snapshot(self, now):
        self.cells={k:v for k,v in self.cells.items() if 0<=now-v[2]<=self.ttl}
        current=self.stamp is not None and 0<=now-self.stamp<=.5
        return {'hits':self.hits if current else [],
                'accumulated':[[x,y,round(now-t,1)] for x,y,t in self.cells.values()],
                'ttl_s':self.ttl,'stamp':self.stamp,'cell_m':self.cell}
