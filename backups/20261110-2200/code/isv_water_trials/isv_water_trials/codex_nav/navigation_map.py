"""Planner perception boundary: scan returns only, SI units, no world objects.

Circle fits are geometric estimates from visible surface points, not simulator
object identities. Occupied memory expires after 2.2 s (the old hit grid takes
about 2.12 s to decay from 20 below its clustering threshold). Free evidence
expires after 5 s; everything else is explicitly unknown, not observed free.
"""
import numpy as np
from isv_water_trials.codex_nav.fast_clearance import compiled_clearance


class NavigationMap:
    occupied_ttl = 2.2
    free_ttl = 5.0

    def __init__(self, width, height, resolution=.25):
        self.width, self.height = width, height
        self.resolution = resolution
        self.xs = np.arange(.9, width-.9, resolution)
        self.ys = np.arange(.9, height-.9, resolution)
        self.gx, self.gy = np.meshgrid(self.xs, self.ys)
        self.free_seen = np.full(self.gx.shape, -np.inf)
        self.tracks = []
        self.obstacles = np.empty((0, 3))
        self.time = 0.
        self.revision = 0

    def observe(self, position, heading, angles, distances, max_range, now):
        self.time = now
        angles = np.asarray(angles)
        distances = np.clip(np.asarray(distances), 0, max_range)
        bearings = heading+angles
        hits = position + distances[:,None]*np.c_[np.cos(bearings),np.sin(bearings)]
        valid = distances < max_range-1e-4
        # A live group always ends at the preceding beam. Batch those identical
        # 2D norms instead of dispatching np.linalg.norm once per beam.
        adjacent_distance=np.linalg.norm(hits[1:]-hits[:-1],axis=1)
        groups=[];group=[]
        for i in range(len(hits)):
            if not valid[i] or (group and adjacent_distance[i-1] > .35):
                if group: groups.append(group)
                group=[]
            if valid[i]:group.append(i)
        if group:groups.append(group)
        if len(groups)>1 and groups[0][0]==0 and groups[-1][-1]==len(hits)-1 and np.linalg.norm(hits[0]-hits[-1])<.35:
            groups[0]=groups.pop()+groups[0]
        detections=[]
        for indices in groups:
            pts=hits[indices]
            # Surveyed map boundaries are explicit prior geometry, not hidden buoys.
            pts=pts[(pts[:,0]>.08)&(pts[:,0]<self.width-.08)&(pts[:,1]>.08)&(pts[:,1]<self.height-.08)]
            if not len(pts):continue
            fitted=False
            if len(pts)>=3:
                origin=pts.mean(axis=0);q=pts-origin
                fit,_,rank,_=np.linalg.lstsq(np.c_[2*q,np.ones(len(q))],np.sum(q*q,axis=1),rcond=None)
                radius=np.sqrt(max(0.,fit[2]+fit[0]**2+fit[1]**2))
                center=origin+fit[:2]
                residual=np.max(abs(np.linalg.norm(pts-center,axis=1)-radius))
                if rank==3 and .05 <= radius <= 1.0 and residual<.01:
                    detections.append(np.r_[center,radius+.015]);fitted=True
            if not fitted:
                # Unfittable surfaces (including walls): conservative hit discs.
                detections.extend(np.c_[pts,np.full(len(pts),.06)])
        self.tracks=[(o,t) for o,t in self.tracks if now-t<=self.occupied_ttl]
        centers=np.empty((len(self.tracks)+len(detections),2))
        for i,(obstacle,_) in enumerate(self.tracks):
            centers[i]=obstacle[:2]
        for obstacle in detections:
            if self.tracks:
                distance=np.linalg.norm(centers[:len(self.tracks)]-obstacle[:2],axis=1)
                idx=int(distance.argmin())
                if distance[idx]<max(.12,min(.5,float(obstacle[2])*.8)):
                    self.tracks[idx]=(obstacle,now)
                    centers[idx]=obstacle[:2]
                    continue
            centers[len(self.tracks)]=obstacle[:2]
            self.tracks.append((obstacle,now))
        self.obstacles=np.array([o for o,_ in self.tracks]).reshape(-1,3)
        dx,dy=self.gx-position[0],self.gy-position[1]
        radial=np.hypot(dx,dy)
        relative=(np.arctan2(dy,dx)-heading-angles[0])%(2*np.pi)
        beam=np.rint(relative/(2*np.pi/len(angles))).astype(int)%len(angles)
        # Cell must be before a measured surface and within scan range.
        seen=radial+.18<distances[beam]
        self.free_seen[seen]=now
        self.revision+=1

    @property
    def known_free(self):
        return self.time-self.free_seen<=self.free_ttl

    def clearance(self, points, hull_radius=.54):
        points=np.asarray(points).reshape(-1,2)
        if not len(self.obstacles):return np.full(len(points),np.inf)
        if compiled_clearance is not None:
            return compiled_clearance(points,self.obstacles,hull_radius)
        return np.min(np.linalg.norm(points[:,None,:]-self.obstacles[None,:,:2],axis=2)-self.obstacles[None,:,2]-hull_radius,axis=1)
