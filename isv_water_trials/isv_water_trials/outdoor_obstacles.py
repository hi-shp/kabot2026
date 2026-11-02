"""Bounded scan-cluster tracking in ENU; no identities supplied by perception.

Keep current surface returns for collision checks even when tracking fails.
Predictions are constant velocity with growing uncertainty, not intent estimates.
"""
from dataclasses import dataclass
import numpy as np


@dataclass
class Track:
    id: int
    position: np.ndarray
    velocity: np.ndarray
    radius: float
    stamp: float
    first: float
    hits: int = 1
    error: float = .1


class ObstacleTracker:
    def __init__(self, max_tracks=48, ttl=1., max_speed=4.):
        self.max_tracks, self.ttl, self.max_speed = max_tracks, ttl, max_speed
        self.tracks = []
        self.stamp = None
        self.next_id = 1

    def update(self, angles, ranges, position, yaw, stamp, max_range):
        if self.stamp is not None and stamp <= self.stamp:
            if stamp < self.stamp-1.:
                self.tracks.clear(); self.stamp = None
            else:
                return
        a = np.asarray(angles)+yaw
        r = np.asarray(ranges)
        valid = np.isfinite(r) & (r > .1) & (r < max_range-1e-4)
        points = np.asarray(position)+np.where(valid,r,0.)[:, None]*np.c_[np.cos(a), np.sin(a)]
        groups, group = [], []
        for i in range(len(r)):
            # Angular spacing grows with distance; cap gaps to avoid joining
            # neighboring people/boats into one velocity observation.
            gap = min(.65, .2+.015*r[i]) if valid[i] else .2
            if not valid[i] or (group and np.linalg.norm(points[i]-points[i-1]) > gap):
                if group: groups.append(group)
                group = []
            if valid[i]: group.append(i)
        if group: groups.append(group)
        if len(groups)>1 and groups[0][0]==0 and groups[-1][-1]==len(r)-1 and np.linalg.norm(points[0]-points[-1]) < .35:
            groups[0] = groups.pop()+groups[0]
        detections = []
        for g in groups:
            if len(g)<3: continue
            p = points[g]; center = (p.min(axis=0)+p.max(axis=0))/2
            radius = float(np.linalg.norm(p-center, axis=1).max())+.12
            if radius > 1.5: continue  # Long walls stay in the static map.
            detections.append((center, radius))
        detections.sort(key=lambda d: np.linalg.norm(d[0]-position))
        detections = detections[:self.max_tracks]
        self.tracks = [t for t in self.tracks if 0 <= stamp-t.stamp <= self.ttl]
        # Globally sorted one-to-one gated association; bounded 48 x 48.
        pairs = []
        for i, t in enumerate(self.tracks):
            dt = stamp-t.stamp
            for j, (center, radius) in enumerate(detections):
                distance = np.linalg.norm(center-(t.position+t.velocity*dt))
                if distance < .35+self.max_speed*dt and abs(radius-t.radius)<.6:
                    pairs.append((distance, i, j))
        used_t, used_d = set(), set()
        for _, i, j in sorted(pairs):
            if i in used_t or j in used_d: continue
            t = self.tracks[i]; center, radius = detections[j]; dt = stamp-t.stamp
            if dt < .04: continue
            measured = (center-t.position)/dt
            if np.linalg.norm(measured)>self.max_speed: continue
            residual = float(np.linalg.norm(center-t.position-t.velocity*dt))
            t.velocity = .65*t.velocity+.35*measured
            t.position = center; t.radius = radius; t.stamp = stamp; t.hits += 1
            t.error = .7*t.error+.3*residual
            used_t.add(i); used_d.add(j)
        for j, (center, radius) in enumerate(detections):
            if j not in used_d and len(self.tracks)<self.max_tracks:
                self.tracks.append(Track(self.next_id, center, np.zeros(2), radius, stamp, stamp))
                self.next_id += 1
        self.stamp = stamp

    def prediction(self, now, origin):
        """Rows: x,y,r,vx,vy, uncertainty growth m/s (local map coordinates)."""
        rows, display = [], []
        for t in self.tracks:
            age = now-t.stamp
            if not 0 <= age <= self.ttl: continue
            mature = t.hits>=3 and t.stamp-t.first>=.3
            velocity = t.velocity if mature else np.zeros(2)
            speed = float(np.linalg.norm(velocity))
            # Keep immature and briefly occluded tracks conservatively too.
            growth = .15+min(.8, t.error/.2)+(0. if mature else .5)
            center = t.position+velocity*age
            radius = t.radius+growth*age
            rows.append([*(center-origin), radius, *velocity, growth])
            display.append({'id':t.id, 'position':center.tolist(), 'velocity':velocity.tolist(),
                            'radius_m':radius, 'speed_m_s':speed, 'confirmed':mature,
                            'age_s':age, 'uncertainty_rate_m_s':growth})
        return np.asarray(rows, dtype=float).reshape(-1,6), display


def compact_obstacles(obstacles, cell=.3):
    """Merge same-cell discs into an enclosing disc; never drop a surface."""
    obstacles = np.asarray(obstacles).reshape(-1,3)
    if not len(obstacles): return obstacles
    keys = np.floor(obstacles[:,:2]/cell).astype(int)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    count = int(inverse.max())+1
    centers = np.column_stack([np.bincount(inverse, weights=obstacles[:,i], minlength=count) for i in range(2)])
    centers /= np.bincount(inverse, minlength=count)[:,None]
    radius = np.zeros(count)
    np.maximum.at(radius, inverse, np.linalg.norm(obstacles[:,:2]-centers[inverse],axis=1)+obstacles[:,2])
    return np.c_[centers,radius]
