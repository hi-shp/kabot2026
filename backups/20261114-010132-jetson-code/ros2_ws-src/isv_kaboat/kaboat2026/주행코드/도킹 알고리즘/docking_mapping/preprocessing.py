"""Bounded DBSCAN-style structure filtering and scan motion rejection."""
from collections import deque
import math
import numpy as np
from scipy.spatial import cKDTree


def clustered_points(points,eps,min_samples,min_cluster_points):
    points=np.asarray(points,dtype=float).reshape(-1,3)
    if not len(points):return points
    tree=cKDTree(points)
    distances,_=tree.query(points,k=min(min_samples,len(points)))
    if len(points)<min_samples:return np.empty((0,3))
    core=np.asarray(distances)[:,-1]<=eps
    labels=np.full(len(points),-1,dtype=int)
    cluster=0
    for seed in np.flatnonzero(core):
        if labels[seed]>=0:continue
        labels[seed]=cluster
        queue=deque([int(seed)])
        while queue:
            current=queue.popleft()
            for neighbor in tree.query_ball_point(points[current],eps):
                if labels[neighbor]>=0:continue
                labels[neighbor]=cluster
                if core[neighbor]:queue.append(neighbor)
        cluster+=1
    keep=np.zeros(len(points),dtype=bool)
    for label in range(cluster):
        selected=labels==label
        if np.count_nonzero(selected)>=min_cluster_points:keep|=selected
    return points[keep]


def scan_motion_reason(duration,angular_speed_rad_s,speed_bound_mps,config):
    """Reject excessive within-scan motion; this is not per-point deskew.

    Duration is max(time)-min(time) from the scan's actual point time field.
    Translation uses a configured conservative speed bound, not noisy GPS deltas.
    """
    if duration is None or not math.isfinite(duration) or not 0<duration<=config['max_scan_duration_s']:
        return 'WAIT_SCAN_TIMING'
    if not math.isfinite(angular_speed_rad_s):return 'INVALID_IMU_ANGULAR_RATE'
    if speed_bound_mps*duration>config['max_scan_translation_m']:return 'EXCESSIVE_SCAN_TRANSLATION'
    if math.degrees(abs(angular_speed_rad_s)*duration)>config['max_scan_rotation_deg']:
        return 'EXCESSIVE_SCAN_ROTATION'
    return ''
