"""Conservative observed free-space gate, never treating absent returns as clear.

This LiDAR gate cannot certify submerged hazards. A measured hull/height envelope
must be configured and verified before it can permit even a simulated manoeuvre.
"""
import numpy as np


def corridor_clear(points_base,config,reverse=False):
    if not config['envelope_verified']:return False
    points=np.asarray(points_base,dtype=float).reshape(-1,3)
    points=points[np.isfinite(points).all(axis=1)]
    if not len(points):return False
    x=points[:,0]*(-1 if reverse else 1);y=points[:,1];z=points[:,2]
    length=config['reverse_length_m'] if reverse else config['forward_length_m']
    width=config['half_width_m'];start=config['hull_half_length_m']
    occupied=(x>start)&(x<start+length)&(np.abs(y)<width)&(z>=config['z_min_m'])&(z<=config['z_max_m'])
    if occupied.any():return False
    # Each angular bin must have a measured return beyond the whole corridor.
    half=np.arctan2(width,start+length)
    angle=np.arctan2(y,x);edges=np.linspace(-half,half,config['coverage_bins']+1)
    # Use a conservative maximum obstacle height for visibility: floor/water points
    # alone do not establish free space at collision height.
    visible=(x>=start+length)&(z>=config['visibility_z_min_m'])&(z<=config['visibility_z_max_m'])
    for low,high in zip(edges[:-1],edges[1:]):
        if np.count_nonzero(visible&(angle>=low)&(angle<=high))<config['min_returns_per_bin']:return False
    return True
