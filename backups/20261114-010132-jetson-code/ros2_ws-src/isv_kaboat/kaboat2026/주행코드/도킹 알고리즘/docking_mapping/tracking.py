"""Fixed dock_ref source -> current LiDAR target; GPS is only a display anchor.

Matrices are absolute transforms. Neither reference points nor hopping coordinates
are rewritten when the boat, docking structure, or GPS anchor moves.
"""
import math
import numpy as np
from .geometry import apply, rigid, voxel
from .registration import align


def relative_goal(base_from_dock, hopping):
    matrix=rigid(base_from_dock)
    point=apply([hopping.point_dock],matrix)[0]
    heading=None
    if hopping.heading_dock_deg is not None:
        a=math.radians(hopping.heading_dock_deg)
        direction=matrix[:3,:3]@np.array([math.cos(a),math.sin(a),0.])
        if np.linalg.norm(direction[:2])<.5:raise ValueError('Invalid projected docking heading')
        heading=math.degrees(math.atan2(direction[1],direction[0]))
    return dict(point_base=point.tolist(),distance_m=float(np.linalg.norm(point[:2])),
                bearing_deg=math.degrees(math.atan2(point[1],point[0])),heading_error_deg=heading)


class DockTracker:
    def __init__(self,reference,hopping,base_from_lidar,config):
        hopping.check_reference(reference)
        self.reference=reference;self.hopping=hopping
        self.extrinsic=rigid(base_from_lidar).copy();self.config=config
        self.source=voxel(reference.points,config['voxel_m'],config['max_points'])
        self.lidar_from_dock=None;self.world_rotation=None;self.stamp=None
        self.world_from_dock=None;self.goal=None;self.metrics={};self.valid=False
        self.target=np.empty((0,3))

    def invalidate(self,reason):
        self.valid=False;self.goal=None;self.metrics=dict(accepted=False,reason=reason)

    def process(self,points_lidar,stamp,world_from_base=None,rotation_world_base=None):
        self.invalidate('WAIT_REGISTRATION')
        if not math.isfinite(stamp) or (self.stamp is not None and stamp<=self.stamp):
            raise ValueError('Non-increasing tracking stamp')
        cfg=self.config
        rotation=(None if rotation_world_base is None else np.asarray(rotation_world_base)@self.extrinsic[:3,:3])
        if self.lidar_from_dock is None:
            if world_from_base is None:raise ValueError('WAIT_VALID_GPS_FOR_INITIAL_ALIGNMENT')
            initial=np.linalg.inv(rigid(world_from_base)@self.extrinsic)@self.reference.enu_from_dock
        else:
            if stamp-self.stamp>cfg['reacquire_gap_s']:
                raise ValueError('TRACKING_GAP_REQUIRES_EXPLICIT_RESTART')
            initial=self.lidar_from_dock.copy()
            if rotation is not None and self.world_rotation is not None:
                delta=np.eye(4);delta[:3,:3]=rotation.T@self.world_rotation
                initial=delta@initial
        points=np.asarray(points_lidar,dtype=float).reshape(-1,3)
        points=points[np.isfinite(points).all(axis=1)]
        ranges=np.linalg.norm(points,axis=1)
        points=points[(ranges>=cfg['min_range_m'])&(ranges<=cfg['max_range_m'])]
        # Bounding box follows the last relative dock pose, never a fresh GPS jump.
        expected=apply(self.source,initial);margin=cfg['roi_margin_m']
        low=expected.min(axis=0)-margin;high=expected.max(axis=0)+margin
        target=points[np.all((points>=low)&(points<=high),axis=1)]
        target=voxel(target,cfg['voxel_m'],cfg['max_points'])
        self.target=target
        result=align(self.source,target,initial,cfg['registration'])
        self.metrics=result.metrics()
        if not result.accepted:return False
        self.lidar_from_dock=rigid(result.matrix).copy()
        self.stamp=stamp;self.world_rotation=None if rotation is None else rotation.copy()
        self.goal=relative_goal(self.extrinsic@self.lidar_from_dock,self.hopping)
        self.valid=True
        self.world_from_dock=(None if world_from_base is None else
                              rigid(world_from_base)@self.extrinsic@self.lidar_from_dock)
        return True

    @property
    def base_from_dock(self):
        return None if self.lidar_from_dock is None else self.extrinsic@self.lidar_from_dock
