"""Fixed-map LiDAR localization and existing V3 planning; no actuator interface."""
from collections import deque
from copy import deepcopy
import math
import numpy as np
from .geometry import apply,rigid,voxel
from .registration import align


def relative_target(map_from_base,point_map_xy):
    pose=rigid(map_from_base);point=np.asarray(point_map_xy,dtype=float)
    if point.shape!=(2,) or not np.isfinite(point).all():raise ValueError('Invalid map target')
    if np.linalg.norm(pose[:2,0])<.5:raise ValueError('Boat heading cannot be projected horizontally')
    yaw=math.atan2(pose[1,0],pose[0,0]);c,s=math.cos(yaw),math.sin(yaw)
    dx,dy=point-pose[:2,3]
    xy=np.array([c*dx+s*dy,-s*dx+c*dy])
    return dict(point_base_xy_m=xy.tolist(),distance_m=float(np.linalg.norm(xy)),
                bearing_deg=math.degrees(math.atan2(xy[1],xy[0])),frame='boat_horizontal')


class MapLidarTracker:
    def __init__(self,points_map,base_from_lidar,config):
        self.config=deepcopy(config);t=config['autonomy']['tracking']
        self.points_map=voxel(points_map,t['voxel_m'],config['mapping']['max_voxels'])
        if len(self.points_map)<config['registration']['min_correspondences']:
            raise ValueError('INSUFFICIENT_REFERENCE_POINTS')
        self.points_map.flags.writeable=False
        self.extrinsic=rigid(base_from_lidar).copy()
        self.poses=deque(maxlen=2);self.rotation=None;self.pose=None;self.metrics={};self.valid=False

    @property
    def initialized(self):return bool(self.poses)

    def process(self,cloud,stamp,rotation_world_base,initial_map_from_base=None):
        self.valid=False;t=self.config['autonomy']['tracking'];m=self.config['mapping']
        if not math.isfinite(stamp):raise ValueError('Invalid scan stamp')
        rotation=np.asarray(rotation_world_base,dtype=float)
        rigid(np.block([[rotation,np.zeros((3,1))],[np.zeros((1,3)),np.ones((1,1))]]))
        if self.initialized:
            old_stamp,old=self.poses[-1]
            if stamp<=old_stamp:raise ValueError('Non-increasing tracking stamp')
            if stamp-old_stamp>t['reacquire_gap_s']:raise ValueError('TRACKING_GAP_REQUIRES_RESTART')
            guess=old.copy()
            guess[:3,:3]=old[:3,:3]@self.rotation.T@rotation
            # A GPS initial pose is deliberately ignored after the first accepted match.
        else:
            if initial_map_from_base is None:raise ValueError('WAIT_INITIAL_ALIGNMENT')
            guess=rigid(initial_map_from_base).copy()
        initial=guess@self.extrinsic
        cloud=np.asarray(cloud,dtype=float).reshape(-1,3)
        cloud=cloud[np.isfinite(cloud).all(axis=1)]
        radius=np.linalg.norm(cloud,axis=1)
        cloud=cloud[(radius>=t['min_range_m'])&(radius<=t['max_range_m'])]
        expected=apply(self.points_map,np.linalg.inv(initial));margin=t['roi_margin_m']
        cloud=cloud[np.all((cloud>=expected.min(axis=0)-margin)&(cloud<=expected.max(axis=0)+margin),axis=1)]
        source=voxel(cloud,t['voxel_m'],t['max_points'])
        registration=deepcopy(t['registration'] if self.initialized else self.config['registration'])
        registration['method']='gicp'
        result=align(source,self.points_map,initial,registration)
        self.metrics=dict(result.metrics(),scan_points=len(source),reference_points=len(self.points_map),method='gicp')
        if not result.accepted:return False
        pose=result.matrix@np.linalg.inv(self.extrinsic)
        if self.initialized:
            dt=stamp-self.poses[-1][0]
            if np.linalg.norm(pose[:3,3]-self.pose[:3,3])>m['gps_jump_margin_m']+m['max_boat_speed_mps']*dt:
                self.metrics.update(accepted=False,reason='CORRECTED_MOTION_JUMP');return False
        self.pose=rigid(pose).copy();self.rotation=rotation.copy();self.poses.append((stamp,self.pose.copy()))
        self.valid=True;return True


class MapObstacleScan:
    """Add fixed-map obstacles to measured scan; never turn missing beams into free space."""
    def __init__(self,scan,map_obstacles_base):
        self.scan=scan
        for name in ('angle_min','angle_increment','range_min','range_max','stamp','yaw','x','y'):
            setattr(self,name,getattr(scan,name))
        self.ranges=np.asarray(scan.ranges,dtype=float).copy()
        body=np.asarray(map_obstacles_base,dtype=float).reshape(-1,2)
        body=body[np.isfinite(body).all(axis=1)]
        dx,dy=(body-np.array([scan.x,scan.y])).T;c,s=math.cos(scan.yaw),math.sin(scan.yaw)
        self.map_sensor=np.column_stack((c*dx+s*dy,-s*dx+c*dy))
        distances=np.linalg.norm(self.map_sensor,axis=1)
        keep=(distances>=scan.range_min)&(distances<=scan.range_max)
        self.map_sensor=self.map_sensor[keep];distances=distances[keep]
        if len(self.ranges) and abs(scan.angle_increment)>0 and len(distances):
            angle=np.arctan2(self.map_sensor[:,1],self.map_sensor[:,0])
            relative=(np.sign(scan.angle_increment)*(angle-scan.angle_min))%(2*math.pi)
            indices=np.rint(relative/abs(scan.angle_increment)).astype(int)
            inside=(indices>=0)&(indices<len(self.ranges))
            indices=indices[inside];distances=distances[inside]
            measured=((np.isfinite(self.ranges)&(self.ranges>=scan.range_min)&(self.ranges<=scan.range_max))|np.isposinf(self.ranges))
            valid=measured[indices]
            np.minimum.at(self.ranges,indices[valid],distances[valid])

    def points(self,yaw=0.,tx=0.,ty=0.):
        c,s=math.cos(yaw),math.sin(yaw);p=self.map_sensor
        mapped=np.column_stack((c*p[:,0]-s*p[:,1]+tx,s*p[:,0]+c*p[:,1]+ty))
        return np.concatenate((self.scan.points(yaw,tx,ty),mapped))


def plan_v3(map_from_base,target_xy,points_map,scan,route_config,radius,raw_scan=None):
    # Import the actual deployed V3; no copied ranking function or new PID.
    from kaboat_avoidance.avoidance_v3 import choose_gap_v3,RouteMissionV3
    from isv_shape_navigation.local_dock import motion_clear
    from isv_shape_navigation.supplied_pwm import heading_pwm,obstacle_pwm,soften_turn_pwm,amplify_turn_pwm
    goal=relative_target(map_from_base,target_xy)
    result=dict(goal=goal,pwm=[1500,1500],state='WAIT_SCAN',hardware_output_enabled=False)
    if goal['distance_m']<=route_config.wp_radius:
        return dict(result,state='ARRIVED')
    map_base=apply(points_map,np.linalg.inv(map_from_base))[:,:2]
    fused=MapObstacleScan(scan,map_base)
    raw=None if raw_scan is None else MapObstacleScan(raw_scan,map_base)
    desired=math.radians(goal['bearing_deg'])
    angle,report=choose_gap_v3(fused,desired,route_config,radius,raw_scan=raw)
    result.update(avoidance=report,map_obstacle_points=len(fused.map_sensor))
    if angle is None:return dict(result,state='BLOCKED')
    if abs(goal['bearing_deg'])>route_config.avoidance_fov_deg:
        if not motion_clear(fused,np.zeros(2),radius):return dict(result,state='ALIGN_BLOCKED')
        pwm=heading_pwm(goal['distance_m'],-goal['bearing_deg'],forward_boost=route_config.forward_pwm_boost)
        state='ALIGN_TARGET'
    elif report.get('selected_direct'):
        pwm=heading_pwm(goal['distance_m'],-math.degrees(angle),forward_boost=route_config.forward_pwm_boost)
        state='APPROACH_TARGET'
    else:
        pwm=obstacle_pwm(report['selected_distance'],-math.degrees(angle),forward_boost=route_config.forward_pwm_boost)
        state='AVOID_OBSTACLE'
    pwm=soften_turn_pwm(pwm,route_config.turn_pwm_scale,route_config.turn_pwm_max_difference)
    pwm=tuple(round(1500+(v-1500)*route_config.propulsion_pwm_scale) for v in pwm)
    pwm=amplify_turn_pwm(pwm,RouteMissionV3.output_turn_gain)
    return dict(result,state=state,pwm=list(pwm),selected_angle_deg=math.degrees(angle))
