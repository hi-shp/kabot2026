"""Bounded, fixed-origin mapping state. Rejected scans never mutate the map."""
from collections import OrderedDict, deque
from copy import deepcopy
from datetime import datetime, timezone
import json
from pathlib import Path
import uuid

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from .geometry import ENU, apply, boat_pose, lidar_extrinsic, rigid, voxel
from .registration import align
from .preprocessing import clustered_points


def validate_config(c):
    for section in ('sensors', 'preprocess', 'registration', 'mapping'):
        if not isinstance(c.get(section), dict):
            raise ValueError('Missing configuration section: ' + section)
    p, r, m, s = (c[k] for k in ('preprocess', 'registration', 'mapping', 'sensors'))
    for section, fields in [(p, ['min_range_m','max_range_m','voxel_m']),
                            (r, ['max_correspondence_m','max_rmse_m','translation_tolerance_m',
                                 'rotation_tolerance_deg','max_translation_correction_m','max_rotation_correction_deg']),
                            (m, ['voxel_m','local_radius_m','max_gps_sigma_m','gps_jump_margin_m',
                                 'max_boat_speed_mps','max_prediction_gap_s','processing_hz','gui_hz','gui_voxel_m',
                                 'gps_sync_tolerance_s','imu_sync_tolerance_s','input_timeout_s','rc_timeout_s'])]:
        for name in fields:
            if isinstance(section[name], bool) or not np.isfinite(section[name]) or section[name] <= 0:
                raise ValueError('Invalid positive setting: ' + name)
    for section, fields, lower in [(p, ['min_points','max_points'], 20),
                                   (r, ['min_correspondences','covariance_neighbors','max_iterations','inner_iterations'], 3),
                                   (m, ['max_voxels','max_path_points','gui_max_points'], 1)]:
        for name in fields:
            if type(section[name]) is not int or section[name] < lower:
                raise ValueError('Invalid integer setting: '+name)
    if r['method'] not in ('icp','gicp') or not 0 < r['trim_fraction'] <= 1 or not 0 < r['min_inlier_ratio'] <= 1:
        raise ValueError('Invalid registration settings')
    if not 0 < r['min_information_ratio'] < 1:
        raise ValueError('Invalid information ratio')
    if p['min_range_m'] >= p['max_range_m'] or p['min_points'] > p['max_points']:
        raise ValueError('Invalid range/point bounds')
    for low, high in [('roi_min_base_m','roi_max_base_m')]:
        if np.asarray(p[low]).shape != (3,) or np.asarray(p[high]).shape != (3,) or np.any(np.asarray(p[low]) >= p[high]):
            raise ValueError('Invalid body ROI')
    if not p['height_min_relative_m'] < p['height_max_relative_m']:
        raise ValueError('Invalid leveled height limits')
    if s['heading_reference'] not in ('magnetic', 'true', 'unknown', 'relative'):
        raise ValueError('Invalid heading_reference')
    for field in ('imu_from_base_rpy_deg','gps_antenna_in_base_m','lidar_in_base_rpy_deg','lidar_in_base_xyz_m'):
        value = np.asarray(s[field], dtype=float)
        if value.shape != (3,) or not np.isfinite(value).all():
            raise ValueError('Invalid extrinsic: '+field)
    if type(s['extrinsics_verified']) is not bool:
        raise ValueError('extrinsics_verified must be boolean')
    if s['magnetic_declination_deg'] is not None and not np.isfinite(s['magnetic_declination_deg']):
        raise ValueError('Invalid declination')
    if p.get('range_mode','radial') not in ('radial','horizontal'):
        raise ValueError('Invalid range mode')
    if type(p.get('front_only',False)) is not bool:
        raise ValueError('Invalid front-only flag')
    if type(p.get('height_filter_enabled',True)) is not bool:
        raise ValueError('Invalid height filter flag')
    if p.get('cluster_enabled',False):
        if (not np.isfinite(p['cluster_eps_m']) or p['cluster_eps_m']<=0
                or type(p['cluster_min_samples']) is not int or p['cluster_min_samples']<2
                or type(p['cluster_min_points']) is not int or p['cluster_min_points']<2):
            raise ValueError('Invalid clustering parameters')
    motion=c.get('scan_motion',{})
    if motion.get('enabled',False):
        for name in ('max_scan_duration_s','max_scan_translation_m','max_scan_rotation_deg'):
            if not np.isfinite(motion[name]) or motion[name]<=0:raise ValueError('Invalid scan motion limit')
        duration=motion.get('fallback_duration_s')
        if duration is not None and (not np.isfinite(duration) or duration<=0):
            raise ValueError('Invalid fallback scan duration')
    reference=c.get('reference')
    if reference is not None:
        if type(reference['min_points']) is not int or reference['min_points']<3:
            raise ValueError('Invalid reference minimum points')
        if reference['origin_mode']!='selection_centroid' or not np.isfinite(reference['yaw_enu_deg']):
            raise ValueError('Invalid dock_ref frame definition')
    hopping=c.get('hopping')
    if hopping is not None:
        for name in ('max_reference_distance_m','default_radius_m','default_hold_seconds'):
            if isinstance(hopping[name],bool) or not np.isfinite(hopping[name]) or hopping[name]<=0:
                raise ValueError('Invalid hopping setting: '+name)
        height=hopping['plane_height_enu_m']
        if height is not None and (isinstance(height,bool) or not np.isfinite(height)):
            raise ValueError('Invalid hopping ENU plane height')
    return c


class VoxelMap:
    def __init__(self, size, capacity):
        self.size, self.capacity = size, capacity
        self.cells = OrderedDict()
        self.evictions = 0

    def add(self, points):
        for point in voxel(points, self.size):
            key = tuple(np.floor(point/self.size).astype(np.int64))
            if key in self.cells:
                previous, count = self.cells.pop(key)
                count = min(count, 100)
                self.cells[key] = ((previous*count+point)/(count+1), count+1)
            else:
                if len(self.cells) >= self.capacity:
                    self.cells.popitem(last=False)
                    self.evictions += 1
                self.cells[key] = (point.copy(), 1)

    def points(self):
        return np.asarray([value[0] for value in self.cells.values()], dtype=float).reshape(-1,3)


def json_safe(value):
    if isinstance(value, dict): return {k:json_safe(v) for k,v in value.items()}
    if isinstance(value, (list,tuple)): return [json_safe(v) for v in value]
    if isinstance(value,np.ndarray): return json_safe(value.tolist())
    if isinstance(value,(float,np.floating)): return float(value) if np.isfinite(value) else None
    if isinstance(value,np.integer): return int(value)
    if isinstance(value,np.bool_): return bool(value)
    return value


class Mapper:
    def __init__(self, config):
        self.config = validate_config(deepcopy(config))
        self.reset()

    def reset(self):
        m=self.config['mapping']
        self.enu=None
        self.map=VoxelMap(m['voxel_m'],m['max_voxels'])
        self.confirmed=False
        self.path=deque(maxlen=m['max_path_points'])
        self.history=deque(maxlen=1000)
        self.accepted_poses=deque(maxlen=2)
        self.last_good_gps=None
        self.current_scan=np.empty((0,3))
        self.gps_pose=self.initial_pose=self.corrected_pose=None
        self.gps_pose_stamp=None
        self.display_anchor=None
        self.status={'state':'WAIT_GPS','map_points':0}

    def fail(self, state, **details):
        self.current_scan=np.empty((0,3))
        self.status=dict(state=state,map_points=len(self.map.cells) if self.confirmed else 0,
                         provisional_points=0 if self.confirmed else len(self.map.cells),**details)
        return self.status

    def preprocess(self, cloud, base_pose):
        p=self.config['preprocess']
        cloud=np.asarray(cloud,dtype=float).reshape(-1,3)
        cloud=cloud[np.isfinite(cloud).all(axis=1)]
        radius=np.linalg.norm(cloud[:,:2] if p.get('range_mode')=='horizontal' else cloud,axis=1)
        cloud=cloud[(radius>=p['min_range_m'])&(radius<=p['max_range_m'])]
        base=apply(cloud,lidar_extrinsic(self.config['sensors']))
        axes=3 if p.get('height_filter_enabled',True) else 2
        keep=np.all((base[:,:axes]>=p['roi_min_base_m'][:axes])&(base[:,:axes]<=p['roi_max_base_m'][:axes]),axis=1)
        if p.get('front_only',False):
            keep &= base[:,0]>=0.
        # Height is measured along ENU up after full roll/pitch compensation,
        # relative to base_link, independent of GPS ellipsoid altitude noise.
        relative_up=(base @ base_pose[:3,:3].T)[:,2]
        if p.get('height_filter_enabled',True):
            keep &= (relative_up>=p['height_min_relative_m'])&(relative_up<=p['height_max_relative_m'])
        if p['water_filter_enabled']:
            if not p['water_level_verified']:
                raise ValueError('WAIT_WATER_LEVEL')
            keep &= np.abs(relative_up-p['water_height_relative_m'])>p['water_exclusion_half_width_m']
        result=voxel(cloud[keep],p['voxel_m'],p['max_points'])
        if p.get('cluster_enabled',False):
            result=clustered_points(result,p['cluster_eps_m'],p['cluster_min_samples'],p['cluster_min_points'])
        return result

    def predicted_position(self, stamp):
        if not self.accepted_poses:
            return None
        previous_time,previous=self.accepted_poses[-1]
        dt=stamp-previous_time
        if not 0<=dt<=self.config['mapping']['max_prediction_gap_s']:
            return None
        velocity=np.zeros(3)
        if len(self.accepted_poses)==2:
            old_time,old=self.accepted_poses[0]
            if previous_time>old_time:
                velocity=(previous[:3,3]-old[:3,3])/(previous_time-old_time)
                speed=np.linalg.norm(velocity)
                velocity*=min(1,self.config['mapping']['max_boat_speed_mps']/max(speed,1e-9))
        return previous[:3,3]+velocity*dt

    def process(self, cloud, lla, quaternion, stamp, sigma, fix_status=0, commit_guard=None):
        m=self.config['mapping']
        if fix_status<0 or not np.isfinite(sigma) or sigma<0 or sigma>m['max_gps_sigma_m']:
            return self.fail('WAIT_VALID_GPS')
        if self.accepted_poses and stamp<=self.accepted_poses[-1][0]:
            return self.fail('OUT_OF_ORDER_SCAN')
        if self.enu is None:
            candidate=ENU(lla)
        else:
            candidate=self.enu
        gps_enu=candidate.to_enu(lla)
        try:
            gps_pose=boat_pose(gps_enu,quaternion,self.config['sensors'])
            source=self.preprocess(cloud,gps_pose)
        except ValueError as exc:
            return self.fail(str(exc))
        if self.enu is None:
            self.enu=candidate
        self.gps_pose=gps_pose
        self.gps_pose_stamp=stamp
        if self.display_anchor is None:
            yaw=Rotation.from_matrix(np.array(gps_pose[:3,:3],copy=True)).as_euler('xyz',degrees=True)[2]
            self.display_anchor=dict(origin_xyz=gps_pose[:3,3].tolist(),yaw_deg=float(yaw))
        if len(source)<self.config['preprocess']['min_points']:
            return self.fail('INSUFFICIENT_SCAN',scan_points=len(source))
        initial=gps_pose.copy()
        jump=False
        if self.last_good_gps is not None:
            old_stamp,old_position,old_sigma=self.last_good_gps
            dt=max(0,stamp-old_stamp)
            limit=m['gps_jump_margin_m']+m['max_boat_speed_mps']*dt+2*np.hypot(sigma,old_sigma)
            jump=np.linalg.norm(gps_pose[:3,3]-old_position)>limit
        predicted=self.predicted_position(stamp)
        if jump:
            if predicted is None:
                return self.fail('GPS_JUMP_NO_PREDICTION',gps_outlier=True)
            initial[:3,3]=predicted
        self.initial_pose=initial.copy()
        extrinsic=lidar_extrinsic(self.config['sensors'])
        initial_lidar=initial@extrinsic
        if not len(self.map.cells):
            if commit_guard is not None and not commit_guard():
                return self.fail('WAIT_RC')
            self.map.add(apply(source,initial_lidar))
            self.last_good_gps=(stamp,gps_pose[:3,3].copy(),sigma)
            self.accepted_poses.append((stamp,initial.copy()))
            self.history.append(dict(stamp=stamp,raw_gps_lla=np.asarray(lla).copy(),gps_antenna_enu=gps_enu.copy(),
                gps_sigma_m=sigma,initial=initial.copy(),gps=gps_pose.copy(),corrected=None,reason='PROVISIONAL_SEED'))
            return self.fail('INITIALIZING',provisional_seed=True)
        target=self.map.points()
        target=target[np.linalg.norm(target-initial_lidar[:3,3],axis=1)<=m['local_radius_m']]
        target=voxel(target,self.config['preprocess']['voxel_m'],self.config['preprocess']['max_points']*3)
        result=align(source,target,initial_lidar,self.config['registration'])
        corrected=result.matrix@np.linalg.inv(extrinsic)
        # Physical motion bound is independent of the allowed ICP correction.
        if result.accepted and self.accepted_poses:
            dt=stamp-self.accepted_poses[-1][0]
            max_step=m['gps_jump_margin_m']+m['max_boat_speed_mps']*dt
            if np.linalg.norm(corrected[:3,3]-self.accepted_poses[-1][1][:3,3])>max_step:
                result.accepted=False;result.reason='CORRECTED_MOTION_JUMP'
        details=result.metrics()
        self.history.append(dict(stamp=stamp,raw_gps_lla=np.asarray(lla).copy(),gps_antenna_enu=gps_enu.copy(),
                                 gps_sigma_m=sigma,gps=gps_pose.copy(),initial=initial.copy(),
                                 corrected=corrected.copy(),gps_outlier=jump,**details))
        if not result.accepted:
            return self.fail(result.reason,gps_outlier=jump,**{k:v for k,v in details.items() if k!='reason'})
        if commit_guard is not None and not commit_guard():
            return self.fail('WAIT_RC')
        self.current_scan=apply(source,result.matrix)
        self.map.add(self.current_scan)
        self.confirmed=True
        self.corrected_pose=corrected
        self.accepted_poses.append((stamp,corrected.copy()))
        if not jump:
            self.last_good_gps=(stamp,gps_pose[:3,3].copy(),sigma)
        self.path.append((stamp,*corrected[:3,3]))
        self.status=dict(state='CONVERGED',map_points=len(self.map.cells),provisional_points=0,
                         gps_outlier=jump,voxel_evictions=self.map.evictions,
                         position_difference_m=float(np.linalg.norm(corrected[:3,3]-gps_pose[:3,3])),
                         **{k:v for k,v in details.items() if k!='reason'})
        return self.status

    def save(self, maps_dir):
        if self.enu is None or not len(self.map.cells):
            raise ValueError('No georeferenced map to save')
        folder=Path(maps_dir)/('map_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid.uuid4().hex[:8])
        folder.mkdir(parents=True,exist_ok=False)
        points=self.map.points()
        metadata=json_safe(dict(schema=1,origin_lla=self.enu.origin,frame='ENU_WGS84',
            altitude_datum='WGS84_ellipsoid',created_utc=datetime.now(timezone.utc).isoformat(),
            confirmed=self.confirmed,config=self.config,status=self.status,history=list(self.history),display_anchor=self.display_anchor))
        try:
            np.savez_compressed(folder/'docking_map.npz',points=points,path=np.asarray(self.path).reshape(-1,4))
            header=f'# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA ascii\n'
            with (folder/'docking_map.pcd').open('w',encoding='ascii') as stream:
                stream.write(header)
                np.savetxt(stream,points,fmt='%.6f')
            temp=folder/'metadata.tmp'
            temp.write_text(yaml.safe_dump(metadata,allow_unicode=True,sort_keys=False),encoding='utf-8')
            temp.replace(folder/'docking_map_metadata.yaml')
        except Exception:
            # No metadata marker means the incomplete snapshot cannot be loaded.
            raise
        return folder

    def load(self, maps_dir):
        root=Path(maps_dir).resolve()
        candidates=sorted(p for p in root.glob('map_*') if p.is_dir() and not p.is_symlink()
                          and (p/'docking_map_metadata.yaml').is_file())
        if not candidates:
            raise ValueError('No complete saved map')
        folder=candidates[-1]
        meta=yaml.safe_load((folder/'docking_map_metadata.yaml').read_text(encoding='utf-8'))
        if (not isinstance(meta,dict) or meta.get('schema')!=1 or meta.get('frame')!='ENU_WGS84'
                or 'origin_lla' not in meta or type(meta.get('confirmed')) is not bool):
            raise ValueError('Map lacks origin/frame/schema metadata')
        enu=ENU(meta['origin_lla'])
        stored=validate_config(meta['config'])
        if stored['sensors'] != self.config['sensors']:
            raise ValueError('Saved sensor calibration differs from active YAML')
        if stored['mapping']['voxel_m'] != self.config['mapping']['voxel_m']:
            raise ValueError('Saved voxel size differs from active YAML')
        with np.load(folder/'docking_map.npz',allow_pickle=False) as archive:
            points=archive['points'];path=archive['path']
            if (points.ndim!=2 or points.shape[1]!=3 or not len(points) or not np.isfinite(points).all()
                    or path.ndim!=2 or path.shape[1]!=4 or not np.isfinite(path).all()
                    or len(points)>self.config['mapping']['max_voxels']):
                raise ValueError('Malformed or oversized saved map')
            newmap=VoxelMap(self.map.size,self.map.capacity)
            newmap.add(points)
        self.reset()
        self.enu,self.map,self.confirmed=enu,newmap,meta['confirmed']
        self.path.extend(map(tuple,path))
        self.display_anchor=meta.get('display_anchor')
        self.status=dict(state='LOADED',map_points=len(newmap.cells) if self.confirmed else 0,
                         provisional_points=0 if self.confirmed else len(newmap.cells))
        return folder
