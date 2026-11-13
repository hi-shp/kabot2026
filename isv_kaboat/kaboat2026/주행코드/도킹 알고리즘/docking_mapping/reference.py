"""Immutable dock_ref geometry with versioned WGS84/ENU metadata."""
from copy import deepcopy
from datetime import datetime,timezone
import hashlib
from pathlib import Path
import uuid

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from .geometry import ENU,apply,rigid,transform
from .mapper import json_safe


def digest(points):
    return hashlib.sha256(np.ascontiguousarray(points,dtype='<f8').tobytes()).hexdigest()


class ReferenceMap:
    def __init__(self,points_dock,origin_lla,enu_from_dock,sensor_config,voxel_m,
                 selection,reference_id=None,version=1,created_utc=None,path_enu=None,quality=None):
        points=np.array(points_dock,dtype=float,copy=True)
        if points.ndim!=2 or points.shape[1]!=3 or not len(points) or not np.isfinite(points).all():
            raise ValueError('Invalid 3D reference points')
        origin=ENU(origin_lla).origin.copy()
        matrix=rigid(enu_from_dock).copy()
        if type(version) is not int or version<1 or not np.isfinite(voxel_m) or voxel_m<=0:
            raise ValueError('Invalid reference version or voxel size')
        identity=reference_id or uuid.uuid4().hex
        if not isinstance(identity,str) or uuid.UUID(identity).hex!=identity:
            raise ValueError('Invalid reference map identifier')
        points.flags.writeable=False;origin.flags.writeable=False;matrix.flags.writeable=False
        self.points=points
        self.origin_lla=origin
        self.enu_from_dock=matrix
        self.reference_id=identity;self.version=version
        self.sensor_config=deepcopy(sensor_config)
        self.voxel_m=float(voxel_m)
        self.selection=deepcopy(selection)
        self.created_utc=created_utc or datetime.now(timezone.utc).isoformat()
        self.path_enu=np.array(path_enu if path_enu is not None else [],dtype=float).reshape(-1,4)
        if not np.isfinite(self.path_enu).all():raise ValueError('Invalid saved path')
        self.path_enu.flags.writeable=False
        self.quality=deepcopy(quality or {})

    @classmethod
    def from_mapper(cls,mapper,config):
        if not mapper.confirmed or mapper.enu is None:
            raise ValueError('A confirmed, georeferenced map is required')
        low,high=config['roi_min_enu_m'],config['roi_max_enu_m']
        if low is None or high is None:
            raise ValueError('REFERENCE_SELECTION_REQUIRED: set structure ROI in ENU')
        low,high=np.asarray(low,dtype=float),np.asarray(high,dtype=float)
        if low.shape!=(3,) or high.shape!=(3,) or not np.isfinite([low,high]).all() or np.any(low>=high):
            raise ValueError('Invalid structure selection ROI')
        points=mapper.map.points()
        points=points[np.all((points>=low)&(points<=high),axis=1)]
        if len(points)<config['min_points']:raise ValueError('Insufficient selected structure points')
        if config['origin_mode']!='selection_centroid':raise ValueError('Unsupported dock_ref origin mode')
        yaw=config['yaw_enu_deg']
        if not np.isfinite(yaw):raise ValueError('Invalid dock_ref yaw')
        matrix=transform(Rotation.from_euler('z',yaw,degrees=True).as_matrix(),points.mean(axis=0))
        # No projection to a plane: z is preserved in the rigid 3D transform.
        dock=apply(points,np.linalg.inv(matrix))
        return cls(dock,mapper.enu.origin,matrix,mapper.config['sensors'],mapper.map.size,
                   dict(roi_min_enu_m=low.tolist(),roi_max_enu_m=high.tolist(),origin_mode=config['origin_mode']),
                   path_enu=list(mapper.path),quality=dict(mapper.status,display_anchor=mapper.display_anchor))

    def points_enu(self):
        return apply(self.points,self.enu_from_dock)

    def metadata(self):
        return json_safe(dict(schema=2,kind='docking_reference',frame='dock_ref',
            reference_id=self.reference_id,version=self.version,created_utc=self.created_utc,
            origin_lla=self.origin_lla,enu_frame='ENU_WGS84',altitude_datum='WGS84_ellipsoid',
            initial_enu_from_dock=self.enu_from_dock,sensors=self.sensor_config,voxel_m=self.voxel_m,
            selection=self.selection,point_count=len(self.points),points_sha256=digest(self.points),
            quality=self.quality,hopping_point=None))

    def save(self,maps_dir,companion_writer=None):
        stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
        folder=Path(maps_dir)/f'reference_{stamp}_{self.reference_id}_v{self.version}'
        folder.mkdir(parents=True,exist_ok=False)
        np.savez_compressed(folder/'docking_reference.npz',points_dock=self.points,path_enu=self.path_enu)
        header=f'# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 8 8 8\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(self.points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(self.points)}\nDATA ascii\n'
        with (folder/'docking_reference.pcd').open('w',encoding='ascii') as stream:
            stream.write(header);np.savetxt(stream,self.points,fmt='%.17g')
        if companion_writer is not None:companion_writer(folder)
        metadata=self.metadata()
        if (folder/'hopping_point.yaml').is_file():metadata['hopping_point']='hopping_point.yaml'
        temporary=folder/'reference_metadata.tmp'
        temporary.write_text(yaml.safe_dump(metadata,allow_unicode=True,sort_keys=False),encoding='utf-8')
        temporary.replace(folder/'docking_reference_metadata.yaml')
        return folder

    @classmethod
    def load_latest(cls,maps_dir,expected_sensors,max_points):
        candidates=sorted(folder for folder in Path(maps_dir).glob('reference_*')
                          if folder.is_dir() and not folder.is_symlink()
                          and (folder/'docking_reference_metadata.yaml').is_file())
        if not candidates:raise ValueError('No saved dock_ref reference map')
        folder=candidates[-1]
        meta=yaml.safe_load((folder/'docking_reference_metadata.yaml').read_text(encoding='utf-8'))
        required=('reference_id','version','origin_lla','initial_enu_from_dock','sensors',
                  'voxel_m','selection','created_utc','points_sha256','point_count')
        if (not isinstance(meta,dict) or meta.get('schema')!=2 or meta.get('kind')!='docking_reference'
                or meta.get('frame')!='dock_ref' or meta.get('enu_frame')!='ENU_WGS84'
                or meta.get('altitude_datum')!='WGS84_ellipsoid' or any(k not in meta for k in required)):
            raise ValueError('Missing or incompatible reference metadata')
        if meta['sensors']!=expected_sensors:raise ValueError('Saved sensor calibration differs from active YAML')
        if type(meta['point_count']) is not int or not 0<meta['point_count']<=max_points:
            raise ValueError('Invalid or oversized reference point count')
        with np.load(folder/'docking_reference.npz',allow_pickle=False) as archive:
            points=archive['points_dock'];path=archive['path_enu']
        if len(points)!=meta['point_count'] or digest(points)!=meta['points_sha256']:
            raise ValueError('Reference geometry checksum mismatch')
        result=cls(points,meta['origin_lla'],meta['initial_enu_from_dock'],meta['sensors'],meta['voxel_m'],
                   meta['selection'],meta['reference_id'],meta['version'],meta['created_utc'],path,meta.get('quality'))
        return result,folder
