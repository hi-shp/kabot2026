"""Bounded WGS84 top-view data for the existing GPS map GUI."""
import numpy as np
from scipy.spatial.transform import Rotation
from .geometry import voxel,apply
from .mapper import json_safe


def mapping_preview(mapper,reference,hopping,config,report,world_from_dock=None):
    result=dict(preview_schema=1,status=json_safe(report),map_points=[],current_scan=[],rc_path=[],origin_lla=None,
                gps_position=None,corrected_position=None,hopping=None,
                map_points_xyz=[],current_scan_xyz=[],rc_path_xyz=[],
                coordinate_frame='initial_bow_relative',relative_reference=config['sensors']['heading_reference']=='relative',
                gps_offset_lidar_m=config['sensors']['gps_antenna_in_base_m'],
                range_filter_m=[config['preprocess']['min_range_m'],config['preprocess']['max_range_m']])
    if mapper.enu is None:return result
    result['origin_lla']=mapper.enu.origin.tolist()
    m=config['mapping'];limit=min(6000,m['gui_max_points'])
    def latlon(points,cap=limit):
        points=voxel(points,m['gui_voxel_m'],cap)
        return [] if not len(points) else mapper.enu.to_lla(points)[:,:2].tolist()
    points=(mapper.map.points() if reference is None else reference.points_enu() if world_from_dock is None
            else apply(reference.points,world_from_dock))
    anchor=mapper.display_anchor or (None if reference is None else reference.quality.get('display_anchor'))
    # Loaded legacy references use dock_ref axes rather than inventing an initial heading.
    if anchor is None and reference is not None:
        matrix=reference.enu_from_dock
        anchor=dict(origin_xyz=matrix[:3,3].tolist(),yaw_deg=float(Rotation.from_matrix(np.array(matrix[:3,:3],copy=True)).as_euler('xyz',degrees=True)[2]))
        result['coordinate_frame']='dock_ref_relative'
    origin=np.asarray(anchor['origin_xyz']) if anchor else np.zeros(3)
    yaw=anchor['yaw_deg'] if anchor else 0.
    rotation=Rotation.from_euler('z',-yaw,degrees=True).as_matrix()
    def relative(points,cap=limit):
        xyz=voxel(points,m['gui_voxel_m'],cap)
        return ((xyz-origin)@rotation.T).tolist()
    result['display_anchor']=anchor
    result['map_points_xyz']=relative(points)
    result['current_scan_xyz']=relative(mapper.current_scan,min(1000,limit)) if report['state']=='CONVERGED' else []
    result['map_points']=latlon(points)
    result['current_scan']=latlon(mapper.current_scan,min(1000,limit)) if report['state']=='CONVERGED' else []
    path=np.asarray(mapper.path,dtype=float).reshape(-1,4)
    if len(path):
        path=path[np.linspace(0,len(path)-1,min(1000,len(path)),dtype=int)]
        result['rc_path']=mapper.enu.to_lla(path[:,1:])[:,:2].tolist()
        result['rc_path_xyz']=((path[:,1:]-origin)@rotation.T).tolist()
    def pose(matrix,stamp):
        if matrix is None or stamp is None:return None
        lla=mapper.enu.to_lla(matrix[:3,3])
        yaw=Rotation.from_matrix(np.array(matrix[:3,:3],copy=True)).as_euler('xyz',degrees=True)[2]
        return dict(latlon=lla[:2].tolist(),enu=matrix[:3,3].tolist(),yaw_enu_deg=float(yaw),stamp_s=float(stamp),
                    age_s=max(0.,report.get('time_now_s',stamp)-stamp))
    result['gps_position']=pose(mapper.gps_pose,mapper.gps_pose_stamp)
    result['corrected_position']=pose(mapper.corrected_pose,mapper.accepted_poses[-1][0] if mapper.accepted_poses else None)
    for key in ('gps_position','corrected_position'):
        pose=result[key]
        if pose:
            pose['relative_xyz']=((np.asarray(pose['enu'])-origin)@rotation.T).tolist()
            pose['yaw_relative_deg']=float((pose['yaw_enu_deg']-yaw+180)%360-180)
    if hopping is not None:result['hopping']=hopping.preview(reference,world_from_dock)
    return result
