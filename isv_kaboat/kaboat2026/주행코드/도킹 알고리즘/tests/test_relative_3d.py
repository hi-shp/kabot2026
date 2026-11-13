from pathlib import Path
import unittest
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from docking_mapping.geometry import ENU, boat_pose
from docking_mapping.mapper import Mapper
from docking_mapping.preview import mapping_preview
from docking_mapping.gui_bridge import MappingBridge
import json

def field_config():
    return yaml.safe_load((Path(__file__).resolve().parents[1]/'config/mapping.yaml').read_text())

class Relative3DTests(unittest.TestCase):
    def test_production_offset_and_driver_convention(self):
        c=field_config();s=c['sensors']
        self.assertEqual(s['heading_reference'],'relative')
        self.assertEqual(s['gps_antenna_in_base_m'],[-.4,0,0])
        self.assertEqual(s['imu_from_base_rpy_deg'],[0,0,90])
        pose=boat_pose([0,0,0],[0,0,0,1],s)
        np.testing.assert_allclose(pose[:3,3],[0,.4,0],atol=1e-12)

    def test_mapping_horizontal_two_meters_has_no_height_cutoff(self):
        c=field_config();p=c['preprocess']
        self.assertTrue(p['front_only'])
        self.assertEqual(p['max_range_m'],2.)
        self.assertFalse(p['height_filter_enabled'])
        # Isolate range/height behavior from density and voxel averaging.
        p.update(voxel_m=.001,cluster_enabled=False)
        cloud=np.array([[.09,0,0],[.1,0,-20],[2.,0,20],[2.0001,0,0],[1.,1.,40]])
        points=Mapper(c).preprocess(cloud,np.eye(4))
        self.assertEqual(len(points),3)
        self.assertEqual(set(points[:,2]),{-20,20,40})
        self.assertEqual(c['autonomy']['tracking']['max_range_m'],25.)
        self.assertTrue(c['autonomy']['dry_run'])
        self.assertFalse(c['autonomy']['enable_output'])

    def test_front_sector_uses_current_body_axes_after_mount_transform(self):
        c=field_config();c['preprocess'].update(voxel_m=.001,cluster_enabled=False)
        c['sensors']['lidar_in_base_rpy_deg']=[0,0,90]
        # A LiDAR point at Y- is in front after a +90deg mount rotation.
        cloud=np.array([[0,-1,20],[0,1,-20],[0,-2,40],[0,-2.01,40]])
        points=Mapper(c).preprocess(cloud,np.eye(4))
        self.assertEqual(len(points),2)
        self.assertEqual(set(points[:,2]),{20,40})

    def test_relative_yaw_wrap_and_xyz_in_preview(self):
        c=field_config();mapper=Mapper(c);mapper.enu=ENU([35.12,129.1,0])
        origin=np.array([3,4,5]);r=Rotation.from_euler('z',179,degrees=True).as_matrix()
        mapper.display_anchor=dict(origin_xyz=origin.tolist(),yaw_deg=179.)
        mapper.map.add(np.array([[.2,0,2.]])@r.T+origin)
        mapper.gps_pose=np.eye(4)
        mapper.gps_pose[:3,:3]=Rotation.from_euler('z',-179,degrees=True).as_matrix()
        mapper.gps_pose[:3,3]=origin;mapper.gps_pose_stamp=1
        preview=mapping_preview(mapper,None,None,c,{'state':'CONVERGED'})
        np.testing.assert_allclose(preview['map_points_xyz'],[[.2,0,2]],atol=1e-10)
        self.assertAlmostEqual(preview['gps_position']['yaw_relative_deg'],2.)
        self.assertEqual(preview['coordinate_frame'],'initial_bow_relative')
        bridge=MappingBridge();bridge.update(json.dumps(preview))
        self.assertTrue(bridge.snapshot()['connected'])
        invalid=dict(preview,map_points_xyz=[[0,0,float('nan')]])
        bridge2=MappingBridge();bridge2.update(json.dumps(invalid))
        self.assertFalse(bridge2.snapshot()['connected'])

if __name__=='__main__':unittest.main()
