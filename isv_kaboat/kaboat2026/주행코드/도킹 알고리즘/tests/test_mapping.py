from copy import deepcopy
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from docking_mapping.geometry import ENU, apply, boat_pose, lidar_extrinsic, rigid, transform, voxel
from docking_mapping.mapper import Mapper, VoxelMap, validate_config
from docking_mapping.registration import align


def config():
    c=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/mapping.yaml').read_text())
    c['sensors'].update(extrinsics_verified=True,heading_reference='true',imu_from_base_rpy_deg=[0.,0.,0.])
    c['sensors']['gps_antenna_in_base_m']=[0.,0.,0.]
    c['preprocess'].update(min_range_m=.5,max_range_m=25.,range_mode='radial',height_filter_enabled=True,front_only=False)
    c['registration'].update(max_iterations=70)
    return c


def structure():
    rng=np.random.default_rng(7)
    surfaces=[]
    for axis,value in [(0,4.),(1,5.),(2,-1.),(0,-3.)]:
        p=rng.uniform([-3.,-3.,-1.],[4.,5.,3.],size=(400,3))
        p[:,axis]=value
        surfaces.append(p)
    return np.vstack(surfaces)


class GeometryTests(unittest.TestCase):
    def test_origin_and_wgs84_round_trip(self):
        enu=ENU([35.12,129.10,12.])
        np.testing.assert_allclose(enu.to_enu(enu.origin),0.,atol=1e-9)
        xyz=np.array([[1.,0.,0.],[0.,1.,0.],[2.,-3.,.7]])
        np.testing.assert_allclose(enu.to_enu(enu.to_lla(xyz)),xyz,atol=2e-8)
        east,north=enu.to_lla(xyz[:2])
        self.assertGreater(east[1],enu.origin[1]);self.assertGreater(north[0],enu.origin[0])

    def test_yaw_roll_pitch_and_offsets(self):
        c=config()['sensors']
        for yaw in [0.,90.,180.]:
            rotation=Rotation.from_euler('xyz',[20.,-10.,yaw],degrees=True)
            pose=boat_pose([5.,6.,7.],rotation.as_quat(),c)
            np.testing.assert_allclose(pose[:3,:3],rotation.as_matrix(),atol=1e-12)
        c['gps_antenna_in_base_m']=[1.,0.,0.]
        c['lidar_in_base_xyz_m']=[0.,2.,0.]
        r=Rotation.from_euler('z',90.,degrees=True)
        pose=boat_pose([5.,6.,0.],r.as_quat(),c)
        np.testing.assert_allclose(pose[:3,3],[5.,5.,0.],atol=1e-12)
        np.testing.assert_allclose(apply([[0.,0.,0.]],pose@lidar_extrinsic(c)),[[3.,5.,0.]],atol=1e-12)

    def test_vectornav_rfu_and_declination(self):
        c=config()['sensors']
        c.update(imu_from_base_rpy_deg=[0.,0.,90.],heading_reference='magnetic',magnetic_declination_deg=10.)
        north_rfu=[0.,0.,0.,1.]
        pose=boat_pose([0.,0.,0.],north_rfu,c)
        yaw=Rotation.from_matrix(pose[:3,:3]).as_euler('xyz',degrees=True)[2]
        self.assertAlmostEqual(yaw,80.)

    def test_unverified_calibration_and_missing_declination_rejected(self):
        c=config()['sensors'];c['extrinsics_verified']=False
        with self.assertRaises(ValueError):boat_pose([0,0,0],[0,0,0,1],c)
        c.update(extrinsics_verified=True,heading_reference='magnetic',magnetic_declination_deg=None)
        with self.assertRaises(ValueError):boat_pose([0,0,0],[0,0,0,1],c)

    def test_invalid_transform_and_geographic_values(self):
        with self.assertRaises(ValueError):ENU([100,0,0])
        matrix=np.eye(4);matrix[0,0]=2
        with self.assertRaises(ValueError):rigid(matrix)


class RegistrationTests(unittest.TestCase):
    def test_one_and_two_meter_gps_errors_reduce(self):
        target=structure()
        true=transform(Rotation.from_euler('z',4.,degrees=True).as_matrix(),[.3,-.2,.1])
        source=apply(target,np.linalg.inv(true))
        for method in ('icp','gicp'):
            for error in (1.,2.):
                with self.subTest(method=method,error=error):
                    initial=true.copy();initial[:3,3]+=[error,.25,0.]
                    cfg=config()['registration'];cfg['method']=method
                    result=align(source,target,initial,cfg)
                    distance=np.linalg.norm(result.matrix[:3,3]-true[:3,3])
                    print(f'{method}: initial_error={np.linalg.norm(initial[:3,3]-true[:3,3]):.3f}m final_error={distance:.4f}m rmse={result.rmse:.4f} accepted={result.accepted} reason={result.reason}')
                    self.assertTrue(result.accepted,result.metrics())
                    self.assertLess(distance,.15)

    def test_absolute_transform_no_double_initial(self):
        target=structure()
        true=transform(translation=[1.,.5,.2])
        source=apply(target,np.linalg.inv(true))
        result=align(source,target,true,config()['registration'])
        self.assertTrue(result.accepted,result.metrics())
        np.testing.assert_allclose(apply(source,result.matrix),target,atol=1e-7)
        correction=result.matrix@np.linalg.inv(true)
        np.testing.assert_allclose(correction@true,result.matrix,atol=1e-10)

    def test_single_wall_low_rmse_is_not_observable(self):
        rng=np.random.default_rng(2)
        wall=rng.uniform([-3.,-3.,0.],[3.,3.,0.],size=(500,3))
        result=align(wall,wall,np.eye(4),config()['registration'])
        self.assertFalse(result.accepted)
        self.assertEqual(result.reason,'DEGENERATE_GEOMETRY')

    def test_no_overlap_rejected(self):
        target=structure()
        result=align(target+100,target,np.eye(4),config()['registration'])
        self.assertFalse(result.accepted)

    def test_partial_multiview_alignment(self):
        target=structure()
        for mask in (target[:,0]>-2.8,target[:,1]>-1.,target[:,1]<4.):
            subset=target[mask]
            true=transform(Rotation.from_euler('z',3.,degrees=True).as_matrix(),[.7,-.3,.1])
            source=apply(subset,np.linalg.inv(true))
            initial=true.copy();initial[0,3]+=.35
            result=align(source,target,initial,config()['registration'])
            self.assertTrue(result.accepted,result.metrics())
            self.assertLess(np.linalg.norm(result.matrix[:3,3]-true[:3,3]),.15)


class MappingTests(unittest.TestCase):
    def seed(self):
        mapper=Mapper(config());cloud=structure();origin=[35.12,129.10,10.]
        status=mapper.process(cloud,origin,[0,0,0,1],1.,.1)
        self.assertEqual(status['state'],'INITIALIZING')
        self.assertFalse(mapper.confirmed)
        return mapper,cloud,origin

    def test_invalid_fix_never_creates_origin(self):
        mapper=Mapper(config())
        mapper.process(structure(),[35.,129.,0.],[0,0,0,1],1.,1.,fix_status=-1)
        self.assertIsNone(mapper.enu)
        self.assertEqual(len(mapper.map.cells),0)

    def test_confirmed_map_and_jump_prediction(self):
        mapper,cloud,origin=self.seed()
        status=mapper.process(cloud,origin,[0,0,0,1],1.5,.1)
        self.assertEqual(status['state'],'CONVERGED',status)
        original=mapper.enu.origin.copy()
        jumped=mapper.enu.to_lla([50.,0.,0.])
        status=mapper.process(cloud,jumped,[0,0,0,1],2.,.1)
        self.assertEqual(status['state'],'CONVERGED',status)
        self.assertTrue(status['gps_outlier'])
        self.assertLess(np.linalg.norm(mapper.corrected_pose[:3,3]),.1)
        np.testing.assert_array_equal(mapper.enu.origin,original)

    def test_rejected_scan_does_not_change_map(self):
        mapper,cloud,origin=self.seed()
        before=mapper.map.points().copy()
        mapper.process(cloud+100.,origin,[0,0,0,1],1.5,.1)
        np.testing.assert_array_equal(mapper.map.points(),before)
        self.assertEqual(len(mapper.current_scan),0)

    def test_map_voxel_thickness_and_capacity(self):
        mapper,cloud,origin=self.seed()
        for time in (1.5,2.,2.5):
            mapper.process(cloud,origin,[0,0,0,1],time,.1)
        self.assertLess(len(mapper.map.cells),len(cloud)*1.2)
        limited=VoxelMap(.1,10);limited.add(np.arange(90).reshape(-1,3))
        self.assertEqual(len(limited.cells),10)

    def test_save_load_restores_origin_map_and_rejects_missing_origin(self):
        mapper,cloud,origin=self.seed()
        mapper.process(cloud,origin,[0,0,0,1],1.5,.1)
        with tempfile.TemporaryDirectory() as directory:
            folder=mapper.save(directory)
            other=Mapper(config());other.load(directory)
            np.testing.assert_allclose(other.enu.origin,mapper.enu.origin)
            # Hash-cell iteration order is not part of the saved map geometry.
            from scipy.spatial import cKDTree
            self.assertEqual(len(other.map.cells),len(mapper.map.cells))
            self.assertLess(cKDTree(mapper.map.points()).query(other.map.points())[0].max(),1e-8)
            self.assertLess(cKDTree(other.map.points()).query(mapper.map.points())[0].max(),1e-8)
            self.assertTrue((folder/'docking_map.pcd').is_file())
            metadata=yaml.safe_load((folder/'docking_map_metadata.yaml').read_text())
            del metadata['origin_lla']
            (folder/'docking_map_metadata.yaml').write_text(yaml.safe_dump(metadata))
            before=other.map.points().copy()
            with self.assertRaises(ValueError):other.load(directory)
            np.testing.assert_array_equal(other.map.points(),before)

    def test_water_filter_requires_measured_level(self):
        c=config();c['preprocess']['water_filter_enabled']=True
        mapper=Mapper(c)
        status=mapper.process(structure(),[35,129,0],[0,0,0,1],1.,.1)
        self.assertEqual(status['state'],'WAIT_WATER_LEVEL')
        self.assertFalse(mapper.map.cells)

    def test_multiview_accumulation_stays_on_structure(self):
        from scipy.spatial import cKDTree
        mapper,cloud,origin=self.seed()
        for index,(position,yaw) in enumerate([([.5,0,0],8.),([0,.5,0],-8.),([-.5,0,0],5.)]):
            true=transform(Rotation.from_euler('z',yaw,degrees=True).as_matrix(),position)
            partial=cloud[cloud[:,index]>-2.5]
            source=apply(partial,np.linalg.inv(true))
            status=mapper.process(source,mapper.enu.to_lla(position),
                Rotation.from_matrix(true[:3,:3]).as_quat(),2.+index,.1)
            self.assertEqual(status['state'],'CONVERGED',status)
        self.assertLess(cKDTree(cloud).query(mapper.map.points())[0].max(),.15)

    def test_rc_change_before_commit_rejects_scan(self):
        mapper,cloud,origin=self.seed()
        before=mapper.map.points().copy()
        mapper.process(cloud,origin,[0,0,0,1],1.5,.1,commit_guard=lambda:False)
        self.assertEqual(mapper.status['state'],'WAIT_RC')
        np.testing.assert_array_equal(mapper.map.points(),before)


if __name__=='__main__':unittest.main()
