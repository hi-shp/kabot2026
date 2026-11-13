from copy import deepcopy
from pathlib import Path
import tempfile
import unittest

import numpy as np
import yaml

from docking_mapping.geometry import ENU,apply
from docking_mapping.mapper import Mapper
from docking_mapping.reference import ReferenceMap
from docking_mapping.preprocessing import clustered_points,scan_motion_reason
from docking_mapping.session import MappingSession
from test_mapping import config,structure


class SessionTests(unittest.TestCase):
    def test_rc_alone_never_starts_mapping(self):
        session=MappingSession()
        self.assertFalse(session.active)
        session.lose_rc()
        self.assertEqual(session.state,'IDLE')

    def test_start_and_resume_require_rc(self):
        session=MappingSession()
        with self.assertRaises(ValueError):session.start(False)
        session.start(True);session.pause()
        with self.assertRaises(ValueError):session.resume(False)
        self.assertFalse(session.active)

    def test_rc_loss_latches_until_operator_resume(self):
        session=MappingSession();session.start(True);session.lose_rc()
        self.assertEqual(session.state,'PAUSED_RC_LOST')
        self.assertFalse(session.active)
        with self.assertRaises(ValueError):session.start(True)
        session.resume(True);self.assertTrue(session.active)

    def test_finish_and_frozen_reference_cannot_accumulate(self):
        session=MappingSession()
        with self.assertRaises(ValueError):session.can_freeze()
        session.start(True)
        with self.assertRaises(ValueError):session.can_freeze()
        session.finish();session.can_freeze();session.reference_ready()
        self.assertEqual(session.phase,'REFERENCE_READY')
        with self.assertRaises(ValueError):session.resume(True)
        with self.assertRaises(ValueError):session.start(True)
        self.assertFalse(session.active)
        session.reset();session.start(True);self.assertTrue(session.active)


class ReferenceTests(unittest.TestCase):
    def fixture(self):
        mapper=Mapper(config())
        mapper.enu=ENU([35.12,129.10,10.])
        mapper.map.add(structure())
        mapper.confirmed=True
        mapper.status=dict(state='CONVERGED',rmse=.01)
        selection=deepcopy(mapper.config['reference'])
        selection.update(roi_min_enu_m=[-3.1,-3.1,-1.1],roi_max_enu_m=[4.1,5.1,3.1],yaw_enu_deg=27.)
        return mapper,selection

    def test_selected_structure_roundtrip_preserves_height(self):
        mapper,selection=self.fixture()
        before=mapper.map.points().copy()
        ref=ReferenceMap.from_mapper(mapper,selection)
        np.testing.assert_allclose(ref.points_enu(),before,atol=1e-12)
        self.assertGreater(np.ptp(ref.points[:,2]),3.)
        np.testing.assert_array_equal(mapper.map.points(),before)
        np.testing.assert_allclose(apply(ref.points_enu(),np.linalg.inv(ref.enu_from_dock)),ref.points,atol=1e-12)

    def test_explicit_roi_is_required(self):
        mapper,selection=self.fixture();selection['roi_min_enu_m']=None
        with self.assertRaises(ValueError):ReferenceMap.from_mapper(mapper,selection)

    def test_roi_excludes_other_scene_points(self):
        mapper,selection=self.fixture()
        selection['roi_min_enu_m']=[3.9,-3.1,-1.1]
        selection['roi_max_enu_m']=[4.1,5.1,3.1]
        ref=ReferenceMap.from_mapper(mapper,selection)
        self.assertTrue(np.all(ref.points_enu()[:,0]>3.9))
        self.assertLess(len(ref.points),len(mapper.map.cells))

    def test_provisional_and_empty_selection_rejected(self):
        mapper,selection=self.fixture();mapper.confirmed=False
        with self.assertRaises(ValueError):ReferenceMap.from_mapper(mapper,selection)
        mapper.confirmed=True;selection.update(roi_min_enu_m=[50,50,50],roi_max_enu_m=[51,51,51])
        with self.assertRaises(ValueError):ReferenceMap.from_mapper(mapper,selection)

    def test_reference_arrays_are_readonly(self):
        mapper,selection=self.fixture();ref=ReferenceMap.from_mapper(mapper,selection)
        for array in (ref.points,ref.origin_lla,ref.enu_from_dock,ref.path_enu):
            self.assertFalse(array.flags.writeable)
        with self.assertRaises(ValueError):ref.points[0,0]+=2

    def test_saved_reference_id_version_transform_and_sensor_metadata(self):
        mapper,selection=self.fixture();ref=ReferenceMap.from_mapper(mapper,selection)
        with tempfile.TemporaryDirectory() as directory:
            folder=ref.save(directory)
            loaded,_=ReferenceMap.load_latest(directory,mapper.config['sensors'],150000)
            self.assertEqual((loaded.reference_id,loaded.version),(ref.reference_id,1))
            np.testing.assert_array_equal(loaded.points,ref.points)
            np.testing.assert_array_equal(loaded.enu_from_dock,ref.enu_from_dock)
            np.testing.assert_array_equal(loaded.origin_lla,ref.origin_lla)
            self.assertEqual(loaded.sensor_config,mapper.config['sensors'])
            self.assertTrue((folder/'docking_reference.pcd').is_file())
            again=ref.save(directory)
            self.assertNotEqual(again,folder)
            self.assertEqual(len(list(Path(directory).glob('reference_*'))),2)

    def test_missing_origin_and_wrong_sensor_calibration_rejected(self):
        mapper,selection=self.fixture();ref=ReferenceMap.from_mapper(mapper,selection)
        with tempfile.TemporaryDirectory() as directory:
            folder=ref.save(directory)
            wrong=deepcopy(mapper.config['sensors']);wrong['gps_antenna_in_base_m']=[1.,0.,0.]
            with self.assertRaises(ValueError):ReferenceMap.load_latest(directory,wrong,150000)
            filename=folder/'docking_reference_metadata.yaml'
            metadata=yaml.safe_load(filename.read_text());del metadata['origin_lla']
            filename.write_text(yaml.safe_dump(metadata))
            with self.assertRaises(ValueError):ReferenceMap.load_latest(directory,mapper.config['sensors'],150000)

    def test_geometry_checksum_and_transform_validation(self):
        mapper,selection=self.fixture();ref=ReferenceMap.from_mapper(mapper,selection)
        with tempfile.TemporaryDirectory() as directory:
            folder=ref.save(directory)
            with np.load(folder/'docking_reference.npz',allow_pickle=False) as data:
                points=data['points_dock'].copy();path=data['path_enu'].copy()
            points[0,0]+=1.
            np.savez_compressed(folder/'docking_reference.npz',points_dock=points,path_enu=path)
            with self.assertRaises(ValueError):ReferenceMap.load_latest(directory,mapper.config['sensors'],150000)
        matrix=ref.enu_from_dock.copy();matrix[0,0]=3.
        with self.assertRaises(ValueError):ReferenceMap(ref.points,ref.origin_lla,matrix,ref.sensor_config,.12,{})


class PreprocessingTests(unittest.TestCase):
    def test_clusters_keep_structure_and_remove_isolated_returns(self):
        rng=np.random.default_rng(21)
        structure_points=rng.normal(0,.05,(100,3))
        isolated=np.array([[10,10,10],[20,20,20],[30,30,30]],dtype=float)
        result=clustered_points(np.vstack([structure_points,isolated]),.3,4,20)
        self.assertEqual(len(result),100)
        self.assertLess(np.linalg.norm(result,axis=1).max(),1.)

    def test_point_time_motion_gate(self):
        motion=config()['scan_motion']
        self.assertEqual(scan_motion_reason(.1,.01,2.,motion),'')
        self.assertEqual(scan_motion_reason(None,.01,2.,motion),'WAIT_SCAN_TIMING')
        self.assertEqual(scan_motion_reason(.1,1.,2.,motion),'EXCESSIVE_SCAN_ROTATION')
        self.assertEqual(scan_motion_reason(.2,.01,2.,motion),'EXCESSIVE_SCAN_TRANSLATION')


if __name__=='__main__':unittest.main()
