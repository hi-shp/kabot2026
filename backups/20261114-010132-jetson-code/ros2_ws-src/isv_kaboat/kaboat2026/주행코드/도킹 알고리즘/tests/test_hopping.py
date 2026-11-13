from copy import deepcopy
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch
from types import SimpleNamespace
import json
import numpy as np
import yaml
from docking_mapping.geometry import ENU,apply
from docking_mapping.hopping import HoppingPoint,click_on_plane,write_hopping,read_hopping,roi_from_clicks
from docking_mapping.reference import ReferenceMap
from docking_mapping.gui_commands import execute
from docking_mapping.preview import mapping_preview
from docking_mapping.gui_bridge import MappingBridge
from docking_mapping.session import MappingSession
import test_reference


class HoppingTests(unittest.TestCase):
    def fixture(self):
        mapper,selection=test_reference.ReferenceTests().fixture()
        ref=ReferenceMap.from_mapper(mapper,selection)
        point_enu=ref.enu_from_dock[:3,3]+[1.,2.,0.]
        ll=mapper.enu.to_lla(point_enu)
        hop=HoppingPoint.from_click(ref,*ll[:2],float(point_enu[2]),
            dict(name='hop1',radius_m=1.2,hold_seconds=7.,heading_dock_deg=30.),100.)
        return mapper,ref,hop,point_enu

    def test_click_plane_and_dock_inverse_roundtrip(self):
        mapper,ref,hop,point=self.fixture()
        np.testing.assert_allclose(apply([hop.point_dock],ref.enu_from_dock)[0],point,atol=2e-7)
        view=hop.preview(ref)
        np.testing.assert_allclose(view['latlon'],mapper.enu.to_lla(point)[:2],atol=1e-10)
        self.assertAlmostEqual(view['height_enu_m'],point[2],places=7)

    def test_click_requires_explicit_height_and_nearby_reference(self):
        mapper,ref,hop,_=self.fixture()
        with self.assertRaises(ValueError):click_on_plane(mapper.enu,35.12,129.1,None)
        ll=mapper.enu.to_lla([1000.,1000.,0.])
        with self.assertRaises(ValueError):HoppingPoint.from_click(ref,*ll[:2],0.,dict(radius_m=1.,hold_seconds=10.),100.)

    def test_settings_and_map_version_validation(self):
        _,ref,hop,_=self.fixture()
        for changed in [dict(radius_m=0.),dict(hold_seconds=0.),dict(name=''),dict(heading_dock_deg=float('nan'))]:
            fields=deepcopy(hop.__dict__);fields.update(changed)
            with self.assertRaises(ValueError):HoppingPoint(**fields)
        wrong=deepcopy(ref);wrong.version=2
        with self.assertRaises(ValueError):hop.preview(wrong)

    def test_point_moves_with_reference_transform_without_changing_dock_coordinates(self):
        _,ref,hop,_=self.fixture();before=np.array(hop.point_dock)
        original=hop.preview(ref)['point_enu']
        shifted=deepcopy(ref);matrix=ref.enu_from_dock.copy();matrix[:3,3]+=[2.,0.,0.];shifted.enu_from_dock=matrix
        np.testing.assert_allclose(np.array(hop.preview(shifted)['point_enu'])-original,[2.,0.,0.],atol=1e-9)
        np.testing.assert_array_equal(hop.point_dock,before)

    def test_save_load_deleted_marker_and_previous_backup(self):
        _,ref,hop,_=self.fixture()
        with tempfile.TemporaryDirectory() as directory:
            write_hopping(directory,ref,hop)
            self.assertEqual(read_hopping(directory,ref),hop)
            write_hopping(directory,ref,None)
            self.assertIsNone(read_hopping(directory,ref))
            self.assertTrue((Path(directory)/'hopping_point.previous.yaml').is_file())

    def test_map_bundle_and_mismatched_link_rejected(self):
        mapper,ref,hop,_=self.fixture()
        with tempfile.TemporaryDirectory() as directory:
            folder=ref.save(directory,lambda p:write_hopping(p,ref,hop))
            loaded,_=ReferenceMap.load_latest(directory,mapper.config['sensors'],150000)
            self.assertEqual(read_hopping(folder,loaded),hop)
            file=folder/'hopping_point.yaml';data=yaml.safe_load(file.read_text());data['reference_version']=2
            file.write_text(yaml.safe_dump(data))
            with self.assertRaises(ValueError):read_hopping(folder,loaded)

    def test_preview_downsampling_bound_and_reference_immutability(self):
        mapper,ref,hop,_=self.fixture();mapper.config['mapping']['gui_max_points']=7
        before=ref.points.copy()
        preview=mapping_preview(mapper,ref,hop,mapper.config,dict(state='REFERENCE_LOADED'))
        self.assertLessEqual(len(preview['map_points']),7)
        self.assertEqual(preview['hopping']['reference_id'],ref.reference_id)
        np.testing.assert_array_equal(before,ref.points)

    def test_roi_from_map_corners(self):
        mapper,_,_,_=self.fixture()
        ll=mapper.enu.to_lla([[-2.,-3.,0.],[4.,5.,0.]])[:,:2].tolist()
        low,high=roi_from_clicks(mapper.enu,ll,-1.,3.)
        np.testing.assert_allclose(low,[-2.,-3.,-1.],atol=2e-5)
        np.testing.assert_allclose(high,[4.,5.,3.],atol=2e-5)

    def test_gui_commands_map_binding_rc_and_dirty_state(self):
        mapper,ref,hop,_=self.fixture();session=MappingSession();session.reference_ready()
        node=SimpleNamespace(map_lock=threading.Lock(),session=session,reference=ref,mapper=mapper,
            hopping=hop,hopping_dirty=False,rc_ready=lambda:True,reference_folder=None,config=mapper.config)
        binding=dict(reference_id=ref.reference_id,reference_version=1)
        execute(node,dict(action='delete_hopping',**binding));self.assertIsNone(node.hopping);self.assertTrue(node.hopping_dirty)
        with self.assertRaises(ValueError):execute(node,dict(action='delete_hopping',reference_id='wrong',reference_version=1))
        node.rc_ready=lambda:False
        with self.assertRaises(ValueError):execute(node,dict(action='set_hopping',**binding))


class BridgeTests(unittest.TestCase):
    def value(self):return dict(preview_schema=1,status=dict(state='PAUSED'),map_points=[[35.,129.]],current_scan=[],rc_path=[])

    def test_bridge_bounded_input_and_expiration(self):
        bridge=MappingBridge(controls_enabled=True,transport=lambda p:dict(success=True,message='queued'))
        bridge.update(json.dumps(self.value()));self.assertTrue(bridge.snapshot()['controls_enabled'])
        too_big=self.value();too_big['map_points']*=6001
        bridge.update(json.dumps(too_big));self.assertEqual(len(bridge.snapshot()['map_points']),1)
        with patch('docking_mapping.gui_bridge.time.monotonic',return_value=bridge.received+4):
            self.assertFalse(bridge.snapshot()['controls_enabled'])

    def test_mapping_only_actions_and_separate_permission(self):
        calls=[]
        bridge=MappingBridge(controls_enabled=True,transport=lambda p:(calls.append(p) or dict(success=True,message='queued')))
        bridge.update(json.dumps(self.value()))
        result=bridge.send(dict(action='load'));self.assertTrue(result['success']);self.assertIn('request_id',calls[0])
        self.assertTrue(bridge.send(dict(action='auto_start'))['success'])
        for action in ('arm','pwm','reverse'):
            with self.assertRaises(ValueError):bridge.send(dict(action=action))
        bridge.controls_enabled=False
        with self.assertRaises(PermissionError):bridge.send(dict(action='load'))


if __name__=='__main__':unittest.main()
