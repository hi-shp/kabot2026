from pathlib import Path
from types import SimpleNamespace as NS
from copy import deepcopy
import tempfile,threading,unittest
import numpy as np
from docking_mapping.geographic_map import GeographicMapStore
from docking_mapping.geometry import ENU
from docking_mapping.mapper import Mapper
from docking_mapping.node import DockingMapping
from docking_mapping.preview import mapping_preview
from docking_mapping.session import MappingSession
from test_mapping import config

class GeographicMapTests(unittest.TestCase):
    def sample(self):
        mapper=Mapper(config());mapper.enu=ENU([35.12,129.1,3.])
        mapper.map.add(np.array([[1.,2.,.3],[3.,4.,1.2],[5.,-2.,-3.]]))
        mapper.confirmed=True;mapper.display_anchor={'origin_xyz':[.4,0,0],'yaw_deg':32.}
        mapper.path.append((100.,1.,2.,.3));mapper.status={'state':'CONVERGED'}
        return mapper
    def test_store_preserves_wgs84_xyz_path_and_relative_display_anchor(self):
        with tempfile.TemporaryDirectory() as directory:
            store=GeographicMapStore(directory);self.assertFalse(store.available())
            mapper=self.sample();ll=mapper.enu.to_lla(mapper.map.points())
            folder=store.save(mapper);self.assertTrue(store.available())
            loaded,loaded_folder=store.load(config())
            self.assertEqual(folder,loaded_folder)
            np.testing.assert_allclose(loaded.enu.to_lla(loaded.map.points()),ll,atol=1e-10)
            self.assertEqual(loaded.display_anchor,mapper.display_anchor)
            np.testing.assert_allclose(list(loaded.path),list(mapper.path))
            p=mapping_preview(loaded,None,None,config(),{'state':'LOADED'})
            self.assertEqual(len(p['map_points']),3);self.assertEqual(len(p['map_points_xyz']),3)
    def test_gui_save_then_load_controls_overlay_visibility_without_roi(self):
        with tempfile.TemporaryDirectory() as directory:
            mapper=self.sample();session=MappingSession();session.state='FINISHED'
            node=NS(map_lock=threading.RLock(),autonomy=NS(active=False,lock=threading.RLock(),controller=NS(state='IDLE'),world_from_dock=None),
                mapper=mapper,config=config(),session=session,geographic_maps=GeographicMapStore(directory),
                gps_map_loaded=False,gps_map_name=None,mode='STOPPED',reference=None,reference_folder=None,hopping=None,hopping_dirty=False,last_output_stamp=None)
            DockingMapping.execute_command(node,'save_gps_map')
            self.assertFalse(node.gps_map_loaded,'save alone must not overlay the 2D map')
            node.mapper=Mapper(config())
            DockingMapping.execute_command(node,'load_gps_map')
            self.assertTrue(node.gps_map_loaded);self.assertIsNone(node.reference)
            self.assertEqual(node.state,'GPS_MAP_LOADED');self.assertEqual(len(node.mapper.map.cells),3)
            self.assertFalse(node.session.active)
    def test_active_mapping_and_empty_save_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            store=GeographicMapStore(directory)
            with self.assertRaises(ValueError):store.save(Mapper(config()))
            self.assertFalse(store.available())
            with self.assertRaises(ValueError):store.load(config())

if __name__=='__main__':unittest.main()
