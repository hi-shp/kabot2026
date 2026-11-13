import json,tempfile,threading,unittest
from pathlib import Path
from types import SimpleNamespace as NS
from docking_mapping.geographic_map import GeographicMapStore
from docking_mapping.gui_commands import execute
from docking_mapping.node import DockingMapping
from docking_mapping.session import MappingSession
import test_geographic_map as fixtures

class TargetTests(unittest.TestCase):
    def test_target_roundtrip_is_bound_to_map_and_preserves_geometry(self):
        with tempfile.TemporaryDirectory() as tmp:
            store=GeographicMapStore(tmp);mapper=fixtures.GeographicMapTests().sample();folder=store.save(mapper)
            before=(folder/'docking_map.npz').read_bytes();self.assertIsNone(store.read_target(folder.name))
            goal=store.write_target(folder.name,35.12023,129.10258)
            self.assertEqual(goal,store.read_target(folder.name));self.assertEqual(before,(folder/'docking_map.npz').read_bytes())
            self.assertFalse(list(folder.glob('*.tmp')))
            second=store.save(mapper);self.assertIsNone(store.read_target(second.name))
            for lat,lon in [(True,129),(float('nan'),129),(91,129),(35,181)]:
                with self.assertRaises(ValueError):store.write_target(folder.name,lat,lon)
            with self.assertRaises(ValueError):store.write_target('../elsewhere',35,129)
            tampered=dict(goal,map_name=second.name);(folder/'gps_target.json').write_text(json.dumps(tampered))
            with self.assertRaises(ValueError):store.read_target(folder.name)
    def test_gui_click_save_reload_and_stale_map_rejection_without_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            store=GeographicMapStore(tmp);mapper=fixtures.GeographicMapTests().sample();folder=store.save(mapper)
            session=MappingSession();session.state='FINISHED'
            # No output or actuation interface exists on this adapter fixture.
            node=NS(map_lock=threading.RLock(),autonomy=NS(active=False,lock=threading.RLock(),controller=NS(state='IDLE'),world_from_dock=None),
                session=session,geographic_maps=store,gps_map_loaded=True,gps_map_name=folder.name,gps_target=None,
                mapper=mapper,config=mapper.config,reference=None,reference_folder=None,hopping=None,hopping_dirty=False)
            payload=dict(action='set_gps_target',map_name=folder.name,latitude=35.12023,longitude=129.10258)
            with self.assertRaises(ValueError):execute(node,dict(payload,map_name='stale-map'))
            execute(node,payload);self.assertEqual(node.gps_target['longitude'],129.10258)
            node.gps_target=None;DockingMapping.execute_command(node,'load_gps_map')
            self.assertEqual(node.gps_target['latitude'],35.12023)
            DockingMapping.execute_command(node,'save_gps_map')
            self.assertEqual(store.read_target(node.gps_map_name)['latitude'],35.12023)

if __name__=='__main__':unittest.main()
