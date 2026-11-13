"""Object selection, radius hold and fresh-input behavior without hardware."""
import math
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from kaboat_gps_hopping.gps_hopping import Fix, GPSHoppingConfig, GPSHoppingMission
from kaboat_object_gps.gps_mission import ObjectGPSMission, load_object_goals


class ObjectGPSTests(unittest.TestCase):
    def make(self):
        cfg = SimpleNamespace(max_percent=16., target_label='red triangle')
        goals = {'red triangle': dict(latitude=35., longitude=129., radius_m=3.),
                 'green circle': dict(latitude=35., longitude=129.0001, radius_m=1.)}
        mission = ObjectGPSMission(cfg, lambda *a, percent: (percent, percent), GPSHoppingConfig(), goals)
        mission.heading, mission.heading_stamp = 0., 10.
        return mission

    def test_selected_object_sets_coordinates_and_radius(self):
        m = self.make()
        self.assertTrue(m.select_object('green_circle'))
        self.assertEqual((m.gps_config.goal_longitude, m.gps_config.gps_arrival_radius), (129.0001, 1.))

    def test_unknown_object_clears_previous_target(self):
        m = self.make()
        self.assertFalse(m.select_object('missing'))
        self.assertEqual(m.step(10.), ('WAIT_GPS_GOAL', (0., 0.)))

    def test_moves_without_camera_scan_or_slam(self):
        m = self.make()
        m.fix = Fix(35., 128.9999, .1, 10.)
        state, output = m.step(10.)
        self.assertEqual(state, 'GPS_TRANSIT')
        self.assertEqual(output, (12., 12.))

    def test_radius_hold_then_drift_reapproach(self):
        m = self.make()
        m.fix = Fix(35., 129., .1, 10.)
        self.assertEqual(m.step(10.), ('GPS_ARRIVAL_HOLD', (0., 0.)))
        m.fix = Fix(35., 129., .1, 12.)
        self.assertEqual(m.step(12.), ('GPS_HOLD', (0., 0.)))
        m.fix = Fix(35., 128.9999, .1, 13.)
        m.heading_stamp = 13.
        self.assertEqual(m.step(13.)[0], 'GPS_TRANSIT')

    def test_selected_target_change_retains_sensor_state(self):
        m = self.make()
        m.fix = Fix(35., 129., .1, 10.)
        m.step(10.)
        m.select_object('green circle')
        self.assertEqual(m.step(10.)[0], 'GPS_TRANSIT')

    def test_stale_fix_or_heading_stops(self):
        m = self.make()
        m.fix = Fix(35., 128.9999, .1, 10.)
        self.assertEqual(m.step(13.)[0], 'WAIT_GPS')
        m.fix = Fix(35., 128.9999, .1, 13.)
        self.assertEqual(m.step(13.)[0], 'WAIT_HEADING')

    def test_new_target_uses_same_hopping_engine(self):
        m = self.make()
        self.assertIsInstance(m, GPSHoppingMission)
        m.fix = Fix(35., 128.9999, .1, 10.)
        m.heading = math.pi
        state, output = m.step(10.)
        self.assertEqual(state, 'ALIGN_GPS')
        self.assertEqual(output[0], -output[1])

    def test_catalog_validation(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/'objects.yaml'
            path.write_text('objects: {marker: {latitude: 35, longitude: 129, radius_m: 2}}')
            self.assertEqual(load_object_goals(path)['marker']['radius_m'], 2.)
            for data in ('objects: {marker: {latitude: 91, longitude: 129, radius_m: 2}}',
                         'objects: {marker: {latitude: 35, longitude: 129, radius_m: 0}}',
                         'objects: {marker: {latitude: .nan, longitude: 129, radius_m: 2}}',
                         'objects: [invalid', 'objects: []'):
                path.write_text(data)
                with self.assertRaises(ValueError): load_object_goals(path)

if __name__ == '__main__': unittest.main()
