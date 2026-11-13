import math
from types import SimpleNamespace as N
import unittest
from unittest.mock import patch

from jetson_server import Store
from telemetry_core import imu_value, vectornav_value, vectornav_heading


class SensorSources(unittest.TestCase):
    def common(self, yaw=42., pitch=2., roll=-3.):
        return N(header=N(frame_id='vectornav'),group_fields=0x0008,
                 yawpitchroll=N(x=yaw,y=pitch,z=roll))

    def test_raw_attitude_fallback_has_explicit_angles(self):
        value=vectornav_value(self.common())
        self.assertTrue(value['orientation_valid'])
        self.assertEqual(value['orientation']['deg_signed'],dict(yaw=42.,pitch=2.,roll=-3.))

    def test_invalid_sensor_attitude_is_never_replaced_with_zero(self):
        value=vectornav_value(self.common(math.nan,90.,math.nan))
        self.assertFalse(value['orientation_valid'])
        self.assertNotIn('orientation',value)
        self.assertIsNone(vectornav_heading(value,{})['heading_deg'])
        imu=imu_value(N(header=N(frame_id='vectornav'),orientation=N(x=math.nan,y=math.nan,z=math.nan,w=math.nan),
                        orientation_covariance=[.01]*9,angular_velocity=N(x=0.,y=0.,z=0.),
                        linear_acceleration=N(x=0.,y=0.,z=9.8)))
        self.assertFalse(imu['orientation_valid'])

    def test_heading_matches_navigation_offsets_and_wrap(self):
        value=vectornav_heading(dict(vectornav_value(self.common(-5.)),topic='/vectornav/raw/common'),
                                dict(magnetic_declination_deg=-8.45,heading_offset_deg=2.))
        self.assertAlmostEqual(value['heading_deg'],348.55)
        self.assertEqual(value['reference'],'true_north')
        self.assertEqual(vectornav_heading(vectornav_value(self.common()),{})['reference'],'magnetic_north')

    def test_raw_mode_exposes_sensor_yaw_without_declination(self):
        raw=vectornav_heading(vectornav_value(self.common(-25.)),dict(
            magnetic_declination_deg=0.,heading_offset_deg=0.,imu_heading_mode='absolute_magnetic_raw'))
        self.assertEqual(raw['heading_deg'],335.)
        self.assertEqual(raw['reference'],'magnetic_north')
        store=Store(N(max_points=5000,cloud_hz=5,max_range=100))
        with patch('jetson_server.time.monotonic',return_value=100.):
            store.value('heading',{'heading_deg':326.55,'reference':'true_north'})
            store.value('heading_raw',raw)
            self.assertEqual(store.snapshot()['values']['heading']['heading_deg'],335.)

    def test_navigation_heading_priority_then_fresh_raw_then_stale(self):
        store=Store(N(max_points=5000,cloud_hz=5,max_range=100))
        with patch('jetson_server.time.monotonic',return_value=100.):
            store.value('heading',{'heading_deg':10.})
            store.value('heading_raw',{'heading_deg':20.})
            self.assertEqual(store.snapshot()['values']['heading']['heading_deg'],10.)
        with patch('jetson_server.time.monotonic',return_value=102.):
            store.value('heading_raw',{'heading_deg':21.})
            self.assertEqual(store.snapshot()['values']['heading']['heading_deg'],21.)
        with patch('jetson_server.time.monotonic',return_value=105.):
            self.assertGreater(store.snapshot()['values']['heading']['age_s'],2.)

    def test_invalid_navigation_heading_uses_valid_raw_only(self):
        store=Store(N(max_points=5000,cloud_hz=5,max_range=100))
        with patch('jetson_server.time.monotonic',return_value=100.):
            store.value('heading',{'heading_deg':None})
            store.value('heading_raw',{'heading_deg':25.})
            self.assertEqual(store.snapshot()['values']['heading']['heading_deg'],25.)
            store.value('heading_raw',{'heading_deg':None})
            self.assertIsNone(store.snapshot()['values']['heading']['heading_deg'])

    def test_topic_changes_follow_fresh_data_without_overwriting_live_primary(self):
        store=Store(N())
        for key,preferred in [('imu',('/vectornav/imu','/imu')),('gps',('/gps/fix','/fix')),('scan',('/scan',))]:
            with patch('jetson_server.time.monotonic',return_value=100.):
                self.assertTrue(store.accepts_sensor(key,'/renamed/'+key,preferred))
                store.value(key,{'topic':'/renamed/'+key})
                self.assertTrue(store.accepts_sensor(key,'/renamed/'+key,preferred))
                self.assertTrue(store.accepts_sensor(key,preferred[0],preferred))
                store.value(key,{'topic':preferred[0]})
                self.assertFalse(store.accepts_sensor(key,'/other/'+key,preferred))
            with patch('jetson_server.time.monotonic',return_value=103.):
                self.assertTrue(store.accepts_sensor(key,'/other/'+key,preferred))


if __name__=='__main__':unittest.main()
