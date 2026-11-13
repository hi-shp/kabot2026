import contextlib
import io
import json
from pathlib import Path
import struct
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from sensor_msgs.msg import Image,NavSatFix,Imu,PointCloud2,PointField,LaserScan
from vectornav_msgs.msg import CommonGroup
from std_msgs.msg import String
from camera_test import inspect_image
from gps_test import inspect_fix
from lidar_test import inspect_cloud,inspect_scan
from imu_test import inspect_imu
from _common import actuator_details
import thruster_test

class SensorValidationTests(unittest.TestCase):
    def test_camera_complete_and_truncated_payload(self):
        m=Image(width=2,height=2,step=6,encoding='bgr8',data=bytes(12))
        self.assertTrue(inspect_image(m)['valid'])
        m.data=bytes(11);self.assertFalse(inspect_image(m)['valid'])
    def test_no_fix_is_rejected_even_with_numeric_coordinates(self):
        m=NavSatFix(latitude=35.,longitude=129.,altitude=10.)
        m.status.status=-1;self.assertFalse(inspect_fix(m)['valid'])
    def test_horizontal_fix_accepts_unknown_altitude(self):
        m=NavSatFix(latitude=35.,longitude=129.,altitude=float('nan'))
        m.status.status=0;self.assertTrue(inspect_fix(m)['valid'])
        self.assertFalse(inspect_fix(m)['altitude_valid'])
    def test_invalid_latitude_is_rejected(self):
        m=NavSatFix(latitude=100.,longitude=129.);m.status.status=0
        self.assertFalse(inspect_fix(m)['valid'])
    def cloud(self,big=False):
        fields=[PointField(name=k,offset=i*4,datatype=7,count=1) for i,k in enumerate(('x','y','z'))]
        return PointCloud2(width=2,height=1,fields=fields,point_step=12,row_step=24,is_bigendian=big,data=struct.pack(('>' if big else '<')+'ffffff',1.,2.,3.,4.,5.,6.))
    def test_cloud_endianness(self):
        for endian in (False,True):
            result=inspect_cloud(self.cloud(endian));self.assertTrue(result['valid'])
            self.assertEqual(result['xyz_example'],[1.,2.,3.])
    def test_truncated_cloud_rejected(self):
        m=self.cloud();m.data=bytes(10);self.assertFalse(inspect_cloud(m)['valid'])
    def test_nonfinite_xyz_rejected(self):
        m=self.cloud();m.data=struct.pack('<ffffff',*[float('nan')]*6)
        self.assertFalse(inspect_cloud(m)['valid'])
    def test_scan_uses_only_finite_in_range_returns(self):
        m=LaserScan(range_min=.1,range_max=10.,angle_increment=.01,ranges=[float('inf'),float('nan'),.01,1.,20.])
        result=inspect_scan(m);self.assertTrue(result['valid']);self.assertEqual(result['valid_ranges'],1)
        m.ranges=[float('inf')];self.assertFalse(inspect_scan(m)['valid'])
    def test_stationary_imu_can_be_valid_but_inactive_fields_cannot(self):
        m=Imu();m.orientation.w=1.;m.linear_acceleration.z=9.81
        raw=CommonGroup(group_fields=0x28)
        self.assertTrue(inspect_imu(m,raw)['valid'])
        raw.group_fields=0;self.assertFalse(inspect_imu(m,raw)['valid'])
    def test_received_imu_numpy_covariance_is_json_serializable(self):
        import numpy as np
        m=Imu();m.orientation.w=1.;m.orientation_covariance=np.zeros(9,dtype=np.float64)
        json.dumps(inspect_imu(m,CommonGroup(group_fields=0x28)),allow_nan=False)
    def test_zero_quaternion_rejected(self):
        m=Imu();m.orientation.w=0.;self.assertFalse(inspect_imu(m,CommonGroup(group_fields=0x28))['valid'])
    def test_actuator_bridge_message_is_not_mcu_receipt(self):
        state=dict(receiving=False,left=1500,right=1500,error=None)
        self.assertFalse(actuator_details({'t':String(data=json.dumps(state))},'t')['valid'])
        state['receiving']=True
        self.assertTrue(actuator_details({'t':String(data=json.dumps(state))},'t')['valid'])
    def test_actuator_wrong_json_type_rejected(self):
        self.assertFalse(actuator_details({'t':String(data='[]')},'t')['valid'])

class ThrusterDispatchTests(unittest.TestCase):
    def test_output_requires_mcu_reported_pulse_widths(self):
        state=dict(receiving=True,left=1500,right=1500,error=None)
        self.assertFalse(thruster_test.inspect_thruster({'t':String(data=json.dumps(state))},'t')['valid'])
        state.update(observed_left_us=1480,observed_right_us=1480)
        self.assertTrue(thruster_test.inspect_thruster({'t':String(data=json.dumps(state))},'t')['valid'])
    def test_default_only_runs_passive_monitor(self):
        with patch.object(thruster_test,'run',return_value=0) as monitor:
            self.assertEqual(thruster_test.main([]),0)
            monitor.assert_called_once()
    def test_file_variables_are_independent(self):
        with patch.multiple(thruster_test,LEFT_PWM=1600,RIGHT_PWM=1400):
            self.assertEqual(thruster_test.resolve_pwm(),(1600,1400))
    def test_invalid_file_value_rejected_before_sender(self):
        with patch.multiple(thruster_test,LEFT_PWM=1000,RIGHT_PWM=1500):
            with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit) as error:
                thruster_test.main(['--execute'])
            self.assertEqual(error.exception.code,2)
    def test_timed_option_not_silently_ignored(self):
        with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
            thruster_test.main(['--execute','--duration','1'])
    def test_execute_uses_file_variables_with_mock_sender(self):
        with patch.multiple(thruster_test,LEFT_PWM=1600,RIGHT_PWM=1450):
            with patch('thruster_output.run_pair') as sender:
                self.assertEqual(thruster_test.main(['--execute']),0)
                sender.assert_called_once_with(1600,1450)
    def test_legacy_cli_override_remains_explicit(self):
        with patch('thruster_output.run_pair') as sender:
            self.assertEqual(thruster_test.main(['--execute','--side','right','--percent','5']),0)
            sender.assert_called_once_with(1500,1520)

if __name__=='__main__': unittest.main()
