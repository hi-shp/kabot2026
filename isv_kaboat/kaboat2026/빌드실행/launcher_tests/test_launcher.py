import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import yaml

spec=importlib.util.spec_from_file_location('launcher_check',Path(__file__).resolve().parents[1]/'launcher_check.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class PreflightTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parent);self.root=Path(self.tmp.name)
        (self.root/'model.pt').write_bytes(b'test');(self.root/'params.yaml').write_text('test')
        self.calib=self.root/'measured.yaml'
        self.data={'camera':{'width':640,'height':480,'K':[500.,0.,320.,0.,500.,240.,0.,0.,1.],'D':[0.,0.,0.,0.,0.]},
                   'extrinsics':{'T':[1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.]}}
        self.save();self.options=dict(params=str(self.root/'params.yaml'),model=str(self.root/'model.pt'),
                calibration=str(self.calib),camera='/missing/camera',gps='/missing/gps',imu='/missing/imu',arduino='/missing/arduino')
    def tearDown(self):self.tmp.cleanup()
    def save(self):self.calib.write_text(yaml.safe_dump(self.data))
    def test_structural_real_size_is_accepted_without_starting_any_node(self):
        with patch.object(module,'calibration_path',side_effect=lambda p:Path(p)):
            self.assertEqual(module.validate(self.options,'autonomy',devices=False),self.calib)
    def test_missing_devices_cannot_enable_autonomy(self):
        with patch.object(module,'calibration_path',side_effect=lambda p:Path(p)):
            with self.assertRaisesRegex(ValueError,'Devices not connected'):module.validate(self.options,'autonomy')
    def test_example_file_is_preview_only(self):
        example=self.root/'calib_example.yaml';example.write_text(self.calib.read_text());self.options['calibration']=str(example)
        with patch.object(module,'calibration_path',side_effect=lambda p:Path(p)):
            module.validate(self.options,'preview',devices=False)
            module.validate(self.options,'autonomy',devices=False)
            self.assertFalse(module.measured_calibration(self.options))
    def test_wrong_image_size_blocks_docking_but_not_gps_leg(self):
        self.data['camera']['height']=360;self.save()
        with patch.object(module,'calibration_path',side_effect=lambda p:Path(p)):
            module.validate(self.options,'autonomy',devices=False)
            self.assertFalse(module.measured_calibration(self.options))
    def test_nan_calibration_is_rejected(self):
        self.data['camera']['K'][0]=float('nan');self.save()
        with self.assertRaisesRegex(ValueError,'finite'):module.validate(self.options,'preview',devices=False)
    def test_malformed_transform_is_rejected(self):
        self.data['extrinsics']['T']=[];self.save()
        with self.assertRaisesRegex(ValueError,'extrinsics'):module.validate(self.options,'preview',devices=False)
    def test_missing_model_is_rejected_before_launch(self):
        self.options['model']=str(self.root/'missing.pt')
        with self.assertRaisesRegex(ValueError,'model'):module.validate(self.options,'preview',devices=False)


class AutoStartTests(unittest.TestCase):
    def setUp(self):
        self.report={'stage':'TO_WP1','state':'TO_WP1','output':{'enabled':True,'active':False,'error':'','start_ready':True}}
        self.mcu={'receiving':True,'mode':2,'estop_active':False,'stop_latched':False}
        self.ages={key:.1 for key in ('GUI','MCU','GPS','IMU','LIDAR')}
    def ready(self):return module.may_auto_start(self.report,self.mcu,self.ages)
    def test_fresh_auto_data_can_start(self):self.assertTrue(self.ready())
    def test_mcu_initialization_blocks_auto_start(self):self.report['output']['start_ready']=False;self.assertFalse(self.ready())
    def test_old_mission_without_readiness_cannot_auto_start(self):del self.report['output']['start_ready'];self.assertFalse(self.ready())
    def test_rc_mode_cannot_start(self):self.mcu['mode']=1;self.assertFalse(self.ready())
    def test_estop_cannot_start(self):self.mcu['estop_active']=True;self.assertFalse(self.ready())
    def test_stale_lidar_cannot_start(self):self.ages['LIDAR']=1.;self.assertFalse(self.ready())
    def test_missing_gps_cannot_start(self):del self.ages['GPS'];self.assertFalse(self.ready())
    def test_preview_cannot_start(self):self.report['output']['enabled']=False;self.assertFalse(self.ready())
    def test_running_does_not_start_again(self):self.report['output']['active']=True;self.assertFalse(self.ready())
    def test_fault_does_not_restart(self):self.report['output']['error']='stopped';self.assertFalse(self.ready())
    def test_later_mission_stage_does_not_restart(self):self.report['stage']='DOCK_TO_WP3';self.assertFalse(self.ready())


class SplitStartupTests(unittest.TestCase):
    def test_mission_reuses_existing_equipment_nodes(self):
        self.assertEqual(module.component_conflicts(module.EQUIPMENT_NODES,'mission'),set())
    def test_second_mission_or_thruster_sender_is_rejected(self):
        self.assertEqual(module.component_conflicts({'usb_cam','course_mission','rc_auto_mode_test'},'mission'),
                         {'course_mission','rc_auto_mode_test'})
    def test_equipment_cannot_be_started_twice(self):
        self.assertEqual(module.component_conflicts({'usb_cam'},'bringup'),{'usb_cam'})
    def test_split_mission_never_auto_starts(self):
        plan=module.execution_plan('autonomy','mission','web')
        self.assertTrue(plan['mission']);self.assertTrue(plan['output_enabled'])
        self.assertFalse(plan['equipment']);self.assertFalse(plan['auto_start'])
    def test_bringup_never_enables_propulsion(self):
        plan=module.execution_plan('autonomy','bringup','auto')
        self.assertTrue(plan['equipment']);self.assertFalse(plan['mission'])
        self.assertFalse(plan['output_enabled']);self.assertFalse(plan['auto_start'])


if __name__=='__main__':unittest.main()
