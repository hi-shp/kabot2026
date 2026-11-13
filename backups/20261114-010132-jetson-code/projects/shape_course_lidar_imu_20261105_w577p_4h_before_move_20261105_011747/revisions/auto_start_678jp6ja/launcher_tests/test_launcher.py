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
            with self.assertRaisesRegex(ValueError,'actual 640x480'):module.validate(self.options,'autonomy',devices=False)
    def test_wrong_image_size_is_rejected_for_autonomy(self):
        self.data['camera']['height']=360;self.save()
        with patch.object(module,'calibration_path',side_effect=lambda p:Path(p)):
            with self.assertRaisesRegex(ValueError,'640x480'):module.validate(self.options,'autonomy',devices=False)
    def test_nan_calibration_is_rejected(self):
        self.data['camera']['K'][0]=float('nan');self.save()
        with self.assertRaisesRegex(ValueError,'finite'):module.validate(self.options,'preview',devices=False)
    def test_malformed_transform_is_rejected(self):
        self.data['extrinsics']['T']=[];self.save()
        with self.assertRaisesRegex(ValueError,'extrinsics'):module.validate(self.options,'preview',devices=False)
    def test_missing_model_is_rejected_before_launch(self):
        self.options['model']=str(self.root/'missing.pt')
        with self.assertRaisesRegex(ValueError,'model'):module.validate(self.options,'preview',devices=False)


if __name__=='__main__':unittest.main()
