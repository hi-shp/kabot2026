import json,time,sys,unittest,urllib.request,urllib.error
from pathlib import Path
from unittest.mock import patch,Mock
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from viewer_runtime import RuntimeManager
from windows_viewer import create_dashboard

class LifecycleTests(unittest.TestCase):
 def test_prepare_v3_never_auto_starts(self):
  manager=RuntimeManager(mission_report=lambda:{'stamp':time.time(),'output':{'active':False,'armed':False}})
  manager.snapshot=Mock(return_value={});manager.stop_processes=Mock();manager.spawn=Mock()
  with patch('viewer_runtime.processes',return_value=[]):manager.command('v3_autonomy')
  argv=manager.spawn.call_args.args[0]
  self.assertIn('--autonomy',argv);self.assertIn('--wait-start',argv)
  self.assertNotIn('v3_cli_runner.py',' '.join(argv))
 def test_active_mission_cannot_be_replaced(self):
  manager=RuntimeManager(mission_report=lambda:{'stamp':time.time(),'output':{'active':True}});manager.spawn=Mock();manager.stop_processes=Mock()
  with patch('viewer_runtime.processes',return_value=[]),self.assertRaises(ValueError):manager.command('course_autonomy')
  manager.spawn.assert_not_called();manager.stop_processes.assert_not_called()
 def test_stale_status_cannot_switch(self):
  manager=RuntimeManager(mission_report=lambda:{'stamp':time.time()-10,'output':{'active':False}});manager.spawn=Mock()
  with patch('viewer_runtime.processes',return_value=[(123,[])]),self.assertRaises(ValueError):manager.command('v3_autonomy')
  manager.spawn.assert_not_called()
 def test_docking_unmeasured_calibration_blocks_output(self):
  manager=RuntimeManager();manager.spawn=Mock();manager.stop_processes=Mock()
  with patch('viewer_runtime.processes',return_value=[]),self.assertRaises(ValueError):manager.command('docking_autonomy')
  manager.spawn.assert_not_called();manager.stop_processes.assert_not_called()
 def test_forwarded_origin_and_refresh(self):
  calls=[];dash=create_dashboard(0,lambda c:(calls.append(c) or {'success':True,'message':'mock'}))
  url='http://127.0.0.1:'+str(dash.port)
  dash.publish({'stamp':time.time(),'intended_pwm':[1500,1500],'state':'TO_WP1','output':{'active':False}})
  def post(host,origin):
   req=urllib.request.Request(url+'/api/command',data=b'{"action":"set_target","target_label":"red triangle"}',headers={'Host':host,'Origin':origin,'Content-Type':'application/json'})
   try:return urllib.request.urlopen(req,timeout=3).status
   except urllib.error.HTTPError as e:return e.code
  try:
   self.assertEqual(post('127.0.0.1:8772','http://127.0.0.1:8772'),200)
   self.assertEqual(post('localhost:18871','http://localhost:18871'),200)
   self.assertEqual(post('127.0.0.1:8772','http://other.example'),403)
   self.assertEqual(post('other.example:8772','http://other.example:8772'),403)
   with patch('gui_runtime.ensure_runtime') as bootstrap:
    self.assertEqual(urllib.request.urlopen(url+'/',timeout=3).status,200);bootstrap.assert_not_called()
   self.assertEqual(len(calls),2)
  finally:dash.close()

if __name__=='__main__':unittest.main(verbosity=2)
