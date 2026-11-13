"""Existing GUI HTTP endpoints with a fake mapping transport; no ROS writes."""
import json
from pathlib import Path
import sys
import unittest
import urllib.request
import urllib.error
from docking_mapping.gui_bridge import MappingBridge

sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'빌드실행'))
from windows_viewer import create_dashboard


class MappingHTTPTests(unittest.TestCase):
    def test_existing_map_assets_mapping_acl_and_mission_separation(self):
        calls=[]
        bridge=MappingBridge(controls_enabled=True,transport=lambda p:(calls.append(p) or dict(success=True,message='queued')))
        bridge.update(json.dumps(dict(preview_schema=1,status=dict(state='REFERENCE_LOADED'),map_points=[[35.,129.]],current_scan=[],rc_path=[])))
        dashboard=create_dashboard(0,None,docking=bridge);url='http://127.0.0.1:'+str(dashboard.port)
        def post(path,payload,origin=url,host=None):
            headers={'Origin':origin,'Content-Type':'application/json'}
            if host is not None:headers['Host']=host
            request=urllib.request.Request(url+path,data=json.dumps(payload).encode(),
                headers=headers)
            try:
                with urllib.request.urlopen(request,timeout=3) as response:return response.status,json.load(response)
            except urllib.error.HTTPError as exc:return exc.code,json.load(exc)
        try:
            html=urllib.request.urlopen(url).read().decode()
            self.assertIn('dockHopApply',html);self.assertIn('id="map"',html)
            self.assertIn('dockAutoEmergency',html)
            for asset in ('app.js','docking_map.js','docking_map.css','lidar3d.js'):
                self.assertEqual(urllib.request.urlopen(url+'/'+asset+'?v=test').status,200)
            preview=json.load(urllib.request.urlopen(url+'/api/docking'))
            self.assertTrue(preview['controls_enabled'])
            code,result=post('/api/docking/command',dict(action='load'))
            self.assertEqual(code,200);self.assertTrue(result['success']);self.assertEqual(len(calls),1)
            self.assertEqual(post('/api/docking/command',dict(action='load'),
                origin='http://127.0.0.1:18871',host='127.0.0.1:18871')[0],200)
            self.assertEqual(len(calls),2)
            self.assertEqual(post('/api/docking/command',dict(action='auto_start'))[0],200)
            self.assertEqual(post('/api/docking/command',dict(action='auto_emergency'))[0],200)
            self.assertEqual(post('/api/docking/command',dict(action='load'),
                origin='http://evil.example:18871',host='evil.example:18871')[0],403)
            self.assertEqual(post('/api/docking/command',dict(action='pwm'))[0],400)
            self.assertEqual(post('/api/docking/command',dict(action='load'),'https://invalid.example')[0],403)
            self.assertEqual(post('/api/command',dict(action='start'))[0],403)
            bridge.controls_enabled=False
            self.assertEqual(post('/api/docking/command',dict(action='load'))[0],403)
            self.assertEqual(len(calls),4)
        finally:dashboard.close()


if __name__=='__main__':unittest.main()
