import json
from pathlib import Path
import urllib.request
import urllib.error
import unittest
from isv_shape_navigation.mission_dashboard import Dashboard


class DashboardTests(unittest.TestCase):
    def setUp(self):
        self.dashboard=Dashboard(Path(__file__).resolve().parents[1]/'web',0)
        self.base='http://127.0.0.1:'+str(self.dashboard.port)

    def tearDown(self):self.dashboard.close()

    def test_dashboard_assets_and_state(self):
        for path,content in (('/',b'GPS'),('/app.js',b'WP2_HOLD'),('/style.css',b'grid')):
            with urllib.request.urlopen(self.base+path) as response:
                self.assertIn(content,response.read())
        self.dashboard.publish(dict(stamp=100.,state='WP2_HOLD',intended_pwm=[1500,1500],hold_remaining=9.))
        with urllib.request.urlopen(self.base+'/api/state') as response:state=json.load(response)
        self.assertEqual(state['hold_remaining'],9.)
        self.assertEqual(state['history'][-1]['pwm'],[1500,1500])

    def test_same_origin_command_is_queued_not_executed_by_web_thread(self):
        request=urllib.request.Request(self.base+'/api/command',data=b'{"action":"stop"}',
            headers={'Content-Type':'application/json','Origin':self.base},method='POST')
        with urllib.request.urlopen(request) as response:self.assertEqual(response.status,202)
        self.assertEqual(self.dashboard.commands.get_nowait(),{'action':'stop'})

    def test_cross_origin_and_unknown_commands_rejected(self):
        for origin,action in (('http://untrusted.example','start'),(self.base,'shell')):
            request=urllib.request.Request(self.base+'/api/command',
                data=json.dumps({'action':action}).encode(),headers={'Content-Type':'application/json','Origin':origin},method='POST')
            try:
                urllib.request.urlopen(request)
                self.fail('Unsafe command unexpectedly accepted')
            except urllib.error.HTTPError as error:
                error.close()
        self.assertTrue(self.dashboard.commands.empty())


if __name__=='__main__':unittest.main()
