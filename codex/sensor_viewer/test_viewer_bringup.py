"""No SSH/ROS/browser launch: check dashboard bringup detection and tab reuse."""
from types import SimpleNamespace
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock,patch
from run_viewer import BrowserWake,ssh_settings,bind_dashboard
from jetson_server import Store,runtime_status


class ViewerBringupTests(unittest.TestCase):
    def test_watertrial_controls_are_serialized_and_scoped(self):
        store=Store(SimpleNamespace())
        with patch('jetson_server.threading.Thread') as thread:
            self.assertEqual(store.request_watertrial_control('stop_other')[0],202)
            self.assertEqual(store.request_watertrial_control('stop_watertrial')[0],409)
            thread.return_value.start.assert_called_once()
        with patch('jetson_server.subprocess.run') as run:
            run.return_value.stdout=b'{"ok":true}\n';run.return_value.returncode=0
            store._run_watertrial_control('stop_watertrial')
        code=run.call_args.args[0][-1]
        compile(code,'<watertrial-control>','exec')
        self.assertIn("for name in NAMES",code)
        with patch('jetson_server.subprocess.run') as run:
            run.return_value.stdout=b'{"ok":true}\n';run.return_value.returncode=0
            store._run_watertrial_control('stop_other')
        code=run.call_args.args[0][-1]
        compile(code,'<watertrial-control>','exec')
        self.assertIn('disarm_first=False',code)
        self.assertEqual(store.request_watertrial_control('start_sensors')[0],400)

    def test_esc_initialization_ack_failure_and_exclusion(self):
        store=Store(SimpleNamespace())
        with patch('jetson_server.threading.Thread') as thread:
            self.assertEqual(store.initialize_esc()[0],202)
            self.assertEqual(store.initialize_esc()[0],409)
            self.assertEqual(store.restart_sensors()[0],409)
            thread.return_value.start.assert_called_once()
        with patch('jetson_server.subprocess.run') as run,patch('jetson_server.time.sleep'):
            store._initialize_esc()
        self.assertTrue(store.esc_initialize['acknowledged'])
        self.assertEqual(run.call_args.args[0][-2:],['-m','isv_water_trials.reinitialize_esc'])
        with patch('jetson_server.subprocess.run',side_effect=OSError):store._initialize_esc()
        self.assertFalse(store.esc_initialize['busy'])
        self.assertFalse(store.esc_initialize['acknowledged'])
        self.assertTrue(store.esc_initialize['error'])
        store.sensor_restart['busy']=True
        self.assertEqual(store.initialize_esc()[0],409)

    def test_restart_is_serialized_and_reports_failure(self):
        store=Store(SimpleNamespace())
        with patch('jetson_server.threading.Thread') as thread:
            self.assertEqual(store.restart_sensors()[0],202)
            self.assertEqual(store.restart_sensors()[0],409)
            thread.return_value.start.assert_called_once()
        with patch('jetson_server.subprocess.run',side_effect=OSError):
            store._restart_sensors()
        self.assertFalse(store.sensor_restart['busy'])
        self.assertTrue(store.sensor_restart['error'])
        with patch('jetson_server.subprocess.run') as run:
            store._restart_sensors()
        self.assertIsNone(store.sensor_restart['error'])
        code=run.call_args.args[0][-1]
        self.assertIn("main(['stop','sensors'])",code)
        self.assertIn("previous.get('args',[])",code)
        self.assertNotIn("'all'",code)

    def test_reserved_port_falls_back_using_the_bound_server(self):
        server=Mock()
        with patch('run_viewer.ThreadingHTTPServer',side_effect=[PermissionError('reserved'),server]) as factory:
            self.assertIs(bind_dashboard(8765),server)
            self.assertEqual([call.args[0] for call in factory.call_args_list],[('127.0.0.1',8765),('127.0.0.1',18765)])

    def test_runtime_processes_and_retry_are_distinct_from_stopped(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            def process(pid,args,state='S'):
                p=root/str(pid);p.mkdir()
                (p/'stat').write_text(f'{pid} (python3) {state} 1 2 3')
                (p/'cmdline').write_bytes(('\0'.join(args)+'\0').encode())
            self.assertEqual(runtime_status(root),dict.fromkeys(('sensors','mapping','trial'),'stopped'))
            process(1,['python3','-m','isv_water_trials.tank_session','_worker','trial'])
            process(2,['python3','/opt/ros/humble/bin/ros2','launch','mechaship_bringup','all_sensors.launch.py'])
            process(3,['python3','/home/isv/ros2_ws/install/isv_water_trials/lib/isv_water_trials/tank_pose'])
            process(4,['python3','-m','isv_water_trials.indoor_trial'],'Z')
            self.assertEqual(runtime_status(root),dict(sensors='running',mapping='running',trial='restarting'))
            (root/'4'/'stat').write_text('4 (python3) S 1 2 3')
            self.assertEqual(runtime_status(root)['trial'],'running')
            self.assertEqual(runtime_status(root/'missing')['trial'],'unknown')

    def test_closed_tab_opens_once_on_bringup(self):
        now=[0.];opened=Mock();wake=BrowserWake('http://localhost',opener=opened,clock=lambda:now[0])
        wake.observe(False);opened.assert_not_called()
        wake.observe(True);wake.observe(True);opened.assert_called_once_with('http://localhost')
        now[0]=40;wake.observe(False);wake.observe(True)
        self.assertEqual(opened.call_count,2)

    def test_open_tab_is_reused_across_sensor_and_ssh_restarts(self):
        now=[100.];opened=Mock();wake=BrowserWake('http://localhost',opener=opened,clock=lambda:now[0])
        wake.viewed();wake.observe(True);wake.observe(False);now[0]+=5;wake.observe(True)
        self.assertEqual(opened.call_count,2)
        wake.observe(False);now[0]+=11;wake.observe(True);self.assertEqual(opened.call_count,3)

    def test_repeated_start_focuses_even_if_sensor_session_is_already_running(self):
        now=[100.];opened=Mock();wake=BrowserWake('http://localhost',opener=opened,clock=lambda:now[0])
        wake.viewed();wake.observe(True,'request1');wake.observe(True,'request1')
        opened.assert_called_once()
        wake.observe(True,'request2');self.assertEqual(opened.call_count,2)
        wake.observe(True);wake.observe(True,'request2');self.assertEqual(opened.call_count,2)

    def test_ssh_alias_preserves_host_key_and_identity(self):
        with tempfile.TemporaryDirectory() as tmp:
            home=Path(tmp);(home/'.ssh').mkdir()
            (home/'.ssh/config').write_text('Host jetson\n HostName 192.168.80.20\n User isv\n HostKeyAlias trusted-jetson\n IdentityFile ~/.ssh/jetson_key\n')
            keys=Mock();key=object();keys.lookup.return_value={'ssh-ed25519':key}
            with patch('run_viewer.Path.home',return_value=home):
                value=ssh_settings(SimpleNamespace(host='jetson',ssh_port=None,user=None),keys)
            self.assertEqual(value['hostname'],'192.168.80.20');self.assertEqual(value['username'],'isv')
            self.assertTrue(value['key_filename'][0].endswith('jetson_key'))
            keys.lookup.assert_called_once_with('trusted-jetson')
            keys.add.assert_called_once_with('192.168.80.20','ssh-ed25519',key)

    def test_initial_open_and_no_browser_avoid_duplicate_tabs(self):
        now=[0.];opened=Mock();wake=BrowserWake('http://localhost',opener=opened,clock=lambda:now[0])
        wake.open();wake.observe(True);self.assertEqual(opened.call_count,1)
        silent=BrowserWake('http://localhost',enabled=False,opener=opened,clock=lambda:now[0])
        silent.open();silent.observe(True);self.assertEqual(opened.call_count,1)

    def test_bringup_health_ignores_monitor_and_stale_data(self):
        store=Store(SimpleNamespace())
        store.graph['nodes']=['/kabot_sensor_monitor','/tank_pose']
        self.assertFalse(store.sensors_running())
        store.graph['nodes'].append('/vectornav');self.assertTrue(store.sensors_running())
        store.graph['nodes']=[]
        with patch('jetson_server.time.monotonic',return_value=100):
            store.values['scan']={'received_monotonic':90};self.assertFalse(store.sensors_running())
            store.values['scan']['received_monotonic']=99;self.assertTrue(store.sensors_running())


if __name__=='__main__':unittest.main()
