"""No SSH/ROS/browser launch: check dashboard bringup detection and tab reuse."""
from types import SimpleNamespace
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock,patch
from run_viewer import BrowserWake
from jetson_server import Store,runtime_status


class ViewerBringupTests(unittest.TestCase):
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
        opened.assert_not_called()
        wake.observe(False);now[0]+=11;wake.observe(True);opened.assert_called_once()

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
