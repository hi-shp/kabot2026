"""Inspect launch actions using doubles; never run ROS, processes or devices."""
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch


ROOT=Path(__file__).resolve().parents[2]


class Action:
    def __init__(self,*args,**kwargs):self.args,self.kwargs=args,kwargs


class SplitLaunchTests(unittest.TestCase):
    def actions(self,components,mode='autonomy',gui='auto'):
        modules={}
        def module(name,**attrs):
            value=types.ModuleType(name);vars(value).update(attrs);modules[name]=value
        def kind(name):return type(name,(Action,),{})
        module('launch',LaunchDescription=kind('LaunchDescription'))
        module('launch.actions',**{name:kind(name) for name in
            ['DeclareLaunchArgument','IncludeLaunchDescription','OpaqueFunction','TimerAction','ExecuteProcess','LogInfo']})
        module('launch.launch_description_sources',PythonLaunchDescriptionSource=kind('PythonLaunchDescriptionSource'))
        class Configuration:
            def __init__(self,name):self.name=name
            def perform(self,context):return context[self.name]
        module('launch.substitutions',LaunchConfiguration=Configuration)
        module('launch_ros');module('launch_ros.actions',Node=kind('Node'))
        module('ament_index_python');module('ament_index_python.packages',get_package_share_directory=lambda name:'/unused/'+name)
        options=dict(params=str(ROOT/'설정/course_mission.yaml'),camera='/camera',gps='/gps',imu='/imu',arduino='/mcu',
                     lidar_model='VLP16',lidar_ip='192.168.1.201',model='/model.pt',calibration='/calib.yaml')
        module('launcher_check',settings=lambda _:options,validate=lambda *a,**k:None,measured_calibration=lambda _:False)
        spec=importlib.util.spec_from_file_location('split_launch',ROOT/'빌드실행/all_in_one.launch.py')
        with patch.dict(sys.modules,modules),patch.dict('os.environ',{'KABOAT_GUI':''}):
            launch=importlib.util.module_from_spec(spec);spec.loader.exec_module(launch)
            return launch.assemble(dict(components=components,mode=mode,gui=gui))

    def flatten(self,actions):
        for action in actions:
            yield action
            yield from self.flatten(action.kwargs.get('actions',[]))

    def test_bringup_contains_sensors_and_perception_without_mission(self):
        actions=list(self.flatten(self.actions('bringup')))
        nodes=[a for a in actions if type(a).__name__=='Node']
        self.assertEqual([a.kwargs['executable'] for a in nodes],['docking_camera'])
        includes=[a for a in actions if type(a).__name__=='IncludeLaunchDescription']
        self.assertEqual(len(includes),2)  # sensor launch + YOLO; fusion is headless.
        commands=[a.kwargs['cmd'] for a in actions if type(a).__name__=='ExecuteProcess']
        self.assertTrue(any('fusion_headless.py' in str(cmd) for cmd in commands))
        self.assertFalse(any('--auto-start' in cmd for cmd in commands))

    def test_mission_has_only_controller_and_monitor(self):
        actions=self.actions('mission',gui='web')
        self.assertEqual([type(a).__name__ for a in actions],['LogInfo','Node','ExecuteProcess'])
        mission=actions[1]
        self.assertEqual(mission.kwargs['executable'],'course_mission')
        params=mission.kwargs['parameters'][-1]
        self.assertTrue(params['enable_output']);self.assertTrue(params['gui_enabled'])
        self.assertEqual(params['gui_backend'],'web')
        self.assertFalse(params['dock_calibration_verified'])
        self.assertIn('--monitor',actions[2].kwargs['cmd'])
        self.assertNotIn('--auto-start',actions[2].kwargs['cmd'])

    def test_split_preview_disables_output(self):
        mission=self.actions('mission',mode='preview',gui='desktop')[1]
        self.assertFalse(mission.kwargs['parameters'][-1]['enable_output'])

    def test_original_full_stack_keeps_all_components(self):
        actions=list(self.flatten(self.actions('all')))
        self.assertTrue(any(type(a).__name__=='Node' for a in actions))
        self.assertEqual(sum(type(a).__name__=='IncludeLaunchDescription' for a in actions),3)
        self.assertTrue(any('--auto-start' in a.kwargs.get('cmd',[]) for a in actions))


if __name__=='__main__':unittest.main()
