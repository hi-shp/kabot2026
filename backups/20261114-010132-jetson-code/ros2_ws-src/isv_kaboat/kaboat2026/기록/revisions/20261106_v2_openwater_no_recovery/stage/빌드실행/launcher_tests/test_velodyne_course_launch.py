"""Inspect launch arguments without running any ROS launch action."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
from launch import LaunchContext

ROOT=Path(__file__).resolve().parents[1]


def load(name,file):
    spec=importlib.util.spec_from_file_location(name,ROOT/file)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


class CourseVelodyneLaunchTests(unittest.TestCase):
    def test_transform_minimum_is_ten_centimetres_without_launching_nodes(self):
        module=load('course_lidar_minimum','velodyne_course.launch.py')
        context=LaunchContext()
        context.launch_configurations.update(velodyne_model='VLP16',velodyne_ip='192.168.1.201',
            velodyne_ring='8',min_range='0.1')
        nodes=[]
        with patch.object(module,'Node',side_effect=lambda **kw:(nodes.append(kw) or NS(**kw))):
            module.nodes(context)  # Constructs actions only; no LaunchService is run.
        transform=next(row for row in nodes if row['name']=='velodyne_transform_node')
        self.assertEqual(transform['parameters'][0]['min_range'],.1)
        ring=next(row for row in nodes if row['name']=='velodyne_laserscan_node')
        self.assertEqual(ring['parameters'][0]['ring'],8)

    def test_sensor_bringup_routes_override_to_project_local_launch(self):
        module=load('course_sensor_parent','sensors_one_command.launch.py')
        context=LaunchContext()
        context.launch_configurations.update(camera_device='/dev/video0',gps_device='/dev/gps',
            imu_device='/dev/imu',arduino_device='/dev/mcu',velodyne_model='VLP16',
            velodyne_ip='192.168.1.201',lidar_min_range='0.2')
        captured=[]
        def include(source,launch_arguments):
            captured.append((source,dict(launch_arguments)));return NS()
        with patch.object(module,'Node',return_value=NS()),patch.object(module,'ExecuteProcess',return_value=NS()),\
             patch.object(module,'get_package_share_directory',return_value='/unused/read-only'),\
             patch.object(module,'PythonLaunchDescriptionSource',side_effect=lambda source:source),\
             patch.object(module,'IncludeLaunchDescription',side_effect=include):
            module.sensors(context)
        self.assertEqual(len(captured),1)
        self.assertEqual(Path(captured[0][0]).name,'velodyne_course.launch.py')
        self.assertEqual(captured[0][1]['min_range'],'0.2')
        self.assertEqual(Path(captured[0][0]).parent,ROOT)


if __name__=='__main__':unittest.main()
