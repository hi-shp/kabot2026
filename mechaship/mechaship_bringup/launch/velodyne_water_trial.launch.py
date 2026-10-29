"""Velodyne -> /velodyne_points -> /scan. No actuator control."""
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    model = LaunchConfiguration('velodyne_model').perform(context)
    calibration_names = {'VLP16': 'VLP16db.yaml', 'VLP32C': 'VeloView-VLP-32C.yaml', 'VLS128': 'VLS128.yaml'}
    if model not in calibration_names:
        raise ValueError('velodyne_model must be VLP16, VLP32C, or VLS128; inspect actual sensor label')
    share = get_package_share_directory('velodyne_pointcloud')
    with open(os.path.join(share, 'config', model + '-velodyne_transform_node-params.yaml')) as f:
        transform = yaml.safe_load(f)['velodyne_transform_node']['ros__parameters']
    transform['calibration'] = os.path.join(share, 'params', calibration_names[model])
    transform.update(fixed_frame='', target_frame='', min_range=0.4)
    return [LogInfo(msg='Velodyne model=' + model + '; model/IP/ring and mounting require hardware confirmation'),
            Node(package='velodyne_driver', executable='velodyne_driver_node', name='velodyne_driver_node', output='screen',
                 parameters=[{'device_ip': LaunchConfiguration('velodyne_ip').perform(context),
                              'model': model, 'frame_id': 'velodyne', 'port': 2368, 'rpm': 600.0,
                              'gps_time': False, 'enabled': True}]),
            Node(package='velodyne_pointcloud', executable='velodyne_transform_node', name='velodyne_transform_node',
                 output='screen', parameters=[transform]),
            Node(package='velodyne_laserscan', executable='velodyne_laserscan_node', name='velodyne_laserscan_node',
                 output='screen', parameters=[{'ring': int(LaunchConfiguration('velodyne_ring').perform(context)),
                                               'resolution': 0.007}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('velodyne_model', default_value='VLP16', description='Provisional default; verify physical model'),
        DeclareLaunchArgument('velodyne_ip', default_value='192.168.1.201'),
        DeclareLaunchArgument('velodyne_ring', default_value='-1', description='-1 auto ring; verify horizontal obstacles/water reflections'),
        OpaqueFunction(function=nodes)])
