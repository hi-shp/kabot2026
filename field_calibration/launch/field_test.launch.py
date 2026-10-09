import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def start(context):
    share = get_package_share_directory('field_calibration')
    profile = LaunchConfiguration('profile').perform(context)
    if profile not in ('indoor', 'outdoor'):
        raise ValueError('profile must be indoor or outdoor')
    params = [LaunchConfiguration('config'), os.path.join(share, 'config', profile+'.yaml'), {
        'test_mode': ParameterValue(LaunchConfiguration('test_mode'), value_type=str),
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
    }]
    return [Node(package='field_calibration', executable=name, name=name, output='screen', parameters=params)
            for name in ('field_logger', 'state_monitor', 'motion_predictor', 'calibration_node')]


def generate_launch_description():
    share = get_package_share_directory('field_calibration')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=os.path.join(share, 'config', 'field_test.yaml')),
        DeclareLaunchArgument('profile', default_value='indoor'),
        DeclareLaunchArgument('test_mode', default_value='manual_log'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=start),
    ])
