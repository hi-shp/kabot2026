"""Compatibility launch: selected object -> registered GPS -> radius hold."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('isv_shape_navigation'))
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=str(share/'config/gps_dock.yaml')),
        DeclareLaunchArgument('target_label', default_value='red triangle'),
        DeclareLaunchArgument('enable_output', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='isv_shape_navigation', executable='object_gps', name='object_gps', output='screen',
             parameters=[LaunchConfiguration('params_file'), {
                 'target_label': ParameterValue(LaunchConfiguration('target_label'), value_type=str),
                 'enable_output': ParameterValue(LaunchConfiguration('enable_output'), value_type=bool),
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
             }]),
    ])
