"""Independent mission; optionally reuse upstream passive tank SLAM launch."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def navigation_node(context):
    config = LaunchConfiguration
    overrides = {
        'enable_output': ParameterValue(config('enable_output'), value_type=bool),
        'use_sim_time': ParameterValue(config('use_sim_time'), value_type=bool),
    }
    for key in ('target_label', 'target_color', 'detector_source'):
        value = config(key).perform(context)
        if value:
            overrides[key] = value
    return [Node(package='isv_shape_navigation', executable='shape_navigation', name='shape_navigation',
                 output='screen', parameters=[config('params_file'), overrides])]


def generate_launch_description():
    share = Path(get_package_share_directory('isv_shape_navigation'))
    upstream = Path(get_package_share_directory('isv_water_trials'))
    config = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=str(share/'config/shape_navigation.yaml')),
        DeclareLaunchArgument('enable_output', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_slam', default_value='false'),
        DeclareLaunchArgument('slam_mode', default_value='mapping'),
        DeclareLaunchArgument('map_file', default_value=''),
        DeclareLaunchArgument('target_label', default_value=''),
        DeclareLaunchArgument('target_color', default_value=''),
        DeclareLaunchArgument('detector_source', default_value=''),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(upstream/'launch/tank_mapping.launch.py')),
            condition=IfCondition(config('start_slam')),
            launch_arguments={'mode': config('slam_mode'), 'map_file': config('map_file'),
                              'autosave': 'false'}.items()),
        OpaqueFunction(function=navigation_node),
    ])
