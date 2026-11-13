"""Three-waypoint mission and local dashboard; no hardware bringup side effects."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share=Path(get_package_share_directory('isv_shape_navigation'))
    return LaunchDescription([
        DeclareLaunchArgument('params_file',default_value=str(share/'config/course_mission.yaml')),
        DeclareLaunchArgument('enable_output',default_value='false'),
        DeclareLaunchArgument('target_label',default_value='red triangle'),
        DeclareLaunchArgument('gui_port',default_value='8770'),
        DeclareLaunchArgument('gui_backend',default_value='desktop',choices=['desktop','web']),
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        Node(package='isv_shape_navigation',executable='course_mission',name='course_mission',output='screen',
             parameters=[LaunchConfiguration('params_file'),{
                 'enable_output':ParameterValue(LaunchConfiguration('enable_output'),value_type=bool),
                 'target_label':ParameterValue(LaunchConfiguration('target_label'),value_type=str),
                 'gui_port':ParameterValue(LaunchConfiguration('gui_port'),value_type=int),
                 'gui_backend':ParameterValue(LaunchConfiguration('gui_backend'),value_type=str),
                 'use_sim_time':ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool),
             }]),
    ])
