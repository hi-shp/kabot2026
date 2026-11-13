"""객체별 등록 GPS를 선택하여 반경 내 호핑을 실행한다."""
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
        DeclareLaunchArgument('objects_file', default_value=str(share/'config/object_gps.yaml')),
        DeclareLaunchArgument('target_label', default_value='red triangle'),
        DeclareLaunchArgument('enable_output', default_value='false'),
        Node(package='isv_shape_navigation', executable='object_gps', name='object_gps', output='screen',
             parameters=[LaunchConfiguration('params_file'), {
                 'object_goals_file': ParameterValue(LaunchConfiguration('objects_file'), value_type=str),
                 'target_label': ParameterValue(LaunchConfiguration('target_label'), value_type=str),
                 'enable_output': ParameterValue(LaunchConfiguration('enable_output'), value_type=bool),
             }]),
    ])
