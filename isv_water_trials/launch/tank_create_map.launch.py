"""Create a fresh map using an already running /scan sensor stream."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    source = Path(get_package_share_directory('isv_water_trials')) / 'launch/tank_mapping.launch.py'
    return LaunchDescription([
        DeclareLaunchArgument('autosave', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(source)),
            launch_arguments={'mode': 'mapping', 'autosave': LaunchConfiguration('autosave')}.items()),
    ])
