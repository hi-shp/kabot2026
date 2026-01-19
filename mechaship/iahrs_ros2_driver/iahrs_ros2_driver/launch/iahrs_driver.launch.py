import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share_directory = get_package_share_directory("iahrs_ros2_driver")

    iahrs_driver_config = os.path.join(pkg_share_directory, "param", "config.yaml")
    iahrs_driver_node = Node(
        package="iahrs_ros2_driver",
        executable="iahrs_ros2_driver_node",
        output="screen",
        name="iahrs_ros2_driver",
        namespace="",
        parameters=[iahrs_driver_config],
        # arguments=["--ros-args", "--log-level", "DEBUG"],
        emulate_tty=True,
    )

    return LaunchDescription(
        [
            iahrs_driver_node,
        ]
    )
