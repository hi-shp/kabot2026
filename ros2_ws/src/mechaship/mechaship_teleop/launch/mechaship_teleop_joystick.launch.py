import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        namespace="",
        output="screen",
        emulate_tty=True,
        parameters=[],
    )

    teleop_joystick = Node(
        package="mechaship_teleop",
        executable="mechaship_teleop_joystick",
        name="teleop_joystick",
        namespace="",
        output="screen",
        emulate_tty=True,
        parameters=[],
    )

    return LaunchDescription(
        [
            joy_node,
            teleop_joystick,
        ]
    )
