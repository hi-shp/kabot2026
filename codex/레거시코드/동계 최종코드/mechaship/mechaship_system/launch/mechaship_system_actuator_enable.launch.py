import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 파라미터 폴더
    pkg_share_dir_param = os.path.join(
        get_package_share_directory("mechaship_system"), "param"
    )

    mechaship_actuator_parameter = LaunchConfiguration(
        "mechaship_actuator_parameter",
        default=os.path.join(pkg_share_dir_param, "actuator.yaml"),
    )
    actuator_enable_launch_arg = DeclareLaunchArgument(
        "mechaship_actuator_parameter",
        default_value=mechaship_actuator_parameter,
    )

    actuator_enable_node = Node(
        executable="actuator_enable_node",
        package="mechaship_system",
        name="actuator_enable_node",
        namespace="",
        parameters=[mechaship_actuator_parameter],
        emulate_tty=True,
        # output="screen", # debug
    )

    return LaunchDescription(
        [
            actuator_enable_launch_arg,
            actuator_enable_node,
        ]
    )
