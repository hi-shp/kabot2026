import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # 파라미터 폴더
    pkg_share_dir_param = os.path.join(
        get_package_share_directory("mechaship_system"), "param"
    )
    actuator_params = LaunchConfiguration(
        "actuator_params",
        default=os.path.join(
            get_package_share_directory("mechaship_system"),
            "param",
            "actuator.yaml",
        ),
    )
    actuator_params_arg = DeclareLaunchArgument(
        "actuator_params",
        default_value=actuator_params,
    )

    actuator_enable_node = Node(
        package="mechaship_system",
        executable="actuator_enable_node",
        name="actuator_enable_node",
        namespace="",
        output="screen",
        emulate_tty=True,
        parameters=[actuator_params],
    )

    return LaunchDescription(
        [
            actuator_params_arg,
            actuator_enable_node,
        ]
    )
