import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def load_param_from_yaml(yaml_file_path):
    with open(yaml_file_path, "r") as file:
        params = yaml.safe_load(file)
    return params


def generate_launch_description():
    # 파라미터 폴더
    pkg_share_dir_param = os.path.join(
        get_package_share_directory("mechaship_system"), "param"
    )
    service_params_path = os.path.join(pkg_share_dir_param, "service.yaml")
    service_params = LaunchConfiguration(
        "service_params",
        default=os.path.join(
            get_package_share_directory("mechaship_system"),
            "param",
            "service.yaml",
        ),
    )
    service_params_arg = DeclareLaunchArgument(
        "service_params",
        default_value=service_params,
    )

    params = load_param_from_yaml(service_params_path)

    # YAML 파일에서 port 값 가져오기
    device_name = (
        params.get("/**", {}).get("ros__parameters", {}).get("port", "/dev/ttyMCU")
    )  # 기본값 "/dev/ttyMCU"

    # Micro-ROS Agent
    micro_ros_agent_node = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="micro_ros_agent",
        namespace="",
        arguments=["serial", "--dev", device_name],
        emulate_tty=True,
    )

    service_node = Node(
        package="mechaship_system",
        executable="service_node",
        name="service_node",
        namespace="",
        emulate_tty=True,
        parameters=[service_params],
    )

    return LaunchDescription(
        [
            micro_ros_agent_node,
            service_params_arg,
            service_node,
        ]
    )
