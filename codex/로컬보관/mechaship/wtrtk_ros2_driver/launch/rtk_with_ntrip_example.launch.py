import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share_directory = get_package_share_directory("wtrtk_ros2_driver")

    wtrtk_ros2_driver_config = os.path.join(
        pkg_share_directory, "param", "WTRTK-960.yaml"
    )
    wtrtk_ros2_driver = Node(
        package="wtrtk_ros2_driver",
        executable="wtrtk_ros2_driver",
        output="screen",
        name="wtrtk_ros2_driver",
        namespace="",
        parameters=[wtrtk_ros2_driver_config],
        # arguments=["--ros-args", "--log-level", "DEBUG"],
        emulate_tty=True,
    )

    nmea_topic_driver = Node(
        package="nmea_navsat_driver",
        executable="nmea_topic_driver",
        output="screen",
        name="nmea_topic_driver",
        namespace="",
        parameters=[],
        # arguments=["--ros-args", "--log-level", "DEBUG"],
        emulate_tty=True,
    )

    ntrip_client_launch_param_host = DeclareLaunchArgument("host", default_value="")
    ntrip_client_launch_param_mountpoint = DeclareLaunchArgument(
        "mountpoint", default_value=""
    )
    ntrip_client_launch_param_username = DeclareLaunchArgument(
        "username", default_value=""
    )
    ntrip_client_launch_param_password = DeclareLaunchArgument(
        "password", default_value=""
    )

    ntrip_client_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ntrip_client"),
                "ntrip_client_launch.py",
            )
        ),
        launch_arguments={
            "host": LaunchConfiguration("host"),
            "mountpoint": LaunchConfiguration("mountpoint"),
            "username": LaunchConfiguration("username"),
            "password": LaunchConfiguration("password"),
        }.items(),
    )

    return LaunchDescription(
        [
            wtrtk_ros2_driver,
            nmea_topic_driver,
            ntrip_client_launch_param_host,
            ntrip_client_launch_param_mountpoint,
            ntrip_client_launch_param_username,
            ntrip_client_launch_param_password,
            ntrip_client_launch,
        ]
    )
