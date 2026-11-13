import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 시뮬레이션 시간(가제보) 사용 여부
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    # model.sdf 파일 경로
    sdf_path = os.path.join(
        "sdf_path",
        get_package_share_directory("mechaship_description"),
        "models",
        "mechaship",
        "model.sdf",
    )

    with open(sdf_path, "r") as sdf:
        robot_desc = sdf.read()

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock if true",
            ),
            Node(
                executable="robot_state_publisher",
                package="robot_state_publisher",
                name="robot_state_publisher",
                namespace="",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"robot_description": robot_desc},
                ],
                # debug
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
