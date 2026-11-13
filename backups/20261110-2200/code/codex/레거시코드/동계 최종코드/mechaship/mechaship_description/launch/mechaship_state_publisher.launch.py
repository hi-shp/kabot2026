import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 시뮬레이션 시간(Gazebo) 사용 여부
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    # sdf 파일 경로
    sdf_path = os.path.join(
        get_package_share_directory("mechaship_description"),
        "models",
        os.environ.get("MODEL_NAME", "mechaship"),
        "model.sdf",
    )

    with open(sdf_path, "r") as sdf:
        robot_description = sdf.read()

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
                    {
                        "use_sim_time": use_sim_time,
                        "robot_description": robot_description,
                    }
                ],
                emulate_tty=True,
                # output="screen", # debug
            ),
            Node(
                executable="joint_state_publisher",
                package="joint_state_publisher",
                name="joint_state_publisher",
                namespace="",
                condition=UnlessCondition(use_sim_time),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "use_gui": False,
                        "publish_default_positions": True,
                    }
                ],
                emulate_tty=True,
                # output="screen", # debug
            ),
        ]
    )
