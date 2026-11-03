import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 시뮬레이션 시간(가제보) 사용 여부 가져오기
    param_stdout = subprocess.run(
        ["ros2", "param", "get", "/robot_state_publisher", "use_sim_time"],
        capture_output=True,
        text=True,
    ).stdout
    get_use_sim_time = "true" if "True" in param_stdout else "false"
    print("use_sim_time :", get_use_sim_time)

    use_sim_time = LaunchConfiguration("use_sim_time", default=get_use_sim_time)

    # rviz2
    rviz_config_dir = os.path.join(
        get_package_share_directory("mechaship_slam"),
        "rviz",
        "mechaship_cartographer.rviz",
    )
    rviz2_node = Node(
        executable="rviz2",
        package="rviz2",
        name="rviz2",
        namespace="",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["-d", rviz_config_dir],
        # debug
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=get_use_sim_time,
                description="Use simulation (Gazebo) clock if true",
            ),
            rviz2_node,
        ]
    )
