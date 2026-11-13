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

    # SLAM ToolBox
    slam_params_file = LaunchConfiguration(
        "slam_params_file",
        default=os.path.join(
            get_package_share_directory("mechaship_slam"), "config", "slam_toolbox.yaml"
        ),
    )
    start_async_slam_toolbox_node = Node(
        executable="async_slam_toolbox_node",
        package="slam_toolbox",
        name="slam_toolbox",
        namespace="",
        parameters=[slam_params_file, {"use_sim_time": use_sim_time}],
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
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=slam_params_file,
                description="Full path to the ROS2 parameters file to use for the slam_toolbox node",
            ),
            start_async_slam_toolbox_node,
        ]
    )
