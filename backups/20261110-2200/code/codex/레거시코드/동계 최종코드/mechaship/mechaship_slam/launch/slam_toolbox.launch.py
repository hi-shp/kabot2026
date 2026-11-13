import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    # 파라미터 폴더
    pkg_share_dir_param = os.path.join(
        get_package_share_directory("mechaship_slam"), "param"
    )

    # 시뮬레이션 시간(가제보) 사용 여부 가져오기
    param_stdout = subprocess.run(
        ["ros2", "param", "get", "/robot_state_publisher", "use_sim_time"],
        capture_output=True,
        text=True,
    ).stdout
    get_use_sim_time = "true" if "True" in param_stdout else "false"
    print("use_sim_time :", get_use_sim_time)

    use_sim_time = LaunchConfiguration("use_sim_time", default=get_use_sim_time)

    # odom publish
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("mechaship_slam"), "launch", "ekf.launch.py"
            )
        )
    )

    # SLAM ToolBox
    slam_toolbox_parameter = LaunchConfiguration(
        "slam_toolbox_parameter",
        default=os.path.join(pkg_share_dir_param, "slam_toolbox.yaml"),
    )
    slam_toolbox_launch_arg = DeclareLaunchArgument(
        "mechaship_gps_parameter", default_value=slam_toolbox_parameter
    )
    async_slam_toolbox_node = LifecycleNode(
        executable="async_slam_toolbox_node",
        package="slam_toolbox",
        name="slam_toolbox",
        namespace="",
        parameters=[slam_toolbox_parameter, {"use_sim_time": use_sim_time}],
        emulate_tty=True,
        # output="screen", # debug
    )

    # slam_toolbox에 configure 요청
    configure_slam_toolbox = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(async_slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    # configuring → inactive 도달하면 자동 activate
    activate_slam_toolbox = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=async_slam_toolbox_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[Lifecycle] slam_toolbox node is activating."),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(async_slam_toolbox_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=get_use_sim_time,
                description="Use simulation (Gazebo) clock if true",
            ),
            ekf_launch,
            slam_toolbox_launch_arg,
            async_slam_toolbox_node,
            configure_slam_toolbox,
            activate_slam_toolbox,
        ]
    )
