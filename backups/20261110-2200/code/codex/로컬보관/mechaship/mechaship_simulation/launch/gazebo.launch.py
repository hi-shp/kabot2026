import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 시뮬레이션 시간(Gazebo) 사용 여부
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    # world 파일 경로
    world_path = os.path.join(
        get_package_share_directory("mechaship_simulation"),
        "worlds",
        f"{os.environ.get("WORLD_NAME", "empty")}.world",
    )

    # 로봇 형상 정보 publish
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("mechaship_description"),
                "launch",
                "mechaship_state_publisher.launch.py",
            )
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # Gazebo ROS
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={"gz_args": world_path, "on_exit_shutdown": "true"}.items(),
    )

    # Gazebo에 로봇 불러오기
    spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("mechaship_simulation"),
                "launch",
                "spawn.launch.py",
            )
        )
    )

    # Gazebo ROS 브릿지
    gazebo_bridge_parameter = os.path.join(
        get_package_share_directory("mechaship_simulation"),
        "param",
        "mechaship_bridge.yaml",
    )
    gazebo_bridge_node = Node(
        executable="parameter_bridge",
        package="ros_gz_bridge",
        name="parameter_bridge",
        namespace="",
        parameters=[
            {
                "config_file": gazebo_bridge_parameter,
                "qos_overrides./scan.publisher.reliability": "best_effort",
                "qos_overrides./scan.publisher.durability": "volatile",
            }
        ],
        emulate_tty=True,
        # output="screen", # debug
    )

    # Gazebo ROS 이미지 브릿지
    gazebo_image_bridge_node = Node(
        executable="image_bridge",
        package="ros_gz_image",
        name="image_bridge",
        namespace="",
        arguments=["/image_raw"],
        emulate_tty=True,
        # output="screen", # debug
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (Gazebo) clock if true",
            ),
            robot_state_publisher,
            gazebo_launch,
            spawn_launch,
            gazebo_bridge_node,
            gazebo_image_bridge_node,
        ]
    )
