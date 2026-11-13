import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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

    # LiDAR 기반 Odometry publish
    rf2o_laser_odometry_node = Node(
        executable="rf2o_laser_odometry_node",
        package="rf2o_laser_odometry",
        name="rf2o_laser_odometry",
        namespace="",
        parameters=[
            {
                "laser_scan_topic": "scan",
                "odom_topic": "scan/odom",
                "publish_tf": True,
                "base_frame_id": "base_footprint",
                "odom_frame_id": "odom",
                "init_pose_from_topic": "",
                "freq": 10.0,
            }
        ],
        arguments=["--ros-args", "--log-level", "warn"],
        emulate_tty=True,
        # output="screen", # debug
    )

    # GPS 기반 Odometry publish
    navsat_transform_parameter = LaunchConfiguration(
        "navsat_transform_parameter",
        default=os.path.join(pkg_share_dir_param, "navsat_transform.yaml"),
    )
    navsat_transform_launch_arg = DeclareLaunchArgument(
        "mechaship_gps_parameter", default_value=navsat_transform_parameter
    )
    navsat_transform_node = Node(
        executable="navsat_transform_node",
        package="robot_localization",
        name="navsat_transform_node",
        namespace="",
        parameters=[{"use_sim_time": use_sim_time}, navsat_transform_parameter],
        remappings=[("odometry/filtered", "/odom"), ("odometry/gps", "/gps/odom")],
        emulate_tty=True,
        # output="screen", # debug
    )

    # 통합 Odometry publish
    ekf_filter_parameter = LaunchConfiguration(
        "ekf_filter_parameter",
        default=os.path.join(pkg_share_dir_param, "ekf.yaml"),
    )
    ekf_filter_launch_arg = DeclareLaunchArgument(
        "ekf_filter_parameter", default_value=ekf_filter_parameter
    )
    ekf_filter_node = Node(
        executable="ekf_node",
        package="robot_localization",
        name="ekf_filter_node",
        namespace="",
        parameters=[{"use_sim_time": use_sim_time}, ekf_filter_parameter],
        remappings=[("odometry/filtered", "/odom")],
        emulate_tty=True,
        # output="screen", # debug
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=get_use_sim_time,
                description="Use simulation (Gazebo) clock if true",
            ),
            rf2o_laser_odometry_node,
            navsat_transform_launch_arg,
            navsat_transform_node,
            ekf_filter_launch_arg,
            ekf_filter_node,
        ]
    )
