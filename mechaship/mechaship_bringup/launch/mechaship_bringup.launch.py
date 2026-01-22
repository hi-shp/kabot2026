import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    # 파라미터 폴더
    pkg_share_dir_param = os.path.join(
        get_package_share_directory("mechaship_bringup"), "param"
    )

    imu_reformatter_node = Node(
        package="mechaship_bringup",
        executable="imu_reformatter.py", # CMake에서 설치한 파일명과 일치해야 함
        name="imu_reformatter",
        output="screen",
        emulate_tty=True,
    )

    # micro-ros
    micro_ros_node = Node(
        executable="micro_ros_agent",
        package="micro_ros_agent",
        name="micro_ros_agent",
        namespace="",
        arguments=["serial", "--dev", "/dev/ttyUROS"],
        emulate_tty=True,
        # output="screen", # debug
    )

    # 액추에이터
    mechaship_actuator_parameter = LaunchConfiguration(
        "mechaship_actuator_parameter",
        default=os.path.join(pkg_share_dir_param, "mechaship_actuator.yaml"),
    )
    actuator_cfg_launch_arg = DeclareLaunchArgument(
        "mechaship_actuator_parameter", default_value=mechaship_actuator_parameter
    )
    actuator_cfg_node = Node(
        executable="actuator_enable_node",
        package="mechaship_system",
        name="actuator_cfg_node",
        namespace="",
        parameters=[mechaship_actuator_parameter],
        emulate_tty=True,
        # output="screen", # debug
    )

    # GPS 드라이버
    mechaship_gps_parameter = LaunchConfiguration(
        "mechaship_gps_parameter",
        default=os.path.join(pkg_share_dir_param, "mechaship_gps.yaml"),
    )
    gps_driver_launch_arg = DeclareLaunchArgument(
        "mechaship_gps_parameter", default_value=mechaship_gps_parameter
    )
    gps_driver_node = Node(
        executable="wtrtk_ros2_driver",
        package="wtrtk_ros2_driver",
        name="wtrtk_ros2_driver",
        namespace="",
        parameters=[mechaship_gps_parameter],
        emulate_tty=True,
        # output="screen", # debug
    )

    # GPS NMEA 파서
    gps_nmea_parser_node = Node(
        executable="nmea_topic_driver",
        package="nmea_navsat_driver",
        name="nmea_topic_driver",
        namespace="",
        parameters=[],
        remappings=[("fix", "gps/fix")],
        emulate_tty=True,
        # output="screen", # debug
    )

    # GPS NTRIP 클라이언트
    ntrip_parameter = LaunchConfiguration(
        "ntrip_parameter",
        default=os.path.join(pkg_share_dir_param, "ntrip_info.yaml"),
    )
    ntrip_client_launch_arg = DeclareLaunchArgument(
        "ntrip_parameter", default_value=ntrip_parameter
    )
    ntrip_client_node = Node(
        executable="ntrip_ros.py",
        package="ntrip_client",
        name="ntrip_client",
        namespace="",
        parameters=[ntrip_parameter],
        remappings=[("fix", "gps/fix")],
        emulate_tty=True,
        # output="screen", # debug
    )

    # USB 카메라 드라이버
    mechaship_camera_parameter = LaunchConfiguration(
        "mechaship_camera_parameter",
        default=os.path.join(pkg_share_dir_param, "mechaship_camera.yaml"),
    )
    usb_camera_launch_arg = DeclareLaunchArgument(
        "mechaship_camera_parameter", default_value=mechaship_camera_parameter
    )
    usb_camera_node = Node(
        executable="usb_cam_node_exe",
        package="usb_cam",
        name="usb_cam_node_exe",
        namespace="",
        parameters=[mechaship_camera_parameter],
        emulate_tty=True,
        # output="screen", # debug
    )

    # LiDAR 드라이버
    mechaship_lidar_parameter = LaunchConfiguration(
        "mechaship_lidar_parameter",
        default=os.path.join(pkg_share_dir_param, "mechaship_lidar.yaml"),
    )
    lidar_driver_launch_arg = DeclareLaunchArgument(
        "mechaship_lidar_parameter", default_value=mechaship_lidar_parameter
    )
    lidar_driver_node = LifecycleNode(
        executable="ydlidar_ros2_driver_node",
        package="ydlidar_ros2_driver",
        name="ydlidar_ros2_driver_node",
        namespace="",
        parameters=[mechaship_lidar_parameter],
        emulate_tty=True,
        # output="screen", # debug
    )

    # IMU 드라이버
    mechaship_imu_parameter = LaunchConfiguration(
        "mechaship_imu_parameter",
        default=os.path.join(pkg_share_dir_param, "mechaship_imu.yaml"),
    )
    imu_driver_launch_arg = DeclareLaunchArgument(
        "mechaship_imu_parameter", default_value=mechaship_imu_parameter
    )
    imu_driver_node = Node(
        executable="iahrs_ros2_driver_node",
        package="iahrs_ros2_driver",
        name="iahrs_ros2_driver",
        namespace="",
        parameters=[mechaship_imu_parameter],
        remappings=[("imu/data", "imu"), ("imu/mag", "mag")],
        emulate_tty=True,
        # output="screen", # debug
    )

    # 로봇 형상 정보 publish
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("mechaship_description"),
                "launch",
                "mechaship_state_publisher.launch.py",
            )
        )
    )

    return LaunchDescription(
        [
            micro_ros_node,
            imu_reformatter_node,
            actuator_cfg_launch_arg,
            actuator_cfg_node,
            gps_driver_launch_arg,
            gps_driver_node,
            gps_nmea_parser_node,
            ntrip_client_launch_arg,
            ntrip_client_node,
            usb_camera_launch_arg,
            usb_camera_node,
            lidar_driver_launch_arg,
            lidar_driver_node,
            imu_driver_launch_arg,
            imu_driver_node,
            robot_state_publisher,
        ]
    )
