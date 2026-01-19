import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # YOLOv8 객체 인식 시각화
    yolov8_params = LaunchConfiguration(
        "yolov8_params",
        default=os.path.join(
            get_package_share_directory("mechaship_yolov8"),
            "param",
            "yolov8_params.yaml",
        ),
    )
    yolov8_params_arg = DeclareLaunchArgument(
        "yolov8_params",
        default_value=yolov8_params,
    )

    yolov8_visualize_node = Node(
        executable="visualize_node",
        package="mechaship_yolov8",
        name="visualize_node",
        namespace="",
        parameters=[yolov8_params],
        emulate_tty=True,
        # output="screen", # debug
    )

    return LaunchDescription([yolov8_params_arg, yolov8_visualize_node])
