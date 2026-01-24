# import os

# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node


# def generate_launch_description():
#     # YOLOv8 객체 인식 시각화
#     yolov8_params = LaunchConfiguration(
#         "yolov8_params",
#         default=os.path.join(
#             get_package_share_directory("mechaship_yolov8"),
#             "param",
#             "yolov8_params.yaml",
#         ),
#     )
#     yolov8_params_arg = DeclareLaunchArgument(
#         "yolov8_params",
#         default_value=yolov8_params,
#     )

#     yolov8_visualize_node = Node(
#         executable="visualize_node",
#         package="mechaship_yolov8",
#         name="visualize_node",
#         namespace="",
#         parameters=[yolov8_params],
#         emulate_tty=True,
#         # output="screen", # debug
#     )

#     return LaunchDescription([yolov8_params_arg, yolov8_visualize_node])

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # YOLOv8 객체 인식 시각화 파라미터 파일
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

    # ✅ 디텍션 토픽이 끊겼을 때 clear까지 기다릴 시간(초)
    # 기본값 0.3초. 실행 시 detection_timeout_sec:=0.1 처럼 0.1 단위로 변경 가능
    detection_timeout_sec = LaunchConfiguration("detection_timeout_sec", default="0.3")
    detection_timeout_sec_arg = DeclareLaunchArgument(
        "detection_timeout_sec",
        default_value=detection_timeout_sec,
        description="Clear visualization if no detection msg arrives for this duration (sec). e.g. 0.1, 0.2, 0.3 ...",
    )

    yolov8_visualize_node = Node(
        executable="visualize_node",
        package="mechaship_yolov8",
        name="visualize_node",
        namespace="",
        parameters=[
            yolov8_params,  # 기존 yaml
            {"detection_timeout_sec": detection_timeout_sec},  # ✅ launch에서 오버라이드 가능
        ],
        emulate_tty=True,
        # output="screen",  # debug
    )

    return LaunchDescription(
        [
            yolov8_params_arg,
            detection_timeout_sec_arg,
            yolov8_visualize_node,
        ]
    )
