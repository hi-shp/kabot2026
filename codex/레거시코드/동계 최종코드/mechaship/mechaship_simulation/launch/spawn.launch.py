import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Gazebo에 로봇 불러오기
    spawn_entity_node = Node(
        executable="create",
        package="ros_gz_sim",
        name="create",
        namespace="",
        parameters=[
            {
                "world": os.environ.get("WORLD_NAME", "empty"),
                "topic": "robot_description",
                "name": "mechaship",
                "x": 0.0,
                "y": 0.0,
                "z": 0.02,
            }
        ],
        emulate_tty=True,
        # output="screen", # debug
    )

    return LaunchDescription([spawn_entity_node])
