import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Course 1 노드 정의
    course1_node = Node(
        package='your_package_name', # 네 패키지 이름으로 수정해
        executable='course1_executable', # setup.py에 등록한 실행 이름
        name='Course1',
        output='screen'
    )

    # 2. Course 2 노드 정의
    course2_node = Node(
        package='your_package_name',
        executable='course2_executable',
        name='Course2',
        output='screen'
    )

    # 3. Course 3 노드 정의
    course3_node = Node(
        package='your_package_name',
        executable='course3_executable',
        name='Course3',
        output='screen'
    )

    # 이벤트 핸들러: Course1이 종료되면 Course2 실행
    start_course2_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=course1_node,
            on_exit=[
                LogInfo(msg='Course 1 finished. Starting Course 2...'),
                course2_node
            ]
        )
    )

    # 이벤트 핸들러: Course2가 종료되면 Course3 실행
    start_course3_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=course2_node,
            on_exit=[
                LogInfo(msg='Course 2 finished. Starting Course 3...'),
                course3_node
            ]
        )
    )

    return LaunchDescription([
        course1_node,
        start_course2_handler,
        start_course3_handler
    ])