from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Declare all launch-time parameters
    args = [
        DeclareLaunchArgument('image_topic', default_value='/camera1/image_raw'),
        # CHANGED: detections_topic 기본값을 tracking으로
        DeclareLaunchArgument('detections_topic', default_value='/yolo/tracking'),
        DeclareLaunchArgument('cloud_topic', default_value='/ouster/points'),
        DeclareLaunchArgument('calib_file', default_value='calib_example.yaml'),

        DeclareLaunchArgument('enable_projection', default_value='true'),
        DeclareLaunchArgument('depth_min', default_value='1.0'),
        DeclareLaunchArgument('depth_max', default_value='5.0'),  #최소 최대 거리 지정
        DeclareLaunchArgument('reject_out_of_depth_range', default_value='false'),   #on/off

        DeclareLaunchArgument('min_draw_interval_ms', default_value='50'),
        # 여유있게 2500ms로 시작(타임미스매치 확인 후 줄이기)  20fs 정도
        DeclareLaunchArgument('max_age_ms', default_value='300'),
        DeclareLaunchArgument('force_projection_without_time_match', default_value='true'),

        DeclareLaunchArgument('debug', default_value='true'),
        DeclareLaunchArgument('debug_every', default_value='30'),
        DeclareLaunchArgument('draw_debug_text', default_value='true'),
        DeclareLaunchArgument('trace_very_verbose', default_value='false'),
    ]

    node = Node(
        package='fusion_viewer',
        executable='viewer_node',   # 패키지 엔트리포인트 이름 유지
        name='fusion_viewer',
        output='screen',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'detections_topic': LaunchConfiguration('detections_topic'),
            'cloud_topic': LaunchConfiguration('cloud_topic'),
            'calib_file': LaunchConfiguration('calib_file'),

            'enable_projection': LaunchConfiguration('enable_projection'),
            'depth_min': LaunchConfiguration('depth_min'),
            'depth_max': LaunchConfiguration('depth_max'),
            'reject_out_of_depth_range': LaunchConfiguration('reject_out_of_depth_range'),

            'min_draw_interval_ms': LaunchConfiguration('min_draw_interval_ms'),
            'max_age_ms': LaunchConfiguration('max_age_ms'),
            'force_projection_without_time_match': LaunchConfiguration('force_projection_without_time_match'),

            'debug': LaunchConfiguration('debug'),
            'debug_every': LaunchConfiguration('debug_every'),
            'draw_debug_text': LaunchConfiguration('draw_debug_text'),
            'trace_very_verbose': LaunchConfiguration('trace_very_verbose'),
        }]
    )

    return LaunchDescription(args + [node])

