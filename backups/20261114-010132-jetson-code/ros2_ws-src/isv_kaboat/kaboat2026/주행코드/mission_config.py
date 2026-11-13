"""전체 주행 설정. 이 파일의 값만 수정하고 주행 프로그램을 재시작하세요.

GPS는 (위도, 경도), 미정 좌표는 (None, None)입니다.
thrust_percentage는 기본 전진 추력(%), reverse_pwm는 좌우 후진 PWM(마이크로초)입니다.
PD 회전 보정으로 최종 좌우 PWM은 기본 전진 추력과 달라질 수 있습니다.
시간은 초 단위입니다. 설정은 시작 시 한 번 읽으며 실시간으로 적용되지 않습니다.
ROS 환경변수와 명시한 ROS 파라미터는 기존과 같이 설정 기본값보다 우선합니다.
status 4의 model_path는 외부 YOLO 노드에도 같은 경로로 지정해야 합니다.
"""

# 통신 환경의 기본값
ROS_CONFIG = {
    'domain_id': 0,
    'rmw_implementation': 'rmw_fastrtps_cpp',
    'localhost_only': '0',
}

# 공통 센서·PD·출력 설정. V3 상세 회피 설정은 v3_settings_file의 기존 값을 사용합니다.
COMMON_CONFIG = {
    'output_settle_seconds': 0.0,  # 초기 출력 대기 시간(초)
    'gps_topic': '/gps/fix',
    'heading_topic': '/vectornav/raw/common',
    'heading_frame': 'vectornav',
    'heading_offset_deg': 0.0,
    'gps_timeout': 2.0,
    'heading_timeout': 0.5,
    'max_gps_sigma': 2.0,
    'image_topic': '/camera/image_raw/compressed',
    'detection_topic': '/yolo/detections',
    'kp': 8.0,
    'kd': 0.0,
    'turn_limit_us': 400.0,
    'arrival_radius_m': 2.0,
    'enable_output': True,
    'auto_follow': True,
    'output_rate_hz': 10.0,
    'v3_settings_file': '~/ros2_ws/src/isv_kaboat/kaboat2026/설정/course_mission.yaml',
}

# 단계별 GPS·목표·기본 전진 추력·후진 출력·시간
STATUS_CONFIG = {
    0: {  # 경로 시작점 접근
        'gps': (None, None),
        'thrust_percentage': 40.0,
    },
    1: {  # 경로 GPS 3개 순차 추종
        'avoidance_distance_weight': 1.0,
        'avoidance_angle_weight':75.0,
        'gps_points': [(None, None), (None, None), (None, None)],
        'thrust_percentage': 50.0,
    },
    2: {  # 호핑 → 후진 → 대기 → status 3
        'avoidance_distance_weight': 1.0,  # 거리 여유: 클수록 먼 장애물 방향 선호
        'avoidance_angle_weight': 75.0,    # 목표각: 클수록 목표 방향 선호
        'gps':(None, None),
        'thrust_percentage': 40.0,
        'reverse_pwm': (1300, 1300),
        'reverse_seconds': 1.0,
        'wait_seconds': 5.0,
    },
    3: {  # V3 회피로 도킹 시작점 접근 → 후진 → status 4
        'avoidance_distance_weight': 1.0,
        'avoidance_angle_weight': 75.0,
        'gps': (None, None),
        'thrust_percentage': 30.0,
        'reverse_pwm': (1300, 1300),
        'reverse_seconds': 2.0,
    },
    4: {  # 목표 도형 카메라 추종 / 미검출 시 대응 GPS 추종
        'arrival_radius_m': 1.0,  # 도킹 GPS 도달 반경(m), Status 4 전용
        'avoidance_distance_weight': 1.0,
        'avoidance_angle_weight': 75.0,
        'target_shape': 'green circle',
        'model_path': '/home/isv/ros2_ws/weights/best3.pt',  # 외부 YOLO 노드의 model에도 같은 경로를 지정
        'gps_by_shape': {
            'blue rectangular': (None, None),
            'red triangle': (None, None),
            'green circle':(None, None),
        },
        'thrust_percentage': 20.0,
        'horizontal_fov_deg': 90.0,
        'confidence_threshold': 0.50,
        'detection_timeout_sec': 0.5,
        'image_timeout_sec': 1.0,
    },
    5: {  # 선택한 부표를 한 바퀴 돌고 status 6으로 전환
        'avoidance_distance_weight': 1.0,
        'avoidance_angle_weight': 75.0,
        'target_buoy': 'green_buoy',
        'gps_by_buoy': {
            'green_buoy': (None, None),
            'black buoy': (None, None),
            'red_buoy': (None, None),
        },
        'orbit_radius_m': 4.0,
        'orbit_point_count': 4,
        'orbit_arrival_radius_m': 1.5,
        'orbit_direction_by_buoy': {
            'green_buoy': 'cw',
            'black buoy': 'ccw',
            'red_buoy': 'cw',
        },
        'thrust_percentage': 40.0,
    },
    6: {  # 회피 시작점 접근 → 후진 → 대기 → status 7
        'gps': (None, None),
        'thrust_percentage': 20.0,
        'reverse_pwm': (1300, 1300),
        'reverse_seconds': 2.0,
        'wait_seconds': 5.0,
    },
    7: {  # V3 장애물 회피로 목표 GPS 추종
        'avoidance_distance_weight': 1.0,
        'avoidance_angle_weight': 20.0,
        'intermediate_gps': [(None, None)],  # [(위도1, 경도1), (위도2, 경도2), ...] 순서대로 통과
        'gps': (None, None),
        'thrust_percentage': 25.0,
    },
}
