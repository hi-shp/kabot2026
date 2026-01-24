#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math
import os
import yaml
from math import degrees, radians, atan2, sin, cos, exp
from sensor_msgs.msg import NavSatFix, LaserScan, Imu
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class CourseOneNavigator(Node):
    def __init__(self):
        super().__init__('course_one_navigator')

        # 1. 파라미터 로드 (isv_params.yaml)
        self.load_params()

        # 2. 상태 변수 초기화
        self.current_pos = {"lat": 0.0, "lon": 0.0}
        self.imu_heading = 0.0
        self.safe_indices = []
        self.target_wp_idx = 0  # 1코스 목표는 첫 번째 웨이포인트

        # 3. QoS 설정 (전달받은 예제 기준)
        lidar_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        gps_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        imu_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)

        # 4. 구독 및 발행
        self.create_subscription(NavSatFix, "/gps/fix", self.gps_cb, gps_qos)
        self.create_subscription(LaserScan, "/scan", self.lidar_cb, lidar_qos)
        self.create_subscription(Imu, "/imu", self.imu_cb, imu_qos)
        
        self.key_pub = self.create_publisher(Float64, "/actuator/key/degree", 10)
        self.thruster_pub = self.create_publisher(Float64, "/actuator/thruster/percentage", 10)

        # 5. 제어 루프 타이머 (YAML의 timer_period_seconds 사용)
        self.create_timer(self.timer_period, self.control_loop)
        self.get_logger().info("Course 1 Navigator Initialized with YAML Params")

    def load_params(self):
        """YAML 파일에서 직접 파라미터 로드"""
        try:
            # 파일 경로 설정 (image_34a76d.png의 구조 반영)
            current_dir = os.path.dirname(os.path.abspath(__file__))
            yaml_path = os.path.join(current_dir, 'isv_params.yaml')
            
            with open(yaml_path, 'r', encoding='utf-8') as f:
                params = yaml.safe_load(f)
            
            # 노드 설정
            self.timer_period = params['node_settings']['timer_period_seconds']
            
            # 서보 설정
            self.min_deg = params['servo']['min_deg']
            self.max_deg = params['servo']['max_deg']
            self.neutral_deg = params['servo']['neutral_deg']
            
            # 내비게이션 (첫 번째 웨이포인트를 WP1으로 설정)
            self.waypoints = params['navigation']['waypoints']
            self.arrival_radius = params['navigation']['arrival_radius_m']
            
            # 추진기 출력 (상태 0: 주행 시작 단계 출력 사용)
            self.default_thrust = params['state_machine']['thruster_defaults']['state0']
            
            # 라이다 위험도 관련 (작년 로직 수치 유지)
            self.max_risk_threshold = 60.0  # 작년 검증값
            self.ship_padding = 23         # 작년 검증값 (배 폭 고려)

        except Exception as e:
            self.get_logger().error(f"Failed to load YAML: {e}")
            # 로드 실패 시 기본값 강제 할당
            self.timer_period = 0.5
            self.neutral_deg = 90.0
            self.default_thrust = 10.0

    def get_bearing(self, lat1, lon1, lat2, lon2):
        dLon = radians(lon2 - lon1)
        y = sin(dLon) * cos(radians(lat2))
        x = cos(radians(lat1)) * sin(radians(lat2)) - sin(radians(lat1)) * cos(radians(lat2)) * cos(dLon)
        return (degrees(atan2(y, x)) + 360) % 360

    def gps_cb(self, msg):
        self.current_pos["lat"] = msg.latitude
        self.current_pos["lon"] = msg.longitude

    def imu_cb(self, msg):
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        # 90도 오프셋 좌표계 보정 필요 시 여기서 조정
        self.imu_heading = (degrees(atan2(siny_cosp, cosy_cosp)) + 360) % 360

    def lidar_cb(self, msg):
        num_ranges = len(msg.ranges)
        mid = num_ranges // 2
        half = num_ranges // 4  # 전방 180도

        relevant_data = np.array(msg.ranges[mid-half : mid+half])
        relevant_data = np.where((relevant_data <= msg.range_min) | (relevant_data >= msg.range_max), 0, relevant_data)

        # 181개로 보간(Interpolation)
        avg_distances = np.interp(np.linspace(0, len(relevant_data)-1, 181), 
                                  np.arange(len(relevant_data)), relevant_data)

        risk_map = np.zeros(181)
        for i, dist in enumerate(avg_distances):
            if dist > 0:
                # 작년 위험도 함수: y = 135.72 * e^(-0.6109x)
                risk = 135.72 * exp(-0.6109 * dist)
                if risk >= self.max_risk_threshold:
                    start = max(0, i - self.ship_padding)
                    end = min(180, i + self.ship_padding)
                    risk_map[start:end+1] = 1 

        self.safe_indices = np.where(risk_map == 0)[0]

    def control_loop(self):
        if self.current_pos["lat"] == 0.0 or not self.waypoints: return

        # 1. 목표 WP 설정 (첫 번째 웨이포인트)
        target = self.waypoints[self.target_wp_idx]
        
        # 2. 목표 방위각 계산
        target_bearing = self.get_bearing(self.current_pos["lat"], self.current_pos["lon"], target[0], target[1])
        
        # 3. 상대 각도 및 목표 인덱스 계산
        relative_angle = (target_bearing - self.imu_heading + 180) % 360 - 180
        desired_idx = 90 + relative_angle

        # 4. 장애물 회피 조향 결정
        if hasattr(self, 'safe_indices') and len(self.safe_indices) > 0:
            final_steering = float(self.safe_indices[np.abs(self.safe_indices - desired_idx).argmin()])
        else:
            final_steering = self.neutral_deg

        # YAML의 서보 한계값 적용
        final_steering = max(self.min_deg, min(self.max_deg, final_steering))

        # 5. 제어 명령 퍼블리시
        self.key_pub.publish(Float64(data=final_steering))
        self.thruster_pub.publish(Float64(data=self.default_thrust))

        # 6. WP 도달 판정 (거리 계산 생략, 도달 시 로깅)
        self.get_logger().info(f"Dist to WP: {target_bearing:.1f}deg, Steer: {final_steering:.1f}")

def main():
    rclpy.init()
    node = CourseOneNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()