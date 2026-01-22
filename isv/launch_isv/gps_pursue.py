#!/usr/bin/env python3
import os
import yaml
import math
import time
import sys
import signal
from math import degrees

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    qos_profile_sensor_data,
)

import numpy as np
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float64
from mechaship_interfaces.msg import RgbwLedColor
from tf_transformations import euler_from_quaternion

def constrain(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

class GPSPursueNode(Node):
    def __init__(self):
        super().__init__("gps_pursue_node")

        self._load_params_from_yaml()

        # 통신 설정
        qos_profile = qos_profile_sensor_data

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, "/imu", self.imu_callback, qos_profile
        )
        self.gps_sub = self.create_subscription(
            NavSatFix, "/gps/fix", self.gps_listener_callback, qos_profile
        )

        # Publishers
        self.key_publisher = self.create_publisher(Float64, "/actuator/key/degree", 10)
        self.thruster_publisher = self.create_publisher(Float64, "/actuator/thruster/percentage", 10)
        self.led_publisher = self.create_publisher(RgbwLedColor, "/actuator/rgbwled/color", 10)

        # Timer (제어 루프 실행)
        self.create_timer(self.timer_period_seconds, self.timer_callback)

        # 상태 변수 초기화
        self.origin = [self.origin_lat, self.origin_lon, 0.0]
        self.origin_set = False
        
        self.waypoints = list(zip(self.goal_gps_lats, self.goal_gps_lons))
        self.wp_index = 0
        self.current_goal_enu = None
        
        self.yaw_rad = None
        self.dist_to_goal_m = None
        self.goal_rel_deg = None  
        self.arrival_radius_m = 3.0  # 도착 인정 범위 (3m)
        self.arrived_all = False

        self.cmd_key_degree = self.servo_neutral_deg
        self.cmd_thruster = 0.0
        self._last_log_time = 0.0

    def _load_params_from_yaml(self):
        # YAML 설정 파일 로드
        script_dir = os.path.dirname(os.path.realpath(__file__))
        yaml_path = os.path.join(script_dir, "isv_params.yaml")
        
        with open(yaml_path, "r") as file:
            params = yaml.safe_load(file)

        self.timer_period_seconds = params["node_settings"]["timer_period_seconds"]
        self.servo_neutral_deg = float(params["servo"]["neutral_deg"])
        self.servo_min_deg = float(params["servo"]["min_deg"])
        self.servo_max_deg = float(params["servo"]["max_deg"])

        nav = params["navigation"]
        self.origin_lat = float(nav["origin"]["lat"])
        self.origin_lon = float(nav["origin"]["lon"])
        self.goal_gps_lats = nav["goal_gps_lats"]
        self.goal_gps_lons = nav["goal_gps_lons"]

        sm = params["state_machine"]["thruster_defaults"]
        self.default_thruster = float(sm["status0"])
        self.led_brightness = int(params["led"]["default_brightness"])

    def normalize_180(self, deg):
        """각도를 -180 ~ 180 사이로 정규화"""
        return (deg + 180.0) % 360.0 - 180.0

    def gps_enu_converter(self, lla):
        """GPS 좌표를 지역 미터 좌표(ENU)로 변환"""
        lat, lon, _ = lla
        lat0, lon0, _ = self.origin
        R = 6378137.0
        dlat = math.radians(lat - lat0)
        dlon = math.radians(lon - lon0)
        latm = math.radians((lat + lat0) * 0.5)
        return dlon * R * math.cos(latm), dlat * R

    def imu_callback(self, msg: Imu):
        q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        _, _, yaw_rad = euler_from_quaternion(q)
        # [중요] 사용자의 요청에 따른 좌우 반전 처리
        self.yaw_rad = -yaw_rad

    def gps_listener_callback(self, gps: NavSatFix):
        if not self.origin_set:
            self.origin_set = True
            self.update_current_goal()

        curr_e, curr_n = self.gps_enu_converter([gps.latitude, gps.longitude, gps.altitude])
        
        if self.current_goal_enu is not None:
            goal_e, goal_n = self.current_goal_enu
            dx, dy = goal_e - curr_e, goal_n - curr_n
            self.dist_to_goal_m = math.hypot(dx, dy)

            # 상대 각도 계산 (선수 방향 0, 시계방향+)
            if self.yaw_rad is not None:
                vec_ang_ccw = math.atan2(dy, dx) # 목표 방향 벡터 (라디안)
                rel_ccw = vec_ang_ccw - self.yaw_rad
                self.goal_rel_deg = self.normalize_180(-degrees(rel_ccw))

    def update_current_goal(self):
        """다음 Waypoint로 목표 갱신"""
        if self.wp_index < len(self.waypoints):
            target_lat, target_lon = self.waypoints[self.wp_index]
            self.current_goal_enu = self.gps_enu_converter([target_lat, target_lon, 0.0])
        else:
            self.current_goal_enu = None
            self.arrived_all = True

    def timer_callback(self):
        # 모든 점에 도착했거나 데이터가 없으면 정지
        if self.arrived_all or self.dist_to_goal_m is None:
            self.cmd_thruster = 0.0
            self.cmd_key_degree = self.servo_neutral_deg
        else:
            # 현재 목표 지점 도착 판정
            if self.dist_to_goal_m <= self.arrival_radius_m:
                self.wp_index += 1
                self.update_current_goal()
                return

            # 조향 및 추진 제어
            self.cmd_thruster = self.default_thruster
            steer_error = self.goal_rel_deg if self.goal_rel_deg is not None else 0.0
            
            # 중립 각도에 오차 각도를 더해 출력 각도 결정
            self.cmd_key_degree = constrain(
                self.servo_neutral_deg + steer_error,
                self.servo_min_deg,
                self.servo_max_deg
            )

        # 명령 발행
        self.key_publisher.publish(Float64(data=float(self.cmd_key_degree)))
        self.thruster_publisher.publish(Float64(data=float(self.cmd_thruster)))

        # 0.2초마다 패널 업데이트
        now = time.time()
        if now - self._last_log_time >= 0.2:
            self._last_log_time = now
            self.display_status()

    def display_status(self):
        sys.stdout.write("\033[H\033[J")
        wp_status = f"{self.wp_index + 1}/{len(self.waypoints)}" if not self.arrived_all else "MISSION COMPLETE"
        dist_val = f"{self.dist_to_goal_m:6.2f}m" if self.dist_to_goal_m else "WAIT"
        rel_ang = f"{self.goal_rel_deg:6.1f}°" if self.goal_rel_deg else "WAIT"
        
        panel = (
            f"┌──────────────── [ gps_pursue_node ] ────────────────┐\n"
            f"│  Waypoint 진행도 : {wp_status:<30} │\n"
            f"│  목표 거리 (DIST) : {dist_val:<30} │\n"
            f"│  상대 각도 (REL)  : {rel_ang:<30} │\n"
            f"│  서보 각도 (SERVO): {self.cmd_key_degree:6.1f}°{' ':<24} │\n"
            f"│  추진력 (THRUST)  : {self.cmd_thruster:6.1f}%{' ':<24} │\n"
            f"└─────────────────────────────────────────────────────┘\n"
        )
        sys.stdout.write(panel)
        sys.stdout.flush()

    def stop_ship(self):
        """종료 시 안전을 위해 모든 출력 정지"""
        stop_key = Float64(data=float(self.servo_neutral_deg))
        stop_thruster = Float64(data=0.0)
        for _ in range(5):
            self.key_publisher.publish(stop_key)
            self.thruster_publisher.publish(stop_thruster)
            time.sleep(0.05)

def main(args=None):
    rclpy.init(args=args)
    node = GPSPursueNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (Ctrl+C)")
    finally:
        node.stop_ship()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()