#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Float32, Float64
from sensor_msgs.msg import NavSatFix, Imu
from tf_transformations import euler_from_quaternion

class ControlPanelNode(Node):
    def __init__(self):
        super().__init__('control_panel_node')

        # ── 상태 변수 (데이터 유지용) ──────────────────────────────────────────
        self.current_gps = ("-", "-")
        self.goal_gps = ("-", "-")
        self.dist_to_goal = -1.0
        self.rel_deg_to_goal = 0.0
        self.current_yaw = 0.0
        self.cmd_servo = 90.0
        self.cmd_thruster = 0.0
        self.lidar_steer = 0.0

        self.has_fix = False
        self.has_goal = False
        self.has_dist = False

        # ── 구독 설정 ──────────────────────────────────────────────────────────
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_cb, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, '/waypoint/goal', self.goal_cb, 10)
        self.create_subscription(Float32, '/waypoint/distance', self.dist_cb, 10)
        self.create_subscription(Float32, '/waypoint/rel_deg', self.rel_deg_cb, 10)
        self.create_subscription(Imu, "/imu", self.imu_cb, qos_profile_sensor_data)
        self.create_subscription(Float64, "/actuator/key/degree", self.servo_cb, 10)
        self.create_subscription(Float64, "/actuator/thruster/percentage", self.thruster_cb, 10)
        self.create_subscription(Float32, "/lidar/risk_angle", self.lidar_cb, 10)

        # ── 타이머 설정 (스레드 대신 10Hz로 화면 갱신) ──────────────────────────
        self.create_timer(0.1, self.display_callback)

    def gps_cb(self, msg):
        if not math.isnan(msg.latitude):
            self.current_gps = (f"{msg.latitude:.7f}", f"{msg.longitude:.7f}")
            self.has_fix = True

    def goal_cb(self, msg):
        self.goal_gps = (f"{msg.latitude:.7f}", f"{msg.longitude:.7f}")
        self.has_goal = True

    def dist_cb(self, msg):
        self.dist_to_goal = float(msg.data)
        self.has_dist = True

    def rel_deg_cb(self, msg):
        self.rel_deg_to_goal = float(msg.data)

    def imu_cb(self, msg):
        q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        _, _, yaw_rad = euler_from_quaternion(q)
        # IMU 방향 반전 적용
        self.current_yaw = -math.degrees(yaw_rad)

    def servo_cb(self, msg):
        self.cmd_servo = msg.data

    def thruster_cb(self, msg):
        self.cmd_thruster = msg.data

    def lidar_cb(self, msg):
        self.lidar_steer = msg.data

    def display_callback(self):
        # 화면 제어용 특수 코드
        sys.stdout.write("\033[H") # 커서를 맨 위로
        EL = "\033[K" # 줄 지우기

        output = []
        output.append(f"┌────────────────────────────────────────────────────────────┐{EL}")
        output.append(f"│                🚢 KABOT MISSION MONITOR v2026              │{EL}")
        output.append(f"└────────────────────────────────────────────────────────────┘{EL}")

        # 1. 목표 정보
        goal_text = f"{self.goal_gps[0]}, {self.goal_gps[1]}" if self.has_goal else "⚠️ [WAITING FOR GOAL...]"
        output.append(f" 📍 Target Goal   : {goal_text}{EL}")

        dist_text = f"{self.dist_to_goal:6.2f} m" if self.has_dist else "⚠️ --- m"
        output.append(f" 📏 Distance      : {dist_text}{EL}")

        if not self.has_fix:
            output.append(f" 🧭 Target REL    : ⚠️ --- °{EL}")
        else: 
            side = "PORT(L)" if self.rel_deg_to_goal < 0 else "STBD(R)"
            output.append(f" 🧭 Target REL    : {abs(self.rel_deg_to_goal):6.1f}° [{side}]{EL}")

        output.append(f"──────────────────────────────────────────────────────────────{EL}")

        # 2. 현재 상태
        pos_text = f"{self.current_gps[0]}, {self.current_gps[1]}" if self.has_fix else "WAITING..."
        output.append(f" 📡 Current Pos   : {pos_text}{EL}")
        output.append(f" 🧭 Current Yaw   : {self.current_yaw:6.1f}°{EL}")

        output.append(f"──────────────────────────────────────────────────────────────{EL}")

        # 3. 제어 값 (0도 중심)
        display_angle = self.cmd_servo - 90.0
        bar_size = 10
        norm_pos = display_angle / 180.0
        idx = int(norm_pos * bar_size)
        bar = ["-"] * (bar_size * 2 + 1)
        target_idx = bar_size + max(-bar_size, min(idx, bar_size))
        bar[target_idx] = "█"
        
        output.append(f" ⚓ Servo Angle   : {display_angle:6.1f}°   [ {''.join(bar)} ]{EL}")
        output.append(f" 🚀 Thruster      : {self.cmd_thruster:6.1f}%    {'#' * int(abs(self.cmd_thruster)/5)}{EL}")

        output.append(f"──────────────────────────────────────────────────────────────{EL}")

        # 4. 장애물
        s = self.lidar_steer
        if abs(s) < 0.1: lidar_msg = "SAFE (Path Clear)"
        elif s > 0: lidar_msg = f"AVOIDING LEFT  (+{s:4.1f}°)"
        else: lidar_msg = f"AVOIDING RIGHT ({s:4.1f}°)"
        output.append(f" ⚠️ LiDAR Risk    : {lidar_msg}{EL}")

        output.append(f"──────────────────────────────────────────────────────────────{EL}")
        
        # 전체 내용을 한 번에 출력
        sys.stdout.write("\n".join(output) + "\n")
        sys.stdout.flush()

def main(args=None):
    rclpy.init(args=args)
    # 처음 실행 시 화면을 한 번 깔끔하게 지움
    sys.stdout.write("\033[2J\033[H")
    node = ControlPanelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()