#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Float32, Float64
from sensor_msgs.msg import NavSatFix, Imu
from tf_transformations import euler_from_quaternion

class ControlPanelNode(Node):
    def __init__(self):
        super().__init__('control_panel_node')

        # ── 상태 변수 ─────────────────────────────────────────────────────────
        self.current_gps = ("-", "-")
        self.goal_gps = ("-", "-")
        self.dist_to_goal = -1.0
        self.rel_deg_to_goal = 0.0
        self.current_yaw = 0.0
        self.cmd_servo = 90.0
        self.cmd_thruster = 0.0
        self.lidar_steer = 0.0

        # 데이터 수신 확인용 타임스탬프
        self.last_received = {k: 0.0 for k in ['fix', 'goal', 'dist', 'rel_deg', 'imu', 'servo', 'thruster']}

        # ── 구독 설정 (Subscribers) ──────────────────────────────────────────
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_cb, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, '/waypoint/goal', self.goal_cb, 10)
        self.create_subscription(Float32, '/waypoint/distance', self.dist_cb, 10)
        self.create_subscription(Float32, '/waypoint/rel_deg', self.rel_deg_cb, 10)
        self.create_subscription(Imu, "/imu", self.imu_cb, qos_profile_sensor_data)
        self.create_subscription(Float64, "/actuator/key/degree", self.servo_cb, 10)
        self.create_subscription(Float64, "/actuator/thruster/percentage", self.thruster_cb, 10)
        self.create_subscription(Float32, "/lidar/risk_angle", self.lidar_cb, 10)

        # UI 출력 스레드 시작
        self.print_thread = threading.Thread(target=self.display_loop, daemon=True)
        self.print_thread.start()

    # ── 콜백 함수 ─────────────────────────────────────────────────────────
    def gps_cb(self, msg):
        self.current_gps = (f"{msg.latitude:.7f}", f"{msg.longitude:.7f}")
        self.last_received['fix'] = time.monotonic()

    def goal_cb(self, msg):
        self.goal_gps = (f"{msg.latitude:.7f}", f"{msg.longitude:.7f}")
        self.last_received['goal'] = time.monotonic()

    def dist_cb(self, msg):
        self.dist_to_goal = float(msg.data)
        self.last_received['dist'] = time.monotonic()

    def rel_deg_cb(self, msg):
        self.rel_deg_to_goal = float(msg.data)
        self.last_received['rel_deg'] = time.monotonic()

    def imu_cb(self, msg):
        q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        _, _, yaw_rad = euler_from_quaternion(q)
        self.current_yaw = math.degrees(yaw_rad)
        self.last_received['imu'] = time.monotonic()

    def servo_cb(self, msg):
        self.cmd_servo = msg.data
        self.last_received['servo'] = time.monotonic()

    def thruster_cb(self, msg):
        self.cmd_thruster = msg.data
        self.last_received['thruster'] = time.monotonic()

    def lidar_cb(self, msg):
        self.lidar_steer = msg.data

    # ── 디스플레이 루프 ───────────────────────────────────────────────────────
    def display_loop(self):
        while rclpy.ok():
            now = time.monotonic()
            sys.stdout.write("\033[2J\033[H") # 화면 클리어

            print("┌────────────────────────────────────────────────────────────┐")
            print("│                🚢 KABOT MISSION MONITOR v2026              │")
            print("└────────────────────────────────────────────────────────────┘")

            # 1. 목표 항법 정보 (YAML 파라미터 기반 목표물 확인)
            print(f" 📍 Target Goal  : ", end="")
            if now - self.last_received['goal'] > 3.0: print("⚠️ [WAITING FOR GOAL...]")
            else: print(f"{self.goal_gps[0]}, {self.goal_gps[1]}")

            print(f" 📏 Distance     : ", end="")
            if now - self.last_received['dist'] > 3.0: print("⚠️ --- m")
            else: print(f"{self.dist_to_goal:6.2f} m")

            print(f" 🧭 Target REL   : ", end="")
            if now - self.last_received['rel_deg'] > 3.0: print("⚠️ --- °")
            else: 
                side = "PORT(L)" if self.rel_deg_to_goal < 0 else "STBD(R)"
                print(f"{abs(self.rel_deg_to_goal):6.1f}° [{side}]")

            print("──────────────────────────────────────────────────────────────")

            # 2. 센서 데이터 정보
            print(f" 📡 Current Pos  : {self.current_gps[0]}, {self.current_gps[1]}")
            print(f" 🧭 Current Yaw  : {self.current_yaw:6.1f}°")

            print("──────────────────────────────────────────────────────────────")

            # 3. 액추에이터 제어 정보
            print(f" ⚓ Servo Angle  : {self.cmd_servo:6.1f}° ", end="")
            # 서보 가시화 (45~135도 범위 반영)
            bar_size = 10
            normalized_pos = (self.cmd_servo - 90.0) / 45.0 # -1.0 ~ 1.0
            pos = int(normalized_pos * bar_size)
            bar = ["-"] * (bar_size * 2 + 1)
            bar[bar_size + max(-bar_size, min(pos, bar_size))] = "█"
            print(f"  [ {''.join(bar)} ]")

            print(f" 🚀 Thruster     : {self.cmd_thruster:6.1f}% ", end="")
            print(f" {'#' * int(abs(self.cmd_thruster)/5)}")

            print("──────────────────────────────────────────────────────────────")

            # 4. 장애물 회피 상태 (LiDAR)
            print(f" ⚠️ LiDAR Risk   : ", end="")
            s = self.lidar_steer
            if abs(s) < 0.1: print("SAFE (Path Clear)")
            elif s > 0: print(f"AVOIDING LEFT  (+{s:4.1f}°)")
            else: print(f"AVOIDING RIGHT ({s:4.1f}°)")

            print("──────────────────────────────────────────────────────────────")
            
            sys.stdout.flush()
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
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