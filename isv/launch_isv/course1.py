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
from rclpy.qos import qos_profile_sensor_data

import numpy as np
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float64, Float32
from tf_transformations import euler_from_quaternion

def constrain(v, lo, hi):
    """값을 최소/최대 범위 내로 제한하는 함수"""
    if math.isnan(v): return lo + (hi - lo) / 2.0
    return lo if v < lo else hi if v > hi else v

class GPSPursueNode(Node):
    def __init__(self):
        super().__init__("gps_pursue_node")
        
        # 1. 파라미터 로드
        self._load_params_from_yaml()
        
        # 2. 퍼블리셔 설정
        self.key_publisher = self.create_publisher(Float64, "/actuator/key/degree", 10)
        self.thruster_publisher = self.create_publisher(Float64, "/actuator/thruster/percentage", 10)
        self.dist_publisher = self.create_publisher(Float32, "/waypoint/distance", 10)
        self.rel_deg_publisher = self.create_publisher(Float32, "/waypoint/rel_deg", 10)
        self.goal_publisher = self.create_publisher(NavSatFix, "/waypoint/goal", 10)
        self.curr_yaw_publisher = self.create_publisher(Float32, "/current_yaw", 10)
        
        # 3. 서브스크라이버 설정 (Sensor Data QoS 사용)
        qos_profile = qos_profile_sensor_data
        self.imu_sub = self.create_subscription(Imu, "/imu", self.imu_callback, qos_profile)
        self.gps_sub = self.create_subscription(NavSatFix, "/gps/fix", self.gps_listener_callback, qos_profile)
        
        # 4. 상태 변수 초기화
        self.origin = None          # 현재 위치를 기준으로 설정될 LLA 원점
        self.origin_set = False     # 원점 설정 여부
        self.wp_index = 0           # 현재 추적 중인 웨이포인트 인덱스
        self.current_goal_enu = None
        self.current_yaw_rel = 0.0
        self.initial_yaw_abs = None # 시작 시점의 절대 방위
        self.dist_to_goal_m = None
        self.goal_rel_deg = None  
        self.arrived_all = False
        
        self.cmd_key_degree = self.servo_neutral_deg
        self.cmd_thruster = 0.0

        # 5. 제어 타이머 생성
        self.create_timer(self.timer_period_seconds, self.timer_callback)
        self.get_logger().info("🚀 GPS Pursue Node가 시작되었습니다. GPS 신호를 대기합니다...")

    def _load_params_from_yaml(self):
        """YAML 파일로부터 설정값을 읽어오는 함수"""
        script_dir = os.path.dirname(os.path.realpath(__file__))
        yaml_path = os.path.join(script_dir, "isv_params.yaml")
        
        try:
            with open(yaml_path, "r") as file:
                params = yaml.safe_load(file)
            
            # YAML 구조에 따른 매핑
            self.timer_period_seconds = float(params["node_settings"]["timer_period"])
            self.servo_neutral_deg = float(params["servo"]["neutral_deg"])
            self.servo_min_deg = float(params["servo"]["min_deg"])
            self.servo_max_deg = float(params["servo"]["max_deg"])
            
            nav = params["navigation"]
            self.waypoints = nav["waypoints"] 
            self.arrival_radii = nav.get("arrival_radius", [1.0, 1.0])
            
            self.thruster_cfg = params["state"]
            
        except Exception as e:
            self.get_logger().error(f"파라미터 로드 실패: {e}")
            sys.exit(1)

    def normalize_180(self, deg):
        """각도를 -180 ~ 180도 사이로 정규화"""
        return (deg + 180.0) % 360.0 - 180.0

    def gps_enu_converter(self, lla):
        """LLA(위경도)를 설정된 Origin 기준의 ENU(평면 좌표)로 변환"""
        if self.origin is None:
            return 0.0, 0.0
        
        lat, lon, _ = lla
        lat0, lon0, _ = self.origin
        R = 6378137.0 # 지구 반경 (m)
        
        dlat = math.radians(lat - lat0)
        dlon = math.radians(lon - lon0)
        latm = math.radians((lat + lat0) * 0.5)
        
        x = dlon * R * math.cos(latm)
        y = dlat * R
        return x, y

    def imu_callback(self, msg: Imu):
        """IMU 데이터를 받아 현재 Yaw(방위각) 계산"""
        q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        _, _, yaw_rad = euler_from_quaternion(q)
        
        # 센서 좌표계에 따라 부호 조정 (- 붙임)
        current_yaw_abs = -yaw_rad 

        if self.initial_yaw_abs is None:
            self.initial_yaw_abs = current_yaw_abs
            self.get_logger().info(f"🧭 초기 방위 설정 완료: {degrees(self.initial_yaw_abs):.2f}°")

        # 시작 시점 방위를 0도로 하는 상대 방위 계산
        rel_yaw_deg = self.normalize_180(degrees(current_yaw_abs - self.initial_yaw_abs))
        self.current_yaw_rel = rel_yaw_deg
        self.curr_yaw_publisher.publish(Float32(data=float(rel_yaw_deg)))

    def gps_listener_callback(self, gps: NavSatFix):
        """GPS 데이터를 받아 목적지까지의 거리와 각도 계산"""
        if math.isnan(gps.latitude) or math.isnan(gps.longitude):
            return

        # [중요] 최초 위치를 Origin으로 설정
        if not self.origin_set:
            if self.initial_yaw_abs is None:
                self.get_logger().warn("IMU 데이터가 아직 없습니다. 대기 중...")
                return
            self.origin = [gps.latitude, gps.longitude, gps.altitude]
            self.origin_set = True
            self.get_logger().info(f"🛰️ Origin 설정 완료! (현재 위치 기준): {self.origin[:2]}")
            self.update_current_goal()

        # 현재 위치를 ENU 좌표로 변환
        curr_e, curr_n = self.gps_enu_converter([gps.latitude, gps.longitude, gps.altitude])
        
        if self.current_goal_enu is not None:
            goal_e, goal_n = self.current_goal_enu
            dx, dy = goal_e - curr_e, goal_n - curr_n
            
            # 거리 계산
            self.dist_to_goal_m = math.hypot(dx, dy)
            
            # 목표 지점의 절대 각도 (지도 기준)
            target_ang_abs = degrees(math.atan2(dy, dx))
            
            # 목표 지점의 상대 각도 (로봇 초기 시작 방향 기준)
            target_ang_rel = self.normalize_180(target_ang_abs - degrees(self.initial_yaw_abs))
            
            # 로봇 현재 정면 대비 꺾어야 할 각도 (목표 상대 각도 - 현재 로봇 상대 각도)
            self.goal_rel_deg = self.normalize_180(target_ang_rel - self.current_yaw_rel)

    def update_current_goal(self):
        """다음 웨이포인트로 목표 갱신"""
        if self.wp_index < len(self.waypoints):
            target_lat, target_lon = self.waypoints[self.wp_index]
            # 위경도 좌표를 ENU 좌표로 변환하여 저장
            self.current_goal_enu = self.gps_enu_converter([target_lat, target_lon, 0.0])
            
            # 가시화를 위한 목표 GPS 퍼블리시
            goal_msg = NavSatFix()
            goal_msg.latitude = target_lat
            goal_msg.longitude = target_lon
            self.goal_publisher.publish(goal_msg)
            self.get_logger().info(f"🎯 목표 갱신: WP {self.wp_index} ({target_lat}, {target_lon})")
        else:
            self.current_goal_enu = None
            self.arrived_all = True
            self.get_logger().info("🏁 모든 목표 지점에 도착하였습니다!")

    def timer_callback(self):
        """주기적인 제어 명령 계산 및 발행"""
        if not self.origin_set or self.arrived_all or self.dist_to_goal_m is None:
            self.cmd_thruster = 0.0
            self.cmd_key_degree = self.servo_neutral_deg
        else:
            # 1. 도착 판정 (현재 인덱스에 맞는 반경 적용)
            current_radius = self.arrival_radii[min(self.wp_index, len(self.arrival_radii)-1)]
            
            if self.dist_to_goal_m <= current_radius:
                self.get_logger().info(f"✅ WP {self.wp_index} 도달! (거리: {self.dist_to_goal_m:.2f}m)")
                self.wp_index += 1
                self.update_current_goal()
                return

            # 2. 추진력 설정 (state0, state1...)
            state_key = f"state{self.wp_index}"
            self.cmd_thruster = float(self.thruster_cfg.get(state_key, 10.0))

            # 3. 조향 각도 계산 (중립 각도 + 목표 오차 각도)
            # 오차 각도(goal_rel_deg)만큼 서보를 회전
            self.cmd_key_degree = constrain(
                self.servo_neutral_deg + self.goal_rel_deg,
                self.servo_min_deg,
                self.servo_max_deg
            )

        # 4. 명령 발행
        self.key_publisher.publish(Float64(data=float(self.cmd_key_degree)))
        self.thruster_publisher.publish(Float64(data=float(self.cmd_thruster)))
        
        # 상태 정보 발행 (디버깅용)
        if self.dist_to_goal_m is not None:
            self.dist_publisher.publish(Float32(data=float(self.dist_to_goal_m)))
        if self.goal_rel_deg is not None:
            self.rel_deg_publisher.publish(Float32(data=float(self.goal_rel_deg)))

    def send_stop_commands(self):
        """종료 시 안전을 위해 모터 정지 명령 송신"""
        self.get_logger().warn("모터 정지 명령 송신 중...")
        safe_key = Float64(data=float(self.servo_neutral_deg))
        safe_thruster = Float64(data=0.0)
        for _ in range(5):
            self.key_publisher.publish(safe_key)
            self.thruster_publisher.publish(safe_thruster)
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = GPSPursueNode()
    
    def signal_handler(sig, frame):
        node.send_stop_commands()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.send_stop_commands()
            node.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    main()