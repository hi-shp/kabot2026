#!/usr/bin/env python3
import os, yaml, math, time, sys, signal
import numpy as np
from math import degrees, radians, exp
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from sensor_msgs.msg import NavSatFix, Imu, LaserScan
from std_msgs.msg import Float64, Float32
from tf_transformations import euler_from_quaternion

def constrain(v, lo, hi):
    if math.isnan(v): return lo + (hi - lo) / 2.0
    return lo if v < lo else hi if v > hi else v

def set_risk_zone(array, center, spread):
    """장애물 지점(center) 주변으로 위험 구역(spread)을 설정"""
    array[center] = 1
    for i in range(1, spread + 1):
        if center - i >= 0: array[center - i] = 1
        if center + i <= 180: array[center + i] = 1
    return array

class GPSLidarFusionNode(Node):
    def __init__(self):
        super().__init__("gps_lidar_fusion_node")
        self._load_params_from_yaml()
        
        # --- QoS 설정 (참고 코드 반영) ---
        lidar_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        
        # Publishers
        self.key_publisher = self.create_publisher(Float64, "/actuator/key/degree", 10)
        self.thruster_publisher = self.create_publisher(Float64, "/actuator/thruster/percentage", 10)
        self.dist_publisher = self.create_publisher(Float32, "/waypoint/distance", 10)
        self.rel_deg_publisher = self.create_publisher(Float32, "/waypoint/rel_deg", 10)
        
        # Subscribers
        self.imu_sub = self.create_subscription(Imu, "/imu", self.imu_callback, lidar_qos)
        self.gps_sub = self.create_subscription(NavSatFix, "/gps/fix", self.gps_listener_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, "/scan", self.lidar_callback, lidar_qos)
        
        # 주행 및 제어 변수
        self.origin = None 
        self.origin_set = False
        self.wp_index = 0
        self.current_goal_enu = None
        self.current_yaw_rel = 0.0
        self.initial_yaw_abs = None
        self.dist_to_goal_m = None
        self.goal_rel_deg = None  
        self.arrived_all = False
        
        # --- 라이다 및 위험도 관련 설정 (참고 코드 수치 적용) ---
        self.max_risk_threshold = 60.0  # 위험도 임계값
        self.safe_angles = list(range(181)) # 초기값은 전구간 안전
        self.final_heading = 90.0

        self.create_timer(self.timer_period_seconds, self.timer_callback)

    def _load_params_from_yaml(self):
        script_dir = os.path.dirname(os.path.realpath(__file__))
        yaml_path = os.path.join(script_dir, "isv_params.yaml")
        try:
            with open(yaml_path, "r", encoding='utf-8') as file:
                params = yaml.safe_load(file)
        except Exception as e:
            self.get_logger().error(f"YAML 로드 실패: {e}"); sys.exit(1)

        self.timer_period_seconds = params["node_settings"]["timer_period"]
        self.servo_neutral_deg = float(params["servo"]["neutral_deg"])
        self.servo_min_deg = float(params["servo"]["min_deg"])
        self.servo_max_deg = float(params["servo"]["max_deg"])
        self.waypoints = params["navigation"]["waypoints"] 
        self.arrival_radii = params["navigation"]["arrival_radius"]
        self.state_cfg = params["state"]

    def normalize_180(self, deg):
        return (deg + 180.0) % 360.0 - 180.0

    def gps_enu_converter(self, lla):
        if self.origin is None: return 0.0, 0.0
        lat, lon = lla[0], lla[1]
        lat0, lon0 = self.origin[0], self.origin[1]
        R = 6378137.0
        dlat, dlon = math.radians(lat - lat0), math.radians(lon - lon0)
        latm = math.radians((lat + lat0) * 0.5)
        return dlon * R * math.cos(latm), dlat * R

    def imu_callback(self, msg: Imu):
        q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        _, _, yaw_rad = euler_from_quaternion(q)
        current_yaw_abs = -yaw_rad 
        if self.initial_yaw_abs is None: self.initial_yaw_abs = current_yaw_abs
        self.current_yaw_rel = self.normalize_180(degrees(current_yaw_abs - self.initial_yaw_abs))

    def lidar_callback(self, data):
        """라이다 데이터를 처리하여 안전한 각도 리스트(safe_angles) 도출"""
        ranges = np.array(data.ranges)
        # 참고 코드의 전방 relevant_data 추출 (인덱스 500~1500)
        relevant_data = ranges[500:1500]
        relevant_data = relevant_data[(relevant_data != 0) & (relevant_data != float("inf"))]
        
        if len(relevant_data) == 0: return

        cumulative_distance = np.zeros(181)
        sample_count = np.zeros(181)
        average_distance = np.zeros(181)
        risk_values = np.zeros(181)
        risk_map = np.zeros(181)

        # 1. 거리 데이터를 180도 인덱스로 매핑
        for i in range(len(relevant_data)):
            length = relevant_data[i]
            angle_index = round((len(relevant_data) - 1 - i) * 180 / len(relevant_data))
            cumulative_distance[angle_index] += length
            sample_count[angle_index] += 1

        # 2. 평균 거리 및 위험도 계산 (작년 위험도 함수 적용)
        for j in range(181):
            if sample_count[j] != 0:
                average_distance[j] = cumulative_distance[j] / sample_count[j]
                # y = 135.72 * e^(-0.6109x)
                risk_values[j] = 135.72 * math.exp(-0.6109 * average_distance[j])

        # 3. 위험 구역 설정 (스프레드 23 적용)
        for k in range(181):
            if risk_values[k] >= self.max_risk_threshold:
                set_risk_zone(risk_map, k, 23)

        # 4. 안전한 각도 리스트 업데이트
        self.safe_angles = np.where(risk_map == 0)[0].tolist()

    def gps_listener_callback(self, gps: NavSatFix):
        if math.isnan(gps.latitude) or math.isnan(gps.longitude): return
        if not self.origin_set:
            self.origin = [gps.latitude, gps.longitude]
            self.origin_set = True
            self.update_current_goal(); return

        if self.initial_yaw_abs is None: return
        curr_e, curr_n = self.gps_enu_converter([gps.latitude, gps.longitude])
        
        if self.current_goal_enu is not None:
            goal_e, goal_n = self.current_goal_enu
            dx, dy = goal_e - curr_e, goal_n - curr_n
            self.dist_to_goal_m = math.hypot(dx, dy)
            target_ang_abs = degrees(math.atan2(dy, dx))
            target_ang_rel = self.normalize_180(target_ang_abs - degrees(self.initial_yaw_abs))
            # GPS가 원하는 상대 각도 (90도가 정면인 체계로 변환)
            self.goal_rel_deg = self.normalize_180(target_ang_rel - self.current_yaw_rel) + 90.0

    def update_current_goal(self):
        if self.wp_index < len(self.waypoints):
            target_lat, target_lon = self.waypoints[self.wp_index]
            self.current_goal_enu = self.gps_enu_converter([target_lat, target_lon])
            radius = self.arrival_radii[self.wp_index] if self.wp_index < len(self.arrival_radii) else 1.0
            self.get_logger().info(f"🎯 WP {self.wp_index} 목표: {target_lat}, {target_lon}")
        else:
            self.current_goal_enu = None; self.arrived_all = True
            self.get_logger().info("🏁 모든 목적지 도착 완료")

    def timer_callback(self):
        if not self.origin_set or self.arrived_all or self.dist_to_goal_m is None or self.goal_rel_deg is None:
            cmd_t, cmd_k = 0.0, self.servo_neutral_deg
        else:
            # WP 도착 판정
            current_radius = self.arrival_radii[self.wp_index] if self.wp_index < len(self.arrival_radii) else 1.0
            if self.dist_to_goal_m <= current_radius:
                self.wp_index += 1; self.update_current_goal(); return

            # 1. GPS 기반 목적지 각도 (desired_heading)
            desired_heading = self.goal_rel_deg 

            # 2. 세이프 존 중 목적지와 가장 가까운 각도 선택
            if len(self.safe_angles) > 0:
                self.final_heading = float(min(self.safe_angles, key=lambda x: abs(x - desired_heading)))
            else:
                self.final_heading = 90.0 # 안전 구역 없으면 정면 (또는 정지 로직)

            # 3. 조향 제한 (45~135도)
            self.final_heading = constrain(self.final_heading, 45.0, 135.0)

            # 4. 최종 출력 각도 계산 (중립 90도 기준 보정)
            # cmd_k = self.servo_neutral_deg + (self.final_heading - 90.0)
            # 참고 코드 방식에 따라 final_heading 자체가 출력 각도로 사용됨
            cmd_k = self.final_heading
            
            state_key = f"state{self.wp_index}"
            cmd_t = float(self.state_cfg.get(state_key, 20.0))

        # 발행
        self.key_publisher.publish(Float64(data=float(cmd_k)))
        self.thruster_publisher.publish(Float64(data=float(cmd_t)))
        self.dist_publisher.publish(Float32(data=float(self.dist_to_goal_m or 0.0)))
        self.rel_deg_publisher.publish(Float32(data=float(self.final_heading - 90.0)))

    def send_stop_commands(self):
        if not rclpy.ok(): return
        for _ in range(5):
            self.key_publisher.publish(Float64(data=self.servo_neutral_deg))
            self.thruster_publisher.publish(Float64(data=0.0))
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = GPSLidarFusionNode()
    def signal_handler(sig, frame):
        node.send_stop_commands(); node.destroy_node(); rclpy.shutdown(); sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if rclpy.ok(): node.send_stop_commands(); node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()