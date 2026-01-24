#!/usr/bin/env python3
import os, yaml, math, time, sys, signal
from math import degrees
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float64, Float32
from tf_transformations import euler_from_quaternion

def constrain(v, lo, hi):
    if math.isnan(v): return lo + (hi - lo) / 2.0
    return lo if v < lo else hi if v > hi else v

class GPSPursueNode(Node):
    def __init__(self):
        super().__init__("gps_pursue_node")
        self._load_params_from_yaml()
        
        # Publishers
        self.key_publisher = self.create_publisher(Float64, "/actuator/key/degree", 10)
        self.thruster_publisher = self.create_publisher(Float64, "/actuator/thruster/percentage", 10)
        self.dist_publisher = self.create_publisher(Float32, "/waypoint/distance", 10)
        self.rel_deg_publisher = self.create_publisher(Float32, "/waypoint/rel_deg", 10)
        
        # Subscribers
        qos_profile = qos_profile_sensor_data
        qos_lidar = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Imu, "/imu", self.imu_callback, qos_profile)
        self.create_subscription(NavSatFix, "/gps/fix", self.gps_callback, qos_profile)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos_lidar)
        self.create_subscription(Float32, '/imu/yaw_refined', self.imu_current_callback, 10)
        
        # 주행 상태 변수
        self.origin = None 
        self.origin_set = False
        self.wp_index = 0
        self.current_goal_enu = None
        self.current_yaw_rel = 0.0
        self.initial_yaw_abs = None
        self.dist_to_goal_m = None
        self.goal_rel_deg = None  
        self.arrived_all = False
        self.max_risk_threshold = 60.0
        self.current_yaw = 0.0
        self.goal_rel_deg = 0.0
        self.key_target_degree = 0.0
        self.cmd_thruster = 0.0

        self.create_timer(self.timer_period_seconds, self.timer_callback)

    def _load_params_from_yaml(self):
        script_dir = os.path.dirname(os.path.realpath(__file__))
        yaml_path = os.path.join(script_dir, "isv_params.yaml")
        try:
            with open(yaml_path, "r", encoding='utf-8') as file:
                params = yaml.safe_load(file)
        except Exception as e:
            sys.exit(1)
        self.timer_period_seconds = params["node_settings"]["timer_period"]
        servo = params["servo"]
        self.servo_neutral_deg = float(servo["neutral_deg"])
        self.servo_min_deg = float(servo["min_deg"])
        self.servo_max_deg = float(servo["max_deg"])
        nav = params["navigation"]
        self.waypoints = nav["waypoints"] 
        self.arrival_radius = nav["arrival_radius"]
        self.state_cfg = params["state"]

    def set_risk_zone(self, array, center, spread):
        array[center] = 1
        for i in range(1, spread + 1):
            if center - i >= 0:
                array[center - i] = 1
            if center + i <= 180:
                array[center + i] = 1
        return array

    def normalize_180(self, deg):
        return (deg + 180.0) % 360.0 - 180.0

    def gps_converter(self, lla):
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

        if self.initial_yaw_abs is None:
            self.initial_yaw_abs = current_yaw_abs

        self.current_yaw_rel = self.normalize_180(degrees(current_yaw_abs - self.initial_yaw_abs))

    def imu_current_callback(self, msg):
        self.current_yaw = msg.data

    def lidar_callback(self, msg):
        ranges = np.array(msg.ranges)
        relevant_msg = ranges[500:1500]
        relevant_msg = relevant_msg[
            (relevant_msg != 0) & (relevant_msg != float("inf"))
        ]
        cumulative_distance = np.zeros(181)
        sample_count = np.zeros(181)
        average_distance = np.zeros(181)
        risk_values = np.zeros(181)
        risk_map = np.zeros(181)

        for i in range(len(relevant_msg)):
            length = relevant_msg[i]
            angle_index = round((len(relevant_msg) - 1 - i) * 180 / len(relevant_msg))
            cumulative_distance[angle_index] += length
            sample_count[angle_index] += 1

        for j in range(181):
            if sample_count[j] != 0:
                average_distance[j] = cumulative_distance[j] / sample_count[j]

        for k in range(181):
            if average_distance[k] != 0:
                risk_values[k] = 135.72 * math.exp(-0.6109 * average_distance[k])

        for k in range(181):
            if risk_values[k] >= self.max_risk_threshold:
                self.set_risk_zone(risk_map, k, 10)

        safe_angles = np.where(risk_map == 0)[0].tolist()

        if len(safe_angles) > 0:
            heading = float(min(safe_angles, key=lambda x: abs(x - (self.goal_rel_deg+90.0))))
        else:
            heading = 0.0 # 안전한 각도가 없을 때?

        if heading > self.servo_max_deg:
            heading = self.servo_max_deg
        if heading < self.servo_min_deg:
            heading = self.servo_min_deg

        self.key_target_degree = heading
        print(safe_angles)

    def gps_callback(self, gps: NavSatFix):
        if math.isnan(gps.latitude) or math.isnan(gps.longitude): return
        if not self.origin_set:
            self.origin = [gps.latitude, gps.longitude]
            self.origin_set = True
            self.get_logger().info(f"📍 현재 위치: {self.origin}")
            self.update_current_goal()
            return

        if self.initial_yaw_abs is None: return

        curr_e, curr_n = self.gps_converter([gps.latitude, gps.longitude])
        
        if self.current_goal_enu is not None:
            goal_e, goal_n = self.current_goal_enu
            dx, dy = goal_e - curr_e, goal_n - curr_n
            
            self.dist_to_goal_m = math.hypot(dx, dy)
            target_ang_abs = degrees(math.atan2(dy, dx))
            target_ang_rel = self.normalize_180(target_ang_abs - degrees(self.initial_yaw_abs))
            self.goal_rel_deg = self.normalize_180(target_ang_rel - self.current_yaw_rel) - 90.0

    def update_current_goal(self):
        if self.wp_index < len(self.waypoints):
            target_lat, target_lon = self.waypoints[self.wp_index]
            self.current_goal_enu = self.gps_converter([target_lat, target_lon])
            radius = self.arrival_radius[self.wp_index] if self.wp_index < len(self.arrival_radius) else 1.0
        else:
            self.current_goal_enu = None
            self.arrived_all = True
            self.get_logger().info("모든 목적지 도착 완료")

    def timer_callback(self):
        if self.arrived_all or self.dist_to_goal_m is None or self.goal_rel_deg is None:
            self.cmd_key_degree = self.servo_neutral_deg
        else:
            # 수정된 부분: 현재 WP의 도착 반경을 안전하게 가져옵니다.
            current_radius = self.arrival_radius[self.wp_index] if self.wp_index < len(self.arrival_radius) else 1.0
            
            if self.dist_to_goal_m <= current_radius: # radius 대신 current_radius 사용
                self.wp_index += 1
                self.update_current_goal()
                return
            
            # 조향 명령: 중립 + 오차(P제어)
            # goal_rel_deg가 (+)면 왼쪽으로 조향, (-)면 오른쪽으로 조향
            self.cmd_key_degree = constrain(
                self.servo_neutral_deg + self.goal_rel_deg,
                self.servo_min_deg,
                self.servo_max_deg
            )

        self.key_publisher.publish(Float64(data=float(self.cmd_key_degree)))
        self.thruster_publisher.publish(Float64(data=float(self.cmd_thruster)))
        
        if self.dist_to_goal_m is not None:
            self.dist_publisher.publish(Float32(data=float(self.dist_to_goal_m)))
        if self.goal_rel_deg is not None:
            self.rel_deg_publisher.publish(Float32(data=float(self.goal_rel_deg)))


    def send_stop_commands(self):
        if not rclpy.ok(): return
        for _ in range(5):
            self.key_publisher.publish(Float64(data=self.servo_neutral_deg))
            self.thruster_publisher.publish(Float64(data=0.0))
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = GPSPursueNode()
    
    def signal_handler(sig, frame):
        node.get_logger().warn("Stopped")
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