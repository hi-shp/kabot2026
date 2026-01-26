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
from sensor_msgs.msg import NavSatFix, Imu, LaserScan
from std_msgs.msg import Float64, Float32, String
from tf_transformations import euler_from_quaternion

def constrain(v, lo, hi):
    if math.isnan(v): return lo + (hi - lo) / 2.0
    return lo if v < lo else hi if v > hi else v

class Course1(Node):
    def __init__(self):
        super().__init__("Course1")
        self.load_params_from_yaml()
        self.key_publisher = self.create_publisher(Float64, "/actuator/key/degree", 10)
        self.thruster_publisher = self.create_publisher(Float64, "/actuator/thruster/percentage", 10)
        self.dist_publisher = self.create_publisher(Float32, "/waypoint/distance", 10)
        self.rel_deg_publisher = self.create_publisher(Float32, "/waypoint/rel_deg", 10)
        self.goal_publisher = self.create_publisher(NavSatFix, "/waypoint/goal", 10)
        self.curr_yaw_publisher = self.create_publisher(Float32, "/current_yaw", 10)
        self.safe_angle_list_publisher = self.create_publisher(String, "/safe_angles_list", 10)
        self.safe_angle_publisher = self.create_publisher(Float32, "/safe_angle", 10)
        self.imu_sub = self.create_subscription(Imu, "/imu", self.imu_callback, qos_profile_sensor_data)
        self.gps_sub = self.create_subscription(NavSatFix, "/gps/fix", self.gps_listener_callback, qos_profile_sensor_data)
        self.lidar_sub = self.create_subscription(LaserScan, "/scan", self.lidar_callback, qos_profile_sensor_data)
        self.safe_angles_list = []
        self.dist_threshold = 0.3 # 장애물로 인식할 거리 (m)
        self.side_margin = 10 # 장애물로 처리할 좌우 각도
        self.origin = None
        self.origin_set = False
        self.wp_index = 0
        self.current_goal_enu = None
        self.current_yaw_rel = 0.0
        self.initial_yaw_abs = None
        self.dist_to_goal_m = None
        self.goal_rel_deg = None  
        self.arrived_all = False
        self.cmd_key_degree = self.servo_neutral_deg
        self.cmd_thruster = 0.0
        self.create_timer(self.timer_period, self.timer_callback)

    def load_params_from_yaml(self):
        script_dir = os.path.dirname(os.path.realpath(__file__))
        yaml_path = os.path.join(script_dir, "isv_params.yaml")
        with open(yaml_path, "r") as file:
            params = yaml.safe_load(file)
        self.timer_period = float(params["node_settings"]["timer_period"])
        self.servo_neutral_deg = float(params["servo"]["neutral_deg"])
        self.servo_min_deg = float(params["servo"]["min_deg"])
        self.servo_max_deg = float(params["servo"]["max_deg"])
        nav = params["navigation"]
        self.waypoints = nav["waypoints"] 
        self.arrival_radii = nav.get("arrival_radius", [1.0]) 
        self.thruster_cfg = params["state"]
        self.default_thruster = float(self.thruster_cfg.get("state0", 10.0))

    def normalize_180(self, deg):
        return (deg + 180.0) % 360.0 - 180.0

    def gps_enu_converter(self, lla):
        if self.origin is None:
            return 0.0, 0.0
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
        current_yaw_abs = -yaw_rad 
        if self.initial_yaw_abs is None:
            self.initial_yaw_abs = current_yaw_abs
        rel_yaw_deg = self.normalize_180(degrees(current_yaw_abs - self.initial_yaw_abs))
        self.current_yaw_rel = rel_yaw_deg
        self.curr_yaw_publisher.publish(Float32(data=float(rel_yaw_deg)))

    def lidar_callback(self, data):
        ranges = np.array(data.ranges)
        start_idx, end_idx = 500, 1500
        subset = ranges[start_idx:end_idx]
        cumulative_distance = np.zeros(181)
        sample_count = np.zeros(181)
        dist_180 = np.zeros(181)
        for i in range(len(subset)):
            length = subset[i]
            if length <= data.range_min or length >= data.range_max or not np.isfinite(length):
                continue
            angle_index = round((len(subset) - 1 - i) * 180 / len(subset))
            if 0 <= angle_index <= 180:
                cumulative_distance[angle_index] += length
                sample_count[angle_index] += 1
        for j in range(181):
            if sample_count[j] > 0:
                dist_180[j] = cumulative_distance[j] / sample_count[j]
            else:
                dist_180[j] = 0.0
        danger_flags = (dist_180 > 0) & (dist_180 <= self.dist_threshold)
        expanded_danger = np.copy(danger_flags)
        n = self.side_margin
        for i in range(181):
            if danger_flags[i]:
                low = max(0, i - n)
                high = min(181, i + n + 1)
                expanded_danger[low:high] = True
        self.safe_angles_list = [int(deg) for deg in range(181) if not expanded_danger[deg]]

        if self.safe_angles_list is not None:
            list_msg = String()
            list_msg.data = str(self.safe_angles_list)
            self.safe_angle_list_publisher.publish(list_msg)
        if self.safe_angles_list:
            safe_arr = np.array(self.safe_angles_list)
            best_angle = safe_arr[np.argmin(np.abs(safe_arr - 90))]
            self.safe_angle_publisher.publish(Float32(data=float(best_angle)))

    def gps_listener_callback(self, gps: NavSatFix):
        if math.isnan(gps.latitude) or math.isnan(gps.longitude) or self.initial_yaw_abs is None:
            return
        if not self.origin_set:
            self.origin = [gps.latitude, gps.longitude, gps.altitude]
            self.origin_set = True
            self.get_logger().info(f"시작 위치: {self.origin[:2]}")
            self.update_current_goal()
        curr_e, curr_n = self.gps_enu_converter([gps.latitude, gps.longitude, gps.altitude])
        if self.current_goal_enu is not None:
            goal_e, goal_n = self.current_goal_enu
            dx, dy = goal_e - curr_e, goal_n - curr_n
            self.dist_to_goal_m = math.hypot(dx, dy)
            target_ang_abs = degrees(math.atan2(dx, dy))
            target_ang_rel = self.normalize_180(target_ang_abs - degrees(self.initial_yaw_abs))
            self.goal_rel_deg = self.normalize_180(target_ang_rel - self.current_yaw_rel)

    def update_current_goal(self):
        if self.wp_index < len(self.waypoints):
            target_lat, target_lon = self.waypoints[self.wp_index]
            self.current_goal_enu = self.gps_enu_converter([target_lat, target_lon, 0.0])
            goal_msg = NavSatFix()
            goal_msg.latitude = target_lat
            goal_msg.longitude = target_lon
            self.goal_publisher.publish(goal_msg)
            self.get_logger().info(f"웨이포인트 목표: {self.wp_index+1}/{len(self.waypoints)}")
        else:
            self.current_goal_enu = None
            self.arrived_all = True
            self.get_logger().info("모든 웨이포인트 도착")

    def timer_callback(self):
        if self.arrived_all or self.dist_to_goal_m is None or self.goal_rel_deg is None:
            self.cmd_thruster = 0.0
            self.cmd_key_degree = self.servo_neutral_deg
        else:
            current_radius = self.arrival_radii[min(self.wp_index, len(self.arrival_radii)-1)]
            if self.dist_to_goal_m <= current_radius:
                self.wp_index += 1
                self.update_current_goal()
                return

            if self.safe_angles_list:
                safe_angles_deg = np.array(self.safe_angles_list) - 90
                diff = np.abs(safe_angles_deg - self.goal_rel_deg)
                best_idx = np.argmin(diff)
                chosen_safe_angle = safe_angles_deg[best_idx]
                steering_angle = self.servo_neutral_deg + chosen_safe_angle
                state_key = f"state{self.wp_index}"
                self.cmd_thruster = float(self.thruster_cfg.get(state_key, self.default_thruster))
                
                self.cmd_key_degree = normalize_180ain(
                    steering_angle, 
                    self.servo_min_deg, 
                    self.servo_max_deg)
            else:
                self.cmd_thruster = 0.0
                self.cmd_key_degree = self.servo_neutral_deg

        self.key_publisher.publish(Float64(data=float(self.cmd_key_degree)))
        self.thruster_publisher.publish(Float64(data=float(self.cmd_thruster)))
        if self.dist_to_goal_m is not None:
            self.dist_publisher.publish(Float32(data=float(self.dist_to_goal_m)))
        if self.goal_rel_deg is not None:
            self.rel_deg_publisher.publish(Float32(data=float(self.goal_rel_deg)))

    def send_stop_commands(self):
        if not rclpy.ok(): return
        safe_key = Float64(data=float(self.servo_neutral_deg))
        safe_thruster = Float64(data=0.0)
        for _ in range(5):
            self.key_publisher.publish(safe_key)
            self.thruster_publisher.publish(safe_thruster)
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = Course1()
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