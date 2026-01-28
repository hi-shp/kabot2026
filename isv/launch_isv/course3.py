#!/usr/bin/env python3
import rclpy
import time
import sys
import math
import signal
import numpy as np
import os
import yaml
from math import degrees, atan2
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import Float64, String
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from tf_transformations import euler_from_quaternion

def constrain(v, lo, hi):
    if math.isnan(v): return lo + (hi - lo) / 2.0
    return lo if v < lo else hi if v > hi else v

class Course3(Node):
    def __init__(self):
        super().__init__("Course3")
        self.load_params_from_yaml()
        self.create_subscription(LaserScan, "/scan", self.lidar_callback, qos_profile_sensor_data)
        self.create_subscription(Imu, "/imu", self.imu_callback, qos_profile_sensor_data)
        self.key_publisher = self.create_publisher(Float64, "/actuator/key/degree", 10)
        self.thruster_publisher = self.create_publisher(Float64, "/actuator/thruster/percentage", 10)
        self.curr_yaw_publisher = self.create_publisher(Float64, "/current_yaw", 10)
        self.safe_angle_list_publisher = self.create_publisher(String, "/safe_angles_list", 10)
        self.max_distance_publisher= self.create_publisher(Float64, "/max_distance", 10)
        self.best_angle_publisher = self.create_publisher(Float64,"/best_angle", 10)
        self.cmd_key_degree = self.servo_neutral_deg
        self.cmd_thruster = self.default_thrust
        self.dist_threshold = 0.3 # 장애물로 인식할 거리 (m)
        self.side_margin = 10 # 장애물로 처리할 좌우 각도
        self.imu_heading = 0.0
        self.stop_dist_m = 0.3
        self.initial_yaw_abs = None
        self.get_logger().info("Course 3")

    def load_params_from_yaml(self):
        script_dir = os.path.dirname(os.path.realpath(__file__))
        yaml_path = os.path.join(script_dir, "isv_params.yaml")
        with open(yaml_path, "r") as file:
            params = yaml.safe_load(file)
        self.timer_period = float(params["node_settings"]["timer_period"])
        self.servo_neutral_deg = float(params["servo"]["neutral_deg"])
        self.servo_min_deg = float(params["servo"]["min_deg"])
        self.servo_max_deg = float(params["servo"]["max_deg"])
        self.default_thrust = float(params["thruster"]["course3"])

    def normalize_180(self, deg: float) -> float:
        return (deg + 180.0) % 360.0 - 180.0

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_deg_360 = (degrees(atan2(siny_cosp, cosy_cosp)) + 360.0) % 360.0
        self.imu_heading = -self.normalize_180(yaw_deg_360)
        q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        _, _, yaw_rad = euler_from_quaternion(q)
        current_yaw_abs = -yaw_rad 
        if self.initial_yaw_abs is None:
            self.initial_yaw_abs = current_yaw_abs
        rel_yaw_deg = self.normalize_180(degrees(current_yaw_abs - self.initial_yaw_abs))
        self.curr_yaw_publisher.publish(Float64(data=float(rel_yaw_deg)))

    def lidar_callback(self, data):
        ranges = np.array(data.ranges)
        start_idx, end_idx = 500, 1500
        subset = ranges[start_idx:end_idx]
        dist_buckets = [[] for _ in range(181)] 
        dist_180 = np.zeros(181)

        for i in range(len(subset)):
            length = subset[i]
            if length <= data.range_min or length >= data.range_max or not np.isfinite(length):
                continue
            angle_index = int(round((len(subset) - 1 - i) * 180 / len(subset)))
            if 0 <= angle_index <= 180:
                dist_buckets[angle_index].append(length)
        for j in range(181):
            if dist_buckets[j]:
                dist_180[j] = np.median(dist_buckets[j])
            else:
                dist_180[j] = 0.0
        danger_flags = (dist_180 > 0) & (dist_180 <= self.dist_threshold)
        expanded_danger = np.copy(danger_flags)
        n = self.side_margin
        for i in range(181):
            if danger_flags[i]:
                expanded_danger[max(0, i - n) : min(181, i + n + 1)] = True
        self.safe_angles_list = [int(deg) for deg in range(181) if not expanded_danger[deg]]

        if self.safe_angles_list is not None:
            list_msg = String()
            list_msg.data = str(self.safe_angles_list)
            self.safe_angle_list_publisher.publish(list_msg)
        safe_distances = [dist_180[deg] for deg in self.safe_angles_list]
        relative_best_idx = np.argmax(safe_distances)
        best_angle_idx = self.safe_angles_list[relative_best_idx]
        max_distance = float(dist_180[best_angle_idx])

        self.max_distance_publisher.publish(Float64(data=max_distance))
        self.best_angle = float(best_angle_idx)
        self.best_angle_publisher.publish(Float64(data=self.best_angle))

        if max_distance <= self.stop_dist_m:
            self.key_publisher.publish(Float64(data=float(self.servo_neutral_deg)))
            self.thruster_publisher.publish(Float64(data=0.0))
            return

        self.cmd_key_degree = constrain(self.best_angle, self.servo_min_deg, self.servo_max_deg)
        self.key_publisher.publish(Float64(data=self.cmd_key_degree))
        self.thruster_publisher.publish(Float64(data=self.cmd_thruster))

    def send_stop_commands(self):
        if not rclpy.ok(): return
        for _ in range(5):
            self.key_publisher.publish(Float64(data=float(self.servo_neutral_deg)))
            self.thruster_publisher.publish(Float64(data=0.0))
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = Course3()
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
