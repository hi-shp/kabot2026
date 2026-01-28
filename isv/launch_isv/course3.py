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
from std_msgs.msg import Float64
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
        self.max_distance_publisher= self.create_publisher(Float64, "/max_distance", 10)
        self.best_angle_publisher = self.create_publisher(Float64,"/best_angle", 10)
        self.init_yaw_sub = self.create_subscription(Float64, "/imu/target_yaw", self.init_yaw_callback, 10)
        self.stop_dist_m = 0.3
        self.initial_yaw_abs = None
        self.yaw_offset = None
        self.current_yaw_rel = 0.0
        self.cmd_key_degree = self.servo_neutral_deg
        self.cmd_thruster = self.default_thruster
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
        self.default_thruster = float(params["thruster"]["course3"])

    def init_yaw_callback(self, msg: Float64):
        if self.initial_yaw_abs is None:
            self.initial_yaw_abs = msg.data
            self.destroy_subscription(self.init_yaw_sub)

    def normalize_180(self, deg: float) -> float:
        return (deg + 180.0) % 360.0 - 180.0

    def imu_callback(self, msg: Imu):
        if self.initial_yaw_abs is None:
            return
        q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        _, _, yaw_rad = euler_from_quaternion(q)
        current_yaw_raw_deg = degrees(-yaw_rad)
        if self.yaw_offset is None:
            self.yaw_offset = self.initial_yaw_abs - current_yaw_raw_deg
        corrected_yaw = current_yaw_raw_deg + self.yaw_offset
        rel_yaw_deg = self.normalize_180(corrected_yaw)
        self.current_yaw_rel = rel_yaw_deg
        self.curr_yaw_publisher.publish(Float64(data=float(rel_yaw_deg)))

    def lidar_callback(self, data):
        ranges = np.array(data.ranges)
        start_idx, end_idx = 500, 1500
        subset = ranges[start_idx:end_idx]
        dist_180 = np.zeros(181)
        dist_buckets = [[] for _ in range(181)]
        for i in range(len(subset)):
            length = subset[i]
            if length <= data.range_min or length >= data.range_max or not np.isfinite(length):
                continue
            angle_index = int(round((len(subset) - 1 - i) * 180 / len(subset)))
            if 0 <= angle_index <= 180:
                dist_buckets[angle_index].append(length)
        for j in range(181):
            if dist_buckets[j]:
                rounded_lengths = [round(val * 20) / 20 for val in dist_buckets[j]]
                counts = {}
                for val in rounded_lengths:
                    counts[val] = counts.get(val, 0) + 1
                dist_180[j] = max(counts, key=counts.get)
            else:
                dist_180[j] = 0.0
        best_i = int(np.argmax(dist_180))
        new_max_distance = float(dist_180[best_i])
        if not hasattr(self, 'prev_max_dist'):
            self.prev_max_dist = new_max_distance
        if abs(new_max_distance - self.prev_max_dist) > 1.0:
            max_distance = self.prev_max_dist
        else:
            max_distance = new_max_distance
            self.prev_max_dist = new_max_distance
        best_lidar_angle = float(-90 + best_i)
        self.max_distance_publisher.publish(Float64(data=max_distance))
        self.best_angle = best_lidar_angle + 90.0
        self.best_angle_publisher.publish(Float64(data=float(self.best_angle)))
        if max_distance <= self.stop_dist_m:
            self.key_publisher.publish(Float64(data=float(self.servo_neutral_deg)))
            self.thruster_publisher.publish(Float64(data=0.0))
            return
        self.cmd_key_degree = constrain(
            self.best_angle, 
            self.servo_min_deg, 
            self.servo_max_deg
        )
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
            
if __name__ == "__main__":
    main()