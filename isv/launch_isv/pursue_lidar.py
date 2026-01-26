#!/usr/bin/env python3
import os
import yaml
import math
import time
import sys
import signal
from math import degrees, radians

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, Float32, String

def constrain(v, lo, hi):
    if math.isnan(v): return lo + (hi - lo) / 2.0
    return lo if v < lo else hi if v > hi else v

class LidarDirectNode(Node):
    def __init__(self):
        super().__init__("lidar_direct_node")
        self._load_params_from_yaml()
        
        # 1. 퍼블리셔 설정
        self.key_publisher = self.create_publisher(Float64, "/actuator/key/degree", 10)
        self.thruster_publisher = self.create_publisher(Float64, "/actuator/thruster/percentage", 10)
        self.safe_angle_list_publisher = self.create_publisher(String, "/safe_angles_list", 10)
        self.safe_angle_publisher = self.create_publisher(Float32, "/safe_angle", 10)
        self.lidar_sub = self.create_subscription(
            LaserScan, "/scan", self.lidar_callback, qos_profile_sensor_data
        )
        self.safe_angles_list = []
        self.goal_rel_deg = self.servo_neutral_deg
        self.max_clear_dist = 0.0
        self.dist_threshold = 1.0  # 1미터 임계값

        self.create_timer(self.timer_period_seconds, self.timer_callback)
        self.get_logger().info("LiDAR 테스트")

    def _load_params_from_yaml(self):
        script_dir = os.path.dirname(os.path.realpath(__file__))
        yaml_path = os.path.join(script_dir, "isv_params.yaml")
        with open(yaml_path, "r") as file:
            params = yaml.safe_load(file)
        self.timer_period_seconds = float(params["node_settings"]["timer_period"])
        self.servo_neutral_deg = float(params["servo"]["neutral_deg"]) 
        self.servo_min_deg = float(params["servo"]["min_deg"])
        self.servo_max_deg = float(params["servo"]["max_deg"])
        self.default_thruster = float(params["state"].get("state0", 5.0))

    def lidar_callback(self, msg: LaserScan):
        num_ranges = len(msg.ranges)
        if num_ranges == 0: return

        # 1) 라이다 전방 180도 슬라이싱
        mid = num_ranges // 2
        half = num_ranges // 4
        start_idx = mid - half
        end_idx = mid + half
        
        ranges = np.array(msg.ranges[start_idx:end_idx], dtype=float)
        ranges = np.flip(ranges)
        
        num_front = len(ranges)
        angles_deg = np.linspace(-90, 90, num_front)

        # 2) INVALID 제거
        invalid = (~np.isfinite(ranges)) | (ranges <= msg.range_min) | (ranges >= msg.range_max)
        ranges[invalid] = 0.0

        # 3) 전방 180도를 1도 단위로 평균내기
        dist_180 = np.zeros(181, dtype=float)
        target_degrees = np.arange(0, 181) 

        for target_deg in target_degrees:
            lower = (target_deg - 90) - 0.5
            upper = (target_deg - 90) + 0.5
            
            mask = (angles_deg >= lower) & (angles_deg < upper)
            chunk = ranges[mask]
            valid_pts = chunk[chunk > 0.0]
            dist_180[target_deg] = float(np.mean(valid_pts)) if valid_pts.size > 0 else 0.0

        # 4) 안전 각도 필터링
        safe_angles = [int(deg) for deg, dist in enumerate(dist_180) if dist > self.dist_threshold]
        self.safe_angles_list = safe_angles

        # 5) 디버깅용 safe_angles 리스트 퍼블리시
        if self.safe_angles_list is not None:
            list_msg = String()
            list_msg.data = str(self.safe_angles_list)
            self.safe_angle_list_publisher.publish(list_msg)

        # 6) 최적 조향 결정 및 최종 각도 발행
        if self.safe_angles_list:
            safe_arr = np.array(self.safe_angles_list)
            best_angle = safe_arr[np.argmin(np.abs(safe_arr - 90))]
            
            self.goal_rel_deg = float(best_angle)
            self.max_clear_dist = float(dist_180[int(best_angle)])
            self.safe_angle_publisher.publish(Float32(data=self.goal_rel_deg))
        else:
            self.max_clear_dist = 0.0
            self.goal_rel_deg = self.servo_neutral_deg

    def timer_callback(self):
        if not self.safe_angles_list or self.max_clear_dist <= 0.0:
            self.cmd_thruster = 0.0
            self.cmd_key_degree = self.servo_neutral_deg
        else:
            self.cmd_thruster = self.default_thruster
            self.cmd_key_degree = constrain(float(self.goal_rel_deg), self.servo_min_deg, self.servo_max_deg)

        self.key_publisher.publish(Float64(data=float(self.cmd_key_degree)))
        self.thruster_publisher.publish(Float64(data=float(self.cmd_thruster)))

    def send_stop_commands(self):
        if not rclpy.ok(): return
        for _ in range(5):
            self.key_publisher.publish(Float64(data=float(self.servo_neutral_deg)))
            self.thruster_publisher.publish(Float64(data=0.0))
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = LidarDirectNode()
    def signal_handler(sig, frame):
        node.get_logger().warn("Stopped")
        node.send_stop_commands()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if rclpy.ok():
            node.send_stop_commands()
            node.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    main()