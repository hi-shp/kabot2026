#!/usr/bin/env python3
import os
import yaml
import math
import time
import sys
import signal  # 시그널 처리를 위해 추가
from math import degrees

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import numpy as np
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

        # 퍼블리셔 설정
        self.key_publisher = self.create_publisher(Float64, "/actuator/key/degree", 10)
        self.thruster_publisher = self.create_publisher(Float64, "/actuator/thruster/percentage", 10)
        self.dist_publisher = self.create_publisher(Float32, "/waypoint/distance", 10)
        self.rel_deg_publisher = self.create_publisher(Float32, "/waypoint/rel_deg", 10)
        self.goal_publisher = self.create_publisher(NavSatFix, "/waypoint/goal", 10)

        # 서브스크라이버 설정
        qos_profile = qos_profile_sensor_data
        self.imu_sub = self.create_subscription(Imu, "/imu", self.imu_callback, qos_profile)
        self.gps_sub = self.create_subscription(NavSatFix, "/gps/fix", self.gps_listener_callback, qos_profile)

        # 상태 변수
        self.origin = [self.origin_lat, self.origin_lon, 0.0]
        self.origin_set = False
        self.wp_index = 0
        self.current_goal_enu = None
        
        self.yaw_rad = None
        self.dist_to_goal_m = None
        self.goal_rel_deg = None  
        self.arrived_all = False

        self.cmd_key_degree = self.servo_neutral_deg
        self.cmd_thruster = 0.0

        self.create_timer(self.timer_period_seconds, self.timer_callback)

    def _load_params_from_yaml(self):
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
        self.waypoints = nav["waypoints"] 

        self.thruster_cfg = params["state_machine"]["thruster_defaults"]
        self.default_thruster = float(self.thruster_cfg.get("state0", 10.0))

    def normalize_180(self, deg):
        return (deg + 180.0) % 360.0 - 180.0

    def gps_enu_converter(self, lla):
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
        self.yaw_rad = -yaw_rad

    def gps_listener_callback(self, gps: NavSatFix):
        if math.isnan(gps.latitude) or math.isnan(gps.longitude):
            return

        if not self.origin_set:
            self.origin_set = True
            self.update_current_goal()

        curr_e, curr_n = self.gps_enu_converter([gps.latitude, gps.longitude, gps.altitude])
        
        if self.current_goal_enu is not None:
            goal_e, goal_n = self.current_goal_enu
            dx, dy = goal_e - curr_e, goal_n - curr_n
            self.dist_to_goal_m = math.hypot(dx, dy)

            if self.yaw_rad is not None:
                vec_ang_ccw = math.atan2(dy, dx)
                rel_ccw = vec_ang_ccw - self.yaw_rad
                self.goal_rel_deg = self.normalize_180(-degrees(rel_ccw))

    def update_current_goal(self):
        if self.wp_index < len(self.waypoints):
            target_lat, target_lon = self.waypoints[self.wp_index]
            self.current_goal_enu = self.gps_enu_converter([target_lat, target_lon, 0.0])
            
            goal_msg = NavSatFix()
            goal_msg.latitude = target_lat
            goal_msg.longitude = target_lon
            self.goal_publisher.publish(goal_msg)
        else:
            self.current_goal_enu = None
            self.arrived_all = True

    def timer_callback(self):
        if self.arrived_all or self.dist_to_goal_m is None or self.goal_rel_deg is None:
            self.cmd_thruster = 0.0
            self.cmd_key_degree = self.servo_neutral_deg
        else:
            if self.dist_to_goal_m <= 3.0:
                self.wp_index += 1
                self.update_current_goal()
                return

            state_key = f"state{self.wp_index}"
            self.cmd_thruster = float(self.thruster_cfg.get(state_key, self.default_thruster))
            
            steer_error = self.goal_rel_deg
            self.cmd_key_degree = constrain(
                self.servo_neutral_deg + steer_error,
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
        if not rclpy.ok():
            return
        
        safe_key = Float64(data=float(self.servo_neutral_deg))
        safe_thruster = Float64(data=0.0)
        
        for i in range(10):
            try:
                self.key_publisher.publish(safe_key)
                self.thruster_publisher.publish(safe_thruster)
                time.sleep(0.05)
            except Exception:
                break

def main(args=None):
    rclpy.init(args=args)
    node = GPSPursueNode()
    
    # 신호 핸들러 설정 (Ctrl+C 즉시 대응)
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