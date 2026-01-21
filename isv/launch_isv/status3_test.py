#!/usr/bin/env python3
import os
import yaml
import math
import time
from math import degrees

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    qos_profile_sensor_data,
)

import numpy as np
np.float = float

from sensor_msgs.msg import LaserScan, NavSatFix, Imu
from std_msgs.msg import Float64
from mechaship_interfaces.msg import RgbwLedColor

from tf_transformations import euler_from_quaternion

from obstacle_isv import calculate_angle_risk
from Lidar_Preprocess import detect_and_cluster


def constrain(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class Status0TestNode(Node):
    def __init__(self):
        super().__init__("status0_test_node")
        self.get_logger().info("----- STARTING STATUS 0 TEST NODE -----")

        self._load_params_from_yaml()

        qos_best_effort_1 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.imu_sub = self.create_subscription(
            Imu, "/imu", self.imu_callback, qos_profile_sensor_data
        )
        self.gps_sub = self.create_subscription(
            NavSatFix, "/gps/fix", self.gps_listener_callback, qos_profile_sensor_data
        )
        self.lidar_sub = self.create_subscription(
            LaserScan, "/scan", self.lidar_listener_callback, qos_best_effort_1
        )

        self.scan_publisher = self.create_publisher(
            LaserScan, "/scan_filtered", qos_best_effort_1
        )
        self.key_publisher = self.create_publisher(Float64, "/actuator/key/degree", 10)
        self.thruster_publisher = self.create_publisher(
            Float64, "/actuator/thruster/percentage", 10
        )
        self.led_publisher = self.create_publisher(
            RgbwLedColor, "/actuator/rgbwled/color", 10
        )

        self.create_timer(self.timer_period_seconds, self.timer_callback)

        self.now_heading = None

        self.origin = [self.origin_lat, self.origin_lon, 0.0]
        self.origin_set = False
        self.goal_gps_coords = list(zip(self.goal_gps_lats, self.goal_gps_lons))
        self.goal_xy_coords = []
        self.now_goal = None
        self.dist_to_goal_m = None

        self.latest_angle_risk = None

        self.arrival_radius_m = 2.0
        self.arrived = False

        self.cmd_key_degree = self.servo_neutral_deg
        self.cmd_thruster = 0.0
        self.cmd_led = self.make_led(white=self.led_brightness)

        self.goal_abs_deg = None
        self.goal_rel_deg = None

        self._last_log_time = 0.0

    def normalize_360(self, deg):
        return (deg % 360.0 + 360.0) % 360.0

    def angle_diff_180(self, deg):
        return (deg + 180.0) % 360.0 - 180.0

    def _load_params_from_yaml(self):
        script_dir = os.path.dirname(os.path.realpath(__file__))
        yaml_path = os.path.join(script_dir, "isv_params.yaml")

        with open(yaml_path, "r") as file:
            params = yaml.safe_load(file)

        self.timer_period_seconds = params["node_settings"]["timer_period_seconds"]

        servo_params = params["servo"]
        self.servo_min_deg = float(servo_params["min_deg"])
        self.servo_max_deg = float(servo_params["max_deg"])
        self.servo_neutral_deg = float(servo_params["neutral_deg"])

        nav_params = params["navigation"]
        self.origin_lat = float(nav_params["origin"]["lat"])
        self.origin_lon = float(nav_params["origin"]["lon"])
        self.goal_gps_lats = nav_params["goal_gps_lats"]
        self.goal_gps_lons = nav_params["goal_gps_lons"]

        sm_params = params["state_machine"]
        self.state_params = {
            i: {"thruster": v}
            for i, v in enumerate(sm_params["thruster_defaults"].values())
        }

        lidar_params = params["lidar_obstacle_avoidance"]
        self.distance_weight = lidar_params["risk_calc"]["distance_weight"]
        self.angle_weight = lidar_params["risk_calc"]["angle_weight"]
        self.goal_weight = lidar_params["risk_calc"]["goal_weight"]

        self.led_brightness = int(params["led"]["default_brightness"])

    def check_key_limit(self, deg):
        return constrain(deg, self.servo_min_deg, self.servo_max_deg)

    def make_led(self, red=0, green=0, blue=0, white=0):
        msg = RgbwLedColor()
        msg.red = int(constrain(red, 0, 255))
        msg.green = int(constrain(green, 0, 255))
        msg.blue = int(constrain(blue, 0, 255))
        msg.white = int(constrain(white, 0, 255))
        return msg

    def imu_callback(self, msg):
        q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        _, _, yaw_rad = euler_from_quaternion(q)
        self.now_heading = self.normalize_360(degrees(yaw_rad))

    def gps_listener_callback(self, gps):
        if not self.origin_set:
            self.origin_set = True
            self.goal_xy_coords = [
                self.gps_enu_converter([lat, lon, 0.0])
                for lat, lon in self.goal_gps_coords
            ]
            self.now_goal = self.goal_xy_coords[0]

        e, n = self.gps_enu_converter([gps.latitude, gps.longitude, gps.altitude])
        goal_e, goal_n = self.now_goal

        dx = goal_e - e
        dy = goal_n - n

        self.dist_to_goal_m = math.sqrt(dx * dx + dy * dy)
        self.goal_abs_deg = self.normalize_360(degrees(math.atan2(dx, dy)))

        if self.now_heading is not None:
            self.goal_rel_deg = self.normalize_360(self.goal_abs_deg - self.now_heading)

    def lidar_listener_callback(self, data):
        filtered_range_list = detect_and_cluster(data.ranges)
        data.ranges = filtered_range_list
        self.scan_publisher.publish(data)

        if self.goal_rel_deg is None:
            self.latest_angle_risk = None
            return

        angle = calculate_angle_risk(
            filtered_range_list,
            self.goal_rel_deg,
            0,
            self.distance_weight,
            self.angle_weight,
            self.goal_weight,
            math.degrees(data.angle_increment),
            1,
        )

        self.latest_angle_risk = self.angle_diff_180(angle)

    def gps_enu_converter(self, lla):
        lat, lon, _ = lla
        lat0, lon0, _ = self.origin
        R = 6378137.0
        dlat = math.radians(lat - lat0)
        dlon = math.radians(lon - lon0)
        latm = math.radians((lat + lat0) * 0.5)
        north = dlat * R
        east = dlon * R * math.cos(latm)
        return east, north

    def run_state_0(self):
        if self.dist_to_goal_m is None or self.arrived:
            self.cmd_thruster = 0.0
            self.cmd_key_degree = self.servo_neutral_deg
            return

        if self.dist_to_goal_m <= self.arrival_radius_m:
            self.arrived = True
            self.cmd_thruster = 0.0
            self.cmd_key_degree = self.servo_neutral_deg
            return

        self.cmd_thruster = float(self.state_params[0]["thruster"])

        if self.latest_angle_risk is not None:
            self.cmd_key_degree = self.check_key_limit(
                self.servo_neutral_deg + self.latest_angle_risk
            )
        else:
            self.cmd_key_degree = self.servo_neutral_deg

    def timer_callback(self):
        self.run_state_0()

        self.key_publisher.publish(Float64(data=self.cmd_key_degree))
        self.thruster_publisher.publish(Float64(data=self.cmd_thruster))
        self.led_publisher.publish(self.cmd_led)

        now = time.time()
        if now - self._last_log_time >= 1.0:
            self._last_log_time = now

            if self.goal_abs_deg is not None and self.goal_rel_deg is not None:
                self.get_logger().info(
                    f"[NAV] goal_abs={self.goal_abs_deg:.1f} deg | "
                    f"goal_rel={self.goal_rel_deg:.1f} deg | "
                    f"key={self.cmd_key_degree:.1f} deg | "
                    f"thruster={self.cmd_thruster:.2f}"
                )


def main(args=None):
    rclpy.init(args=args)
    node = Status0TestNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
