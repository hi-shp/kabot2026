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
    """
    STATUS 0만 단독 테스트:
      - 첫 번째 GPS goal로 향함
      - LiDAR 회피각(calculate_angle_risk) 사용
      - 상태 전환 없음
    """

    def __init__(self):
        super().__init__("status0_test_node")
        self.get_logger().info("----- STARTING STATUS 0 TEST NODE -----")

        self._load_params_from_yaml()

        # ✅ Jazzy 호환 QoS
        qos_best_effort_1 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, "/imu", self.imu_callback, qos_profile_sensor_data
        )
        self.gps_sub = self.create_subscription(
            NavSatFix, "/gps/fix", self.gps_listener_callback, qos_profile_sensor_data
        )
        self.lidar_sub = self.create_subscription(
            LaserScan, "/scan", self.lidar_listener_callback, qos_best_effort_1
        )

        # Publishers
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

        # Timer
        self.create_timer(self.timer_period_seconds, self.timer_callback)

        # IMU heading
        self.now_heading = None

        # GPS/Goals
        self.origin = [self.origin_lat, self.origin_lon, 0.0]
        self.origin_set = False
        self.goal_gps_coords = list(zip(self.goal_gps_lats, self.goal_gps_lons))
        self.goal_xy_coords = []
        self.now_goal = None
        self.dist_to_goal_m = None

        # Lidar
        self.angle_increment_deg = None
        self.angle_min_deg = None
        self.latest_scan_filtered = None
        self.latest_angle_risk = None

        # State (locked)
        self.status = 0
        self.state_changed = True
        self.get_logger().info(f"State machine locked to STATUS = {self.status}")

        # 도착 판별 (추가)
        self.arrival_radius_m = 2.0
        self.arrived = False

        # Commands
        self.cmd_key_degree = self.servo_neutral_deg
        self.cmd_thruster = 0.0
        self.cmd_led = self.make_led(white=self.led_brightness)

        # goal angles
        self.goal_abs_deg = None
        self.goal_rel_deg = None

    # ---------------- Utils ----------------
    def normalize_360(self, deg: float) -> float:
        return (deg % 360.0 + 360.0) % 360.0

    def angle_diff_180(self, deg: float) -> float:
        return (deg + 180.0) % 360.0 - 180.0

    def _load_params_from_yaml(self):
        self.get_logger().info("Loading parameters directly from YAML file...")
        script_dir = os.path.dirname(os.path.realpath(__file__))
        yaml_path = os.path.join(script_dir, "isv_params.yaml")

        try:
            with open(yaml_path, "r") as file:
                params = yaml.safe_load(file)
                if params is None:
                    self.get_logger().error(f"YAML file is empty or invalid: {yaml_path}")
                    return

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
                self.risk_min_dist = lidar_params["risk_calc"]["min_safe_dist"]
                self.risk_max_dist = lidar_params["risk_calc"]["max_safe_dist"]
                self.distance_weight = lidar_params["risk_calc"]["distance_weight"]
                self.angle_weight = lidar_params["risk_calc"]["angle_weight"]
                self.goal_weight = lidar_params["risk_calc"]["goal_weight"]

                self.led_brightness = int(params["led"]["default_brightness"])

                self.get_logger().info("Parameters loaded successfully from YAML.")

        except FileNotFoundError:
            self.get_logger().fatal(f"FATAL: Parameter file not found at {yaml_path}. Shutting down.")
            raise
        except Exception as e:
            self.get_logger().fatal(f"FATAL: Failed to load or parse parameter file: {e}")
            raise

    def check_key_limit(self, deg):
        return constrain(deg, self.servo_min_deg, self.servo_max_deg)

    def make_led(self, red=0, green=0, blue=0, white=0):
        msg = RgbwLedColor()
        msg.red = int(constrain(red, 0, 255))
        msg.green = int(constrain(green, 0, 255))
        msg.blue = int(constrain(blue, 0, 255))
        msg.white = int(constrain(white, 0, 255))
        return msg

    def set_led(self, red=0, green=0, blue=0, white=0):
        self.cmd_led = self.make_led(red, green, blue, white)

    # ---------------- Callbacks ----------------
    def imu_callback(self, msg: Imu):
        q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        _, _, yaw_rad = euler_from_quaternion(q)
        yaw_deg = degrees(yaw_rad)
        self.now_heading = self.normalize_360(yaw_deg)

    def gps_listener_callback(self, gps: NavSatFix):
        if (not self.origin_set) and (not math.isnan(gps.latitude)) and (not math.isnan(gps.longitude)):
            if not self.goal_gps_lats or not self.goal_gps_lons:
                self.get_logger().warn("Goal GPS coordinates are not set. Skipping GPS initialization.")
                return

            self.origin_set = True
            self.goal_xy_coords = [
                self.gps_enu_converter([lat, lon, 0.0])
                for (lat, lon) in self.goal_gps_coords
            ]
            self.now_goal = self.goal_xy_coords[0]
            self.get_logger().info("GPS Origin and Goal for STATUS 0 Initialized.")

        if (not self.origin_set) or (self.now_goal is None):
            return

        e, n = self.gps_enu_converter([gps.latitude, gps.longitude, gps.altitude])

        goal_e, goal_n = self.now_goal
        dx = goal_e - e
        dy = goal_n - n

        dist = math.sqrt(dx * dx + dy * dy)
        self.dist_to_goal_m = dist

        self.goal_abs_deg = self.normalize_360(degrees(math.atan2(dx, dy)))

        if self.now_heading is not None:
            self.goal_rel_deg = self.normalize_360(self.goal_abs_deg - self.now_heading)
        else:
            self.goal_rel_deg = None

    def lidar_listener_callback(self, data: LaserScan):
        self.angle_increment_deg = math.degrees(data.angle_increment)
        self.angle_min_deg = math.degrees(data.angle_min)

        filtered_range_list = detect_and_cluster(data.ranges)
        self.latest_scan_filtered = filtered_range_list

        filtered_scan_msg = data
        filtered_scan_msg.ranges = filtered_range_list
        self.scan_publisher.publish(filtered_scan_msg)

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
            self.angle_increment_deg,
            1,
        )

        self.latest_angle_risk = self.angle_diff_180(angle)

    # ---------------- GPS ENU ----------------
    def gps_enu_converter(self, lla):
        lat, lon, _alt = lla
        lat0, lon0, _alt0 = self.origin
        R = 6378137.0
        dlat = math.radians(lat - lat0)
        dlon = math.radians(lon - lon0)
        latm = math.radians((lat + lat0) * 0.5)
        north = dlat * R
        east = dlon * R * math.cos(latm)
        return east, north

    # ---------------- STATUS 0 ----------------
    def run_state_0(self):
        # 거리 계산 전 안전 정지
        if self.dist_to_goal_m is None:
            self.cmd_thruster = 0.0
            self.cmd_key_degree = self.servo_neutral_deg
            self.set_led(white=self.led_brightness)
            return

        # 도착 후 재출발 방지
        if self.arrived:
            self.cmd_thruster = 0.0
            self.cmd_key_degree = self.servo_neutral_deg
            self.set_led(white=self.led_brightness)
            return

        # 도착 판별
        if self.dist_to_goal_m <= self.arrival_radius_m:
            self.arrived = True
            self.cmd_thruster = 0.0
            self.cmd_key_degree = self.servo_neutral_deg
            self.set_led(white=self.led_brightness)
            self.get_logger().info(f"[ARRIVED] dist={self.dist_to_goal_m:.2f}m <= {self.arrival_radius_m:.2f}m")
            return

        # 정상 주행
        self.cmd_thruster = float(self.state_params[0]["thruster"])
        self.set_led(white=self.led_brightness)

        if self.latest_angle_risk is not None:
            self.cmd_key_degree = self.check_key_limit(self.servo_neutral_deg - self.latest_angle_risk)
        else:
            self.cmd_key_degree = self.servo_neutral_deg

    # ---------------- Timer ----------------
    def timer_callback(self):
        self.run_state_0()

        self.key_publisher.publish(Float64(data=float(self.cmd_key_degree)))
        self.thruster_publisher.publish(Float64(data=float(self.cmd_thruster)))
        self.led_publisher.publish(self.cmd_led)

        if self.state_changed:
            self.get_logger().info(f"[ENTER] status={self.status} (Locked for testing)")
            self.state_changed = False


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Status0TestNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception) as e:
        if isinstance(e, KeyboardInterrupt):
            print("Keyboard Interrupt (SIGINT)")
        else:
            if node:
                node.get_logger().error(f"An exception occurred: {e}")
            else:
                print(f"An exception occurred during node initialization: {e}")
    finally:
        if node:
            try:
                node.get_logger().info("Shutting down node, setting actuators to neutral.")
                node.key_publisher.publish(Float64(data=float(node.servo_neutral_deg)))
                node.thruster_publisher.publish(Float64(data=0.0))
                node.led_publisher.publish(node.make_led(white=node.led_brightness))
            except Exception:
                pass
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
