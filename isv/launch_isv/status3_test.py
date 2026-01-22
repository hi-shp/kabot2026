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

        self._load_params_from_yaml()

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
        self.key_publisher = self.create_publisher(Float64, "/actuator/key/degree", 10)
        self.thruster_publisher = self.create_publisher(
            Float64, "/actuator/thruster/percentage", 10
        )
        self.led_publisher = self.create_publisher(
            RgbwLedColor, "/actuator/rgbwled/color", 10
        )

        # Timer
        self.create_timer(self.timer_period_seconds, self.timer_callback)

        # State
        self.origin = [self.origin_lat, self.origin_lon, 0.0]
        self.origin_set = False

        self.goal_gps_coords = list(zip(self.goal_gps_lats, self.goal_gps_lons))
        self.now_goal = None  # (east, north) of current goal

        self.dist_to_goal_m = None

        # IMU yaw (rad) in ENU convention (typically CCW positive from +x(East))
        self.yaw_rad = None

        # For display only (north=0, CW+)
        self.goal_abs_deg = None

        # Boat-frame relative angle to goal (bow=0, CW+), normalized to [-180, 180)
        self.goal_rel_deg = None

        self.latest_angle_risk = None

        self.arrival_radius_m = 2.0
        self.arrived = False

        self.cmd_key_degree = self.servo_neutral_deg
        self.cmd_thruster = 0.0
        self.cmd_led = self.make_led(white=self.led_brightness)

        self._last_log_time = 0.0

    # ----------------- Params -----------------
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
        self.goal_gps_lats = nav["goal_gps_lats"]
        self.goal_gps_lons = nav["goal_gps_lons"]

        sm = params["state_machine"]["thruster_defaults"]
        self.state_params = {i: {"thruster": v} for i, v in enumerate(sm.values())}

        lp = params["lidar_obstacle_avoidance"]["risk_calc"]
        self.distance_weight = lp["distance_weight"]
        self.angle_weight = lp["angle_weight"]
        self.goal_weight = lp["goal_weight"]

        self.led_brightness = int(params["led"]["default_brightness"])

    # ----------------- Angle helpers -----------------
    def normalize_360(self, deg):
        deg= ( deg +360) %360
        return deg 

    def normalize_180(self, deg):
        """-180 <= deg < 180"""
        return ((deg + 180.0) % 360.0 - 180.0)

    # ----------------- Messages -----------------
    def make_led(self, red=0, green=0, blue=0, white=0):
        msg = RgbwLedColor()
        msg.red = int(constrain(red, 0, 255))
        msg.green = int(constrain(green, 0, 255))
        msg.blue = int(constrain(blue, 0, 255))
        msg.white = int(constrain(white, 0, 255))
        return msg

    def gps_enu_converter(self, lla):
        """Simple local tangent plane approx: returns (east, north) meters"""
        lat, lon, _ = lla
        lat0, lon0, _ = self.origin

        R = 6378137.0
        dlat = math.radians(lat - lat0)
        dlon = math.radians(lon - lon0)
        latm = math.radians((lat + lat0) * 0.5)

        north = dlat * R
        east = dlon * R * math.cos(latm)
        return east, north

    # ----------------- Callbacks -----------------
    def imu_callback(self, msg: Imu):
        q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        _, _, yaw_rad = euler_from_quaternion(q)
        yaw_deg = degrees(yaw_rad)
        self.now_heading = self.normalize_360(yaw_deg)


    def gps_listener_callback(self, gps: NavSatFix):
        if (not self.origin_set) and (not math.isnan(gps.latitude)) and (not math.isnan(gps.longitude)):
            if not self.goal_gps_lats or not self.goal_gps_lons:
                self.get_logger().warn("Goal GPS coordinates are not set in parameters. Skipping GPS initialization.")
                return
            self.origin_set = True
            self.goal_xy_coords = [self.gps_enu_converter([lat, lon, 0.0]) for (lat, lon) in self.goal_gps_coords]
            self.now_goal = self.goal_xy_coords[0]
            self.arrival_latched = False
            self.get_logger().info("GPS Origin and Goals Initialized.")

        if (not self.origin_set) or (self.now_goal is None):
            return

        e, n = self.gps_enu_converter([gps.latitude, gps.longitude, gps.altitude])

        goal_e, goal_n = self.now_goal
        dx = goal_e - e
        dy = goal_n - n

        dist = math.sqrt(dx * dx + dy * dy)
        self.dist_to_goal_m = dist
    
        self.goal_abs_deg = self.normalize_360(degrees(math.atan2(dx, dy)))
        self.goal_rel_deg = self.normalize_360(self.goal_abs_deg - self.now_heading)
       

       
        

    def lidar_listener_callback(self, scan: LaserScan):
        filtered = detect_and_cluster(scan.ranges)

        if self.goal_rel_deg is None:
            self.latest_angle_risk = None
            return

        # calculate_angle_risk() 입력 각도는 보트 기준(선수 0, CW+)이라고 가정
        angle = calculate_angle_risk(
            filtered,
            self.goal_rel_deg,
            0,
            self.distance_weight,
            self.angle_weight,
            self.goal_weight,
            math.degrees(scan.angle_increment),
            1,
        )

        # 출력도 동일한 각도계라고 가정하고 -180~180으로 정규화
        self.latest_angle_risk = -self.normalize_180(angle)
        
        
    # ----------------- Control loop -----------------
    def timer_callback(self):
        # Arrival check
        if self.dist_to_goal_m is None or self.arrived:
            self.cmd_thruster = 0.0
            self.cmd_key_degree = self.servo_neutral_deg
        elif self.dist_to_goal_m <= self.arrival_radius_m:
            self.arrived = True
            self.cmd_thruster = 0.0
            self.cmd_key_degree = self.servo_neutral_deg
        else:
            self.cmd_thruster = float(self.state_params[0]["thruster"])

            steer = self.latest_angle_risk if self.latest_angle_risk is not None else 0.0
            self.cmd_key_degree = constrain(
                self.servo_neutral_deg +steer,
                self.servo_min_deg,
                self.servo_max_deg,
            )

        # Publish
        self.key_publisher.publish(Float64(data=float(self.cmd_key_degree)))
        self.thruster_publisher.publish(Float64(data=float(self.cmd_thruster)))
        self.led_publisher.publish(self.cmd_led)

        # Fast panel (0.1s)
        now = time.time()
        if now - self._last_log_time >= 0.1:
            self._last_log_time = now

            h = f"{degrees(self.yaw_rad):6.1f}°" if self.yaw_rad is not None else "  WAIT "
            gabs = f"{self.goal_abs_deg:6.1f}°" if self.goal_abs_deg is not None else "  WAIT "
            grel = f"{self.goal_rel_deg:6.1f}°" if self.goal_rel_deg is not None else "  WAIT "
            risk = f"{self.latest_angle_risk:6.1f}°" if self.latest_angle_risk is not None else "  NONE "
            dist = f"{self.dist_to_goal_m:6.2f}m" if self.dist_to_goal_m is not None else "  WAIT "

            sys.stdout.write("\033[H\033[J")
            panel = (
                f"┌─────────── [ 실시간 항해 컨트롤 패널 ] ───────────┐\n"
                f"│   IMU yaw (raw)       : {h}                      │\n"
                f"│   목표 방위 (ABS, N0 CW+) : {gabs}                │\n"
                f"│   목표 상대각 (REL, Bow0 CW+) : {grel}            │\n"
                f"│   LiDAR risk angle        : {risk}               │\n"
                f"│   목표 거리 (DIST)        : {dist}               │\n"
                f"│   조향 각도 (SERVO)       : {self.cmd_key_degree:6.1f}°             │\n"
                f"│   추진 출력 (THRUST)      : {self.cmd_thruster:6.1f}%              │\n"
                f"└───────────────────────────────────────────────────┘\n"
            )
            sys.stdout.write(panel)
            sys.stdout.flush()

    def send_stop_commands(self):
        """종료 시 안전 명령 반복 발행"""
        sys.stdout.write("\n[EMERGENCY] STOPPING SHIP...")
        sys.stdout.flush()

        safe_key = Float64(data=float(self.servo_neutral_deg))
        safe_thruster = Float64(data=0.0)

        for _ in range(10):
            self.key_publisher.publish(safe_key)
            self.thruster_publisher.publish(safe_thruster)
            time.sleep(0.05)

        print("\n[EMERGENCY] SAFE VALUES SENT. SYSTEM CLOSED.")


def main(args=None):
    rclpy.init(args=args)
    node = Status0TestNode()

    def signal_handler(sig, frame):
        node.send_stop_commands()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        if rclpy.ok():
            node.send_stop_commands()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()