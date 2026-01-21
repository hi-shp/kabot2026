import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    qos_profile_sensor_data,
)

import numpy as np
import os
np.float = float  # NumPy의 float 정의 복구
import yaml

from sensor_msgs.msg import LaserScan, NavSatFix, Imu
import math
from std_msgs.msg import Float32
from simple_pid import PID
from tf_transformations import euler_from_quaternion
from math import degrees
import time

from obstacle_isv import calculate_angle_risk
from Lidar_Preprocess import detect_and_cluster

from vision_msgs.msg import Detection2DArray
from mechaship_interfaces.msg import RgbwLedColor


def constrain(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class MotorControlNode(Node):
    def __init__(self):
        super().__init__("pid_go")
        self.get_logger().info("----- start ISV_2026 (State Machine) Node -----")

        # YAML 파일에서 파라미터 직접 로드
        self._load_params_from_yaml()

        # ---------------- QoS (Jazzy-friendly, no deprecated enums) ----------------
        qos_best_effort_1 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ---------------- Subscribers ----------------
        self.imu_sub = self.create_subscription(Imu, "/imu", self.imu_callback, qos_profile_sensor_data)
        self.gps_sub = self.create_subscription(NavSatFix, "/gps/fix", self.gps_listener_callback, qos_profile_sensor_data)
        self.lidar_sub = self.create_subscription(LaserScan, "/scan", self.lidar_listener_callback, qos_best_effort_1)
        self.detection_subscriber = self.create_subscription(
            Detection2DArray, "/detections", self.detection_callback, qos_profile_sensor_data
        )

        # ---------------- Publishers ----------------
        self.scan_publisher = self.create_publisher(LaserScan, "/scan_filtered", qos_best_effort_1)
        self.key_publisher = self.create_publisher(Float32, "/actuator/key/degree", 10)
        self.thruster_publisher = self.create_publisher(Float32, "/actuator/thruster/percentage", 10)
        self.led_publisher = self.create_publisher(RgbwLedColor, "/actuator/rgbwled/color", 10)

        # ---------------- Timer ----------------
        self.create_timer(self.timer_period_seconds, self.timer_callback)

        # ---------------- Control ----------------
        self.angle_pid = PID(self.p_gain, self.i_gain, self.d_gain)
        self.angle_pid.output_limits = (self.pid_output_min, self.pid_output_max)
        self.last_time = time.time()

        # ---------------- Vision ----------------
        self.object_detected = False
        self.target_x = None
        self.vision_target_angle = None
        self.last_detected_object = None
        self.last_detected_color = None

        # ---------------- IMU/Heading ----------------
        # 시작 시 북쪽 정렬하고 0도로 시작할 예정이므로 보정 없이 yaw(deg)만 사용
        self.now_heading = None

        # ---------------- GPS / Goals ----------------
        self.origin = [self.origin_lat, self.origin_lon, 0.0]
        self.origin_set = False
        self.goal_gps_coords = list(zip(self.goal_gps_lats, self.goal_gps_lons))
        self.goal_xy_coords = []
        self.now_goal = None
        self.arrival_latched = False

        self.dist_to_goal_m = None
        self.goal_abs_deg = None
        self.goal_rel_deg = None

        # ---------------- Lidar ----------------
        self.angle_increment_deg = None
        self.angle_min_deg = None
        self.latest_scan_filtered = None
        self.latest_angle_risk = None

        self.distance_weight = None
        self.angle_weight = None
        self.goal_weight = None

        # ---------------- State Machine ----------------
        self.status = 0
        self.status_enter_time = time.time()
        self.state_changed = True

        # stop_status: status0~2에서 도착 이벤트 플래그
        self.stop_status = 0

        # status3 도킹 단계
        self.status3_docking_phase = False
        self.final_stop_start_time = None

        # ---------------- Outputs (command) ----------------
        self.cmd_key_degree = self.servo_neutral_deg
        self.cmd_thruster = 0.0
        self.cmd_led = self.make_led(white=self.led_brightness)

    # ---------------- Utils ----------------
    def normalize_360(self, deg: float) -> float:
        return (deg % 360.0 + 360.0) % 360.0

    def angle_diff_180(self, deg: float) -> float:
        """0~360을 -180~180으로 변환"""
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

                pid_params = params["pid"]
                self.p_gain = pid_params["p_gain"]
                self.i_gain = pid_params["i_gain"]
                self.d_gain = pid_params["d_gain"]
                self.pid_output_min = pid_params["output_limits"]["min"]
                self.pid_output_max = pid_params["output_limits"]["max"]

                servo_params = params["servo"]
                self.servo_min_deg = servo_params["min_deg"]
                self.servo_max_deg = servo_params["max_deg"]
                self.servo_neutral_deg = servo_params["neutral_deg"]

                nav_params = params["navigation"]
                self.origin_lat = nav_params["origin"]["lat"]
                self.origin_lon = nav_params["origin"]["lon"]
                self.goal_gps_lats = nav_params["goal_gps_lats"]
                self.goal_gps_lons = nav_params["goal_gps_lons"]
                self.arrival_radius_m = nav_params["arrival_radius_m"]
                self.arrival_latch_release_factor = nav_params["arrival_latch_release_factor"]

                sm_params = params["state_machine"]
                # status0~3 기본 thruster
                self.state_params = {i: {"thruster": v} for i, v in enumerate(sm_params["thruster_defaults"].values())}

                docking_final = sm_params.get("docking_final", {})
                self.docking_enter_radius_m = float(docking_final.get("enter_radius_m", 0.2))
                self.docking_stop_hold_time_sec = float(docking_final.get("stop_hold_time_sec", 10.0))

                vision_params = params["vision"]
                self.screen_width = vision_params["screen_width"]
                self.angle_conversion_factor = vision_params["angle_conversion_factor"]

                vision_targets_params = params["vision_targets"]
                self.available_objects = vision_targets_params["available_objects"]
                self.tracking_target = vision_targets_params["tracking_target"]

                lidar_params = params["lidar_obstacle_avoidance"]
                self.risk_min_dist = lidar_params["risk_calc"]["min_safe_dist"]
                self.risk_max_dist = lidar_params["risk_calc"]["max_safe_dist"]
                self.distance_weight = lidar_params["risk_calc"]["distance_weight"]
                self.angle_weight = lidar_params["risk_calc"]["angle_weight"]
                self.goal_weight = lidar_params["risk_calc"]["goal_weight"]

                self.docking_reverse_thrust = lidar_params["docking"]["obstacle_reverse_thrust"]
                self.docking_obs_thresh_m = lidar_params["docking"]["obstacle_distance_threshold_m"]
                self.docking_stop_dist_m = lidar_params["docking"]["final_stop_distance_m"]

                self.led_brightness = params["led"]["default_brightness"]

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
        msg.red = int(red)
        msg.green = int(green)
        msg.blue = int(blue)
        msg.white = int(white)
        return msg

    def set_led(self, red=0, green=0, blue=0, white=0):
        self.cmd_led = self.make_led(red, green, blue, white)

    def set_status(self, new_status: int):
        if new_status == self.status:
            return
        self.status = new_status
        self.status_enter_time = time.time()
        self.state_changed = True
        self.get_logger().info(f"[STATE] -> {self.status}")

        if self.status == 3:
            self.status3_docking_phase = False
            self.final_stop_start_time = None

    # ---------------- Callbacks ----------------
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
       

        if dist > (self.arrival_radius_m * self.arrival_latch_release_factor):
            self.arrival_latched = False

        if self.status < 3:
            if (dist <= self.arrival_radius_m) and (not self.arrival_latched):
                self.arrival_latched = True
                self.stop_status = 1
                self.get_logger().info(f"[GPS] Arrived goal of status={self.status} (dist={dist:.2f}m)")

        elif self.status == 3:
            if dist <= self.docking_enter_radius_m:
                if not self.status3_docking_phase:
                    self.get_logger().info(f"[GPS] Status3 entered docking phase (dist={dist:.2f}m)")
                self.status3_docking_phase = True

    def lidar_listener_callback(self, data: LaserScan):
        self.angle_increment_deg = math.degrees(data.angle_increment)
        self.angle_min_deg = math.degrees(data.angle_min)

        scan_data = data.ranges
        filtered_range_list = detect_and_cluster(scan_data)

        # None/NaN/<=0 등을 inf로 치환해서 downstream 계산 안정화
        sanitized = []
        for r in filtered_range_list:
            if r is None:
                sanitized.append(float("inf"))
                continue
            try:
                rr = float(r)
                if math.isnan(rr) or rr <= 0.0:
                    sanitized.append(float("inf"))
                else:
                    sanitized.append(rr)
            except Exception:
                sanitized.append(float("inf"))

        self.latest_scan_filtered = sanitized

        filtered_scan_msg = data
        filtered_scan_msg.ranges = sanitized
        self.scan_publisher.publish(filtered_scan_msg)

        if self.goal_rel_deg is None:
            self.latest_angle_risk = None
            return

        try:
            angle = calculate_angle_risk(
                sanitized,
                self.goal_rel_deg,
                0,
                self.distance_weight,
                self.angle_weight,
                self.goal_weight,
                self.angle_increment_deg,
                1,
            )
            self.latest_angle_risk = self.angle_diff_180(angle)
        except Exception as e:
            self.latest_angle_risk = None
            self.get_logger().error(f"[LIDAR] calculate_angle_risk failed: {e}")

    def detection_callback(self, msg: Detection2DArray):
        target_detection = None

        for detection in msg.detections:
            if not detection.results:
                continue
            class_id = int(detection.results[0].hypothesis.class_id)
            object_name = self.get_object_name(class_id)
            if object_name == self.tracking_target:
                target_detection = detection
                break

        if target_detection:
            self.object_detected = True
            bbox = target_detection.bbox
            self.target_x = bbox.center.position.x

            angle_offset = (
                (self.target_x - (self.screen_width / 2.0)) / (self.screen_width / 2.0)
            ) * self.angle_conversion_factor
            self.vision_target_angle = -angle_offset

            self.last_detected_object = self.tracking_target
            self.last_detected_color = self.parse_color_from_object_name(self.last_detected_object)

        else:
            self.object_detected = False
            self.target_x = None
            self.vision_target_angle = None
            self.last_detected_object = None
            self.last_detected_color = None

    def parse_color_from_object_name(self, name: str):
        if not name:
            return None
        if name.startswith("red "):
            return "red"
        if name.startswith("green "):
            return "green"
        if name.startswith("blue "):
            return "blue"
        return None

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

    # ---------------- State Behaviors ----------------
    def run_state_0(self):
        self.cmd_thruster = self.state_params[0]["thruster"]
        self.set_led(white=self.led_brightness)
        if self.latest_angle_risk is not None:
            self.cmd_key_degree = self.check_key_limit(self.servo_neutral_deg - self.latest_angle_risk)
        else:
            self.cmd_key_degree = self.servo_neutral_deg

    def run_state_1(self):
        self.cmd_thruster = self.state_params[1]["thruster"]
        self.set_led(white=self.led_brightness)
        if self.latest_angle_risk is not None:
            self.cmd_key_degree = self.check_key_limit(self.servo_neutral_deg - self.latest_angle_risk)
        else:
            self.cmd_key_degree = self.servo_neutral_deg

    def run_state_2(self):
        self.cmd_thruster = self.state_params[2]["thruster"]

        if (
            self.object_detected
            and self.vision_target_angle is not None
            and self.last_detected_object == self.tracking_target
        ):
            self.cmd_key_degree = self.check_key_limit(self.servo_neutral_deg - self.vision_target_angle)

            b = self.led_brightness
            if self.last_detected_color == "red":
                self.set_led(red=b)
            elif self.last_detected_color == "green":
                self.set_led(green=b)
            elif self.last_detected_color == "blue":
                self.set_led(blue=b)
            else:
                self.set_led(green=b)

        else:
            if self.latest_angle_risk is not None:
                self.cmd_key_degree = self.check_key_limit(self.servo_neutral_deg - self.latest_angle_risk)
            else:
                self.cmd_key_degree = self.servo_neutral_deg
            self.set_led(white=self.led_brightness)

    def run_state_3(self):
        # ---------- Phase B: docking/stop ----------
        if self.status3_docking_phase:
            self.set_led(white=self.led_brightness)

            base_thr = self.state_params.get(4, {}).get("thruster", 0.0)
            self.cmd_key_degree = self.servo_neutral_deg
            self.cmd_thruster = base_thr

            min_dist = float("inf")
            min_index = None
            if self.latest_scan_filtered:
                for i, r in enumerate(self.latest_scan_filtered):
                    if math.isinf(r) or math.isnan(r) or r <= 0.0:
                        continue
                    if r < min_dist:
                        min_dist = r
                        min_index = i

            lidar_angle_deg = None
            if min_index is not None and self.angle_min_deg is not None and self.angle_increment_deg is not None:
                lidar_angle_deg = self.angle_min_deg + min_index * self.angle_increment_deg

            if (min_dist <= self.docking_obs_thresh_m) and (lidar_angle_deg is not None):
                self.cmd_key_degree = self.check_key_limit(lidar_angle_deg)
                self.cmd_thruster = self.docking_reverse_thrust
                self.final_stop_start_time = None
                return

            self.cmd_key_degree = self.servo_neutral_deg
            self.cmd_thruster = 0.0

            if self.final_stop_start_time is None:
                self.final_stop_start_time = time.time()
                self.get_logger().info("[FINAL] Stopped safely. Holding for docking hold time...")

            if time.time() - self.final_stop_start_time >= self.docking_stop_hold_time_sec:
                self.get_logger().info("[FINAL] Docking hold complete. Setting status to 4 (shutdown).")
                self.set_status(4)

            return

        # ---------- Phase A: go-to-goal (avoid) ----------
        self.cmd_thruster = self.state_params[3]["thruster"]
        self.set_led(white=self.led_brightness)

        if self.latest_angle_risk is not None:
            self.cmd_key_degree = self.check_key_limit(self.servo_neutral_deg - self.latest_angle_risk)
        else:
            self.cmd_key_degree = self.servo_neutral_deg

    # ---------------- Main Timer ----------------
    def timer_callback(self):
        if self.status == 4:
            self.get_logger().info("STATUS 4 reached. Shutting down node.")
            self.destroy_node()
            return

        if self.stop_status == 1:
            next_status = self.status + 1

            if next_status >= len(self.goal_xy_coords):
                self.set_status(4)
                self.now_goal = None
            else:
                self.set_status(next_status)
                self.now_goal = self.goal_xy_coords[next_status]
                self.arrival_latched = False

            self.stop_status = 0

        if self.status == 0:
            self.run_state_0()
        elif self.status == 1:
            self.run_state_1()
        elif self.status == 2:
            self.run_state_2()
        elif self.status == 3:
            self.run_state_3()
        else:
            self.cmd_thruster = 0.0
            self.cmd_key_degree = self.servo_neutral_deg
            self.set_led(white=self.led_brightness)

        # publish
        self.key_publisher.publish(Float32(data=float(self.cmd_key_degree)))
        self.thruster_publisher.publish(Float32(data=float(self.cmd_thruster)))
        self.led_publisher.publish(self.cmd_led)

        if self.state_changed:
            self.get_logger().info(f"[ENTER] status={self.status}")
            self.state_changed = False

    def get_object_name(self, class_id):
        return self.available_objects[class_id] if 0 <= class_id < len(self.available_objects) else "unknown"


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MotorControlNode()
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
                node.key_publisher.publish(Float32(data=float(node.servo_neutral_deg)))
                node.thruster_publisher.publish(Float32(data=0.0))
                node.led_publisher.publish(node.make_led(white=node.led_brightness))
            except Exception as e:
                try:
                    node.get_logger().warn(f"Failed to publish neutral commands on shutdown: {e}")
                except Exception:
                    pass
            try:
                node.destroy_node()
            except Exception:
                pass

        rclpy.shutdown()


if __name__ == "__main__":
    main()
