#!/usr/bin/env python3
import os, sys, yaml, math, time, signal
import numpy as np
from math import degrees
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64, String
from sensor_msgs.msg import NavSatFix, Imu, LaserScan
from vision_msgs.msg import Detection2DArray
from mechaship_interfaces.msg import RgbwLedColor
from tf_transformations import euler_from_quaternion

def constrain(v, lo, hi):
    if math.isnan(v): return lo + (hi - lo) / 2.0
    return lo if v < lo else hi if v > hi else v

def norm_text(s):
    return " ".join(str(s).strip().lower().split())

class ISV_2026(Node):
    def __init__(self):
        super().__init__("ISV_2026")
        self.load_params_from_yaml()
        self.key_publisher = self.create_publisher(Float64, "/actuator/key/degree", 10)
        self.thruster_publisher = self.create_publisher(Float64, "/actuator/thruster/percentage", 10)
        self.dist_publisher = self.create_publisher(Float64, "/waypoint/distance", 10)
        self.rel_deg_publisher = self.create_publisher(Float64, "/waypoint/rel_deg", 10)
        self.goal_publisher = self.create_publisher(NavSatFix, "/waypoint/goal", 10)
        self.curr_yaw_publisher = self.create_publisher(Float64, "/current_yaw", 10)
        self.safe_angle_list_publisher = self.create_publisher(String, "/safe_angles_list", 10)
        self.safe_angle_publisher = self.create_publisher(Float64, "/safe_angle", 10)
        self.led_publisher = self.create_publisher(RgbwLedColor, "/actuator/rgbwled/color", 10)
        self.led_string_publisher = self.create_publisher(String, "/led_color", 10)
        self.target_name_publisher = self.create_publisher(String, "/target_name", 10)
        self.target_angle_publisher = self.create_publisher(Float64, "/target_angle", 10)
        self.state_publisher = self.create_publisher(String, "/state", 10)
        self.zero_count_publisher = self.create_publisher(String, "/zero_count", 10)
        self.imu_sub = self.create_subscription(Imu, "/imu", self.imu_callback, qos_profile_sensor_data)
        self.gps_sub = self.create_subscription(NavSatFix, "/gps/fix", self.gps_callback, qos_profile_sensor_data)
        self.lidar_sub = self.create_subscription(LaserScan, "/scan", self.lidar_callback, qos_profile_sensor_data)
        self.det_sub = self.create_subscription(Detection2DArray, "/detections", self.detection_callback, qos_profile_sensor_data)
        self.phase = "GPS"    # (GPS ->  HOPING -> DETECTION -> DONE -> WALL)
        self.origin = None
        self.origin_set = False
        self.wp_index = 0
        self.initial_yaw_abs = None
        self.yaw_offset = None
        self.current_yaw_rel = 0.0
        self.dist_to_goal_m = None
        self.goal_rel_deg = None
        self.latest_det = None
        self.arrived_all = False
        self.start_time = self.get_clock().now()
        self.safe_angles_list = []
        self.dist_threshold = 1.6
        self.side_margin = 35
        self.yaw_zero_count = 0
        self.last_zero_time = 0.0
        self.zero_count_cooldown = 8.0
        self.create_timer(self.timer_period, self.timer_callback)
        self.led_by_name("off")
        self.get_logger().info("Course 1")

    def load_params_from_yaml(self):
        script_dir = os.path.dirname(os.path.realpath(__file__))
        yaml_path = os.path.join(script_dir, "isv_params.yaml")
        with open(yaml_path, "r") as file:
            params = yaml.safe_load(file)
        self.timer_period = float(params["node_settings"]["timer_period"])
        self.servo_neutral_deg = float(params["servo"]["neutral_deg"])
        self.servo_min_deg = float(params["servo"]["min_deg"])
        self.servo_max_deg = float(params["servo"]["max_deg"])
        self.waypoints = params["navigation"]["waypoints"]
        self.arrival_radii = params["navigation"]["arrival_radius"]
        self.default_thruster = float(params["thruster"]["course1"])
        v = params["vision"]
        self.screen_width = int(v["screen_width"])
        self.angle_factor = float(v["angle_conversion_factor"])
        self.available_objects = v["available_objects"]
        self.hoping_target = v["hoping_target"]
        self.detection_target = v["detection_target"]
        """
        node_settings:
          timer_period: 0.1
        servo:
          min_deg: 30.0
          max_deg: 150.0
          neutral_deg: 90.0
        navigation:
          waypoints:
            - [35.20417994, 129.21250807]
            - [35.20419992, 129.21243434]
            - [35.20416613, 129.21225879]
          arrival_radius:
            - 0.5
            - 2.0
            - 1.5
        thruster:
          course1: 25.0
          course2: 25.0
          course3: 15.0
        vision:
          screen_width: 640
          angle_conversion_factor: 90.0
          available_objects:
            - "Blue Circle"
            - "Blue Cross"
            - "Blue Triangle"
            - "Green Circle"
            - "Green Cross"
            - "Green Triangle"
            - "Purple Buoy"
            - "Red Circle"
            - "Red Cross"
            - "Red Triangle"
          hoping_target: "Purple Buoy"
          detection_target: "Blue Circle"
        """

    def normalize_180(self, deg):
        return (deg + 180.0) % 360.0 - 180.0
    
    def led_by_name(self, name):
        name = norm_text(name)
        r, g, b, w = 0, 0, 0, 0
        pub_name = "off"
        for color in ["blue", "green", "red", "white"]:
            if name.startswith(color):
                pub_name = color
                if color == "red": g = 200
                elif color == "green": r = 200
                elif color == "blue": b = 200
                elif color == "white": w = 200
                break
        self.led_publisher.publish(RgbwLedColor(red=r, green=g, blue=b, white=w))
        self.led_string_publisher.publish(String(data=pub_name))

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
        self.curr_yaw_publisher.publish(Float64(data=float(rel_yaw_deg)))
        current_time = time.time()
        if self.phase == "HOPING" and -10.0 < rel_yaw_deg < 0.0:
            if (current_time - self.last_zero_time) > self.zero_count_cooldown:
                self.yaw_zero_count += 1
                self.last_zero_time = current_time
                if self.yaw_zero_count >= 2:
                    self.zero_count_publisher.publish(String(data=str(self.yaw_zero_count)))
                    self.led_by_name("off")
                    self.phase = "DETECTION"

    def lidar_callback(self, data):
        if self.current_yaw_rel is None:
            return
        if self.phase == "WALL":
            self.side_margin = 20
            self.dist_threshold = 1.8 if -180.0 <= self.current_yaw_rel <= -60.0 else 1.2
            n_left, n_right = self.side_margin, max(0, self.side_margin)
        else:
            n_left, n_right = self.side_margin, self.side_margin - 5
        ranges = np.array(data.ranges[500:1500])
        num_samples = len(ranges)
        self.dist_180 = np.zeros(181)
        if self.phase == "WALL":
            self.dist_180.fill(np.inf)
            for i, length in enumerate(ranges):
                if data.range_min < length < data.range_max and np.isfinite(length):
                    angle_index = round((num_samples - 1 - i) * 180 / num_samples)
                    if 0 <= angle_index <= 180 and length < self.dist_180[angle_index]:
                        self.dist_180[angle_index] = length
            self.dist_180[np.isinf(self.dist_180)] = 0.0
        else:
            cumulative_distance = np.zeros(181)
            sample_count = np.zeros(181)
            for i, length in enumerate(ranges):
                if data.range_min < length < data.range_max and np.isfinite(length):
                    angle_index = round((num_samples - 1 - i) * 180 / num_samples)
                    if 0 <= angle_index <= 180:
                        cumulative_distance[angle_index] += length
                        sample_count[angle_index] += 1
            valid_mask = sample_count > 0
            self.dist_180[valid_mask] = cumulative_distance[valid_mask] / sample_count[valid_mask]
        danger_flags = (self.dist_180 > 0) & (self.dist_180 <= self.dist_threshold)
        expanded_danger = np.copy(danger_flags)
        for i in np.where(danger_flags)[0]:
            low = max(0, i - n_left)
            high = min(181, i + n_right + 1)
            expanded_danger[low:high] = True
        self.safe_angles_list = [int(deg) for deg in range(181) if not expanded_danger[deg]]
        if self.safe_angles_list is not None:
            self.safe_angle_list_publisher.publish(String(data=str(self.safe_angles_list)))
        if self.safe_angles_list:
            safe_arr = np.array(self.safe_angles_list)
            best_angle = safe_arr[np.argmin(np.abs(safe_arr - 90))]
            self.safe_angle_publisher.publish(Float64(data=float(best_angle)))

    def gps_callback(self, gps: NavSatFix):
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

            self.dist_publisher.publish(Float64(data=float(self.dist_to_goal_m)))
            self.rel_deg_publisher.publish(Float64(data=float(self.goal_rel_deg)))

    def update_current_goal(self):
        if self.wp_index < len(self.waypoints):
            target_lat, target_lon = self.waypoints[self.wp_index]
            self.current_goal_enu = self.gps_enu_converter([target_lat, target_lon, 0.0])
            goal_msg = NavSatFix()
            goal_msg.latitude = target_lat
            goal_msg.longitude = target_lon
            self.goal_publisher.publish(goal_msg)
            if self.wp_index == 1 and self.phase == "HOPING":
                self.get_logger().info("웨이포인트 도달")
            elif self.wp_index == 2:
                self.get_logger().info("웨이포인트 도달")
            self.get_logger().info(f"웨이포인트 목표: {self.wp_index+1}/{len(self.waypoints)}")

    def detection_callback(self, msg):
        self.latest_det = msg

    def timer_callback(self):
        if self.phase == "GPS":
            self.state_publisher.publish(String(data="GPS 모드"))
            elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            if not self.arrived_all and self.wp_index == 0:
                lat, lon = self.waypoints[self.wp_index]
                self.goal_publisher.publish(NavSatFix(latitude=lat, longitude=lon))
            if self.arrived_all or self.dist_to_goal_m is None or self.goal_rel_deg is None:
                self.cmd_thruster = 0.0
                self.cmd_key_degree = self.servo_neutral_deg
            else:
                if elapsed_time >= 40.0:
                    if self.latest_det and self.latest_det.detections:
                        for d in self.latest_det.detections:
                            c_id = int(d.results[0].hypothesis.class_id)
                            if norm_text(self.available_objects[c_id]) == norm_text(self.hoping_target) or self.dist_to_goal_m <= self.arrival_radii[0]:
                                self.wp_index = 1
                                self.phase = "HOPING"
                                self.update_current_goal()
                if self.safe_angles_list:
                    safe_angles_deg = np.array(self.safe_angles_list) - 90
                    diff = np.abs(safe_angles_deg - self.goal_rel_deg)
                    best_idx = np.argmin(diff)
                    chosen_safe_angle = safe_angles_deg[best_idx]
                    steering_angle = self.servo_neutral_deg + chosen_safe_angle
                    self.cmd_thruster = self.default_thruster
                    self.cmd_key_degree = constrain(steering_angle, self.servo_min_deg, self.servo_max_deg)
                else:
                    self.cmd_thruster = 20.0
                    self.cmd_key_degree = self.servo_neutral_deg
            self.key_publisher.publish(Float64(data=float(self.cmd_key_degree)))
            self.thruster_publisher.publish(Float64(data=float(self.cmd_thruster)))

        elif self.phase == "HOPING":
            self.state_publisher.publish(String(data="Hoping 모드"))
            self.led_by_name("blue")
            msg = f"0도 통과 횟수: {self.yaw_zero_count}"
            if self.yaw_zero_count <= 2:
                self.zero_count_publisher.publish(String(data=msg))
            self.key_publisher.publish(Float64(data=40.0))
            for d in self.latest_det.detections:
                c_id = int(d.results[0].hypothesis.class_id)
                target_name = self.available_objects[c_id]
                if norm_text(self.available_objects[c_id]) == norm_text(self.hoping_target):
                    cx = float(d.bbox.center.position.x if hasattr(d.bbox.center, 'position') else d.bbox.center.x)
                    ang = ((cx - (self.screen_width/2)) / (self.screen_width/2)) * self.angle_factor
                    self.target_name_publisher.publish(String(data=target_name))
                    self.target_angle_publisher.publish(Float64(data=ang))
                    self.key_publisher.publish(Float64(data=40.0 if ang <= 10.0 else self.servo_neutral_deg))
                        
        elif self.phase == "DETECTION":
            self.state_publisher.publish(String(data="Detection 모드"))
            error = 5.0 - self.current_yaw_rel
            steer = self.servo_neutral_deg + error
            self.key_publisher.publish(Float64(data=constrain(steer, self.servo_min_deg, self.servo_max_deg)))
            if self.latest_det and self.latest_det.detections:
                for d in self.latest_det.detections:
                    c_id = int(d.results[0].hypothesis.class_id)
                    name = self.available_objects[c_id]
                    self.target_name_publisher.publish(String(data=self.detection_target))
                    if norm_text(name) == norm_text(self.detection_target):
                        cx = float(d.bbox.center.position.x if hasattr(d.bbox.center, 'position') else d.bbox.center.x)
                        ang = ((cx - (self.screen_width/2)) / (self.screen_width/2)) * self.angle_factor
                        self.target_angle_publisher.publish(Float64(data=ang))
                        color_name = name
                        if ang <= 30.0:
                            self.led_by_name(color_name)
                            self.phase = "DONE"

        elif self.phase == "DONE":
            self.state_publisher.publish(String(data="Done 모드"))
            lat, lon = self.waypoints[1]
            self.goal_publisher.publish(NavSatFix(latitude=lat, longitude=lon))
            if self.arrived_all or self.dist_to_goal_m is None:
                self.cmd_thruster = self.default_thruster
                self.cmd_key_degree = self.servo_neutral_deg
            else:
                current_radius = self.arrival_radii[1]
                if self.dist_to_goal_m <= current_radius:
                    self.wp_index = 2
                    self.led_by_name("off")
                    self.phase = "WALL"
                    self.update_current_goal()
            error = -90.0 - self.current_yaw_rel
            steer = self.servo_neutral_deg + error
            self.key_publisher.publish(Float64(data=constrain(steer, self.servo_min_deg, self.servo_max_deg)))
            self.thruster_publisher.publish(Float64(data=float(self.cmd_thruster)))
        
        elif self.phase == "WALL":
            self.state_publisher.publish(String(data="Wall 모드"))
            lat, lon = self.waypoints[2]
            self.goal_publisher.publish(NavSatFix(latitude=lat, longitude=lon))
            if self.arrived_all or self.dist_to_goal_m is None or self.goal_rel_deg is None:
                self.cmd_thruster = 20.0
                self.cmd_key_degree = self.servo_neutral_deg
            else:
                if self.dist_to_goal_m <= self.arrival_radii[2]:
                    self.led_by_name("green")
                    self.cmd_thruster = 0.0
                    self.cmd_key_degree = self.servo_neutral_deg
                    for _ in range(20):
                        self.key_publisher.publish(Float64(data=float(self.cmd_key_degree)))
                        self.thruster_publisher.publish(Float64(data= -20.0))
                        time.sleep(0.1)
                    self.key_publisher.publish(Float64(data=float(self.cmd_key_degree)))
                    self.thruster_publisher.publish(Float64(data = 0.0))
                    self.get_logger().info("최종 웨이포인트 도달")
                    self.destroy_node()
                    sys.exit(0)
                if self.safe_angles_list:
                    safe_angles_deg = np.array(self.safe_angles_list) - 90
                    diff = np.abs(safe_angles_deg - self.goal_rel_deg)
                    best_idx = np.argmin(diff)
                    chosen_safe_angle = safe_angles_deg[best_idx]
                    steering_angle = self.servo_neutral_deg + chosen_safe_angle
                    self.cmd_thruster = self.default_thruster
                    self.cmd_key_degree = constrain(steering_angle, self.servo_min_deg, self.servo_max_deg)
                else:
                    self.cmd_thruster = 20.0
                    self.cmd_key_degree = self.servo_neutral_deg +15
            self.key_publisher.publish(Float64(data=float(self.cmd_key_degree)))
            self.thruster_publisher.publish(Float64(data=float(self.cmd_thruster)))

    def send_stop_commands(self):
        if not rclpy.ok(): return
        for _ in range(5):
            self.key_publisher.publish(Float64(data=float(self.servo_neutral_deg)))
            self.thruster_publisher.publish(Float64(data=0.0))
            self.led_by_name("off")
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = ISV_2026()
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
