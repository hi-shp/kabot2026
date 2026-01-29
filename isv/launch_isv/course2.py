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
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float64, String
from mechaship_interfaces.msg import RgbwLedColor
from tf_transformations import euler_from_quaternion

def constrain(v, lo, hi):
    if math.isnan(v): return lo + (hi - lo) / 2.0
    return lo if v < lo else hi if v > hi else v

def norm_text(s):
    return " ".join(str(s).strip().lower().split())

class Course2(Node):
    def __init__(self):
        super().__init__("Course2")
        self.load_params_from_yaml()
        self.key_publisher = self.create_publisher(Float64, "/actuator/key/degree", 10)
        self.thruster_publisher = self.create_publisher(Float64, "/actuator/thruster/percentage", 10)
        self.curr_yaw_publisher = self.create_publisher(Float64, "/current_yaw", 10)
        self.rel_deg_publisher = self.create_publisher(Float64, "/waypoint/rel_deg", 10)
        self.goal_publisher = self.create_publisher(NavSatFix, "/waypoint/goal", 10)
        self.dist_publisher = self.create_publisher(Float64, "/waypoint/distance", 10)
        self.led_publisher = self.create_publisher(RgbwLedColor,"/actuator/rgbwled/color",10)
        self.led_string_publisher = self.create_publisher(String, "/led_color", 10)
        self.target_name_publisher = self.create_publisher(String, "/target_name", 10)
        self.target_angle_publisher = self.create_publisher(Float64, "/target_angle", 10)
        self.state_publisher = self.create_publisher(String, "/state", 10)
        self.zero_count_publisher = self.create_publisher(String, "/zero_count", 10)
        self.imu_23_publisher = self.create_publisher(Float64, '/imu/two_three', 10)
        self.imu_sub = self.create_subscription(Imu, "/imu", self.imu_callback, qos_profile_sensor_data)
        self.det_sub = self.create_subscription(Detection2DArray, "/detections", self.detection_callback, qos_profile_sensor_data)
        self.gps_sub = self.create_subscription(NavSatFix, "/gps/fix", self.gps_callback, qos_profile_sensor_data)
        self.init_yaw_sub = self.create_subscription(Float64, "/imu/target_yaw", self.init_yaw_callback, 10)
        self.latest_det = None
        self.initial_yaw_abs = None
        self.yaw_offset = None
        self.origin = None
        self.origin_set = False
        self.dist_to_goal_m = None
        self.goal_rel_deg = None  
        self.arrived_all = False
        self.wp_index = 1
        self.current_goal_enu = None
        self.current_yaw_rel = 0.0
        self.yaw_zero_count = 0
        self.yaw_zero_latched = False
        self.last_zero_time = 0.0
        self.zero_count_cooldown = 5.0
        self.cmd_key_degree = self.servo_neutral_deg
        self.cmd_thruster = self.default_thruster
        self.phase = "IMU"
        self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info("Course 2")

    def load_params_from_yaml(self):
        path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "isv_params.yaml")
        with open(path, "r", encoding='utf-8') as f:
            params = yaml.safe_load(f)
        self.timer_period = float(params["node_settings"]["timer_period"])
        self.servo_neutral_deg = float(params["servo"]["neutral_deg"])
        self.servo_min_deg = float(params["servo"]["min_deg"])
        self.servo_max_deg = float(params["servo"]["max_deg"])
        self.waypoints = params["navigation"]["waypoints"]
        self.arrival_radii = params["navigation"]["arrival_radius"]
        self.default_thruster = float(params["thruster"]["course2"])
        v = params["vision"]
        self.screen_width = int(v["screen_width"])
        self.angle_factor = float(v["angle_conversion_factor"])
        self.available_objects = v["available_objects"]
        self.hoping_target = v["hoping_target"]
        self.detection_target = v["detection_target"]
        
    def init_yaw_callback(self, msg: Float64):
        if self.initial_yaw_abs is None:
            self.initial_yaw_abs = msg.data
            self.destroy_subscription(self.init_yaw_sub)

    def normalize_180(self, deg):
        return (deg + 180.0) % 360.0 - 180.0

    def led_by_name(self, name):
        name = norm_text(name)
        r, g, b, w = 0, 0, 0, 0
        pub_name = "off"
        for color in ["blue", "green", "red", "white"]:
            if name.startswith(color):
                pub_name = color
                if color == "red": r = 100
                elif color == "green": g = 100
                elif color == "blue": b = 100
                elif color == "white": w = 100
                break
        self.led_publisher.publish(RgbwLedColor(red=r, green=g, blue=b, white=w))
        self.led_string_publisher.publish(String(data=pub_name))

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
        current_time = time.time()
        if self.phase == "HOPING" and abs(rel_yaw_deg) < 5.0:
            if (current_time - self.last_zero_time) > self.zero_count_cooldown:
                self.yaw_zero_count += 1
                self.last_zero_time = current_time
                if self.yaw_zero_count >= 2:
                    self.led_by_name("off")
                    self.phase = "DETECTION"

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

    def update_current_goal(self):
        if self.wp_index == 1:
            target_lat, target_lon = self.waypoints[self.wp_index]
            self.current_goal_enu = self.gps_enu_converter([target_lat, target_lon, 0.0])
            goal_msg = NavSatFix()
            goal_msg.latitude = target_lat
            goal_msg.longitude = target_lon
            self.goal_publisher.publish(goal_msg)
            self.get_logger().info(f"웨이포인트 목표: {self.wp_index+1}/{len(self.waypoints)}")
        else:
            self.cmd_thruster = 0.0
            self.cmd_key_degree = self.servo_neutral_deg
            self.key_publisher.publish(Float64(data=float(self.cmd_key_degree)))
            self.thruster_publisher.publish(Float64(data=float(self.cmd_thruster)))
            self.imu_23_publisher.publish(Float64(data=float(self.current_yaw_rel)))
            self.led_by_name("off")
            self.get_logger().info("웨이포인트 도달")
            self.destroy_node()
            sys.exit(0)

    def detection_callback(self, msg):
        self.latest_det = msg

    def timer_callback(self):
        if self.dist_to_goal_m is not None:
            self.dist_publisher.publish(Float64(data=float(self.dist_to_goal_m)))
        if self.initial_yaw_abs is None:
            return
        self.cmd_thruster = self.default_thruster
        self.thruster_publisher.publish(Float64(data=self.cmd_thruster))
        if self.phase == "IMU":
            error = 120.0 - self.current_yaw_rel
            steer = self.servo_neutral_deg + error
            self.key_publisher.publish(Float64(data=constrain(steer, self.servo_min_deg, self.servo_max_deg)))
            self.rel_deg_publisher.publish(Float64(data=120.0))
            if self.latest_det and self.latest_det.detections:
                for d in self.latest_det.detections:
                    c_id = int(d.results[0].hypothesis.class_id)
                    target_name = self.available_objects[c_id]
                    if norm_text(self.available_objects[c_id]) == norm_text(self.hoping_target):
                        self.phase = "HOPING"
        msg = f"0도 통과 횟수: {self.yaw_zero_count}"
        if self.yaw_zero_count <= 2:
            self.zero_count_publisher.publish(String(data=msg))
        if not self.latest_det or not self.latest_det.detections: return
        if self.phase == "HOPING":
            self.state_publisher.publish(String(data="Hoping 모드"))
            self.led_by_name("white")
            for d in self.latest_det.detections:
                c_id = int(d.results[0].hypothesis.class_id)
                target_name = self.available_objects[c_id]
                if norm_text(self.available_objects[c_id]) == norm_text(self.hoping_target):
                    cx = float(d.bbox.center.position.x if hasattr(d.bbox.center, 'position') else d.bbox.center.x)
                    ang = ((cx - (self.screen_width/2)) / (self.screen_width/2)) * self.angle_factor
                    self.target_name_publisher.publish(String(data=target_name))
                    self.target_angle_publisher.publish(Float64(data=ang))
                    self.key_publisher.publish(Float64(data=40.0 if ang <= -30.0 else self.servo_neutral_deg))
        elif self.phase == "DETECTION":
            self.state_publisher.publish(String(data="Detection 모드"))
            error = 0.0 - self.current_yaw_rel
            steer = self.servo_neutral_deg + error
            self.key_publisher.publish(Float64(data=constrain(steer, self.servo_min_deg, self.servo_max_deg)))
            self.rel_deg_publisher.publish(Float64(data=0.0))
            if self.latest_det and self.latest_det.detections:
                for d in self.latest_det.detections:
                    c_id = int(d.results[0].hypothesis.class_id)
                    name = self.available_objects[c_id]
                    self.target_name_publisher.publish(String(data=self.detection_target))
                    if norm_text(name) == norm_text(self.detection_target):
                        cx = float(d.bbox.center.position.x if hasattr(d.bbox.center, 'position') else d.bbox.center.x)
                        ang = ((cx - (self.screen_width/2)) / (self.screen_width/2)) * self.angle_factor
                        self.target_angle_publisher.publish(Float64(data=ang))
                        if ang <= 5.0:
                            self.led_by_name(name)
                            self.phase = "DONE"
                            break
            else:
                self.target_angle_publisher.publish(Float64(data=0.0))
        elif self.phase == "DONE":
            self.state_publisher.publish(String(data="Done 모드"))
            if not self.arrived_all and self.wp_index == 1:
                lat, lon = self.waypoints[self.wp_index]
                self.goal_publisher.publish(NavSatFix(latitude=lat, longitude=lon))
            if self.arrived_all or self.dist_to_goal_m is None:
                self.cmd_thruster = 0.0
                self.cmd_key_degree = self.servo_neutral_deg
            else:
                current_radius = self.arrival_radii[1]
                if self.dist_to_goal_m <= current_radius:
                    self.wp_index += 1
                    self.update_current_goal()
            error = -90.0 - self.current_yaw_rel
            steer = self.servo_neutral_deg + error
            self.key_publisher.publish(Float64(data=constrain(steer, self.servo_min_deg, self.servo_max_deg)))
            self.rel_deg_publisher.publish(Float64(data=-90.0))

    def send_stop_commands(self):
        if not rclpy.ok(): return
        for _ in range(5):
            self.key_publisher.publish(Float64(data=float(self.servo_neutral_deg)))
            self.thruster_publisher.publish(Float64(data=0.0))
            self.led_by_name("off")
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = Course2()
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

if __name__ == "__main__": main()