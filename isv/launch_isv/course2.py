import os
import yaml
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from mechaship_interfaces.msg import RgbwLedColor


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v

class course2(Node):
    def __init__(self):
        super().__init__("course2")
        self._load_params_from_yaml()
        
        self.detection_subscriber = self.create_subscription(
            Detection2DArray,
            "/detection",
            self.detection_cb,
            qos_profile_sensor_data,
        )

        self.key_publisher = self.create_publisher(Float64, "/actuator/key/degree", 10)
        self.thruster_publisher = self.create_publisher(Float64, "/actuator/thruster/percentage", 10)
        self.led_pub = self.create_publisher(RgbwLedColor, "/rgb", 10)
        
        
        self.create_subscription(Imu, '/imu', self.imu_callback, qos_profile_sensor_data)

        self.last_target_time = None
        self.servo_neutral_deg =90.0
        self.current_led = self.make_led(white=self.led_default_white)


    def wrap_deg_180(self, angle_deg: float) -> float:
        a = (angle_deg + 180.0) % 360.0 - 180.0
        if a == -180.0:
            a = 180.0
        return a

    def quat_to_yaw_deg(self, q) -> float:
        x, y, z, w = q.x, q.y, q.z, q.w
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return math.degrees(yaw)

    def imu_callback(self, msg):
        yaw_deg = self.quat_to_yaw_deg(msg.orientation)
        yaw_deg = self.wrap_deg_180(yaw_deg)

        self.get_logger().info(f"yaw: {yaw_deg:.1f} deg")\
        

    
