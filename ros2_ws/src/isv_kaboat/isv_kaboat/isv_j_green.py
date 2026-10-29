import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    qos_profile_sensor_data,
)

import numpy as np

np.float = float  # NumPy의 float 정의 복구

from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan, Imu
import math
from std_msgs.msg import Float32
from tf_transformations import euler_from_quaternion
from math import degrees
import time
from isv_kaboat.obstacle_isv import calculate_angle_risk
from isv_kaboat.ISV_LiDAR_YDH_LIST import detect_and_cluster

from vision_msgs.msg import Detection2DArray
from mechaship_interfaces.msg import RgbwLedColor


class MotorControlNode(Node):
    def __init__(self):
        super().__init__(
            "pid_go",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        self.get_logger().info("----- start Hoping node -----")
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1,
        )

        self.imu_sub = self.create_subscription(
            Imu,
            "/imu",
            self.imu_callback,
            qos_profile_sensor_data,
        )

        self.subscription3 = self.create_subscription(
            LaserScan, "/scan", self.lidar_listener_callback, qos_profile
        )

        self.scan_publisher = self.create_publisher(
            LaserScan, "/scan_filtered", qos_profile
        )

        self.detection_subscriber = self.create_subscription(
            Detection2DArray,
            "/detections",
            self.detection_callback,
            qos_profile_sensor_data,
        )

        self.create_timer(0.1, self.timer_callback)

        self.key_publisher = self.create_publisher(Float32, "/actuator/key/degree", 10)
        self.thruster_publisher = self.create_publisher(
            Float32, "/actuator/thruster/percentage", 10
        )

        self.led_publisher = self.create_publisher(
            RgbwLedColor, "/actuator/rgbwled/color", 10
        )

        self.target_object = (
            "green circle"  # "green circle"  "red triangle"        # 추적할 객체 이름
        )
        self.object_detected = False  # 객체 탐지 여부
        self.target_x = None  # 화면에서의 객체 중심 X좌표

        self.target_angle = None
        self.last_time = time.time()
        self.now_heading = None
        self.angle_to_goal = 0.0
        self.angle_increment = None
        self.status = 0
        self.stop_count = 0
        self.stop_status = 0
        self.thruster_percentage = 40.0

        self.start_time = time.time()  # 추가된 시작 시간 설정

    def imu_callback(self, msg: Imu):
        quaternion = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        roll_rad, pitch_rad, yaw_rad = euler_from_quaternion(quaternion)
        roll_degree = degrees(roll_rad)
        pitch_degree = degrees(pitch_rad)
        yaw_degree = degrees(yaw_rad)
        self.now_heading = yaw_degree

    def lidar_listener_callback(self, data):
        self.angle_increment = math.degrees(data.angle_increment)
        scan_data = data.ranges
        filtered_range_list = detect_and_cluster(scan_data)

        filtered_scan_msg = LaserScan()
        filtered_scan_msg.header = data.header
        filtered_scan_msg.angle_min = data.angle_min
        filtered_scan_msg.angle_max = data.angle_max
        filtered_scan_msg.angle_increment = data.angle_increment
        filtered_scan_msg.time_increment = data.time_increment
        filtered_scan_msg.scan_time = data.scan_time
        filtered_scan_msg.range_min = data.range_min
        filtered_scan_msg.range_max = data.range_max
        filtered_scan_msg.ranges = filtered_range_list
        filtered_scan_msg.intensities = data.intensities
        self.scan_publisher.publish(filtered_scan_msg)

        if self.now_heading is not None:
            current_time = time.time()
            elapsed_time = current_time - self.start_time

            if elapsed_time < 28.75:
                angle_to_goal_rel = (self.now_heading * -1 + 360.0) % 360.0
                self.thruster_percentage = 40.0
            elif 28.75 <= elapsed_time < 37.25:
                angle_to_goal_rel = (self.now_heading * -1 + 90.0) % 360.0
                self.thruster_percentage = 35.0
            elif 37.25 <= elapsed_time < 9000:
                angle_to_goal_rel = (self.now_heading * -1 + 180.0) % 360.0
                self.thruster_percentage = 22.5
            elif 900 <= elapsed_time < 95:
                self.thruster_percentage = -10.0
                self.get_logger().info("Thruster output set to reverse.")
                return
            else:
                self.thruster_percentage = 0.0
                self.get_logger().info("Thruster output set to 0. Stopping operations.")
                return

            self.target_angle = calculate_angle_risk(
                filtered_range_list,
                angle_to_goal_rel,
                0,
                10000000,
                0,
                7,
                self.angle_increment,
                1,
            )
            if self.target_angle > 180:
                self.target_angle = (360 - self.target_angle) * -1

            # self.get_logger().info(f"Current Time: {elapsed_time:.2f} seconds")
            # # self.get_logger().info(f"Target Angle: {self.target_angle}")
            # # self.get_logger().info(f"Angle to Goal: {angle_to_goal_rel}")
            # self.get_logger().info(f"Key.degree : {self.key_degree}")

    def timer_callback(self):
        key_msg = Float32()
        thruster_msg = Float32()

        # LED 초기 상태: White
        led_msg = RgbwLedColor()
        led_msg.red = 0
        led_msg.green = 0
        led_msg.blue = 0
        led_msg.white = 20

        # 1. 목표 타겟이 인식되었는지 확인
        if self.object_detected and self.target_x is not None:
            screen_width = 640  # 가정된 카메라 해상도
            angle_offset = (
                ((self.target_x - (screen_width / 2)) / (screen_width / 2)) * 90 * 0.3
            )
            self.target_angle = -angle_offset  # 목표 타겟 중심을 기준으로 각도 설정
            self.get_logger().info(f"Target Angle from Vision: {self.target_angle}")

            # LED 색상 설정
            if self.target_object in ["red circle", "blue triangle", "green cross"]:
                led_msg.red = 20
            elif self.target_object in ["blue circle", "green triangle", "red cross"]:
                led_msg.blue = 20
            elif self.target_object in ["green circle", "red triangle", "blue cross"]:
                led_msg.green = 20
            led_msg.white = 0

        # 2. 타겟 각도가 None이 아닐 경우 서보모터 제어

        if self.target_angle is not None:
            error = (
                90.0 - self.target_angle
            )  # key_degree 구할 때, target_angle에 배율 고렿해보기 (0.5, 0.25)
            error = check_key_limit(error)  # 각도를 제한
            self.key_degree = error
        else:
            self.key_degree = 90.0

        # 3. 서보모터 및 스러스터 출력
        key_msg.data = self.key_degree
        thruster_msg.data = self.thruster_percentage
        self.key_publisher.publish(key_msg)
        self.thruster_publisher.publish(thruster_msg)

        # 4. LED 상태 발행
        self.led_publisher.publish(led_msg)

        # 5. 로깅 정보 출력
        elapsed_time = time.time() - self.start_time
        self.get_logger().info(f"Key Degree: {self.key_degree}")
        self.get_logger().info(f"Current Time: {elapsed_time:.2f} seconds")
        # self.get_logger().info(f"Target Object: {self.target_object}")
        # self.get_logger().info(f"Object Detected: {self.object_detected}")

    def detection_callback(self, msg: Detection2DArray):
        target_detection = None
        for detection in msg.detections:
            class_id = int(detection.results[0].hypothesis.class_id)
            object_name = self.get_object_name(class_id)
            if object_name == self.target_object:
                target_detection = detection
                break

        if not target_detection:
            self.target_x = None
            self.object_detected = False
            self.get_logger().info(
                f"No object detected. Target object: {self.target_object}"
            )
            return

        bbox = target_detection.bbox
        self.target_x = bbox.center.position.x
        self.object_detected = True

        self.get_logger().info(
            f"Detected object: {object_name}, Target object: {self.target_object}"
        )

    def get_object_name(self, class_id):
        CLASSES = [
            "blue circle",
            "blue cross",
            "blue triangle",
            "green circle",
            "green cross",
            "green triangle",
            "red circle",
            "red cross",
            "red triangle",
        ]
        return CLASSES[class_id]


def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    return input_vel


def check_key_limit(vel):
    return constrain(vel, 47.5, 132.5)


def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        key_msg = Float32()
        key_msg.data = 90.0  # 서보모터 중립
        thruster_msg = Float32()
        thruster_msg.data = 0.0  # 스러스터 정지
        node.key_publisher.publish(key_msg)
        node.thruster_publisher.publish(thruster_msg)
        node.get_logger().info("Shutdown: All motors set to neutral.")
        rclpy.shutdown()


if __name__ == "__main__":
    main()


# 그린 가는 최종 코드
