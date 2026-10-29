import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    qos_profile_sensor_data,
)
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan, NavSatFix, Imu
import math
from std_msgs.msg import Float32
from simple_pid import PID
import pymap3d as pm
import time

from isv_kaboat.obstacle_isv import (
    calculate_angle_risk,
)  # 라이다 노이즈 제거, 목표각 도출
from isv_kaboat.ISV_LiDAR_YDH_LIST import detect_and_cluster

# import numpy as np
# from queue import Queue
# import time
from mechaship_interfaces.msg import Heading

from mechaship_interfaces.srv import Key, ThrottlePercentage, RGBColor, ThrottlePulseWidth
# from filterpy.kalman import KalmanFilter


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

        self.subscription1 = self.create_subscription(
            NavSatFix, "fix", self.gps_listener_callback, qos_profile
        )
        # self.subscription1

        self.subscription2 = self.create_subscription(
            Heading, "/heading", self.imu_callback, qos_profile
        )
        # self.subscription2

        self.subscription3 = self.create_subscription(
            LaserScan, "/scan", self.lidar_listener_callback, qos_profile
        )
        # self.subscription3

        self.scan_publisher = self.create_publisher(
            LaserScan, "/scan_filtered", qos_profile
        )  # Filtered scan publisher

        self.set_throttle_handler_left = self.create_client(
            ThrottlePulseWidth, "/actuators/throttle_left/set_pulse_width"
        )

        self.set_throttle_handler_right = self.create_client(
            ThrottlePulseWidth, "/actuators/throttle_right/set_pulse_width"
        )
        # Wait for the service to be available
        self.set_throttle_handler_left.wait_for_service()
        self.set_throttle_handler_right.wait_for_service()

        self.origin = [35.2316117, 129.08254347, 0.0]  # 수정 예정

        self.angle_pid = PID(24.0 / 3, 0, 0)  # 90기준 300
        self.angle_pid.output_limits = (-400, 400)  # 출력 범위 제한(임의)

        self.target_angle = None

        self.last_time = time.time()

        self.now_heading = None
        self.angle_to_goal = None
        self.angle_increment = None

        # self.goal_gps_coords = [
        #     (35.2315693, 129.08240824), #35.23161553,129.08243432999998
        #     (35.2315665299999, 129.08239976),
        #     (35.23154693, 129.0824671499999),
        #     (35.23161553, 129.08243432999998)
        # ]
        self.goal_gps_coords = [
            (35.23163124, 129.08247798),  # 35.23158747, 129.08245525
            (35.2315962, 129.08246026),
            (35.23157352, 129.08252906),
            (35.2316117, 129.08254347),
        ]

        # 결과를 저장할 리스트 초기화
        self.goal_xy_coords = []

        # 각 GPS 좌표를 변환하여 goal_xy_coords에 추가
        for coord in self.goal_gps_coords:
            # ENU 좌표로 변환
            e, n = self.gps_enu_converter((coord[0], coord[1], 0))  # 고도를 0으로 설정
            # X, Y 좌표로 변환
            # x, y = self.get_xy(e, n)
            # 변환된 좌표를 리스트에 추가
            self.goal_xy_coords.append((e, n))

        self.now_goal = self.goal_xy_coords[0]

        self.status = 0

        self.MIN_PULSE_WIDTH = 1395
        self.MAX_PULSE_WIDTH = 1900

        self.stop_count = 0
        self.stop_status = 0

    def pid_angle(self):
        # 시계방향으로 돌릴지 반시계방향으로 돌릴지 판단 코드
        # 시작 각도와 목표 각도 잡고 pid 제어
        # 모터를 반대 방향으로 같은 크기로 돌리는데 그 크기를 각도 오차를 통해서 pid 제어
        # 이때는 비례상수가 최대 차이 180도 일 때 임의의 같은 출력(ex 60출력) -> 비례상수 1/3수준으로 지정

        # 클러스터링 후 목표 각만 받아와서 self.target_angle이라고 설정하면 됨
        # 여기서 self.target_angle값이 있으면 절대 좌표계 기준으로 각도를 맞춰준 후 error 계산이 되어야함

        # error = self.angle_difference(self.now_heading, self.target_angle)
        error = self.target_angle
        self.angle_pid.setpoint = 0
        #        if error > 5:
        #            error += 30
        #        elif error <-5:
        #            error -= 30
        current_time = time.time()
        dt = current_time - self.last_time

        # error가 양수이면 시계방향 회전, 음수이면 반시계방향 회전
        motor_speed = self.angle_pid(abs(error), dt)  # abs 맞나?
        self.last_time = current_time
        # print("here")
        if error > 0:
            self.turn_angle(abs(motor_speed), 1)
        else:
            self.turn_angle(abs(motor_speed), 2)
        print(f"error : {error}")
        # print(f"motor speed : {motor_speed}")

    def imu_callback(self, data):

        self.now_heading = (
            360 - (data.yaw + 360.0) % 360.0
        )  # 수정 예정___________북쪽 기준

    def gps_listener_callback(self, gps):
        # self.get_logger().info('gps data: "%s"' % gps)
        e, n = self.gps_enu_converter([gps.latitude, gps.longitude, gps.altitude])
        # x, y = self.get_xy(e,n)

        # 목표 좌표에서의 X, Y 값
        goal_x, goal_y = self.now_goal
        delta_x = goal_x - e
        delta_y = goal_y - n

        angle_rad = math.atan2(delta_x, delta_y)  # 북쪽 기준
        angle_deg = math.degrees(angle_rad)
        # 절대 좌표계 기준 목표 지점으로의 각도
        self.angle_to_goal = (angle_deg + 360) % 360

        # 목표에 도착했을 때 좌표 바꾸는거 추가 예정
        distance = math.sqrt(delta_x**2 + delta_y**2)

        # print(f"@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
        # print(f"goal distance : {distance}")
        # print(f"@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")

        if distance <= 1:
            self.stop_status = 1
            print("-------------------------------------------------")
            print("goal!")
            print("-------------------------------------------------")
            self.status += 1
            self.now_goal = self.goal_xy_coords[self.status]

        # print(f"goal : {self.angle_to_goal}, Yaw : {self.now_heading}")

    def lidar_listener_callback(self, data):

        self.angle_increment = math.degrees(
            data.angle_increment
        )  # 한번만 넣어주거나 값으로 바로 넣거나, 라디안 단위인지 확인, 그리고 뒤에 angle_to_rel이나 이런거 None일 수 있으니 확인

        scan_data = data.ranges

        filtered_range_list = detect_and_cluster(scan_data)

        # 새로운 LaserScan 메시지 생성
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
        filtered_scan_msg.intensities = data.intensities  # 필요에 따라 수정 가능

        # Publish the filtered scan
        self.scan_publisher.publish(filtered_scan_msg)

        if self.now_heading != None and self.angle_to_goal != None:
            angle_to_goal_rel = self.absolute_to_relative_angle(
                self.now_heading, self.angle_to_goal
            )

            self.target_angle = calculate_angle_risk(
                filtered_range_list,
                angle_to_goal_rel,
                0,
                1000,
                0.3,
                7,
                self.angle_increment,
                1,
            )  # 거리, 각도0.2, 목표물1
            if self.target_angle > 180:
                self.target_angle = (360 - self.target_angle) * -1
            if self.stop_status == 1:
                self.back_motor()
                self.stop_count += 1
                if self.stop_count == 4:
                    self.stop_count = 0
                    self.stop_status = 0
            else:
                self.pid_angle()
                # pass

    def gps_enu_converter(self, gnss):
        e, n, u = pm.geodetic2enu(
            gnss[0], gnss[1], gnss[2], self.origin[0], self.origin[1], self.origin[2]
        )
        return e, n

    def angle_difference(self, a, b):
        diff = (b - a + 180) % 360 - 180
        return diff

    def turn_angle(self, percentage, direction):  # 음수로 역회전을 하는건가?
        throttle = ThrottlePulseWidth.Request()

        if direction == 1:
            left_pulse_width = 1620
            right_pulse_width = 1620 - percentage
        else:
            left_pulse_width = 1620 - percentage
            right_pulse_width = 1620

        # print(f"left : {left_pulse_width}, right : {right_pulse_width}")

        left_pulse_width = self.constrain(
            left_pulse_width, self.MIN_PULSE_WIDTH, self.MAX_PULSE_WIDTH
        )
        right_pulse_width = self.constrain(
            right_pulse_width, self.MIN_PULSE_WIDTH, self.MAX_PULSE_WIDTH
        )

        throttle.pulse_width = int(left_pulse_width)
        self.set_throttle_handler_left.call_async(throttle)

        throttle.pulse_width = int(right_pulse_width)
        self.set_throttle_handler_right.call_async(throttle)

    def absolute_to_relative_angle(self, yaw, absolute_angle):
        relative_angle = (absolute_angle - yaw + 360) % 360
        return relative_angle

    def constrain(self, input_val, low_bound, high_bound):
        return max(min(input_val, high_bound), low_bound)

    def stop_motor(self):
        left_throttle_set = ThrottlePulseWidth.Request()
        right_throttle_set = ThrottlePulseWidth.Request()

        left_throttle_set.pulse_width = 1500
        right_throttle_set.pulse_width = 1500

        left_future = self.set_throttle_handler_left.call_async(left_throttle_set)
        right_future = self.set_throttle_handler_right.call_async(right_throttle_set)

        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(self)

        executor.spin_until_future_complete(left_future)
        executor.spin_until_future_complete(right_future)

        print("Motors stopped")

    def back_motor(self):
        throttle = ThrottlePulseWidth.Request()
        print("back-------------------------")

        left_pulse_width = 1400
        right_pulse_width = 1400

        # print(f"left : {left_pulse_width}, right : {right_pulse_width}")

        left_pulse_width = self.constrain(
            left_pulse_width, self.MIN_PULSE_WIDTH, self.MAX_PULSE_WIDTH
        )
        right_pulse_width = self.constrain(
            right_pulse_width, self.MIN_PULSE_WIDTH, self.MAX_PULSE_WIDTH
        )

        throttle.pulse_width = int(left_pulse_width)
        self.set_throttle_handler_left.call_async(throttle)

        throttle.pulse_width = int(right_pulse_width)
        self.set_throttle_handler_right.call_async(throttle)


def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
        node.stop_motor()
    finally:
        node.stop_motor()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
