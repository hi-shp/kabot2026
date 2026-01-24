#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math
import os

class ImuQuantizedTracker(Node):
    def __init__(self):
        super().__init__('imu_quantized_tracker')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(Imu, '/imu', self.imu_cb, qos_profile)
        
        # --- 캘리브레이션 설정 ---
        self.is_calibrated = False
        self.calib_count = 1000
        self.samples_x, self.samples_y, self.samples_z = [], [], []
        self.offset_x, self.offset_y, self.offset_z = 0.0, 0.0, 0.0
        
        # --- 물리 계산 변수 ---
        self.last_time = None
        self.vel_x, self.vel_y, self.vel_z = 0.0, 0.0, 0.0
        self.dist_x, self.dist_y, self.dist_z = 0.0, 0.0, 0.0
        
        # --- 필터링 및 기준값 ---
        self.deadzone = 0.20    # 가속도 데드존
        self.gravity = 9.807

        self.get_logger().info("Precision Tracker (2-decimal velocity) Ready.")

    def truncate(self, number, digits):
        """소수점 n자리 아래를 버림"""
        stepper = 10.0 ** digits
        return math.trunc(stepper * number) / stepper

    def imu_cb(self, msg: Imu):
        curr_time = self.get_clock().now().nanoseconds / 1e9
        if self.last_time is None:
            self.last_time = curr_time
            return
        dt = curr_time - self.last_time
        self.last_time = curr_time

        if not self.is_calibrated:
            self.perform_calibration(msg)
            return

        # 1. 가속도 보정
        acc_x = msg.linear_acceleration.x - self.offset_x
        acc_y = msg.linear_acceleration.y - self.offset_y
        acc_z = msg.linear_acceleration.z - self.offset_z - self.gravity

        # 2. 가속도 데드존 적용
        if abs(acc_x) < self.deadzone: acc_x = 0.0
        if abs(acc_y) < self.deadzone: acc_y = 0.0
        if abs(acc_z) < self.deadzone: acc_z = 0.0

        # 3. 속도 계산 및 소수점 2자리 제한 (핵심 수정 부분)
        # 3자리부터 무시하여 미세한 누적 오차 방지
        raw_vel_x = self.vel_x + (acc_x * dt)
        raw_vel_y = self.vel_y + (acc_y * dt)
        raw_vel_z = self.vel_z + (acc_z * dt)

        self.vel_x = self.truncate(raw_vel_x, 2)
        self.vel_y = self.truncate(raw_vel_y, 2)
        self.vel_z = self.truncate(raw_vel_z, 2)

        # [ZUPT] 가속도가 0인데 속도가 미세하게 남은 경우 강제 정지
        if acc_x == 0.0 and abs(self.vel_x) < 0.05: self.vel_x = 0.0
        if acc_y == 0.0 and abs(self.vel_y) < 0.05: self.vel_y = 0.0
        if acc_z == 0.0 and abs(self.vel_z) < 0.05: self.vel_z = 0.0

        # 4. 거리 계산
        self.dist_x += self.vel_x * dt
        self.dist_y += self.vel_y * dt
        self.dist_z += self.vel_z * dt

        self.display_data(acc_x, acc_y, acc_z)

    def perform_calibration(self, msg):
        self.samples_x.append(msg.linear_acceleration.x)
        self.samples_y.append(msg.linear_acceleration.y)
        self.samples_z.append(msg.linear_acceleration.z)
        
        if len(self.samples_x) % 100 == 0:
            os.system('clear')
            print(f" Calibration: {len(self.samples_x)} / {self.calib_count}")

        if len(self.samples_x) >= self.calib_count:
            self.offset_x = sum(self.samples_x) / self.calib_count
            self.offset_y = sum(self.samples_y) / self.calib_count
            self.offset_z = (sum(self.samples_z) / self.calib_count) - self.gravity
            self.is_calibrated = True

    def display_data(self, ax, ay, az):
        os.system('clear')
        print("="*60)
        print(f" [IMU Quantized Tracker - 0.01 Precision Mode]")
        print("="*60)
        print(f" ▶ Accel(C) : X:{ax:+8.3f} | Y:{ay:+8.3f} | Z:{az:+8.3f}")
        print(f" ▶ Veloc(V) : X:{self.vel_x:8.2f} | Y:{self.vel_y:8.2f} | Z:{self.vel_z:8.2f} (Fixed)")
        print("-" * 60)
        print(f" ▶ Dist (M) : X:{self.dist_x:8.3f} | Y:{self.dist_y:8.3f} | Z:{self.dist_z:8.3f}")
        print("="*60)
        print(" * Velocity is truncated at 2nd decimal place to reduce drift.")

def main(args=None):
    rclpy.init(args=args)
    node = ImuQuantizedTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()