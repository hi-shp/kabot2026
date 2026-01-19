#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math
import os

class ImuViewer(Node):
    def __init__(self):
        super().__init__('imu_viewer')
        
        # IMU용 QoS 설정 (일반적으로 Best Effort 사용)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(Imu, '/imu', self.imu_cb, qos_profile)
        self.get_logger().info("IMU Viewer Started. Monitoring /imu (Left Turn = Negative)")

    def euler_from_quaternion(self, x, y, z, w):
        """Quaternion(x,y,z,w)을 Euler 각도(Roll, Pitch, Yaw)로 변환"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # 라디안을 도(degree) 단위로 변환
        # [수정] 좌회전이 음수가 되도록 Yaw 값에 -1을 곱함
        return math.degrees(roll), math.degrees(pitch), -math.degrees(yaw)

    def imu_cb(self, msg: Imu):
        # 1. 자세 (Orientation) 변환
        r, p, y = self.euler_from_quaternion(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )

        # 터미널 출력
        os.system('clear')
        print("="*50)
        print(" [IMU Data Monitor Summary]")
        print("="*50)
        print(f" ▶ Orientation (Euler Degree)")
        print(f"    Roll  : {r:8.2f} °")
        print(f"    Pitch : {p:8.2f} °")
        print(f"    Yaw   : {y:8.2f} ° (Left: -, Right: +)")
        print("="*50)

def main(args=None):
    rclpy.init(args=args)
    node = ImuViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()