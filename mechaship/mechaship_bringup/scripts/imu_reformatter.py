#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from tf_transformations import euler_from_quaternion
import math

class ImuReformatterNode(Node):
    def __init__(self):
        super().__init__('imu_reformatter')
        
        # [해결] 드라이버 설정에 상관없이 통신이 가능하도록 센서 데이터 전용 QoS 프로파일 사용
        # 만약 이래도 안된다면 아래 주석처리된 custom_qos로 교체하세요.
        self.imu_sub = self.create_subscription(
            Imu, 
            '/imu', 
            self.imu_callback, 
            qos_profile_sensor_data # 대부분의 ROS2 센서 드라이버 표준
        )
        
        # 조향 시스템에서 사용할 가공된 Yaw 퍼블리셔
        self.yaw_pub = self.create_publisher(Float32, '/imu/yaw_refined', 10)
        
        self.initial_yaw = None
        self.get_logger().info("IMU Reformatter: System Ready. Waiting for IMU data...")

    def normalize_180(self, deg):
        # -180 ~ 180도 범위 유지
        return (deg + 180.0) % 360.0 - 180.0

    def imu_callback(self, msg):
        # 쿼터니언 -> 오일러(Yaw) 변환
        q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        _, _, yaw_rad = euler_from_quaternion(q)
        
        # 오른쪽 +, 왼쪽 - 방향 보정
        current_yaw_deg = math.degrees(-yaw_rad)

        # 켰을 때 정면(0도) 고정
        if self.initial_yaw is None:
            self.initial_yaw = current_yaw_deg
            self.get_logger().info(f"Standard Heading Fixed at {self.initial_yaw:.2f} degrees")

        # 상대 Yaw 계산 및 정규화
        refined_yaw = self.normalize_180(current_yaw_deg - self.initial_yaw)
        
        # 발행
        self.yaw_pub.publish(Float32(data=float(refined_yaw)))

def main(args=None):
    rclpy.init(args=args)
    node = ImuReformatterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()