# imu에서 출력하는 quaternion을 친숙한 euler로 변환하는 모듈 import
# sudo apt install ros-$ROS_DISTRO-tf-transformations -y
from tf_transformations import euler_from_quaternion
from math import degrees

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32


class ImuKeyExampleNode(Node):
    def __init__(self):
        # 노드의 이름을 "imukey_example_node"로 초기화
        super().__init__("imukey_example_node")

        # "/imu" 토픽을 구독하고, 메시지가 도착하면 imu_callback 호출
        self.create_subscription(
            Imu,
            "/imu",
            self.imu_callback,
            qos_profile_sensor_data,  # QoS 설정 적용
        )

        # "/actuator/key/degree" 토픽에 Float32 메시지를 발행할 퍼블리셔 생성
        self.imu_publisher = self.create_publisher(Float32, "/actuator/key/degree", 10)

        # Key Topic에 Publish하는 주기를 10Hz로 제한하기 위해 0.1초 간격의 타이머 생성
        self.create_timer(0.1, self.timer_callback)

        # 초기 키 목표 각도 설정
        self.key_target_degree = 90.0

    def timer_callback(self):
        # Float32 메시지 객체 생성
        msg = Float32()
        # 현재 키 목표 각도를 메시지 데이터로 설정
        msg.data = self.key_target_degree

        # 메시지를 "/actuator/key/degree" 토픽에 발행
        self.imu_publisher.publish(msg)

    def imu_callback(self, msg: Imu):
        # 수신한 IMU 메시지에서 quaternion 추출
        quaternion = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )

        # quaternion을 euler 각도로 변환 (roll, pitch, yaw)
        roll_rad, pitch_rad, yaw_rad = euler_from_quaternion(quaternion)

        # 라디안을 도 단위로 변환하고 180도 추가하여 각도 범위 조정
        roll_degree = degrees(roll_rad) + 180
        pitch_degree = degrees(pitch_rad) + 180
        yaw_degree = degrees(yaw_rad) + 180

        # yaw 각도를 키 목표 각도로 설정
        self.key_target_degree = yaw_degree

        # 키 목표 각도를 60도와 120도 사이로 제한
        if self.key_target_degree > 120.0:
            self.key_target_degree = 120.0
        if self.key_target_degree < 60.0:
            self.key_target_degree = 60.0

        # 현재 roll, pitch, yaw 각도와 키 목표 각도를 로그로 출력
        self.get_logger().info(
            f"roll: {roll_degree:5.1f}°    pitch: {pitch_degree:5.1f}°    yaw: {yaw_degree:5.1f}°    key: {self.key_target_degree:5.1f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ImuKeyExampleNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        # 키보드 인터럽트(Ctrl+C)가 발생하면 종료
        pass

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
