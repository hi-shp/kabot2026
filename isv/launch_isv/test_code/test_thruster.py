#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class ThrusterTestNode(Node):
    def __init__(self):
        super().__init__("thruster_test_node")

        # 추진기 퍼센트 토픽 설정
        self.publisher = self.create_publisher(
            Float64,
            "/actuator/thruster/percentage",
            10,
        )

        self.msg = Float64()
        self.msg.data = 0.0  # 출력 값

        self.publish_count = 0
        self.max_publish_count = 20   # 총 20번 발행

        # 0.1초마다 timer_callback 함수 실행
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Thruster test node started.")

    def timer_callback(self):
        self.publisher.publish(self.msg)
        self.publish_count += 1

        # 요청하신 형식에 맞춘 출력
        self.get_logger().info(
            f"Publishing thruster percentage {self.msg.data} ({self.publish_count}/{self.max_publish_count})"
        )

        # 20회 도달 시 종료 처리
        if self.publish_count >= self.max_publish_count:
            self.get_logger().info("Thruster test publish finished")
            self.timer.cancel()
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ThrusterTestNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass

if __name__ == "__main__":
    main()