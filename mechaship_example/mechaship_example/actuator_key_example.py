import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32


class ActuatorKeyExample(Node):
    def __init__(self) -> None:
        # 노드의 이름을 "actuator_key_example_node"로 초기화
        super().__init__("actuator_key_example_node")

        # 키의 초기 각도를 60.0도로 설정
        self.key_degree = 60.0

        # "/actuator/key/degree" 토픽에 Float32 메시지를 발행할 퍼블리셔 생성
        self.key_publisher = self.create_publisher(Float32, "/actuator/key/degree", 10)

        # 2초 간격으로 timer_callback 함수를 호출하는 타이머 생성
        self.create_timer(2, self.timer_callback)

    def timer_callback(self):
        # Float32 메시지 객체 생성
        msg = Float32()
        # 메시지 데이터에 현재 키 각도 할당
        msg.data = self.key_degree

        # 메시지를 "/actuator/key/degree" 토픽에 발행
        self.key_publisher.publish(msg)
        # 현재 키 각도를 로그로 출력
        self.get_logger().info(f"Key: {self.key_degree}°")

        # 키 각도를 30도씩 증가
        self.key_degree += 30.0

        # 키 각도가 120도를 초과하면 60도로 초기화
        if self.key_degree > 120.0:
            self.key_degree = 60.0


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorKeyExample()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        # 키보드 인터럽트(Ctrl+C)가 발생하면 종료
        pass

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
