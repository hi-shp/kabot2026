import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32


class ActuatorThrusterExample(Node):
    def __init__(self) -> None:
        # 노드의 이름을 "actuator_thruster_example_node"로 초기화
        super().__init__("actuator_thruster_example_node")

        # 스러스터의 초기 퍼센티지를 0.0%로 설정
        self.thruster_percentage = 0.0

        # "/actuator/thruster/percentage" 토픽에 Float32 메시지를 발행할 퍼블리셔 생성
        self.thruster_publisher = self.create_publisher(
            Float32, "/actuator/thruster/percentage", 10
        )

        # 2초 간격으로 timer_callback 함수를 호출하는 타이머 생성
        self.create_timer(2, self.timer_callback)
        # 노드 초기화 시 바로 timer_callback을 호출하여 첫 메시지를 즉시 발행
        self.timer_callback()

    def timer_callback(self):
        # Float32 메시지 객체 생성
        msg = Float32()
        # 메시지 데이터에 현재 스러스터 퍼센티지 할당
        msg.data = self.thruster_percentage

        # 메시지를 "/actuator/thruster/percentage" 토픽에 발행
        self.thruster_publisher.publish(msg)
        # 현재 스러스터 퍼센티지를 로그로 출력
        self.get_logger().info(f"Thruster: {self.thruster_percentage}%")

        # 스러스터 퍼센티지를 20%씩 증가
        self.thruster_percentage += 20.0

        # 스러스터 퍼센티지가 100%를 초과하면 0%로 초기화
        if self.thruster_percentage > 100.0:
            self.thruster_percentage = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorThrusterExample()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        # 키보드 인터럽트(Ctrl+C)가 발생하면 종료
        pass

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
