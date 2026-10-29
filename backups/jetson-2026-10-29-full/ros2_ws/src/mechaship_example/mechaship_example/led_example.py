import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool


class LedExample(Node):
    def __init__(self) -> None:
        # 노드의 이름을 "led_example_node"로 초기화
        super().__init__("led_example_node")

        # LED 상태를 나타내는 변수 초기화 (0: LED 1 ON, 1: LED 2 ON)
        self.led_status = 0

        # "/actuator/led/user_1" 토픽에 Bool 메시지를 발행할 퍼블리셔 생성
        self.led_1_publisher = self.create_publisher(Bool, "/actuator/led/user_1", 10)

        # "/actuator/led/user_2" 토픽에 Bool 메시지를 발행할 퍼블리셔 생성
        self.led_2_publisher = self.create_publisher(Bool, "/actuator/led/user_2", 10)

        # 1초 간격으로 timer_callback 함수를 호출하는 타이머 생성
        self.create_timer(1, self.timer_callback)
        # 노드 초기화 시 바로 timer_callback을 호출하여 첫 상태를 설정
        self.timer_callback()

    def timer_callback(self):
        # Bool 메시지 객체 생성
        msg = Bool()

        if self.led_status == 0:
            # LED 1을 켜고 LED 2를 끄는 경우
            self.get_logger().info("LED 1 is ON!")

            msg.data = True
            self.led_1_publisher.publish(msg)  # LED 1 ON
            msg.data = False
            self.led_2_publisher.publish(msg)  # LED 2 OFF

            # 상태를 1로 변경하여 다음 번에 LED 2를 켜도록 설정
            self.led_status = 1

        elif self.led_status == 1:
            # LED 2을 켜고 LED 1을 끄는 경우
            self.get_logger().info("LED 2 is ON!")

            msg.data = False
            self.led_1_publisher.publish(msg)  # LED 1 OFF
            msg.data = True
            self.led_2_publisher.publish(msg)  # LED 2 ON

            # 상태를 0으로 변경하여 다음 번에 LED 1을 켜도록 설정
            self.led_status = 0


def main(args=None):
    rclpy.init(args=args)
    node = LedExample()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        # 키보드 인터럽트(Ctrl+C)가 발생하면 종료
        pass

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
