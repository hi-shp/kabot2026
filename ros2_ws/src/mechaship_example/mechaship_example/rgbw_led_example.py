import rclpy
from rclpy.node import Node

from mechaship_interfaces.msg import RgbwLedColor


class RgbwLedExample(Node):
    def __init__(self) -> None:
        # 노드의 이름을 "rgbw_led_example_node"로 초기화
        super().__init__("rgbw_led_example_node")

        # RGBW LED의 현재 상태를 나타내는 변수 초기화
        # 상태는 "R", "G", "B", "W", "N" 중 하나
        self.rgbw_led_status = "R"

        # "/actuator/rgbwled/color" 토픽에 RgbwLedColor 메시지를 발행할 퍼블리셔 생성
        self.rgbw_led_publisher = self.create_publisher(
            RgbwLedColor, "/actuator/rgbwled/color", 10
        )

        # 1초 간격으로 timer_callback 함수를 호출하는 타이머 생성
        self.create_timer(1, self.timer_callback)
        # 노드 초기화 시 바로 timer_callback을 호출하여 첫 상태를 설정
        self.timer_callback()

    def timer_callback(self):
        # RgbwLedColor 메시지 객체 생성 및 초기화
        msg = RgbwLedColor()
        msg.red = 0
        msg.green = 0
        msg.blue = 0
        msg.white = 0

        # 현재 LED 상태에 따라 색상을 설정하고 상태를 변경
        if self.rgbw_led_status == "R":
            # 빨간색 켜기 (강도 20)
            self.get_logger().info("Red")
            msg.red = 20
            self.rgbw_led_status = "G"

        elif self.rgbw_led_status == "G":
            # 초록색 켜기 (강도 20)
            self.get_logger().info("Green")
            msg.green = 20
            self.rgbw_led_status = "B"

        elif self.rgbw_led_status == "B":
            # 파란색 켜기 (강도 20)
            self.get_logger().info("Blue")
            msg.blue = 20
            self.rgbw_led_status = "W"

        elif self.rgbw_led_status == "W":
            # 흰색 켜기 (강도 20)
            self.get_logger().info("White")
            msg.white = 20
            self.rgbw_led_status = "N"

        elif self.rgbw_led_status == "N":
            # 모든 LED 끄기
            self.get_logger().info("OFF")
            self.rgbw_led_status = "R"

        # 설정된 색상을 "/actuator/rgbwled/color" 토픽에 발행
        self.rgbw_led_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RgbwLedExample()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        # 키보드 인터럽트(Ctrl+C)가 발생하면 종료
        pass

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
