import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool


class EmoSwitchExample(Node):
    def __init__(self) -> None:
        # 노드의 이름을 "emo_switch_example"으로 초기화
        super().__init__("emo_switch_example")

        # "/sensor/emo/status" 토픽을 구독하고, 메시지가 도착하면 emo_switch_callback을 호출
        self.create_subscription(
            Bool, "/sensor/emo/status", self.emo_switch_callback, 10
        )

    def emo_switch_callback(self, data: Bool):
        # 수신한 EMO 스위치 상태 데이터 추출
        switch_status = data.data

        if switch_status is True:
            # 스위치가 눌렸을 때 경고 메시지 로그 출력
            self.get_logger().warn("EMO Switch Pressed")
        else:
            # 스위치가 눌리지 않았을 때 아무 동작도 수행하지 않음
            pass


def main(args=None):
    rclpy.init(args=args)
    node = EmoSwitchExample()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        # 키보드 인터럽트(Ctrl+C)가 발생하면 종료
        pass

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
