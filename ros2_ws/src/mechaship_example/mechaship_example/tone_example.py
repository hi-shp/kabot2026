import time

import rclpy
from rclpy.node import Node

from mechaship_interfaces.msg import ToneTopic


class ToneExample(Node):
    def __init__(self) -> None:
        # 노드의 이름을 "tone_example_node"로 초기화
        super().__init__("tone_example_node")

        # "/actuator/tone/play" 토픽에 ToneTopic 메시지를 발행할 퍼블리셔 생성
        self.tone_publisher = self.create_publisher(
            ToneTopic, "/actuator/tone/play", 10
        )

        # 음계 목록을 주파수(Hz) 단위로 정의 (도, 레, 미, 파, 솔, 라, 시, 도)
        self.notes = [
            523,  # 도
            587,  # 레
            659,  # 미
            698,  # 파
            783,  # 솔
            880,  # 라
            987,  # 시
            1046,  # 도 (높은 옥타브)
        ]

        # 무한 루프를 사용하여 음계를 순차적으로 발행
        while True:
            for note in self.notes:
                # ToneTopic 메시지 객체 생성
                msg = ToneTopic()
                msg.duration_ms = 500  # 음의 지속 시간 설정 (밀리초)
                msg.hz = note  # 현재 음의 주파수 설정

                # 메시지를 "/actuator/tone/play" 토픽에 발행
                self.tone_publisher.publish(msg)

                # 0.5초(500밀리초) 동안 대기
                time.sleep(0.5)


def main(args=None):
    rclpy.init(args=args)
    node = ToneExample()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        # 키보드 인터럽트(Ctrl+C)가 발생하면 종료
        pass

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
