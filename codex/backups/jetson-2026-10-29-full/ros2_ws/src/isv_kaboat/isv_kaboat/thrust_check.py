#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node

from mechaship_interfaces.srv import ThrottlePulseWidth


class ThrusterTestNode(Node):
    """
    트러스터 테스트 노드
    - 서비스 형식은 MotorControlNode 와 동일:
      /actuators/throttle_left/set_pulse_width
      /actuators/throttle_right/set_pulse_width
    """

    def __init__(self) -> None:
        super().__init__("thruster_test")

        # 모터 PWM 범위 (MotorControlNode 와 동일 값 사용)
        self.MIN_PULSE_WIDTH = 1395
        self.MAX_PULSE_WIDTH = 1900
        self.STOP_PW = 1500

        # 서비스 클라이언트 생성
        self.cli_left = self.create_client(
            ThrottlePulseWidth,
            "/actuators/throttle_left/set_pulse_width",
        )
        self.cli_right = self.create_client(
            ThrottlePulseWidth,
            "/actuators/throttle_right/set_pulse_width",
        )

        self.get_logger().info("Waiting for throttle services...")
        self.cli_left.wait_for_service()
        self.cli_right.wait_for_service()
        self.get_logger().info("Throttle services are available.")

    # ---------- 유틸 함수들 ----------

    def constrain(self, val: float, lo: float, hi: float) -> float:
        return max(min(val, hi), lo)

    def send_pwm(self, left_pw: int, right_pw: int, label: str = "") -> None:
        """좌/우 PWM 한번에 전송"""
        left_pw = int(self.constrain(left_pw, self.MIN_PULSE_WIDTH, self.MAX_PULSE_WIDTH))
        right_pw = int(self.constrain(right_pw, self.MIN_PULSE_WIDTH, self.MAX_PULSE_WIDTH))

        req_l = ThrottlePulseWidth.Request()
        req_r = ThrottlePulseWidth.Request()
        req_l.pulse_width = left_pw
        req_r.pulse_width = right_pw

        self.cli_left.call_async(req_l)
        self.cli_right.call_async(req_r)

        self.get_logger().info(
            f"[{label}] left={left_pw}, right={right_pw}"
        )

    def stop(self, label: str = "STOP") -> None:
        self.send_pwm(self.STOP_PW, self.STOP_PW, label)

    # ---------- 테스트 시퀀스 ----------

    def run_sequence(self) -> None:
        """
        정지 → 전진 → 정지 → 후진 → 정지 → 우회전 → 정지 → 좌회전 → 정지
        """
        try:
            # 안전하게 정지 상태에서 시작
            self.stop("INIT STOP")
            time.sleep(2.0)

            # 1) 전진 (양쪽 동일, 약 1600)
            self.send_pwm(1600, 1600, "FORWARD")
            time.sleep(3.0)

            # 정지
            self.stop("STOP AFTER FORWARD")
            time.sleep(2.0)

            # 2) 후진 (양쪽 동일, 약 1400)
            self.send_pwm(1400, 1400, "BACKWARD")
            time.sleep(3.0)

            # 정지
            self.stop("STOP AFTER BACKWARD")
            time.sleep(2.0)

            # 3) 우회전 (CW: 왼쪽 세게, 오른쪽 약하게)
            # MotorControlNode.turn_angle/turn_angle2 와 같은 느낌으로
            self.send_pwm(1600, 1400, "TURN RIGHT (CW)")
            time.sleep(3.0)

            # 정지
            self.stop("STOP AFTER RIGHT TURN")
            time.sleep(2.0)

            # 4) 좌회전 (CCW: 왼쪽 약하게, 오른쪽 세게)
            self.send_pwm(1400, 1600, "TURN LEFT (CCW)")
            time.sleep(3.0)

            # 최종 정지
            self.stop("FINAL STOP")
            time.sleep(2.0)

            self.get_logger().info("Thruster test sequence finished.")

        except KeyboardInterrupt:
            self.get_logger().info("KeyboardInterrupt: stopping motors.")
            self.stop("STOP ON CTRL+C")
        except Exception as e:
            self.get_logger().error(f"Exception in test sequence: {e}")
            self.stop("STOP ON EXCEPTION")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ThrusterTestNode()

    # rclpy.spin 을 쓰지 않고, 바로 시퀀스를 실행하고 종료하는 구조
    node.run_sequence()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

