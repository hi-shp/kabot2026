#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

import os
import time


class GPSLogger(Node):
    def __init__(self):
        super().__init__('gps_logger')
        self.latest_gps = None

        # /fix 구독
        self.create_subscription(NavSatFix, "/smc_2000/fix", self.gps_cb, 10)

        # 저장 경로 설정 (홈 디렉토리 기준)
        home = os.path.expanduser("~")
        self.file_path = os.path.join(
            home, "ros2_ws/src/isv_kaboat/isv_kaboat/gps_log.txt"
        )

        # 1초마다 기록하는 타이머
        self.timer = self.create_timer(1.0, self._log_gps)

        self.get_logger().info(
            f"GPS Logger Node Started. Saving every 1 sec to:\n{self.file_path}"
        )

    def gps_cb(self, msg: NavSatFix):
        self.latest_gps = msg

    def _log_gps(self):
        if self.latest_gps is None:
            self.get_logger().warn("아직 /fix 데이터가 없습니다.")
            return

        lat = self.latest_gps.latitude
        lon = self.latest_gps.longitude

        # goal_gps_coords에 바로 복붙 가능한 형식
        line = f"({lat:.7f}, {lon:.7f}),\n"

        # 파일에 append 저장
        try:
            with open(self.file_path, "a") as f:
                f.write(line)
            print(f"[Saved] {line.strip()}")
        except Exception as e:
            self.get_logger().error(f"파일 저장 중 오류: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = GPSLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
