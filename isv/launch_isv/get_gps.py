#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

import sys
import threading
import queue
import termios
import tty
import os
import time

KEY_SPACE = " "
KEY_ENTER = "\n"

class GPSLogger(Node):
    def __init__(self):
        super().__init__('gps_logger')
        self.latest_gps = None

        # /fix 구독
        self.create_subscription(NavSatFix, "/gps/fix", self.gps_cb, 10)

        # 키 입력을 넣을 큐
        self.key_q = queue.Queue()

        # 키보드 스레드 시작
        self.kbd_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self.kbd_thread.start()

        # 10Hz로 큐 비우면서 처리
        self.timer = self.create_timer(0.1, self._process_keys)

        self.get_logger().info("GPS Logger Node Started. Press SPACE (or ENTER) to log position.")

    def gps_cb(self, msg: NavSatFix):
        self.latest_gps = msg

    # ---- 키보드 스레드 (blocking) ----
    def _keyboard_loop(self):
        # TTY가 아닌 환경이면 그냥 리턴 (launch나 파이프에서 입력 없음)
        if not sys.stdin.isatty():
            return

        fd = sys.stdin.fileno()
        old_attr = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)   # 즉시 1글자씩 받기
            while True:
                ch = sys.stdin.read(1)
                # 스페이스/엔터만 큐에 넣기
                if ch in (KEY_SPACE, KEY_ENTER):
                    self.key_q.put(ch)
        except Exception:
            pass
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_attr)

    # ---- 큐에 키가 들어오면 로그 출력 ----
    def _process_keys(self):
        try:
            while True:
                _ = self.key_q.get_nowait()
                if self.latest_gps is None:
                    self.get_logger().warn("아직 /fix 데이터가 없습니다.")
                else:
                    lat = self.latest_gps.latitude
                    lon = self.latest_gps.longitude

                    # altitude 제거, goal_gps_coords에 바로 복붙 가능
                    print(f"({lat:.7f}, {lon:.7f}),")
        except queue.Empty:
            pass


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
