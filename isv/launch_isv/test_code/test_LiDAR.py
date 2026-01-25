# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
# import numpy as np
# import sys

# class LidarTurboSpectrum(Node):
#     def __init__(self):
#         super().__init__('lidar_turbo_spectrum')
#         # BEST_EFFORT 설정으로 데이터 유실이 있더라도 최신 데이터를 우선 받음
#         qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
#         self.create_subscription(LaserScan, '/scan', self.scan_cb, qos)
        
#         # 촘촘한 256색 그라데이션
#         self.color_ramp = [196, 202, 208, 214, 220, 226, 190, 154, 118, 82, 46, 28, 22]
#         self.RESET = '\033[0m'
#         self.HIDE_CURSOR = '\033[?25l'
#         self.SHOW_CURSOR = '\033[?25h'
#         self.MOVE_TOP = '\033[H' # 커서를 맨 위 왼쪽으로 이동 (clear 대신 사용)

#         print(self.HIDE_CURSOR, end="") # 실행 시 커서 숨기기
#         print("\033[2J") # 최초 실행 시에만 한 번 화면 전체 지움

#     def get_smooth_color(self, dist):
#         norm_dist = max(0.0, min(dist, 5.0)) / 5.0
#         idx = int(norm_dist * (len(self.color_ramp) - 1))
#         return f'\033[38;5;{self.color_ramp[idx]}m'

#     def scan_cb(self, msg):
#         num_ranges = len(msg.ranges)
#         if num_ranges == 0: return

#         # 데이터 슬라이싱 및 반전 (기존과 동일)
#         mid = num_ranges // 2
#         half = num_ranges // 4
#         front_raw = list(msg.ranges[mid-half : mid+half])
#         front_raw.reverse()

#         # 해상도 (터미널 폭에 맞게 자동 조절 가능하나 80~100 권장)
#         display_width = 80
#         step = len(front_raw) / display_width
        
#         spectrum_line = ""
#         min_dist = 5.0
        
#         for i in range(display_width):
#             group = front_raw[int(i*step):int((i+1)*step)]
#             valid = [r for r in group if msg.range_min < r < msg.range_max]
            
#             if valid:
#                 d = min(valid)
#                 if d < min_dist: min_dist = d
#                 color = self.get_smooth_color(d)
#                 spectrum_line += f"{color}█{self.RESET}"
#             else:
#                 spectrum_line += " "

#         # [핵심] os.system('clear') 대신 커서 이동만 사용
#         sys.stdout.write(self.MOVE_TOP)
#         sys.stdout.write("\n" + " " * 22 + "                [ Lidar Test ]\n")
#         sys.stdout.write("    " + "—" * display_width + "\n")
#         sys.stdout.write("    " + spectrum_line + "\n")
#         sys.stdout.write("    " + "—" * display_width + "\n")
#         sys.stdout.write(f"    {'LEFT (0°)':<20}{'FRONT (90°)':^40}{'RIGHT (180°)':>20}\n")
        
#         c = self.get_smooth_color(min_dist)
#         sys.stdout.write(f"\n    >> {c}NEAREST: {min_dist:.3f}m{self.RESET}      \n") # 공백 추가로 잔상 제거
#         sys.stdout.flush()

#     def __del__(self):
#         print(self.SHOW_CURSOR) # 종료 시 커서 다시 보이기

# def main():
#     rclpy.init()
#     node = LidarTurboSpectrum()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         print('\033[?25h') # 커서 복구
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


#!/usr/bin/env python3
import os
import csv
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np


class LidarFrontMinus90To0Logger(Node):
    def __init__(self):
        super().__init__("lidar_front_minus90_to_0_logger")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(LaserScan, "/scan", self.scan_cb, qos)

        # ✅ 설정값
        self.sample_period_sec = 2.0
        self.max_dist_m = 2.0
        self.target_count = 90  # -90~0 구간을 90개로

        # ✅ 마지막으로 저장한 시간
        self.last_saved_time = self.get_clock().now()

        # ✅ 저장 파일
        self.csv_path = os.path.expanduser("~/lidar_front_-90_0.csv")

        # 파일이 없으면 헤더 작성
        if not os.path.exists(self.csv_path):
            with open(self.csv_path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["stamp_sec", "k", "angle_deg", "distance_m"])
        self.get_logger().info(f"Logging to {self.csv_path} every {self.sample_period_sec:.1f}s")

    def scan_cb(self, msg: LaserScan):
        now = self.get_clock().now()
        dt = (now - self.last_saved_time).nanoseconds / 1e9

        # ✅ 2초에 1번만 처리
        if dt < self.sample_period_sec:
            return

        self.last_saved_time = now

        num_ranges = len(msg.ranges)
        if num_ranges == 0:
            return

        # 전방 180도 구간
        mid = num_ranges // 2
        half = num_ranges // 4
        start_idx = mid - half
        end_idx = mid + half
        front_ranges = list(msg.ranges[start_idx:end_idx])  # (-90~+90 가정)

        # ✅ -90~0 구간만 (앞 1/2)
        left_len = len(front_ranges) // 2
        # local index 범위: [0, left_len-1]
        sample_pos = np.linspace(0, left_len - 1, self.target_count).astype(int)

        # 메시지 timestamp (가능하면 센서 stamp 사용)
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1e9

        rows = []
        for k, local_i in enumerate(sample_pos):
            global_i = start_idx + local_i
            r = msg.ranges[global_i]

            angle_rad = msg.angle_min + global_i * msg.angle_increment
            angle_deg = float(np.degrees(angle_rad))

            # INVALID 처리 -> 0으로 저장
            if (not np.isfinite(r)) or (r <= msg.range_min) or (r >= msg.range_max):
                r_out = 0.0
            else:
                # 2m 초과 -> 0
                r_out = 0.0 if r > self.max_dist_m else float(r)

            rows.append([stamp_sec, k, angle_deg, r_out])

        # ✅ CSV에 append 저장
        with open(self.csv_path, "a", newline="") as f:
            w = csv.writer(f)
            w.writerows(rows)

        # ✅ 콘솔에 간단 요약만 출력 (원하면 전체 90개도 출력 가능)
        nonzero = sum(1 for r in rows if r[3] > 0.0)
        self.get_logger().info(f"Saved 90 values (-90~0). nonzero={nonzero}/90")


def main():
    rclpy.init()
    node = LidarFrontMinus90To0Logger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
