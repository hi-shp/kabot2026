#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import os

class LidarFrontProcessor(Node):
    def __init__(self):
        super().__init__('lidar_front_processor')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile)
        self.get_logger().info("Lidar Front 180 Line Processor Started.")

    def scan_cb(self, msg: LaserScan):
        num_ranges = len(msg.ranges)
        if num_ranges == 0:
            return

        # 1. 정면 중심 인덱스 설정 (배열 중간을 정면으로 가정)
        center_idx = num_ranges // 2  
        half_view_idx = num_ranges // 4  
        
        start_idx = center_idx - half_view_idx
        end_idx = center_idx + half_view_idx
        
        # 2. 정면 180도 데이터 추출
        front_ranges = msg.ranges[start_idx:end_idx]
        if not front_ranges:
            return

        # 3. 각 1도마다 평균값 계산
        samples_per_degree = len(front_ranges) / 180.0
        averaged_180_list = []

        for i in range(180):
            group_start = int(i * samples_per_degree)
            group_end = int((i + 1) * samples_per_degree)
            
            degree_group = front_ranges[group_start:group_end]
            valid_vals = [r for r in degree_group if msg.range_min < r < msg.range_max]
            
            if valid_vals:
                averaged_180_list.append(round(float(np.mean(valid_vals)), 4))
            else:
                averaged_180_list.append(float('inf'))

        # 좌우 반전 적용 (0: 왼쪽 끝, 180: 오른쪽 끝)
        averaged_180_list.reverse()

        # 4. 가장 가까운 각도 및 거리 계산
        # 리스트 중 최소값 찾기 (inf 제외)
        min_distance = min(averaged_180_list)
        
        if min_distance != float('inf'):
            min_angle = averaged_180_list.index(min_distance) # 0~180 사이의 인덱스(각도)
        else:
            min_angle = None

        # 5. 결과 출력
        os.system('clear') # 터미널 가독성을 위해 화면 초기화
        print("="*60)
        print(f"Data List: {averaged_180_list}")
        print("-" * 60)
        
        if min_angle is not None:
            # 0도는 왼쪽, 90도는 정면, 179도는 오른쪽
            print(f"▶ 최단 거리 장애물: {min_distance:.4f} m")
            print(f"▶ 각도: {min_angle} (0:왼쪽, 90:정면, 180:오른쪽)")
        else:
            print("▶ 감지된 장애물 없음 (모든 거리 inf)")
        print("="*60)

def main(args=None):
    rclpy.init(args=args)
    node = LidarFrontProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()