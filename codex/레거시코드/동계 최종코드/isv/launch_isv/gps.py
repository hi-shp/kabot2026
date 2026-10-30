#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import sys

class GPSAverageNode(Node):
    def __init__(self, target_count=100):
        super().__init__('gps_average_node')
        self.subscription = self.create_subscription(
            NavSatFix, 
            "/gps/fix", 
            self.gps_callback, 
            10
        )
        self.target_count = target_count
        self.lat_list = []
        self.lon_list = []

    def gps_callback(self, msg):
        # NaN 데이터 무시
        if msg.latitude != msg.latitude or msg.longitude != msg.longitude:
            return

        self.lat_list.append(msg.latitude)
        self.lon_list.append(msg.longitude)
        
        # 진행률 표시 (터미널 한 줄에서 갱신)
        sys.stdout.write(f"\r수집 중: {len(self.lat_list)}/{self.target_count}")
        sys.stdout.flush()

        if len(self.lat_list) >= self.target_count:
            self.calculate_and_print()

    def calculate_and_print(self):
        # 상/하위 20개를 제외하기 위해 정렬
        self.lat_list.sort()
        self.lon_list.sort()

        # 인덱스 20부터 79까지(총 60개) 슬라이싱하여 추출
        # 이는 상위 20개와 하위 20개를 자동으로 제외함
        trimmed_lat = self.lat_list[20:80]
        trimmed_lon = self.lon_list[20:80]

        # 정제된 60개 데이터의 평균 계산
        avg_lat = sum(trimmed_lat) / len(trimmed_lat)
        avg_lon = sum(trimmed_lon) / len(trimmed_lon)

        # 줄바꿈 후 기존과 동일한 형식으로 최종 결과 출력
        print("\n")
        print(f"{avg_lat:.8f}, {avg_lon:.8f}")
        print("")

        rclpy.shutdown()
        sys.exit(0)

def main():
    rclpy.init()
    # 100개를 수집하여 중간 60개를 사용
    node = GPSAverageNode(target_count=100)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()