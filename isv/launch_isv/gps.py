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
        avg_lat = sum(self.lat_list) / len(self.lat_list)
        avg_lon = sum(self.lon_list) / len(self.lon_list)

        # 줄바꿈 후 최종 결과만 출력
        print("\n")
        print(f"{avg_lat:.8f}, {avg_lon:.8f}")
        print("")

        rclpy.shutdown()
        sys.exit(0)

def main():
    rclpy.init()
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