#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float64
import yaml
import os
import math
import sys

class ISV2026Monitor(Node):
    def __init__(self):
        super().__init__('isv_2026_monitor')
        
        # 1. 파라미터 로드
        self.load_yaml_params()
        self.path_points = list(zip(self.params['navigation']['goal_gps_lats'], 
                                   self.params['navigation']['goal_gps_lons']))
        self.target_idx = 0
        
        # 2. 데이터 초기화
        self.curr_lat, self.curr_lon = 0.0, 0.0
        self.curr_heading = 0.0
        
        # 3. 구독자 설정 (GPS 및 IMU)
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_cb, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 10) # IAHRS 데이터
        
        # 4. 모니터링 타이머 (0.5초 주기)
        self.create_timer(0.5, self.update_monitor)
        
        self.RESET = '\033[0m'
        self.BOLD = '\033[1m'
        self.CYAN = '\033[36m'

    def load_yaml_params(self):
        # 파라미터 파일 로드
        param_path = os.path.join(os.getcwd(), 'isv/launch_isv/isv_params.yaml')
        with open(param_path, 'r') as f:
            self.params = yaml.safe_load(f)

    def gps_cb(self, msg):
        self.curr_lat = msg.latitude
        self.curr_lon = msg.longitude

    def imu_cb(self, msg):
        # 쿼터니언을 오일러 각도(Yaw)로 변환 (Heading 추출)
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        # 0~360도 형식으로 변환 (북쪽 0도 기준)
        self.curr_heading = (math.degrees(yaw_rad) + 360) % 360

    def get_nav_info(self, lat1, lon1, lat2, lon2):
        # 거리 계산 (Haversine)
        R = 6371000
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        d_phi, d_lam = math.radians(lat2-lat1), math.radians(lon2-lon1)
        a = math.sin(d_phi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(d_lam/2)**2
        dist = 2 * R * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        # 방위각 계산 (Target Bearing)
        y = math.sin(d_lam) * math.cos(phi2)
        x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(d_lam)
        bearing = (math.degrees(math.atan2(y, x)) + 360) % 360
        
        return dist, bearing

    def update_monitor(self):
        if self.curr_lat == 0.0: return
        
        target_lat, target_lon = self.path_points[self.target_idx]
        dist, target_bearing = self.get_nav_info(self.curr_lat, self.curr_lon, target_lat, target_lon)
        
        # 터미널 출력 업데이트
        os.system('clear')
        print(f"\n{self.BOLD}{self.CYAN}  [ ISV 2026 NAVIGATION MONITOR ]{self.RESET}")
        print("  " + "="*45)
        print(f"  ● 현재 위치  : {self.curr_lat:.7f}, {self.curr_lon:.7f}")
        print(f"  ● 목표 지점  : No.{self.target_idx} ({target_lat:.7f}, {target_lon:.7f})")
        print("  " + "-"*45)
        print(f"  ▶ 남은 거리  : {self.BOLD}{dist:.2f} m{self.RESET}")
        print(f"  ▶ 현재 헤딩  : {self.curr_heading:>6.1f}° (배가 보는 방향)")
        print(f"  ▶ 목표 각도  : {target_bearing:>6.1f}° (가야 할 방향)")
        print(f"  ▶ 오차 각도  : {self.BOLD}{(target_bearing - self.curr_heading):>6.1f}°{self.RESET}")
        print("  " + "="*45)
        
        # 도착 시 인덱스만 수동 증가 (테스트용)
        if dist < self.params['navigation']['arrival_radius_m']:
            print(f"  [!] 목표 지점 도달! (다음 지점 대기 중)")

def main():
    rclpy.init()
    node = ISV2026Monitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()