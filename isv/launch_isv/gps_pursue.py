#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import yaml
import os
import math

class ISV2026Controller(Node):
    def __init__(self):
        super().__init__('isv_2026_controller')
        
        # 1. 파라미터 로드
        self.load_yaml_params()
        self.path_points = list(zip(self.params['navigation']['goal_gps_lats'], 
                                   self.params['navigation']['goal_gps_lons']))
        self.target_idx = 0
        
        # 2. 데이터 초기화
        self.curr_lat, self.curr_lon = 0.0, 0.0
        self.curr_heading_raw = 0.0 
        
        # 3. 퍼블리셔 설정
        self.key_pub = self.create_publisher(Float64, '/actuator/key/degree', 10)
        self.thruster_pub = self.create_publisher(Float64, '/actuator/thruster/pwm', 10)
        
        # 4. 구독자 설정 (QoS 적용)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_cb, 10)
        self.create_subscription(Imu, '/imu', self.imu_cb, qos)
        
        # 5. 타이머 설정 (node_settings의 0.5초 주기 적용)
        timer_period = self.params['node_settings']['timer_period_seconds']
        self.create_timer(timer_period, self.control_loop)
        
        self.RESET, self.BOLD = '\033[0m', '\033[1m'
        self.CYAN, self.YELLOW, self.GREEN = '\033[36m', '\033[33m', '\033[32m'

    def load_yaml_params(self):
        param_path = os.path.join(os.getcwd(), 'isv/launch_isv/isv_params.yaml')
        try:
            with open(param_path, 'r') as f:
                self.params = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Param load failed: {e}")
            # 최소한의 기본값 (파일 로드 실패 대비)
            self.params = {
                'pid': {'p_gain': 1.0, 'output_limits': {'min': -30.0, 'max': 30.0}},
                'servo': {'min_deg': 60.0, 'max_deg': 120.0, 'neutral_deg': 90.0},
                'navigation': {'arrival_radius_m': 2.0},
                'state_machine': {'thruster_defaults': {'state0': 15.0}},
                'node_settings': {'timer_period_seconds': 0.5}
            }

    def gps_cb(self, msg):
        self.curr_lat, self.curr_lon = msg.latitude, msg.longitude

    def imu_cb(self, msg):
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        # 물리 방향: 우회전(+), 좌회전(-) 절대 헤딩 (북=0)
        heading_deg = -math.degrees(math.atan2(siny_cosp, cosy_cosp))
        self.curr_heading_raw = (heading_deg + 360) % 360

    def get_nav_info(self, lat1, lon1, lat2, lon2):
        R = 6371000
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        d_phi, d_lam = math.radians(lat2-lat1), math.radians(lon2-lon1)
        a = math.sin(d_phi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(d_lam/2)**2
        dist = 2 * R * math.atan2(math.sqrt(a), math.sqrt(1-a))
        y = math.sin(d_lam) * math.cos(phi2)
        x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(d_lam)
        bearing = (math.degrees(math.atan2(y, x)) + 360) % 360
        return dist, bearing

    def control_loop(self):
        if self.curr_lat == 0.0: return
        
        # 1. 목표 정보 계산
        t_lat, t_lon = self.path_points[self.target_idx]
        dist, t_bearing_raw = self.get_nav_info(self.curr_lat, self.curr_lon, t_lat, t_lon)
        
        # 2. 오차 계산 (절대 기준)
        error = t_bearing_raw - self.curr_heading_raw
        if error > 180: error -= 360
        if error < -180: error += 360

        # 3. 키(Servo) 제어 로직 (PID p_gain 적용)
        p_gain = self.params['pid']['p_gain']
        output_limit = self.params['pid']['output_limits']['max'] # 30.0
        
        # 오차에 gain을 곱하되, 설정된 출력 제한(-30 ~ 30)을 넘지 않게 처리
        pid_output = max(-output_limit, min(output_limit, error * p_gain))
        
        # 중립(90) + PID 출력 -> 최종 서보 각도 (60 ~ 120 제한)
        neutral = self.params['servo']['neutral_deg']
        min_s, max_s = self.params['servo']['min_deg'], self.params['servo']['max_deg']
        target_key = max(min_s, min(max_s, neutral + pid_output))

        # 4. 추진기(Thruster) 제어 로직
        state_key = f'state{self.target_idx}'
        thruster_val = self.params['state_machine']['thruster_defaults'].get(state_key, 0.0)

        # 5. 메시지 발행
        self.key_pub.publish(Float64(data=float(target_key)))
        self.thruster_pub.publish(Float64(data=float(thruster_val)))

        # 6. 화면 출력 (Front=90° 기준)
        display_head = (self.curr_heading_raw + 90) % 360
        display_target = (t_bearing_raw + 90) % 360
        
        os.system('clear')
        print(f"\n{self.BOLD}{self.CYAN} [ ISV 2026 CONTROL MONITOR ]{self.RESET}")
        print(" " + "="*50)
        print(f" ● Status : {self.BOLD}State {self.target_idx}{self.RESET} ({t_lat:.7f}, {t_lon:.7f})")
        print(f" ● Dist   : {self.BOLD}{dist:.2f} m{self.RESET} / {self.params['navigation']['arrival_radius_m']}m")
        print(" " + "-"*50)
        print(f" ▶ HEAD   : {display_head:>6.1f}° (Front=90°)")
        print(f" ▶ TARGET : {display_target:>6.1f}°")
        print(f" ▶ ERROR  : {error:>6.1f}°")
        print(" " + "-"*50)
        print(f" ▶ KEY OUT: {self.YELLOW}{target_key:>6.1f}°{self.RESET} (Neutral: 90)")
        print(f" ▶ THRUST : {self.GREEN}{thruster_val:>6.1f}%{self.RESET}")
        print(" " + "="*50)

        # 7. 도착 판정 및 인덱스 전환
        if dist < self.params['navigation']['arrival_radius_m']:
            if self.target_idx < len(self.path_points) - 1:
                self.target_idx += 1
            else:
                # 최종 목적지 도달 시 추진기 정지 (State 4 처리)
                self.target_idx = 4 

def main():
    rclpy.init()
    node = ISV2026Controller()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()