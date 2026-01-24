#!/usr/bin/env python3
import os
import yaml
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data 

class LiDARSafePathPlanner(Node):
    def __init__(self):
        super().__init__('lidar_process_node')
        
        self._load_params()
        
        # 'safe_angle' 토픽으로 180개 고정 리스트 발행
        self.safe_pub = self.create_publisher(Float64MultiArray, 'safe_angle', 10)
        
        self.scan_sub = self.create_subscription(
            LaserScan, 
            'scan', 
            self.lidar_callback, 
            qos_profile_sensor_data
        )
        
        self.latest_ranges = []
        timer_period = self.cfg['node_settings']['timer_period_seconds']
        self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("LiDAR Process Node: Gauge 시각화 모드 실행 중")

    def _load_params(self):
        script_dir = os.path.dirname(os.path.realpath(__file__))
        yaml_path = os.path.join(script_dir, "isv_params.yaml")
        if not os.path.exists(yaml_path):
            yaml_path = os.path.join(os.path.dirname(script_dir), "isv_params.yaml")
            
        try:
            with open(yaml_path, "r") as file:
                self.cfg = yaml.safe_load(file)
        except Exception as e:
            self.get_logger().error(f"YAML 로드 실패: {e}")
            self.cfg = {'node_settings': {'timer_period_seconds': 0.5},
                        'lidar_obstacle_avoidance': {'risk_calc': {'max_safe_dist': 7.0}}}

    def lidar_callback(self, msg):
        self.latest_ranges = list(msg.ranges)

    def timer_callback(self):
        if not self.latest_ranges:
            return

        r_cfg = self.cfg['lidar_obstacle_avoidance']['risk_calc']
        max_safe_dist = r_cfg['max_safe_dist']
        
        n_raw = len(self.latest_ranges)
        step = n_raw // 180
        
        # 180개 고정 길이 리스트 생성 (기본값 0.0: 위험)
        gauge_data = [0.0] * 180
        
        for i in range(180):
            sample_idx = i * step
            if sample_idx >= n_raw: break
            
            dist = self.latest_ranges[sample_idx]
            
            # 안전하면 1.0, 위험하면 0.0
            if dist == 0.0 or dist > max_safe_dist:
                gauge_data[i] = 1.0
            else:
                gauge_data[i] = 0.0

        # 리스트 발행
        msg = Float64MultiArray()
        msg.data = gauge_data
        self.safe_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LiDARSafePathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()