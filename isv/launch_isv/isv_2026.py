#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import subprocess
import time
from std_msgs.msg import Float64

class ISV_2026(Node):
    def __init__(self):
        super().__init__('isv_2026')
        self.last_yaw = 0.0
        self.yaw_sub = self.create_subscription(Float64, '/imu/one_two', self.yaw_cb, 10)
        self.yaw_pub = self.create_publisher(Float64, '/imu/one_two', 10)
        self.run_manager()

    def yaw_cb(self, msg):
        self.last_yaw = msg.data

    def run_manager(self):
        for mission in ['course1', 'course2', 'course3']:
            self.get_logger().info(f'>>> {mission} 시작')
            
            # Popen으로 비동기 실행 (매니저가 멈추지 않음)
            proc = subprocess.Popen(['ros2', 'run', 'isv', mission])

            # 노드가 실행 중인 동안 계속 루프
            while proc.poll() is None:
                if mission == 'course2':
                    # course2 실행 중일 때만 0.1초 간격(10Hz)으로 각도 발행
                    self.yaw_pub.publish(Float64(data=self.last_yaw))
                
                # 매니저 노드의 콜백(구독) 처리 및 루프 간격 조절
                rclpy.spin_once(self, timeout_sec=0.1)
            
            self.get_logger().info(f'<<< {mission} 종료')

def main(args=None):
    rclpy.init(args=args)
    ISV_2026()
    rclpy.shutdown()

if __name__ == '__main__':
    main()