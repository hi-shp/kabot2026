#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import subprocess
from std_msgs.msg import Float64

class ISV_2026(Node):
    def __init__(self):
        super().__init__('isv_2026')
        self.yaw_12 = 0.0
        self.yaw_23 = 0.0
        self.create_subscription(Float64, '/imu/one_two', self.cb_12, 10)
        self.create_subscription(Float64, '/imu/two_three', self.cb_23, 10)
        self.pub_target = self.create_publisher(Float64, '/imu/target_yaw', 10)
        self.run_manager()
        
    def cb_12(self, msg):
        self.yaw_12 = msg.data
    def cb_23(self, msg):
        self.yaw_23 = msg.data

    def run_manager(self):
        for mission in ['course1', 'course2', 'course3']:
            proc = subprocess.Popen(['ros2', 'run', 'isv', mission])
            while proc.poll() is None:
                if mission == 'course2':
                    self.pub_target.publish(Float64(data=self.yaw_12))
                elif mission == 'course3':
                    self.pub_target.publish(Float64(data=self.yaw_23))
                rclpy.spin_once(self, timeout_sec=0.1)

def main(args=None):
    rclpy.init(args=args)
    try:
        ISV_2026()
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()