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
        self.yaw_pub = self.create_publisher(Float64, '/imu/target_yaw', 10)
        self.run_manager()

    def yaw_cb(self, msg):
        self.last_yaw = msg.data

    def run_manager(self):
        for mission in ['course1', 'course2', 'course3']:
            proc = subprocess.Popen(['ros2', 'run', 'isv', mission])

            while proc.poll() is None:
                if mission == 'course2':
                    self.yaw_pub.publish(Float64(data=self.last_yaw))
                rclpy.spin_once(self, timeout_sec=0.1)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = ISV_2026()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()