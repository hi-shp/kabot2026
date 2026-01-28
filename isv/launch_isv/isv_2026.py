#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import subprocess
import time

class ISV_2026(Node):
    def __init__(self):
        super().__init__('isv_2026')
        try:
            for mission in ['course1', 'course2', 'course3']:
                subprocess.run(['ros2', 'run', 'isv', mission])
                time.sleep(1.0)
        except KeyboardInterrupt:
            pass

def main(args=None):
    rclpy.init(args=args)
    try:
        ISV_2026()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()