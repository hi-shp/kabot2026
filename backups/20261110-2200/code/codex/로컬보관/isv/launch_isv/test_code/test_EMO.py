#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class EmoMonitor(Node):
    def __init__(self):
        super().__init__('emo_monitor')

        self.emo_active = None

        self.subscription = self.create_subscription(
            Bool,
            '/sensor/emo/status',
            self.emo_callback,
            10
        )

        self.get_logger().info('EMO monitor node started.')
        self.get_logger().info('Waiting for /sensor/emo/status...')

    def emo_callback(self, msg: Bool):
        # 상태가 바뀔 때만 출력
        if self.emo_active != msg.data:
            self.emo_active = msg.data

            if self.emo_active:
                self.get_logger().warn('EMERGENCY STOP ACTIVE (EMO = TRUE)')
            else:
                self.get_logger().info('Emergency stop released (EMO = FALSE)')


def main(args=None):
    rclpy.init(args=args)
    node = EmoMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
