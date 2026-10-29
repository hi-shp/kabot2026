#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from vectornav_msgs.msg import CommonGroup  # yawpitchroll.x = yaw (deg)

class ImuHeadingLogger(Node):
    def __init__(self):
        super().__init__('imu_heading_logger')

        # QoS: 센서 데이터는 보통 BEST_EFFORT, depth=1이면 최신값만 받기 충분
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1,
        )

        # VectorNav CommonGroup 구독 (/vectornav/raw/common)
        self.sub_vn = self.create_subscription(
            CommonGroup,
            '/vectornav/raw/common',
            self.vectornav_callback,
            qos
        )

        self.get_logger().info('IMU Heading Logger started (topic: /vectornav/raw/common)')

    def vectornav_callback(self, msg: CommonGroup):
        """
        yawpitchroll.x: yaw (deg), 범위 보통 [-180, 180]
        변환 규칙: 음수면 360 + yaw, 양수/0이면 그대로
        """
        yaw_deg = float(msg.yawpitchroll.x)

        if yaw_deg < 0.0:
            heading = 360.0 + yaw_deg
        else:
            heading = yaw_deg

        # 안전하게 0~360 범위에 고정 (경계치 처리)
        if heading >= 360.0:
            heading -= 360.0
        if heading < 0.0:
            heading += 360.0

        self.get_logger().info(f"raw_yaw={yaw_deg:.2f} deg  ->  heading(0~360, cw)={heading:.2f} deg")


def main(args=None):
    rclpy.init(args=args)
    node = ImuHeadingLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

