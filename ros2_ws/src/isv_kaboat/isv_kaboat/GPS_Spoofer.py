#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus

# 순차적으로 발행할 GPS 지점들
WAYPOINTS = [
    (35.2328965, 129.0798148, 0.0),

]

interval_sec = 2.0  # 2초마다 다음 좌표 발행
loop_forever = True  # 마지막 도달 후 처음으로 돌아갈지 여부


class WaypointGPSSpoofer(Node):
    def __init__(self):
        super().__init__('gps_spoofer_waypoints')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,   # ← 변경됨
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.pub = self.create_publisher(NavSatFix, "/smc_2000/fix", qos)

        self.waypoints = WAYPOINTS
        self.index = 0  # 현재 지점 인덱스

        self.timer = self.create_timer(interval_sec, self.tick)

        self.get_logger().info("Waypoint GPS spoofer start")

    def tick(self):
        lat, lon, alt = self.waypoints[self.index]

        # NavSatFix 메시지 생성
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS

        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt

        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.pub.publish(msg)

        self.get_logger().info(f"Publishing waypoint {self.index}: {lat}, {lon}")

        # 다음 인덱스
        self.index += 1

        # 마지막 지점 도달 시 처리
        if self.index >= len(self.waypoints):
            if loop_forever:
                self.index = 0  # 처음부터 다시
            else:
                self.get_logger().info("All waypoints published. Stopping.")
                self.timer.cancel()


def main():
    rclpy.init()
    node = WaypointGPSSpoofer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
