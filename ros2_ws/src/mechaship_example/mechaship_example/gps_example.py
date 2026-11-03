import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, NavSatStatus


class GpsExampleNode(Node):
    def __init__(self):
        # 노드의 이름을 "gps_example_node"로 초기화
        super().__init__("gps_example_node")

        # "/gps/fix" 토픽을 구독하고, 메시지가 도착하면 gps_callback 호출
        self.create_subscription(NavSatFix, "/gps/fix", self.gps_callback, 10)

    def gps_callback(self, msg: NavSatFix):
        # 수신한 GPS 데이터에서 위도, 경도, fix 상태(GPS 위성 신호가 정상적으로 수신되는지 여부) 추출
        latitude = msg.latitude  # 위도
        longitude = msg.longitude  # 경도
        fix_status = msg.status.status  # fix 상태

        if fix_status == -1:  # not fixed
            self.get_logger().warning(
                f"GPS not fixed: Data may be inaccurate"
            )  # 정확하지 않을 수 있다는 경고 문구 출력
        elif fix_status == 0:  # fixed
            pass  # 아무것도 하지 않음

        self.get_logger().info(
            f"latitude: {latitude}    longitude: {longitude}"
        )  # 위도, 경도 출력


def main(args=None):
    rclpy.init(args=args)
    node = GpsExampleNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        # 키보드 인터럽트(Ctrl+C)가 발생하면 종료
        pass

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
