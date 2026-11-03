import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32


class BatteryExample(Node):
    def __init__(self) -> None:
        # 노드의 이름을 "battery_example_node"로 초기화
        super().__init__("battery_example_node")

        # "/sensor/battery/voltage" 토픽을 구독하고, 메시지가 도착하면 battery_voltage_callback을 호출
        self.create_subscription(
            Float32, "/sensor/battery/voltage", self.battery_voltage_callback, 10
        )

    def battery_voltage_callback(self, data: Float32):
        # 수신한 배터리 전압 데이터 추출
        battery_voltage = data.data

        # 배터리 전압의 최소 및 최대 값 설정
        # 일반적인 리튬 배터리의 셀당 최저 전압은 2.8 V이나, 안전을 위해 3 V를 최저 전압으로 사용
        battery_voltage_min = 9  # 3 V * 3셀 -> 9 V
        battery_voltage_max = 12.6  # 4.2 V * 3셀 -> 12.6 V

        # 배터리 전압이 최대값을 초과하면 최대값으로 제한
        if battery_voltage > battery_voltage_max:
            battery_voltage = battery_voltage_max
        # 배터리 전압이 최소값 미만이면 최소값으로 제한
        if battery_voltage < battery_voltage_min:
            battery_voltage = battery_voltage_min

        # 배터리 퍼센트 계산
        battery_percentage = (
            (battery_voltage - battery_voltage_min)
            / (battery_voltage_max - battery_voltage_min)
        ) * 100.0

        # 배터리 전압과 퍼센트를 로그로 출력
        self.get_logger().info(
            f"voltage: {battery_voltage:.1f} V \t percentage: {battery_percentage:.1f} %"
        )


def main(args=None):
    rclpy.init(args=args)
    node = BatteryExample()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        # 키보드 인터럽트(Ctrl+C)가 발생하면 종료
        pass

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
