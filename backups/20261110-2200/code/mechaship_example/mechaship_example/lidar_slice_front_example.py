from math import radians

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan


class LidarPublisher(Node):
    def __init__(self):
        # 노드의 이름을 "lidar_publisher"로 초기화
        super().__init__("lidar_publisher")

        # "/scan" 토픽을 구독하고, 메시지가 도착하면 lidar_callback 호출
        self.subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.lidar_callback,
            qos_profile_sensor_data,  # QoS 설정 적용
        )

        # "/scan_sliced" 토픽에 LaserScan 메시지를 발행할 퍼블리셔 생성
        self.lidar_sliced_publisher = self.create_publisher(
            LaserScan,
            "/scan_sliced",
            10,  # Publish하는 /scan_sliced Topic은 QoS 설정 적용하지 않음
        )

    def lidar_callback(self, data: LaserScan):
        # 새로운 LaserScan 메시지 객체 생성
        msg = LaserScan()

        # TG15 LiDAR는 왼쪽부터 360° 범위 데이터를 2000개로 쪼개어 ranges 배열에 담음
        # 전방에 해당하는 데이터는 500~1499 범위
        msg.ranges = data.ranges[500:1500]

        # angle_min, angle_max는 Publish되는 데이터의 시작, 종료 각도를 입력
        # 여기서는 -90~+90까지의 전방을 Publish하기 때문에 -90~+90을 Radian으로 바꾸어 입력
        msg.angle_min = radians(-90)  # 시작 각도
        msg.angle_max = radians(90)   # 종료 각도

        # 나머지 데이터는 기존 데이터를 그대로 사용
        msg.header = data.header               # publish되는 데이터를 설명하는 헤더 (publish 시간, 센서 위치 등 저장)
        msg.angle_increment = data.angle_increment  # ranges 배열에 저장된 데이터간의 각도 관계
        msg.time_increment = data.time_increment    # ranges 배열에 저장된 데이터간의 시간 관계
        msg.scan_time = data.scan_time              # 직전 측정 결과와 현재 측정 결과 간의 시간 변위
        msg.range_min = data.range_min              # LiDAR의 최소 측정 가능 거리
        msg.range_max = data.range_max              # LiDAR의 최대 측정 가능 거리

        # "/scan_sliced" 토픽에 슬라이스된 LaserScan 메시지 발행
        self.lidar_sliced_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = LidarPublisher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        # 키보드 인터럽트(Ctrl+C)가 발생하면 종료
        pass

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()