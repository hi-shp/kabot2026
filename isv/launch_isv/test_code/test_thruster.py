import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class ThrusterFixedPublisher(Node):
    def __init__(self):
        super().__init__("thruster_fixed_publisher")

        self.publisher_ = self.create_publisher(
            Float64,
            "/actuator/thruster/percentage",
            10
        )

        msg = Float64()
        msg.data = 10.0

        # 한 번만 publish
        self.publisher_.publish(msg)
        self.get_logger().info(f"Thruster set to {msg.data}%")

        # 바로 종료
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    ThrusterFixedPublisher()


if __name__ == "__main__":
    main()
