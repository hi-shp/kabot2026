import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class KeyAlignNode(Node):
    def __init__(self):
        super().__init__("key_align_node")

        self.publisher = self.create_publisher(
            Float64,
            "/actuator/key/degree",
            10,
        )

        self.msg = Float64()
        self.msg.data = 90.0

        self.publish_count = 0
        self.max_publish_count = 20   # 총 몇 번 보낼지

        # 0.1초마다 publish
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        self.publisher.publish(self.msg)
        self.publish_count += 1

        self.get_logger().info(
            f"Publishing key degree {self.msg.data} ({self.publish_count}/{self.max_publish_count})"
        )

        if self.publish_count >= self.max_publish_count:
            self.get_logger().info("Key align publish finished")
            self.timer.cancel()
            self.destroy_node()
            rclpy.shutdown()


def main():
    rclpy.init()
    node = KeyAlignNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
