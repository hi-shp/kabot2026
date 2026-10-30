import rclpy
from rclpy.node import Node
from mechaship_interfaces.srv import Tone


class ToneMelodyTest(Node):
    def __init__(self):
        super().__init__("tone_melody_test")

        self.client = self.create_client(
            Tone,
            "/actuator/tone/addqueue"
        )

        self.get_logger().info("Waiting for /actuator/tone/addqueue service...")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, retrying...")

        # (Hz, duration_ms)
        self.melody = [
            (523, 300),  # C
            (587, 300),  # D
            (659, 300),  # E
            (523, 300),  # C
            (659, 300),  # E
            (698, 300),  # F
            (784, 600),  # G
        ]

        self.index = 0

        # 0.35초마다 한 음씩 큐에 추가
        self.timer = self.create_timer(0.35, self.timer_callback)

    def timer_callback(self):
        if self.index >= len(self.melody):
            self.get_logger().info("Melody queued completely.")
            self.timer.cancel()
            self.destroy_node()
            rclpy.shutdown()
            return

        hz, duration = self.melody[self.index]

        req = Tone.Request()
        req.hz = hz
        req.duration_ms = duration

        self.get_logger().info(f"Add tone: {hz} Hz, {duration} ms")
        self.client.call_async(req)

        self.index += 1


def main(args=None):
    rclpy.init(args=args)
    node = ToneMelodyTest()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
