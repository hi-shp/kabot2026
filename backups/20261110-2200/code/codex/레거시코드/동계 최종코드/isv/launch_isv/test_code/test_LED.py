import rclpy
from rclpy.node import Node
from mechaship_interfaces.msg import RgbwLedColor


def publish_rgbw_led(publisher, color: str, brightness: int):
    
    brightness = max(0, min(255, int(brightness)))

    msg = RgbwLedColor()

    if color == "red":
        msg.red = brightness
    elif color == "green":
        msg.green = brightness
    elif color == "blue":
        msg.blue = brightness
    elif color == "white":
        msg.white = brightness
    elif color == "off":
        pass
    else:
        return

    publisher.publish(msg)


class RgbwLedTestNode(Node):
    def __init__(self):
        super().__init__("rgbw_led_test_node")

        self.led_publisher = self.create_publisher(
            RgbwLedColor,
            "/actuator/rgbwled/color",
            10
        )

        self.step = 0
        self.timer = self.create_timer(2.0, self.timer_callback)

    def timer_callback(self):
        if self.step == 0:
            publish_rgbw_led(self.led_publisher, "red", 150)
        elif self.step == 1:
            publish_rgbw_led(self.led_publisher, "green", 150)
        elif self.step == 2:
            publish_rgbw_led(self.led_publisher, "blue", 150)
        elif self.step == 3:
            publish_rgbw_led(self.led_publisher, "white", 150)
        elif self.step == 4:
            publish_rgbw_led(self.led_publisher, "off", 0)
        else:
            self.step = -1

        self.step += 1


def main(args=None):
    rclpy.init(args=args)
    node = RgbwLedTestNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        publish_rgbw_led(node.led_publisher, "off", 0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
