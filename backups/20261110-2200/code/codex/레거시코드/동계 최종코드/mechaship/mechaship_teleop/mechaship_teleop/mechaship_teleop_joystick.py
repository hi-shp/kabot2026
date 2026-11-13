import rclpy
from mechaship_interfaces.msg import RgbwLedColor, ToneTopic
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64


def constrain(val, low, high):
    return max(min(val, high), low)


class TeleopJoystickNode(Node):
    __KEY_MIN_DEGREE = 60.0
    __KEY_CENTER_DEGREE = 90.0
    __KEY_MAX_DEGREE = 120.0

    __THROTTLE_MIN_PERCENTAGE = 0.0
    __THROTTLE_MAX_PERCENTAGE = 100.0

    __RGBW_BRIGHTNESS = 30

    __TONE_HZ = 2000
    __TONE_DURATION = 100

    def __init__(self) -> None:
        super().__init__("teleop_joystick")

        # Publisher
        self.__key_publisher_hd = self.create_publisher(
            Float64, "actuator/key/degree", 10
        )
        self.__throttle_publisher_hd = self.create_publisher(
            Float64, "actuator/thruster/percentage", 10
        )
        self.__rgbw_publisher_hd = self.create_publisher(
            RgbwLedColor, "actuator/rgbwled/color", 10
        )
        self.__tone_publisher_hd = self.create_publisher(
            ToneTopic, "actuator/tone/play", 10
        )

        # Subscriber
        self.__joystick_subscriber_hd = self.create_subscription(
            Joy, "joy", self.__joystick_subscriber_callback, 10
        )

        # Timer
        self.__pub_timer_hd = self.create_timer(0.1, self.__pub_timer_callback)
        self.__new_data_check_timer_hd = self.create_timer(
            1.0, self.__new_data_check_timer_callback
        )

        # State flags
        self.__is_new_data = False
        self.__is_pub_active = False
        self.__init_published = False

        # Messages
        self.__key_degree_msg = Float64()
        self.__key_degree_msg.data = self.__KEY_CENTER_DEGREE

        self.__throttle_percentage_msg = Float64()
        self.__throttle_percentage_msg.data = self.__THROTTLE_MIN_PERCENTAGE

        self.__rgbw_msg = RgbwLedColor()
        self.__red_last = 0
        self.__green_last = 0
        self.__blue_last = 0
        self.__white_last = 0

        self.__tone_msg = ToneTopic()
        self.__tone_msg.hz = self.__TONE_HZ
        self.__tone_msg.duration_ms = self.__TONE_DURATION
        self.__tone_last = 0

    def __joystick_subscriber_callback(self, data: Joy):
        self.__is_new_data = True

        axis_value = data.axes[3] * -1  # 방향 반전

        if axis_value < 0:
            scale = self.__KEY_CENTER_DEGREE - self.__KEY_MIN_DEGREE
            degree = self.__KEY_CENTER_DEGREE + axis_value * scale
        else:
            scale = self.__KEY_MAX_DEGREE - self.__KEY_CENTER_DEGREE
            degree = self.__KEY_CENTER_DEGREE + axis_value * scale

        self.__key_degree_msg.data = constrain(
            degree, self.__KEY_MIN_DEGREE, self.__KEY_MAX_DEGREE
        )

        self.__throttle_percentage_msg.data = constrain(
            data.axes[1] * self.__THROTTLE_MAX_PERCENTAGE,
            -self.__THROTTLE_MAX_PERCENTAGE,
            self.__THROTTLE_MAX_PERCENTAGE,
        )

        # RGBW LED
        self.__rgbw_msg.red = 0
        self.__rgbw_msg.green = 0
        self.__rgbw_msg.blue = 0
        self.__rgbw_msg.white = 0

        if data.buttons[0] == 1:
            self.__rgbw_msg.red = self.__RGBW_BRIGHTNESS
        if data.buttons[1] == 1:
            self.__rgbw_msg.green = self.__RGBW_BRIGHTNESS
        if data.buttons[2] == 1:
            self.__rgbw_msg.blue = self.__RGBW_BRIGHTNESS
        if data.buttons[3] == 1:
            self.__rgbw_msg.white = self.__RGBW_BRIGHTNESS

        if (
            self.__red_last != self.__rgbw_msg.red
            or self.__green_last != self.__rgbw_msg.green
            or self.__blue_last != self.__rgbw_msg.blue
            or self.__white_last != self.__rgbw_msg.white
        ):
            self.__red_last = self.__rgbw_msg.red
            self.__green_last = self.__rgbw_msg.green
            self.__blue_last = self.__rgbw_msg.blue
            self.__white_last = self.__rgbw_msg.white
            self.__rgbw_publisher_hd.publish(self.__rgbw_msg)

        if data.buttons[4] == 1 and self.__tone_last != 1:
            self.__tone_publisher_hd.publish(self.__tone_msg)
            self.__tone_last = 1
        elif data.buttons[4] == 0:
            self.__tone_last = 0

    def __pub_timer_callback(self):
        # 초기 정렬 (1회)
        if not self.__init_published:
            self.__key_publisher_hd.publish(self.__key_degree_msg)
            self.__throttle_publisher_hd.publish(self.__throttle_percentage_msg)

            self.get_logger().info(
                f"[INIT] Key aligned to {self.__KEY_CENTER_DEGREE}°, Thruster set to 0%"
            )

            self.__init_published = True
            return

        if not self.__is_pub_active:
            return

        self.get_logger().info(
            f"Throttle: {self.__throttle_percentage_msg.data:+5.1f}% \t "
            f"Key: {self.__key_degree_msg.data:5.1f}°"
        )

        self.__key_publisher_hd.publish(self.__key_degree_msg)
        self.__throttle_publisher_hd.publish(self.__throttle_percentage_msg)

    def __new_data_check_timer_callback(self):
        if self.__is_new_data:
            self.__is_pub_active = True
        else:
            self.get_logger().warning("JoyStick Not Connected!!")
            self.__is_pub_active = False

        self.__is_new_data = False


def main(args=None):
    rclpy.init(args=args)
    node = TeleopJoystickNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
