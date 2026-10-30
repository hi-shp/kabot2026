import serial
import time
import signal
import sys

import rclpy
from rclpy.node import Node

from nmea_msgs.msg import Sentence
from rtcm_msgs.msg import Message


class WtrtkRos2Driver(Node):
    END_CHAR = "\r\n"

    def __init__(self):
        super().__init__("wtrtk_ros2_driver")

        self._is_first_data_rcv = False

        self.declare_parameters(
            namespace="",
            parameters=[
                ("serial_port", "/dev/ttyUSB0"),
                ("serial_baudrate", 115200),
                ("publish_nmea_topic", "nmea_sentence"),
                ("publish_nmea_topic_frame_id", "gps_link"),
                ("subscribe_rtcm_topic", ""),
                ("update_rate_1hz", True),
                ("update_rate_2hz", False),
                ("update_rate_5hz", False),
                ("update_rate_10hz", False),
                ("update_rate_20hz", False),
                ("update_rate_50hz", False),
            ],
        )

        # Serial settings
        self._serial_port = (
            self.get_parameter("serial_port").get_parameter_value().string_value
        )
        self._serial_baudrate = (
            self.get_parameter("serial_baudrate").get_parameter_value().integer_value
        )

        # Topics
        self._publish_nmea_topic = (
            self.get_parameter("publish_nmea_topic").get_parameter_value().string_value
        )
        self._frame_id = (
            self.get_parameter("publish_nmea_topic_frame_id")
            .get_parameter_value()
            .string_value
        )
        self._subscribe_rtcm_topic = (
            self.get_parameter("subscribe_rtcm_topic")
            .get_parameter_value()
            .string_value
        )
        enable_rtcm = False if len(self._subscribe_rtcm_topic) == 0 else True

        # GPS report period
        period = 1
        period = (
            1
            if self.get_parameter("update_rate_1hz").get_parameter_value().bool_value
            else period
        )
        period = (
            0.5
            if self.get_parameter("update_rate_2hz").get_parameter_value().bool_value
            else period
        )
        period = (
            0.2
            if self.get_parameter("update_rate_5hz").get_parameter_value().bool_value
            else period
        )
        period = (
            0.1
            if self.get_parameter("update_rate_10hz").get_parameter_value().bool_value
            else period
        )
        period = (
            0.05
            if self.get_parameter("update_rate_20hz").get_parameter_value().bool_value
            else period
        )
        period = (
            0.02
            if self.get_parameter("update_rate_50hz").get_parameter_value().bool_value
            else period
        )

        self._timer_hd = self.create_timer(
            0.01, self._timer_nmea_polling_callback, autostart=False
        )
        self._pub_hd = self.create_publisher(Sentence, self._publish_nmea_topic, 10)
        if enable_rtcm is True:
            self.create_subscription(
                Message, self._subscribe_rtcm_topic, self._sub_rtcm_callback, 10
            )

        self._connect_serial()

        self._stop_log()

        self.get_logger().info("Configureing GPS module...")

        self._send_command("MODE ROVER AUTOMOTIVE DEFAULT")

        self._send_command(f"GNGGA {period}")
        self._send_command(f"GPGST {period}")
        self._send_command(f"GPVTG {period}")

        self._timer_hd.reset()

    def _stop_log(self):
        self._send_command("")

        MAX_RETRIES = 20
        for attempt in range(1, MAX_RETRIES + 1):
            try:
                self.get_logger().info(
                    f"Stopping log... attempt {attempt}/{MAX_RETRIES}"
                )
                self._send_command("UNLOG")
                break
            except TimeoutError:
                if attempt == MAX_RETRIES:
                    raise TimeoutError("Stopping log failed")
        else:
            raise TimeoutError("Stop log failed")

    def _connect_serial(self):
        self._is_first_data_rcv = False
        while True:
            try:
                self._gps = serial.Serial(
                    port=self._serial_port, baudrate=self._serial_baudrate, timeout=0.1
                )

            except serial.SerialException as e:
                self.get_logger().warning(
                    f"Could not open serial port: I/O error({e.errno}): {e.strerror}"
                )
                self.get_logger().warning("Retry after 5 secounds...")
                time.sleep(5)

            except Exception as e:
                self.get_logger().fatal(f"Unexpected error: {e}")
                self.get_logger().fatal("Shutting down node...")

                exit()

            else:
                self.get_logger().debug("Serial port connected!!")
                break

    def _send_command(self, command: str):
        try:
            self._gps.write((command + self.END_CHAR).encode())

            if len(command) == 0:
                return

            start = time.monotonic()
            while True:
                data = self._gps.readline().decode("utf-8", errors="ignore").strip()
                if f"command,{command},response: OK" in data:
                    break
                if time.monotonic() - start > 1:
                    raise TimeoutError("Command failed")
        except TimeoutError:
            raise

        except Exception as e:
            self.get_logger().fatal(f"Unexpected error: {e}")
            self.get_logger().fatal("Shutting down node...")
            exit()

    def _timer_nmea_polling_callback(self):
        data = self._gps.readline().decode("utf-8", errors="ignore").strip()

        if len(data) == 0:
            return  # 빈 문자열 필터링

        if not data.startswith("$"):
            return  # 디버그 데이터 필터링

        self.get_logger().debug(data)

        if data.startswith("$command") or data.startswith("$devicename"):
            return  # 설정 관련 메시지 필터링

        if self._is_first_data_rcv is False:
            self._is_first_data_rcv = True
            self.get_logger().info("First data received")

        pub = Sentence()
        pub.header.frame_id = self._frame_id
        pub.header.stamp = self.get_clock().now().to_msg()
        pub.sentence = data

        self._pub_hd.publish(pub)

    def _sub_rtcm_callback(self, msg: Message):
        byte_data = bytes(msg.message)
        self._gps.write(byte_data)


def sigint_handler_int(signal_received, frame):
    global node

    node.get_logger().warn("Shutting down node...")

    node._stop_log()

    node.destroy_node()
    sys.exit(0)


def sigint_handler_term(signal_received, frame):
    pass  # ignore


node = None


def main(args=None):
    global node
    rclpy.init(args=args)
    rclpy.signals.uninstall_signal_handlers()

    signal.signal(signal.SIGINT, sigint_handler_int)
    signal.signal(signal.SIGTERM, sigint_handler_term)

    try:
        node = WtrtkRos2Driver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
