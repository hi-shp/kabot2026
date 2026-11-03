import os
import subprocess
import re
import psutil

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.exceptions import ParameterNotDeclaredException

from std_msgs.msg import Bool
from std_msgs.msg import UInt32


class SystemNode(Node):
    def __init__(self) -> None:
        super().__init__("system_service_node")

        self.declare_parameter("auto_off", False)
        self.__auto_off = self.get_parameter_or(
            "auto_off", Parameter("auto_off", Parameter.Type.BOOL, False)
        )

        self.__ip_addr_report_publisher_hd = self.create_publisher(
            UInt32, "system/ip_address_report", 10
        )

        self.__ip_addr_report_timer_hd = self.create_timer(
            1, self.__ip_addr_report_timer_callback
        )

        if self.__auto_off.value is True:
            self.__off_sig_subscriber_hd = self.create_subscription(
                Bool, "system/power/off_sig", self.__off_sig_subscriber_callback, 10
            )

    def __find_ip_interface(self) -> str:
        try:
            # ip route 명령 실행
            result = subprocess.run(
                ["ip", "route"], capture_output=True, text=True, check=True
            )
            output = result.stdout

            # "default via"로 시작하는 줄에서 인터페이스 이름 추출
            match = re.search(r"^default .* dev (\S+)", output, re.MULTILINE)
            if match:
                return match.group(1)
            else:
                return ""
        except subprocess.CalledProcessError as e:
            print(f"Error getting default interface: {e}")
            return ""

    def __ip_addr_report_timer_callback(self):
        msg = UInt32()
        msg.data = 0

        interface = self.__find_ip_interface()
        if not interface:  # default interface 없음
            self.__ip_addr_report_publisher_hd.publish(msg)
            return

        addrs = psutil.net_if_addrs()
        if interface not in addrs:  # default interface에 ip 부여 안됨
            self.__ip_addr_report_publisher_hd.publish(msg)
            return

        for addr in addrs[interface]:
            if addr.family.name == "AF_INET":  # IPv4
                ip_parts = list(map(int, addr.address.split(".")))
                msg.data = (
                    (ip_parts[0] << 24)
                    | (ip_parts[1] << 16)
                    | (ip_parts[2] << 8)
                    | ip_parts[3]
                )

                self.__ip_addr_report_publisher_hd.publish(msg)
                return

            # IP 주소 찾지 못함
            self.__ip_addr_report_publisher_hd.publish(msg)
            return

    def __off_sig_subscriber_callback(self, msg: Bool):
        if msg.data is True:
            self.get_logger().warning(
                "Shutdown signal received. Shutting down the system..."
            )

            os.system("/usr/sbin/poweroff")  # sudo chmod a+s /usr/sbin/poweroff


def main(args=None):
    rclpy.init(args=args)

    system_node = SystemNode()

    try:
        rclpy.spin(system_node)

    except KeyboardInterrupt:
        return

    system_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
