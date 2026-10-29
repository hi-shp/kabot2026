import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.exceptions import ParameterNotDeclaredException

from std_msgs.msg import Bool
from std_msgs.msg import UInt32
from mechaship_interfaces.srv import ActuatorEnable
from mechaship_interfaces.srv import ActuatorDisable


class ActuatorEnableNode(Node):
    def __init__(self) -> None:
        super().__init__("actuator_enable_node")

        self.declare_parameter("key_min_degree", 0.0)
        self.__key_min_degree = self.get_parameter_or(
            "key_min_degree", Parameter("key_min_degree", Parameter.Type.DOUBLE, 0.0)
        )

        self.declare_parameter("key_max_degree", 180.0)
        self.__key_max_degree = self.get_parameter_or(
            "key_max_degree", Parameter("key_max_degree", Parameter.Type.DOUBLE, 180.0)
        )

        self.declare_parameter("key_pulse_0_degree", 500)
        self.__key_pulse_0_degree = self.get_parameter_or(
            "key_pulse_0_degree",
            Parameter("key_pulse_0_degree", Parameter.Type.INTEGER, 500),
        )

        self.declare_parameter("key_pulse_180_degree", 2500)
        self.__key_pulse_180_degree = self.get_parameter_or(
            "key_pulse_180_degree",
            Parameter("key_pulse_180_degree", Parameter.Type.INTEGER, 2500),
        )

        self.declare_parameter("thruster_pulse_0_percentage", 1500)
        self.__thruster_pulse_0_percentage = self.get_parameter_or(
            "thruster_pulse_0_percentage",
            Parameter("thruster_pulse_0_percentage", Parameter.Type.INTEGER, 1500),
        )

        self.declare_parameter("thruster_pulse_100_percentage", 2000)
        self.__thruster_pulse_100_percentage = self.get_parameter_or(
            "thruster_pulse_100_percentage",
            Parameter("thruster_pulse_100_percentage", Parameter.Type.INTEGER, 2000),
        )

        self.__actuator_enable_client_hd = self.create_client(
            ActuatorEnable, "system/actuator/enable"
        )

        self.__actuator_disable_client_hd = self.create_client(
            ActuatorDisable, "system/actuator/disable"
        )

        msg = ActuatorEnable.Request()
        msg.key_min_degree = self.__key_min_degree.value
        msg.key_max_degree = self.__key_max_degree.value
        msg.key_pulse_0_degree = self.__key_pulse_0_degree.value
        msg.key_pulse_180_degree = self.__key_pulse_180_degree.value
        msg.thruster_pulse_0_percentage = self.__thruster_pulse_0_percentage.value
        msg.thruster_pulse_100_percentage = self.__thruster_pulse_100_percentage.value

        self.__actuator_enable_client_hd.call_async(msg)
        self.__actuator_enable_client_hd.call_async(msg)
        self.__actuator_enable_client_hd.call_async(msg)
        self.__actuator_enable_client_hd.call_async(msg)
        future = self.__actuator_enable_client_hd.call_async(msg)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("enable done")

    def shutdown(self):
        msg = ActuatorDisable.Request()

        self.__actuator_disable_client_hd.call_async(msg)
        self.__actuator_disable_client_hd.call_async(msg)
        self.__actuator_disable_client_hd.call_async(msg)
        self.__actuator_disable_client_hd.call_async(msg)
        future = self.__actuator_disable_client_hd.call_async(msg)
        rclpy.spin_until_future_complete(self, future)


def main(args=None):
    # enable actuator
    rclpy.init(args=args)
    _node = ActuatorEnableNode()

    try:
        rclpy.spin(_node)

    except KeyboardInterrupt:
        pass

    finally:
        _node.shutdown()

        _node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
