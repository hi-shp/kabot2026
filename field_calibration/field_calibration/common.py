import json
import signal
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, Float64

MODES = ('stationary', 'straight', 'coast', 'port_turn', 'starboard_turn', 'zigzag', 'manual_log')


class FieldNode(Node):
    def __init__(self, name):
        super().__init__(name, automatically_declare_parameters_from_overrides=True)

    def param(self, name, default):
        if not self.has_parameter(name):
            self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def now(self):
        return self.get_clock().now().nanoseconds/1e9

    def publish_json(self, pub, payload):
        pub.publish(String(data=json.dumps(payload, allow_nan=False, separators=(',', ':'))))

    def json_publisher(self, topic):
        # A disconnected laptop must not create reliable DDS acknowledgement waits
        # in on-boat recording callbacks. Raw messages are recorded independently.
        return self.create_publisher(String, topic,
                                     QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))

    def subscribe_json(self, topic, callback):
        def receive(msg):
            try:
                callback(json.loads(msg.data))
            except (ValueError, KeyError, TypeError) as e:
                self.get_logger().error(f'{topic}: {e}')
        return self.create_subscription(String, topic, receive,
                                        QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec/1e9


def spin(factory, args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = factory()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # ros2 launch can relay SIGINT after the terminal has already delivered it.
        # Do not interrupt file/bag finalization with a second Ctrl+C.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
