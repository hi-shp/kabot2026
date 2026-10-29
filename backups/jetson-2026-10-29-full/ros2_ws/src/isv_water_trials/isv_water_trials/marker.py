import argparse
import json
import time


def main(argv=None):
    parser = argparse.ArgumentParser(description='Mark the START of a trial segment. Use end to close it.')
    parser.add_argument('label', choices=['idle', 'straight', 'coast', 'turn_left', 'turn_right',
                                         'zigzag', 'roll_decay', 'drift', 'end'])
    parser.add_argument('--note', default='')
    args = parser.parse_args(argv)
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    rclpy.init()
    node = Node('water_trial_marker')
    pub = node.create_publisher(String, '/water_trial/mark', 100)
    deadline = time.monotonic() + 3
    while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() == 0:
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError('No recorder subscribed; marker NOT sent')
    pub.publish(String(data=json.dumps(vars(args), ensure_ascii=False)))
    pub.wait_for_all_acked(__import__('rclpy.duration', fromlist=['Duration']).Duration(seconds=1))
    node.destroy_node()
    rclpy.shutdown()
