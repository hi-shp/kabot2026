import math
import select
import sys
import termios
import time
import tty

import rclpy
from std_msgs.msg import Float64

THROTTLE_TOPIC = "actuator/thruster/percentage"
KEY_RAD_TOPIC = "actuator/key/radian"  # for gazebo only
KEY_DEGREE_TOPIC = "actuator/key/degree"

THROTTLE_STEP_SIZE = 5.0
KEY_STEP_SIZE = 5.0

msg = """
Control Your mechaship!
---------------------------
Moving around:
        w
   a    s    d
        x
w/x : increase/decrease throttle percentage (0~100%)
a/d : increase/decrease key degree (60~120°)
space key, s : force stop (throttle 0%, key 90°)
CTRL-C to quit
"""


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def print_status(throttle, key_deg):
    sys.stdout.write("\r" + " " * 50)  # 먼저 기존 줄 지우기
    sys.stdout.write(f"\rcurrently:\tthrottle {int(throttle)}%\tkey {int(key_deg)}°")
    sys.stdout.flush()


def constrain(val, low, high):
    return max(min(val, high), low)


def check_throttle_limit(val):
    return constrain(val, -100.0, 100.0)


def check_key_limit(val):
    return constrain(val, 60.0, 120.0)


def main():
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = rclpy.create_node("teleop_keyboard")
    throttle_publisher = node.create_publisher(Float64, THROTTLE_TOPIC, 10)
    key_rad_publisher = node.create_publisher(Float64, KEY_RAD_TOPIC, 10)
    key_degree_publisher = node.create_publisher(Float64, KEY_DEGREE_TOPIC, 10)

    target_throttle = 0.0
    target_key = 90.0

    try:
        print(msg)
        while True:
            key = get_key(settings)
            if key == "w":
                target_throttle = check_throttle_limit(
                    target_throttle + THROTTLE_STEP_SIZE
                )
            elif key == "x":
                target_throttle = check_throttle_limit(
                    target_throttle - THROTTLE_STEP_SIZE
                )
            elif key == "a":
                target_key = check_key_limit(target_key - KEY_STEP_SIZE)
            elif key == "d":
                target_key = check_key_limit(target_key + KEY_STEP_SIZE)
            elif key == " " or key == "s":
                target_throttle = 0.0
                target_key = 90.0
            elif key == "\x03":  # Ctrl-C
                break
            else:
                continue

            print_status(target_throttle, target_key)

            # publish throttle
            throttle_msg = Float64()
            throttle_msg.data = target_throttle
            throttle_publisher.publish(throttle_msg)

            # publish key angle (radians)
            key_rad_msg = Float64()
            key_degree_msg = Float64()
            key_rad_msg.data = math.radians(target_key)
            key_degree_msg.data = target_key
            key_rad_publisher.publish(key_rad_msg)
            key_degree_publisher.publish(key_degree_msg)

            time.sleep(0.05)

    except Exception as e:
        print(f"\n\nException: {e}")

    finally:
        print("\n\nShutting down, resetting to default state...")

        # reset
        throttle_msg = Float64()
        throttle_msg.data = 0.0
        throttle_publisher.publish(throttle_msg)

        key_rad_msg = Float64()
        key_degree_msg = Float64()
        key_rad_msg.data = math.radians(90.0)
        key_degree_msg.data = 90.0
        key_rad_publisher.publish(key_rad_msg)
        key_degree_publisher.publish(key_degree_msg)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
