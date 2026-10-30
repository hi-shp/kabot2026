import os
import select
import sys

import rclpy
from std_msgs.msg import Float32

import termios
import tty

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
space key, s : force stop
CTRL-C to quit
"""

e = """
Communications Failed
"""


def get_key(settings):
    if os.name == "nt":
        return msvcrt.getch().decode("utf-8")
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ""

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def print_vels(target_throttle, target_key):
    print(
        "currently:\tthrottle percentage {0}%\t key degree {1}°".format(
            target_throttle, target_key
        )
    )


def make_simple_profile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input

    return output


def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    else:
        input_vel = input_vel

    return input_vel


def check_thruster_limit(vel):
    return constrain(vel, -100.0, 100.0)


def check_key_limit(vel):
    return constrain(vel, 60.0, 120.0)


def main():
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()

    node = rclpy.create_node("teleop_keyboard")
    key_publisher = node.create_publisher(Float32, "actuator/key/degree", 10)
    throttle_publisher = node.create_publisher(
        Float32, "actuator/thruster/percentage", 10
    )

    status = 0
    target_throttle = 0.0
    target_key = 90.0

    try:
        print(msg)
        while 1:
            key = get_key(settings)
            if key == "w":
                target_throttle = check_thruster_limit(
                    target_throttle + THROTTLE_STEP_SIZE
                )
                status = status + 1
                print_vels(target_throttle, target_key)
            elif key == "x":
                target_throttle = check_thruster_limit(
                    target_throttle - THROTTLE_STEP_SIZE
                )
                status = status + 1
                print_vels(target_throttle, target_key)
            elif key == "a":
                target_key = check_key_limit(target_key - KEY_STEP_SIZE)
                status = status + 1
                print_vels(target_throttle, target_key)
            elif key == "d":
                target_key = check_key_limit(target_key + KEY_STEP_SIZE)
                status = status + 1
                print_vels(target_throttle, target_key)
            elif key == " " or key == "s":
                target_throttle = 0.0
                target_key = 90.0
                print_vels(target_throttle, target_key)
            else:
                if key == "\x03":
                    break
                continue

            if status == 20:
                print(msg)
                status = 0

            _msg = Float32()

            _msg.data = target_key
            key_publisher.publish(_msg)

            _msg.data = target_throttle
            throttle_publisher.publish(_msg)

    except Exception as e:
        print(e)

    finally:
        _msg = Float32()

        _msg.data = 90.0
        key_publisher.publish(_msg)

        _msg.data = 0.0
        throttle_publisher.publish(_msg)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == "__main__":
    main()
