"""Conservative LiDAR straight/avoidance preview. Publishes advice only."""
import json
import math
import time


def sector_clearance(scan, low_deg, high_deg):
    if not (math.isfinite(scan.angle_min) and math.isfinite(scan.angle_increment)
            and scan.angle_increment > 0 and math.isfinite(scan.range_min)
            and math.isfinite(scan.range_max) and scan.range_max > scan.range_min):
        return None
    observed = []
    for index, value in enumerate(scan.ranges):
        angle = math.degrees(scan.angle_min + index * scan.angle_increment)
        if low_deg <= angle <= high_deg:
            observed.append(float(value))
    # A missing or invalid ray cannot establish that the whole path is clear.
    if len(observed) < 3 or any(not math.isfinite(v) or v < scan.range_min or v > scan.range_max for v in observed):
        return None
    return min(observed)


def advise(scan):
    front = sector_clearance(scan, -20, 20)
    left = sector_clearance(scan, 25, 70)
    right = sector_clearance(scan, -70, -25)
    clearances = {'front_m': front, 'left_m': left, 'right_m': right}
    if front is None:
        return {'action': 'STOP_UNKNOWN', 'left_pwm_us': 1500, 'right_pwm_us': 1500, **clearances}
    if front < 1.5:
        return {'action': 'STOP_NEAR', 'left_pwm_us': 1500, 'right_pwm_us': 1500, **clearances}
    if front >= 3.0:
        return {'action': 'STRAIGHT_10_PERCENT_RANGE', 'left_pwm_us': 1540, 'right_pwm_us': 1540, **clearances}
    # Turning direction and thrust sign are still unverified on the real hull.
    if left is None or right is None or max(left, right) < 3.0:
        action = 'STOP_NO_CLEAR_SIDE'
    else:
        action = 'AVOID_LEFT_PREVIEW' if left >= right else 'AVOID_RIGHT_PREVIEW'
    return {'action': action, 'left_pwm_us': 1500, 'right_pwm_us': 1500, **clearances}


def main(argv=None):
    from .shutdown import keep_terminal_loss
    keep_terminal_loss()
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    rclpy.init(args=argv)
    node = rclpy.create_node('water_trial_avoid_preview')
    publisher = node.create_publisher(String, '/water_trial/avoidance_preview', 10)
    last_scan = time.monotonic()

    def scan_callback(scan):
        nonlocal last_scan
        last_scan = time.monotonic()
        decision = advise(scan)
        decision.update(deployment_ready=False, motor_command_sent=False,
                        scan_frame=scan.header.frame_id,
                        assumption='0 deg points toward bow; mounting not confirmed')
        publisher.publish(String(data=json.dumps(decision)))

    def stale_check():
        if time.monotonic() - last_scan > .5:
            publisher.publish(String(data=json.dumps({
                'action': 'STOP_STALE_SCAN', 'left_pwm_us': 1500, 'right_pwm_us': 1500,
                'deployment_ready': False, 'motor_command_sent': False})))

    subscription = node.create_subscription(LaserScan, '/scan', scan_callback, qos_profile_sensor_data)
    timer = node.create_timer(.2, stale_check)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
