"""Executed from memory over SSH by pc_fast_feed, never installed on the vessel.

Passive subscriptions only: no navigation/config writes, services or actuation.
"""
import json
import math
from collections import deque
from pathlib import Path
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped
from vectornav_msgs.msg import CommonGroup
from ament_index_python.packages import get_package_share_directory
from isv_water_trials.outdoor_config import load_config


def aligned_heading(stamp, headings):
    before=next((p for p in reversed(headings) if p['measurement_stamp_s']<=stamp),None)
    return before['heading_deg'] if before and stamp-before['measurement_stamp_s']<=.3 else None


def aligned_scan_pose(stamp, headings, fixes, velocities):
    """Last measured GPS/IMU at acquisition; never extrapolate position."""
    fix = next((p for p in reversed(fixes) if p['measurement_stamp_s']<=stamp),None)
    h = next((p for p in reversed(headings) if p['measurement_stamp_s']<=stamp),None)
    if fix is None or h is None:
        return None
    dt = stamp-fix['measurement_stamp_s']
    if stamp-h['measurement_stamp_s']>.3 or dt>.5:
        return None
    heading = aligned_heading(stamp,headings)
    return dict(point=[fix['latitude_deg'],fix['longitude_deg']],heading_deg=heading,stamp_s=stamp,
                imu_skew_s=stamp-h['measurement_stamp_s'],gps_offset_s=dt,
                position_source='gps',heading_source='imu',position_compensation_s=0.)


def main():
    config = load_config(Path(get_package_share_directory('isv_water_trials'))/'config/outdoor.json')
    rclpy.init(args=['--ros-args', '--disable-rosout-logs', '--disable-external-lib-logs'])
    node = Node('pc_fast_path_readonly', enable_rosout=False, start_parameter_services=False)
    values = {}
    received = {}
    headings, fixes, velocities = (deque(maxlen=500),deque(maxlen=100),deque(maxlen=100))
    last_key = None
    last_emit = time.monotonic()

    def put(key, value):
        received[key] = time.monotonic()
        values[key] = dict(value, stamp_s=time.time())

    def measurement_time(message):
        stamp = message.header.stamp.sec+message.header.stamp.nanosec/1e9
        return dict(measurement_stamp_s=stamp,
                    measurement_age_s=(node.get_clock().now().nanoseconds/1e9-stamp))

    def heading(message):
        if not message.group_fields & 0x0008:
            return
        yaw = message.yawpitchroll.x
        if math.isfinite(yaw):
            value = dict(heading_deg=(yaw+float(config.get('heading_offset_deg', 0))+
                                          float(config.get('magnetic_declination_deg') or 0)) % 360,
                         topic='/vectornav/raw/common',**measurement_time(message))
            if headings and value['measurement_stamp_s']<=headings[-1]['measurement_stamp_s']:
                return
            headings.append(value);put('heading',value)

    def scan(message):
        pairs = []
        for i, radius in enumerate(message.ranges):
            if math.isfinite(radius) and max(0, message.range_min) < radius < min(12., message.range_max):
                pairs.append([round(math.degrees(message.angle_min+i*message.angle_increment) % 360, 3), round(radius, 3)])
        timing=measurement_time(message)
        pose=aligned_scan_pose(timing['measurement_stamp_s'],headings,fixes,velocities)
        put('scan', dict(polar_deg_m=pairs, rays=len(message.ranges), range_max_m=12., topic='/scan',
                         scan_pose=pose,scan_heading_deg=aligned_heading(timing['measurement_stamp_s'],headings),
                         scan_time_s=message.scan_time,time_increment_s=message.time_increment,
                         **timing))

    def gps(message):
        valid = message.status.status >= 0 and math.isfinite(message.latitude) and math.isfinite(message.longitude)
        value = dict(valid_fix=valid, latitude_deg=message.latitude if valid else None,
                        longitude_deg=message.longitude if valid else None,
                        horizontal_sigma_m=math.sqrt(max(0., message.position_covariance[0], message.position_covariance[4])),
                        **measurement_time(message))
        if valid:fixes.append(value)
        put('gps',value)

    def velocity(message):
        east, north = message.twist.twist.linear.x, message.twist.twist.linear.y
        if math.isfinite(east) and math.isfinite(north):
            value = dict(east_m_s=east, north_m_s=north,
                                     velocity_sigma_m_s=math.sqrt(max(0., message.twist.covariance[0], message.twist.covariance[7])),
                                     **measurement_time(message))
            velocities.append(value);put('gps_velocity',value)

    def virtual(message):
        active = message.status.status >= 0
        put('virtual_gps', dict(active=active, point=[message.latitude, message.longitude] if active else None))

    subscriptions = [node.create_subscription(CommonGroup, '/vectornav/raw/common', heading, qos_profile_sensor_data),
                     node.create_subscription(LaserScan, '/scan', scan, qos_profile_sensor_data),
                     node.create_subscription(NavSatFix, '/gps/fix', gps, qos_profile_sensor_data),
                     node.create_subscription(TwistWithCovarianceStamped, '/gps/fix_velocity', velocity, qos_profile_sensor_data),
                     node.create_subscription(NavSatFix, '/gps/virtual/fix', virtual, qos_profile_sensor_data)]

    def emit():
        nonlocal last_key, last_emit
        now = time.monotonic()
        key = tuple(v.get('stamp_s') for v in values.values())
        if key == last_key and now-last_emit < 1:
            return
        last_key = key
        last_emit = now
        packet = {k: dict(v, age_s=round(now-received[k], 4)) for k, v in values.items()}
        if packet.get('virtual_gps', {}).get('age_s', 100) > .65:
            packet['virtual_gps'] = dict(active=False, point=None, age_s=0, stamp_s=0)
        print(json.dumps(dict(values=packet, server_time_s=time.time(),
                              mount=[*config['lidar_lever_arm_m'], config['lidar_yaw_deg']],
                              dimensions=[config['length_m'], config['width_m']]),
                         separators=(',', ':'), allow_nan=False), flush=True)

    node.create_timer(.025, emit)
    try:
        rclpy.spin(node)
    except (BrokenPipeError, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
