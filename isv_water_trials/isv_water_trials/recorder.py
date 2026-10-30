"""Read-only raw recorder: no actuator clients or actuator publishers."""
import argparse
from collections import Counter
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import time

from .common import load_config, dump_json, clean_json


def main(argv=None):
    from .shutdown import keep_terminal_loss
    keep_terminal_loss()
    parser = argparse.ArgumentParser()
    parser.add_argument('--config')
    parser.add_argument('--output', default=str(Path.home() / 'water_trials'))
    parser.add_argument('--duration', type=float, default=0, help='0 means until Ctrl+C')
    args = parser.parse_args(argv)
    if args.duration < 0:
        parser.error('duration must be >= 0')
    cfg = load_config(args.config)
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.convert import message_to_ordereddict
    from sensor_msgs.msg import NavSatFix, Imu, LaserScan
    from std_msgs.msg import String, Bool, Float32
    from vectornav_msgs.msg import CommonGroup

    rclpy.init()
    node = Node('water_trial_recorder')
    session = Path(args.output).expanduser() / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    session.mkdir(parents=True, exist_ok=False)
    if shutil.disk_usage(session).free < 500 * 1024**2:
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError('Less than 500 MiB free disk space')
    start = time.monotonic()
    meta = {'schema_version': 1, 'utc_start': datetime.now(timezone.utc).isoformat(),
            'config': cfg, 'mode': 'passive', 'ros_domain_id': __import__('os').environ.get('ROS_DOMAIN_ID', '0'),
            'time_basis': 'receiver monotonic clock; source header preserved in raw messages',
            'limitations': ['Service commands from other clients are NOT observed. Use trial or command logging proxy.',
                            'Receive-time alignment includes transport latency. Raw source stamps are preserved.']}
    dump_json(session / 'metadata.json', meta)
    counts = Counter()
    raw = (session / 'raw.jsonl').open('x', encoding='utf-8', buffering=1)
    subscriptions = []

    def receive(kind):
        def callback(message):
            row = {'t': time.monotonic() - start, 'ros_ns': node.get_clock().now().nanoseconds,
                   'kind': kind, 'msg': message_to_ordereddict(message)}
            raw.write(json.dumps(clean_json(row), ensure_ascii=False, allow_nan=False) + '\n')
            counts[kind] += 1
        return callback

    specs = [(NavSatFix, cfg['gps_topic'], 'gps'),
             (CommonGroup, cfg['vectornav_topic'], 'vn'),
             (Imu, cfg['imu_topic'], 'imu'), (Float32, cfg['battery_topic'], 'battery'),
             (Bool, cfg['estop_topic'], 'estop'),
             (String, '/water_trial/command', 'command'), (String, '/water_trial/mark', 'mark'),
             (LaserScan, '/scan', 'scan')]
    for message_type, topic, kind in specs:
        # Sensor BEST_EFFORT also matches RELIABLE publishers. Events are RELIABLE.
        qos = 100 if kind in ('command', 'mark') else qos_profile_sensor_data
        subscriptions.append(node.create_subscription(message_type, topic, receive(kind), qos))
    node.get_logger().info('PASSIVE recording: ' + str(session))
    last_status = start
    result = 'stopped'
    try:
        while rclpy.ok() and (args.duration == 0 or time.monotonic() - start < args.duration):
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() - last_status >= 5:
                node.get_logger().info('received ' + str(dict(counts)))
                last_status = time.monotonic()
                if shutil.disk_usage(session).free < 200 * 1024**2:
                    raise RuntimeError('Recording stopped: less than 200 MiB free')
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        result = 'error'
        raise
    finally:
        raw.flush()
        __import__('os').fsync(raw.fileno())
        raw.close()
        meta.update(utc_end=datetime.now(timezone.utc).isoformat(), duration_s=time.monotonic() - start,
                    counts=dict(counts), result=result)
        dump_json(session / 'metadata.json', meta)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print('SESSION=' + str(session))
