"""Record raw inputs and mission outputs for replay and later manual labeling."""
import argparse
from datetime import datetime
from pathlib import Path
import shutil
import subprocess

TOPICS = [
    '/camera/image_raw', '/camera/camera_info', '/scan', '/velodyne_points',
    '/gps/fix', '/gps/fix_velocity', '/vectornav/imu', '/vectornav/raw/common',
    '/tank/odom', '/tank/pose', '/tank/session', '/map', '/tf', '/tf_static',
    '/actuators/status', '/water_trial/command', '/yolo/detections',
    '/sensor/emo/status', '/sensor/battery/voltage', '/actuators/stop_requested',
    '/shape_navigation/detections', '/shape_navigation/status', '/shape_navigation/command',
    '/shape_navigation/goal', '/shape_navigation/path',
    '/shape_navigation/config',
]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=None)
    parser.add_argument('--without-points', action='store_true')
    parser.add_argument('--extra-topic', action='append', default=[])
    args = parser.parse_args(argv)
    output = args.output or Path.home()/'water_trials/shape_bags'/datetime.now().strftime('shape_%Y%m%d_%H%M%S')
    output = output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        parser.error('Output already exists: '+str(output))
    if shutil.disk_usage(output.parent).free < 2*1024**3:
        parser.error('At least 2 GiB free disk space is required to start recording')
    from ament_index_python.packages import get_package_share_directory
    qos = Path(get_package_share_directory('isv_shape_navigation'))/'config/record_qos.yaml'
    topics = [t for t in TOPICS if not (args.without_points and t == '/velodyne_points')]
    command = ['ros2', 'bag', 'record', '-o', str(output), '--max-bag-duration', '60',
               '--max-bag-size', '268435456', '--qos-profile-overrides-path', str(qos),
               *dict.fromkeys([*topics, *args.extra_topic])]
    print('Recording to '+str(output), flush=True)
    child = subprocess.Popen(command)
    try:
        code = child.wait()
    except KeyboardInterrupt:
        # The same terminal SIGINT also reaches ros2 bag, allowing metadata flush.
        try:
            code = child.wait(timeout=15)
        except subprocess.TimeoutExpired:
            child.terminate(); code = child.wait(timeout=5)
    raise SystemExit(code)


if __name__ == '__main__':
    main()
