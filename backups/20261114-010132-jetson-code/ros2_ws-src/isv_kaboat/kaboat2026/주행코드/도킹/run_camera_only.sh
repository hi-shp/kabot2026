#!/usr/bin/env bash
# All subprocesses inherit domain 0. With --enable-output, targets trigger motion.
set -eo pipefail
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
source "$HOME/ros2_ws/src/isv_kaboat/kaboat2026/.runtime/install/setup.bash"
export CAMERA_ONLY_DIRECTORY="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec /usr/bin/python3 - "$@" <<'PY'
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.signals import SignalHandlerOptions

parser = argparse.ArgumentParser(description='Camera/YOLO/best3.pt/pixel following; domain 0')
parser.add_argument('--target', default='green circle', choices=[
    c + ' ' + s for c in ('blue', 'green', 'red')
    for s in ('circle', 'cross', 'triangle')])
parser.add_argument('--enable-output', action='store_true',
                    help='Enable follow mode: neutral without target, move on detection')
options = parser.parse_args()
directory = Path(os.environ['CAMERA_ONLY_DIRECTORY'])
follower = directory / 'camera_only.py'
model = Path.home() / 'ros2_ws/weights/best3.pt'
if not follower.is_file() or not model.is_file():
    raise SystemExit('camera_only.py or ~/ros2_ws/weights/best3.pt is missing')

children = []
rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
observer = rclpy.create_node('camera_only_bringup_observer')

def interrupted(signum, frame):
    raise KeyboardInterrupt

signal.signal(signal.SIGINT, interrupted)
signal.signal(signal.SIGTERM, interrupted)

def spin(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(observer, timeout_sec=0.1)
        for name, child in children:
            if child.poll() is not None:
                raise RuntimeError(f'{name} exited: {child.returncode}; see its output above')

def launch(name, command):
    child = subprocess.Popen(command, stdin=subprocess.DEVNULL, start_new_session=True)
    children.append((name, child))
    print(f'Started {name}, PID={child.pid}', flush=True)

def wait_for(predicate, timeout, description):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        spin(0.2)
    raise RuntimeError('Timed out waiting for ' + description)

def camera_available():
    return observer.count_publishers('/camera/image_raw') > 0

try:
    spin(3)
    if 'object_pixel_follower' in observer.get_node_names():
        raise RuntimeError('An object_pixel_follower already exists in domain 0')

    # A foreign-domain camera/serial process would own the same physical devices.
    for process in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            argv = process.read_bytes().split(b'\0')
            if not any(token.endswith((b'/usb_cam_node_exe', b'/arduino_bridge'))
                       for token in argv):
                continue
            env = dict(part.split(b'=', 1) for part in
                       (process.parent / 'environ').read_bytes().split(b'\0')
                       if b'=' in part)
            if int(env.get(b'ROS_DOMAIN_ID', b'0')) != 0:
                raise RuntimeError('Sensor process exists in another domain; stop its launch first')
        except (OSError, ProcessLookupError):
            pass

    names = set(observer.get_node_names())
    sensor_names = {'smc_2000', 'vectornav', 'arduino_actuator_bridge',
                    'velodyne_driver_node', 'sensor_status'}
    if not names.intersection(sensor_names):
        launch('sensors', ['ros2', 'launch', 'mechaship_bringup',
            'all_sensors.launch.py', 'use_camera:=true', 'use_outdoor:=false'])
        wait_for(camera_available, 30, 'camera publisher')
    elif not camera_available():
        # Reuse the existing sensor session without restarting its devices.
        launch('camera', ['ros2', 'run', 'usb_cam', 'usb_cam_node_exe',
            '--ros-args', '-r', '__node:=usb_cam', '-r', '__ns:=/camera',
            '-p', 'video_device:=/dev/video0', '-p', 'framerate:=15.0',
            '-p', 'image_width:=640', '-p', 'image_height:=480',
            '-p', 'pixel_format:=yuyv2rgb', '-p', 'io_method:=mmap',
            '-p', 'frame_id:=camera_frame'])
        wait_for(camera_available, 20, 'camera publisher')

    if observer.count_publishers('/yolo/detections') == 0:
        launch('YOLO best3.pt', ['ros2', 'launch', 'yolo_bringup', 'yolo.launch.py',
            'model:=' + str(model), 'input_image_topic:=/camera/image_raw/compressed',
            'image_reliability:=2', 'device:=cuda:0', 'threshold:=0.50',
            'use_tracking:=False', 'use_3d:=False', 'use_debug:=True'])
        wait_for(lambda: observer.count_publishers('/yolo/detections') > 0,
                 60, 'YOLO detection publisher')
    else:
        from rcl_interfaces.srv import GetParameters
        client = observer.create_client(GetParameters, '/yolo/yolo_node/get_parameters')
        if not client.wait_for_service(timeout_sec=3):
            raise RuntimeError('Cannot verify the existing YOLO model')
        future = client.call_async(GetParameters.Request(names=['model']))
        wait_for(future.done, 5, 'YOLO model parameter')
        response = future.result()
        if (response is None or not response.values
                or Path(response.values[0].string_value).expanduser().resolve() != model.resolve()):
            raise RuntimeError('Existing YOLO uses another model; refusing to change it')
        print('Reusing verified YOLO best3.pt', flush=True)

    launch('camera_only', ['/usr/bin/python3', str(follower), '--ros-args',
        '-p', 'target_object:=' + options.target,
        '-p', 'auto_follow:=' + str(options.enable_output).lower(),
        '-p', 'enable_output:=' + str(options.enable_output).lower()])
    print('ROS_DOMAIN_ID=0; target=' + options.target, flush=True)
    print('Output enabled=' + str(options.enable_output)
          + '; automatic follow on detection when enabled. Ctrl+C stops processes started here.', flush=True)
    while True:
        spin(0.5)
except KeyboardInterrupt:
    pass
except Exception as exc:
    print('STARTUP FAILED: ' + str(exc), file=sys.stderr, flush=True)
    sys.exitcode = 1
finally:
    # Stop the follower before its upstream camera and actuator transport.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    for name, child in reversed(children):
        if child.poll() is None:
            try:
                os.killpg(child.pid, signal.SIGINT)
                child.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGTERM)
                try:
                    child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait()
            except ProcessLookupError:
                pass
    observer.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
sys.exit(getattr(sys, 'exitcode', 0))
PY
