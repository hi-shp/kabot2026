
#!/usr/bin/env python3
"""YOLO pixel following with paired thrusters.

Requires the kaboat2026 runtime overlay.
"""

import json
import math
import os
import signal
import sys
import time


class PixelPD:
    def __init__(self, kp=8.0, kd=0.0, turn_limit=400.0):
        self.kp, self.kd, self.limit = kp, kd, turn_limit
        self.reset()

    def reset(self):
        self.previous = None

    def calculate(self, error_deg, now):
        derivative = 0.0
        if self.previous is not None:
            old_error, old_time = self.previous
            dt = now - old_time
            if 0.01 <= dt <= 0.5:
                derivative = (error_deg - old_error) / dt
        self.previous = error_deg, now
        return max(
            -self.limit,
            min(
                self.limit,
                self.kp * error_deg + self.kd * derivative
            )
        )


def pixel_angle(x, width, fov_deg):
    if not all(math.isfinite(v) for v in (x, width, fov_deg)):
        raise ValueError('Nonfinite camera geometry')
    if width <= 0 or not 0 <= x < width or not 0 < fov_deg < 180:
        raise ValueError('Invalid camera geometry')

    focal = width / 2 / math.tan(math.radians(fov_deg / 2))
    return -math.degrees(math.atan2(x - width / 2, focal))


def mix_pwm(base_pwm, turn):
    left_delta = base_pwm - 1500 - turn
    right_delta = base_pwm - 1500 + turn

    scale = max(
        1.0,
        abs(left_delta) / 400,
        abs(right_delta) / 400
    )

    return (
        round(1500 + left_delta / scale),
        round(1500 + right_delta / scale)
    )


def follow_command(ready, base_pwm, turn):
    return mix_pwm(base_pwm, turn) if ready else (1500, 1500)


def detection_state(ready):
    # 0 = detected, 1 = not detected
    return 0 if ready else 1


def camera_output_class(base):
    class CameraOutput(base):
        def _tick(self, now, pwm, telemetry, telemetry_age):
            if self.pending and all(f.done() for f in self.pending):
                self.sent_at = now
            previous_error = self.error
            result = super()._tick(now, pwm, telemetry, telemetry_age)
            # A transient stop-ACK failure must not outlive a later successful stop.
            if self.stop_ack and self.error == 'STOP_UNCONFIRMED':
                self.error = getattr(self, 'first_output_fault', '') or 'OUTPUT_STOPPED'
            elif (self.error and self.error != 'STOP_UNCONFIRMED'
                  and self.error != previous_error):
                self.first_output_fault = self.error
            return result
    return CameraOutput


def load_ros_environment():
    os.environ.setdefault('ROS_DOMAIN_ID', '0')
    os.environ.setdefault(
        'RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'
    )
    os.environ.setdefault('ROS_LOCALHOST_ONLY', '0')

    import importlib.util

    try:
        available = all(
            importlib.util.find_spec(name) is not None
            for name in (
                'rclpy',
                'yolo_msgs',
                'isv_shape_navigation'
            )
        )
    except (ImportError, ModuleNotFoundError):
        available = False

    if available:
        return

    if os.environ.get('CAMERA_ONLY_ENV_LOADED') == '1':
        raise RuntimeError(
            'ROS packages unavailable after sourcing overlays'
        )

    import subprocess

    command = '''set -e
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
source "$HOME/ros2_ws/src/isv_kaboat/kaboat2026/.runtime/install/setup.bash"
exec env -0
'''

    result = subprocess.check_output(
        ['bash', '-c', command]
    )

    for entry in result.split(b'\0'):
        if b'=' in entry:
            key, value = entry.split(b'=', 1)
            os.environ[os.fsdecode(key)] = os.fsdecode(value)

    os.environ['CAMERA_ONLY_ENV_LOADED'] = '1'
    os.execv(sys.executable, [sys.executable, *sys.argv])


# Stage 1: configuration only. No ROS imports, subscriptions or actuator calls.
VDOCK_SHAPES = tuple(
    colour + ' ' + shape
    for colour in ('blue', 'green', 'red')
    for shape in ('circle', 'cross', 'triangle')
)


def vdock_empty_config():
    return dict(version=1, target_shape=None, regions=[
        dict(id=i, latitude=None, longitude=None, shape=None)
        for i in (1, 2, 3)
    ])


def vdock_validate(config):
    if not isinstance(config, dict) or config.get('version') != 1:
        raise ValueError('Unsupported Vdock configuration')
    regions = config.get('regions')
    if (not isinstance(regions, list) or len(regions) != 3
            or any(not isinstance(r, dict) for r in regions)
            or [r.get('id') for r in regions] != [1, 2, 3]):
        raise ValueError('Regions must contain IDs 1, 2, 3 in order')
    assigned = []
    missing = []
    for region in regions:
        lat, lon, shape = (region.get(k) for k in ('latitude', 'longitude', 'shape'))
        if (lat is None) != (lon is None):
            raise ValueError('Latitude and longitude must be specified together')
        if lat is not None:
            if (type(lat) not in (int, float) or type(lon) not in (int, float)
                    or not math.isfinite(lat) or not math.isfinite(lon)
                    or not -90 <= lat <= 90 or not -180 <= lon <= 180):
                raise ValueError('Invalid GPS coordinates')
        else:
            missing.append(f'region_{region["id"]}_gps')
        if shape is not None:
            if shape not in VDOCK_SHAPES:
                raise ValueError('Unknown best3.pt shape: ' + str(shape))
            assigned.append(shape)
        else:
            missing.append(f'region_{region["id"]}_shape')
    if len(assigned) != len(set(assigned)):
        raise ValueError('Assign distinct shapes to avoid ambiguous GPS selection')
    target = config.get('target_shape')
    if target is not None and target not in VDOCK_SHAPES:
        raise ValueError('Unknown target shape')
    if target is None:
        missing.append('target_shape')
    elif target not in assigned:
        missing.append('target_shape_region')
    return dict(ready=not missing, missing=missing)


def vdock_update(config, region_id=None, latitude=None, longitude=None,
                 shape=None, target_shape=None):
    import copy
    result = copy.deepcopy(config)
    vdock_validate(result)
    if region_id is not None:
        if region_id not in (1, 2, 3):
            raise ValueError('Region must be 1, 2 or 3')
        region = result['regions'][region_id - 1]
        if latitude is not None or longitude is not None:
            if latitude is None or longitude is None:
                raise ValueError('Provide latitude and longitude together')
            region.update(latitude=latitude, longitude=longitude)
        if shape is not None:
            region['shape'] = shape
    elif latitude is not None or longitude is not None or shape is not None:
        raise ValueError('Choose --region before editing its GPS or shape')
    if target_shape is not None:
        result['target_shape'] = target_shape
    vdock_validate(result)
    return result


def vdock_setup_main(args=None):
    import argparse
    from pathlib import Path
    import tempfile

    parser = argparse.ArgumentParser(
        description='Vdock stage 1: GPS/shape configuration only; no ROS or motion')
    parser.add_argument('--config', type=Path,
                        default=Path(__file__).with_name('vdock_regions.json'))
    parser.add_argument('--init', action='store_true')
    parser.add_argument('--region', type=int, choices=(1, 2, 3))
    parser.add_argument('--latitude', type=float)
    parser.add_argument('--longitude', type=float)
    parser.add_argument('--shape', choices=VDOCK_SHAPES)
    parser.add_argument('--target', choices=VDOCK_SHAPES)
    options = parser.parse_args(args)
    try:
        if options.config.exists():
            config = json.loads(options.config.read_text(encoding='utf-8'))
        elif options.init:
            config = vdock_empty_config()
        else:
            raise ValueError('Config not found; use --vdock-setup --init first')
        vdock_validate(config)
        changed = (options.init or options.region is not None
                   or options.latitude is not None or options.longitude is not None
                   or options.shape is not None or options.target is not None)
        config = vdock_update(config, options.region, options.latitude,
                              options.longitude, options.shape, options.target)
        if changed:
            with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8',
                                             dir=options.config.parent,
                                             delete=False) as stream:
                temp = Path(stream.name)
                json.dump(config, stream, ensure_ascii=False, indent=2)
                stream.write('\n')
            try:
                os.replace(temp, options.config)
            finally:
                temp.unlink(missing_ok=True)
        print(json.dumps(dict(config_path=str(options.config), config=config,
                              validation=vdock_validate(config),
                              mode='SETUP_ONLY', output_enabled=False),
                         ensure_ascii=False, indent=2))
    except (ValueError, OSError) as error:
        parser.exit(2, 'Vdock setup failed: ' + str(error) + '\n')


class VdockNavigation:
    """Latch one detected GPS target and run a single arrival sequence."""

    def __init__(self, goal, target_index, threshold):
        self.goal = goal
        self.target_index = target_index
        self.threshold = threshold
        self.selected = False
        self.reverse_started = None
        self.complete = False
        self.distance = None
        self.goal_angle = None
        self.selected_angle = None
        self.reverse_skipped = False

    def detect(self, detections):
        if self.goal is None or self.selected:
            return
        self.selected = any(
            item.class_id == self.target_index
            and item.class_name == VDOCK_SHAPES[self.target_index]
            and math.isfinite(item.score) and item.score >= self.threshold
            for item in detections
        )

    def step(self, now, executing, fix, heading, scan, vector, wrap, choose,
             reverse_clear, drive, reverse_seconds, reverse_pwm):
        neutral = (1500, 1500)
        self.selected_angle = None
        if self.complete:
            return 'ARRIVED', neutral
        if self.reverse_started is not None:
            if (not executing or fix is None or heading is None or scan is None
                    or now - self.reverse_started >= reverse_seconds
                    or not reverse_clear(scan)):
                self.complete = True
                return 'ARRIVED', neutral
            return 'ARRIVAL_REVERSE', reverse_pwm
        if not self.selected:
            return ('WAIT_TARGET' if self.goal else 'NO_TARGET_GPS'), neutral
        self.distance = self.goal_angle = None
        if fix is None:
            return 'WAIT_GPS', neutral
        east, north = vector(fix, *self.goal)
        self.distance = math.hypot(east, north)
        if heading is None:
            return 'WAIT_HEADING', neutral
        if scan is None:
            return 'WAIT_SCAN', neutral
        if self.distance <= 1.0:
            if not executing:
                return 'AT_GOAL', neutral
            if not reverse_clear(scan):
                self.complete = self.reverse_skipped = True
                return 'ARRIVED', neutral
            self.reverse_started = now
            return 'ARRIVAL_REVERSE', reverse_pwm
        self.goal_angle = wrap(math.atan2(north, east) - heading)
        self.selected_angle, report = choose(scan, self.goal_angle)
        if self.selected_angle is None:
            return report['reason'], neutral
        return 'GPS_APPROACH', drive(self.selected_angle)


def vdock_gap_selector(v3):
    """Reuse V3 unchanged with a private namespace for its two ranking functions."""
    from types import FunctionType

    scope = dict(v3.choose_gap_v3.__globals__, ANGLE_WEIGHT=500.0)
    scope['score_candidate'] = FunctionType(v3.score_candidate.__code__, scope)
    return FunctionType(v3.choose_gap_v3.__code__, scope,
                        argdefs=v3.choose_gap_v3.__defaults__)


def main(args=None):
    load_ros_environment()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage, LaserScan, NavSatFix
    from std_msgs.msg import String, Empty
    from std_srvs.srv import SetBool, Trigger
    from yolo_msgs.msg import DetectionArray
    from mechaship_interfaces.srv import ThrottlePulseWidth
    from isv_water_trials.actuator_config import command_pwm
    from isv_shape_navigation.output import (
        PairedOutput,
        OutputReadiness
    )
    from isv_shape_navigation.output_runtime import (
        CommandLease,
        acquire_output_lock,
        conflicting_output_nodes
    )

    import cv2
    from dataclasses import fields
    from pathlib import Path
    import numpy as np
    import yaml
    from vectornav_msgs.msg import CommonGroup
    from kaboat_avoidance import avoidance_v3
    from kaboat_avoidance.route_mission import RouteConfig
    from isv_shape_navigation.core import Scan, select_navigation_scan, wrap
    from isv_shape_navigation.gps_input import inspect_fix
    from isv_shape_navigation.gps_mission import gps_vector
    from isv_shape_navigation.local_dock import motion_clear

    directory = Path(__file__).resolve().parent
    course = yaml.safe_load((directory.parents[1] / '설정/course_mission.yaml').read_text(
        encoding='utf-8'))['course_mission']['ros__parameters']
    regions = json.loads((directory / 'vdock_regions.json').read_text(encoding='utf-8'))
    vdock_validate(regions)
    route = RouteConfig(**{field.name: course[field.name] for field in fields(RouteConfig)
                          if field.name in course})
    choose_gap = vdock_gap_selector(avoidance_v3)
    clearance = course['footprint_radius'] + course['clearance']
    reverse_seconds = avoidance_v3.RouteMissionV3.waypoint_reverse_seconds
    reverse_pwm = avoidance_v3.RouteMissionV3.waypoint_reverse_pwm

    class Follower(Node):
        def __init__(self):
            super().__init__('object_pixel_follower_vdock')

            defaults = dict(
                target_object=regions['target_shape'] or 'green circle',
                vdock_gps_topic='/gps/fix',
                horizontal_fov_deg=90.0,
                confidence_threshold=0.50,
                kp=8.0,
                kd=0.0,
                turn_limit_us=400.0,
                thrust_percentage=20.0,
                detection_timeout_sec=0.5,
                image_timeout_sec=1.0,
                enable_output=False,
                auto_follow=False,
                output_rate_hz=10.0
            )

            self.p = {}
            for name, value in defaults.items():
                self.p[name] = self.declare_parameter(
                    name, value
                ).value

            for name, value in self.p.items():
                if isinstance(value, float) and not math.isfinite(value):
                    raise ValueError('Invalid parameter: ' + name)

            classes = [
                c + ' ' + s
                for c in ('blue', 'green', 'red')
                for s in ('circle', 'cross', 'triangle')
            ]

            if self.p['target_object'] not in classes:
                raise ValueError('Unknown best3.pt target class')

            if not (
                0 < self.p['horizontal_fov_deg'] < 180
                and 0 <= self.p['confidence_threshold'] <= 1
                and 0 <= self.p['thrust_percentage'] <= 100
                and 0 <= self.p['turn_limit_us'] <= 400
                and self.p['kp'] >= 0
                and self.p['kd'] >= 0
                and self.p['detection_timeout_sec'] > 0
                and self.p['image_timeout_sec'] > 0
            ):
                raise ValueError('Invalid controller parameters')

            if not 5.0 <= self.p['output_rate_hz'] <= 10.0:
                raise ValueError('Output rate must be 5..10 Hz')

            target_region = next((region for region in regions['regions']
                                  if region['shape'] == self.p['target_object']), None)
            goal = ((target_region['latitude'], target_region['longitude'])
                    if target_region and target_region['latitude'] is not None else None)
            self.navigation = VdockNavigation(goal, VDOCK_SHAPES.index(self.p['target_object']),
                                             self.p['confidence_threshold'])
            self.fix = None
            self.fix_stamp = None
            self.heading = None
            self.heading_stamp = -math.inf
            self.scans = [None, None]
            self.create_subscription(NavSatFix, self.p['vdock_gps_topic'], self.gps_fix,
                                     qos_profile_sensor_data)
            self.create_subscription(CommonGroup, course['heading_topic'], self.heading_fix,
                                     qos_profile_sensor_data)
            for index, topic in enumerate((course['scan_topic'], course['scan_fallback_topic'])):
                self.create_subscription(LaserScan, topic,
                                         lambda msg, slot=index: self.lidar_scan(msg, slot),
                                         qos_profile_sensor_data)

            self.pd = PixelPD(
                self.p['kp'],
                self.p['kd'],
                self.p['turn_limit_us']
            )

            self.width = None
            self.image_at = -math.inf
            self.target_at = -math.inf
            self.telemetry_at = -math.inf

            self.telemetry = {}
            self.target = None
            self.reason = 'WAIT_DATA'

            self.lease = CommandLease()
            self.readiness = OutputReadiness()

            self.output = None
            self.output_lock = None

            # Keep auto-follow enabled across target loss.
            self.auto_follow_allowed = bool(
                self.p['enable_output']
                and self.p['auto_follow']
            )

            # Explicit stop or output fault inhibits auto-restart.
            self.stop_latched = False

            self.conflicts = []
            self.graph_at = -math.inf
            self.last_print_at = -math.inf
            self.logged_output_error = ''

            self.status_pub = self.create_publisher(
                String,
                '~/status',
                10
            )

            self.create_subscription(
                CompressedImage,
                '/camera/image_raw/compressed',
                self.image,
                qos_profile_sensor_data
            )

            self.create_subscription(
                DetectionArray,
                '/yolo/detections',
                self.detections,
                qos_profile_sensor_data
            )

            self.create_subscription(
                String,
                '/actuators/status',
                self.feedback,
                qos_profile_sensor_data
            )

            self.create_subscription(
                Empty,
                '/water_trial/stop',
                self.external_stop,
                10
            )

            if self.p['enable_output']:
                if self.get_parameter('use_sim_time').value:
                    raise ValueError(
                        'Hardware output requires real time'
                    )

                self.output_lock = acquire_output_lock()

                self.arm = self.create_client(
                    SetBool,
                    '/actuators/arm'
                )

                self.stop_client = self.create_client(
                    Trigger,
                    '/actuators/stop'
                )

                self.pwm = [
                    self.create_client(
                        ThrottlePulseWidth,
                        '/actuators/throttle_' + side +
                        '/set_pulse_width'
                    )
                    for side in ('left', 'right')
                ]

                self.output = camera_output_class(PairedOutput)(
                    lambda v: self.arm.call_async(
                        SetBool.Request(data=v)
                    ),
                    lambda l, r: [
                        client.call_async(
                            ThrottlePulseWidth.Request(
                                pulse_width=value
                            )
                        )
                        for client, value in zip(
                            self.pwm, (l, r)
                        )
                    ],
                    lambda: self.stop_client.call_async(
                        Trigger.Request()
                    ),
                    self.services_ready,
                    timeout=0.6,
                    stop_ready=self.stop_client.service_is_ready
                )

            self.get_logger().warning(
                'Camera-only: detection_state 0=FOLLOW, '
                '1=NEUTRAL; '
                f'output={self.p["enable_output"]}, '
                f'auto_follow={self.p["auto_follow"]}. '
                'Target detection can move the vessel.'
            )

            self.create_service(
                Trigger, '~/start', self.start
            )
            self.create_service(
                Trigger, '~/stop', self.stop_service
            )

            self.create_timer(0.1, self.control)
            self.create_timer(
                1.0 / self.p['output_rate_hz'],
                self.output_tick
            )

        def image(self, msg):
            if not msg.data or not self.message_fresh(msg):
                return
            try:
                frame = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8),
                                     cv2.IMREAD_GRAYSCALE)
            except cv2.error:
                return
            if frame is not None:
                self.width = float(frame.shape[1])
                self.image_at = time.monotonic()

        def message_fresh(self, msg):
            stamp = msg.header.stamp

            age = (
                self.get_clock().now().nanoseconds / 1e9
                - (stamp.sec + stamp.nanosec / 1e9)
            )

            return (
                0 <= age
                <= self.p['detection_timeout_sec']
            )

        def ros_now(self):
            return self.get_clock().now().nanoseconds / 1e9

        @staticmethod
        def stamp(message):
            return message.header.stamp.sec + message.header.stamp.nanosec / 1e9

        def gps_fix(self, msg):
            stamp = self.stamp(msg)
            view = inspect_fix(msg, stamp, self.ros_now(), course['gps_timeout'],
                               min(1.0, course['max_gps_sigma']), self.fix_stamp)
            if view['reason'] == 'GPS_OUT_OF_ORDER':
                return
            self.fix = msg if view['nav_valid'] else None
            if self.fix is not None:
                self.fix_stamp = stamp

        def heading_fix(self, msg):
            stamp = self.stamp(msg)
            if (msg.header.frame_id == course['heading_frame'] and stamp > self.heading_stamp
                    and -.1 <= self.ros_now() - stamp <= course['heading_timeout']
                    and math.isfinite(msg.yawpitchroll.x)):
                self.heading = math.pi / 2 - math.radians(
                    msg.yawpitchroll.x + course['heading_offset_deg'])
                self.heading_stamp = stamp

        def lidar_scan(self, msg, slot):
            stamp = self.stamp(msg)
            old = self.scans[slot]
            if (msg.header.frame_id != course['scan_frame'] or not len(msg.ranges)
                    or not -.1 <= self.ros_now() - stamp <= course['scan_timeout']
                    or (old is not None and stamp <= old.stamp)
                    or not all(math.isfinite(v) for v in (msg.angle_min, msg.angle_increment,
                                                          msg.range_min, msg.range_max))
                    or msg.angle_increment == 0
                    or not 0 <= msg.range_min < msg.range_max):
                return
            self.scans[slot] = Scan(np.asarray(msg.ranges), msg.angle_min, msg.angle_increment,
                                    max(msg.range_min, route.avoidance_lidar_min_range),
                                    msg.range_max, stamp, math.radians(course['scan_yaw_deg']),
                                    course['scan_x'], course['scan_y'])

        def navigation_inputs(self):
            now = self.ros_now()
            fix = (self.fix if self.fix is not None
                   and -.1 <= now - self.fix_stamp <= course['gps_timeout'] else None)
            heading = (self.heading if -.1 <= now - self.heading_stamp
                       <= course['heading_timeout'] else None)
            scan, source = select_navigation_scan(*self.scans, now, course['scan_timeout'])
            return fix, heading, scan if source != 'unavailable' else None

        def select_gap(self, scan, desired):
            raw = self.scans[1]
            if (raw is scan or raw is None
                    or abs(scan.stamp - raw.stamp) > course['sync_tolerance']
                    or not -.1 <= self.ros_now() - raw.stamp <= course['scan_timeout']):
                raw = None
            return choose_gap(scan, desired, route, clearance, raw_scan=raw)

        def rear_clear(self, _scan):
            # Check the existing rear corridor against the unfiltered scan.
            raw = self.scans[1]
            return bool(raw is not None
                        and -.1 <= self.ros_now() - raw.stamp <= course['scan_timeout']
                        and motion_clear(raw, np.array([-1.0, 0.0]), clearance))

        def detections(self, msg):
            if self.message_fresh(msg):
                self.navigation.detect(msg.detections)
            candidates = []

            if self.width is not None and self.message_fresh(msg):
                for d in msg.detections:
                    if d.class_name != self.p['target_object']:
                        continue

                    center = d.bbox.center
                    x = float(
                        getattr(center, 'position', center).x
                    )
                    score = float(d.score)

                    if (
                        math.isfinite(score)
                        and math.isfinite(x)
                        and score >= self.p['confidence_threshold']
                        and 0 <= x < self.width
                    ):
                        candidates.append((score, x))

            if not candidates:
                self.target = None

                return

            _, x = max(candidates, key=lambda v: v[0])

            angle = pixel_angle(
                x,
                self.width,
                self.p['horizontal_fov_deg']
            )

            self.target = (
                x,
                angle,
            )
            self.target_at = time.monotonic()

        def feedback(self, msg):
            try:
                value = json.loads(msg.data)
                if not isinstance(value, dict):
                    return
            except (ValueError, TypeError):
                return

            self.telemetry = value
            self.telemetry_at = time.monotonic()

        def navigation_ready(self, now):
            return (
                self.target is not None
                and 0 <= now - self.target_at
                <= self.p['detection_timeout_sec']
                and 0 <= now - self.image_at
                <= self.p['image_timeout_sec']
            )

        def services_ready(self):
            return all(
                c.service_is_ready()
                for c in (
                    self.arm,
                    self.stop_client,
                    *self.pwm
                )
            )

        def start(self, request, response):
            now = time.monotonic()
            blocker = 'OUTPUT_DISABLED'

            if self.output:
                blocker = self.readiness.check(
                    now,
                    self.telemetry,
                    now - self.telemetry_at,
                    self.services_ready()
                )

                conflicts = conflicting_output_nodes(
                    self.get_node_names(),
                    self.get_name()
                )

                if conflicts:
                    blocker = (
                        'OUTPUT_CONFLICT: ' + ','.join(conflicts)
                    )

            response.success = bool(
                not blocker and self.output.start()
            )

            if response.success:
                self.auto_follow_allowed = True
                self.stop_latched = False

            response.message = blocker or (
                'STARTED'
                if response.success
                else 'ALREADY_ACTIVE_OR_STOPPING'
            )
            return response

        def stop(self, reason):
            # Latch explicit stops and output faults.
            self.stop_latched = True

            self.lease.publish(
                time.monotonic(),
                (1500, 1500),
                False
            )

            if self.output:
                self.output.request_stop(reason)

            self.reason = reason

        def external_stop(self, message):
            if (
                self.output
                and (
                    self.output.stopping
                    or self.output.stop_ack
                )
                and self.output.error
            ):
                self.stop_latched = True
                self.lease.publish(
                    time.monotonic(),
                    (1500, 1500),
                    False
                )
                return

            self.stop('EXTERNAL_STOP')

        def stop_service(self, request, response):
            self.stop('OPERATOR_STOP')
            self.pd.reset()

            response.success = True
            response.message = (
                'Stop requested; check status.stop_ack '
                'for MCU acknowledgment'
            )
            return response

        def control(self):
            now = time.monotonic()
            ready = self.navigation_ready(now)
            fix, heading, scan = self.navigation_inputs()
            executing = (not self.p['enable_output'] or bool(
                self.output and self.output.active and self.output.armed and not self.stop_latched))

            def drive(angle):
                base = command_pwm(self.p['thrust_percentage'])[0]
                return mix_pwm(base, self.pd.calculate(math.degrees(angle), now))

            try:
                self.reason, pwm = self.navigation.step(
                    now, executing, fix, heading, scan, gps_vector, wrap, self.select_gap,
                    self.rear_clear, drive, reverse_seconds, reverse_pwm)
            except (ValueError, RuntimeError, OverflowError) as error:
                self.get_logger().error('Vdock planning failed: ' + str(error))
                self.reason, pwm = 'GUIDANCE_INVALID', (1500, 1500)
            if self.reason != 'GPS_APPROACH':
                self.pd.reset()
            if self.stop_latched:
                pwm = (1500, 1500)
            self.lease.publish(time.monotonic(), pwm, not self.stop_latched)

            stopped = bool(
                self.output
                and not self.output.active
                and self.output.error
            )

            status = dict(
                state=(
                    'OUTPUT_STOPPED'
                    if stopped
                    else self.reason
                ),
                target_object=self.p['target_object'],
                detection_state=detection_state(ready),
                selected_class_id=(self.navigation.target_index if self.navigation.selected else None),
                goal_gps=(self.navigation.goal if self.navigation.selected else None),
                goal_distance_m=self.navigation.distance,
                gps_target_angle_deg=(math.degrees(self.navigation.goal_angle)
                                      if self.navigation.goal_angle is not None else None),
                selected_angle_deg=(math.degrees(self.navigation.selected_angle)
                                    if self.navigation.selected_angle is not None else None),
                goal_angle_weight=500.0,
                arrival_reverse_skipped=self.navigation.reverse_skipped,
                target_x=self.target[0] if ready else None,
                target_angle_deg=(
                    self.target[1] if ready else None
                ),
                kp=self.p['kp'],
                kd=self.p['kd'],
                intended_pwm=pwm,
                output_rate_hz=self.p['output_rate_hz'],
                output_enabled=self.output is not None,
                active=bool(
                    self.output and self.output.active
                ),
                auto_start_pending=bool(
                    self.auto_follow_allowed
                    and not self.stop_latched
                    and not self.output.active
                    if self.output else False
                ),
                stop_latched=self.stop_latched,
                armed=bool(
                    self.output and self.output.armed
                ),
                stop_ack=bool(
                    self.output and self.output.stop_ack
                ),
                acknowledged_pwm=(
                    self.output.last_ack_pwm
                    if self.output else None
                ),
                output_error=(
                    self.output.error if self.output else ''
                )
            )

            self.status_pub.publish(
                String(data=json.dumps(status))
            )

            if now - self.last_print_at >= 1.0:
                self.last_print_at = now
                self.get_logger().info(
                    f'detection_state={status["detection_state"]} '
                    f'target={status["target_object"]} '
                    f'x={status["target_x"]} '
                    f'state={status["state"]} distance_m={status["goal_distance_m"]} '
                    f'goal_angle_deg={status["gps_target_angle_deg"]} '
                    f'gap_angle_deg={status["selected_angle_deg"]} '
                    f'left={pwm[0]} right={pwm[1]} '
                    f'active={status["active"]} '
                    f'armed={status["armed"]} '
                    f'error={status["output_error"]}'
                )

        def output_tick(self):
            if not self.output:
                return

            now = time.monotonic()

            self.readiness.observe(
                now,
                self.telemetry,
                now - self.telemetry_at
            )

            if now - self.graph_at >= 1:
                self.conflicts = conflicting_output_nodes(
                    self.get_node_names(),
                    self.get_name()
                )
                self.graph_at = now

            pwm, error = self.lease.read(now)

            # Auto-start remains allowed after target loss.
            # Never auto-rearm after an explicit stop or fault.
            if (
                self.auto_follow_allowed
                and not self.stop_latched
                and not self.output.active
                and not self.output.stopping
                and not self.output.error
            ):
                blocker = self.readiness.check(
                    now,
                    self.telemetry,
                    now - self.telemetry_at,
                    self.services_ready()
                )

                if (
                    not blocker
                    and not self.conflicts
                    and not error
                ):
                    self.output.start()

            if (self.stop_latched or any(value is None for value in self.navigation_inputs())
                    or (self.reason == 'ARRIVAL_REVERSE'
                        and now - self.navigation.reverse_started >= reverse_seconds)):
                pwm = (1500, 1500)

            if (
                error or self.conflicts
            ) and self.output.active:
                self.stop(
                    error or 'DATA_OR_OUTPUT_CONFLICT'
                )

            self.output.tick(
                now,
                pwm,
                self.telemetry,
                now - self.telemetry_at
            )

            if self.output.error != self.logged_output_error:
                self.logged_output_error = self.output.error

                if self.output.error:
                    self.get_logger().error(
                        'OUTPUT_STOPPED: ' + self.output.error
                    )

    from rclpy.signals import SignalHandlerOptions

    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO
    )

    def interrupt(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)

    node = Follower()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if rclpy.ok() and node.output:
            node.stop('SHUTDOWN')

            deadline = time.monotonic() + 2.0

            while (
                not node.output.stop_ack
                and time.monotonic() < deadline
            ):
                node.output_tick()
                rclpy.spin_once(
                    node,
                    timeout_sec=0.02
                )

            if not node.output.stop_ack:
                node.get_logger().error(
                    'STOP_UNCONFIRMED'
                )

        node.destroy_node()

        if node.output_lock:
            node.output_lock.close()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    if '--vdock-setup' in sys.argv[1:]:
        vdock_setup_main([a for a in sys.argv[1:] if a != '--vdock-setup'])
    else:
        main()
