
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


def main(args=None):
    load_ros_environment()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
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

    class Follower(Node):
        def __init__(self):
            super().__init__('object_pixel_follower')

            defaults = dict(
                target_object='green circle',
                horizontal_fov_deg=90.0,
                confidence_threshold=0.50,
                kp=8.0,
                kd=0.0,
                turn_limit_us=400.0,
                thrust_percentage=20.0,
                detection_timeout_sec=0.5,
                image_timeout_sec=1.0,
                enable_output=True,
                auto_follow=True,
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
                Image,
                '/camera/image_raw',
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
            if msg.width > 0 and self.message_fresh(msg):
                self.width = float(msg.width)
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

        def detections(self, msg):
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
                self.pd.reset()

                # Neutral only. Do not cancel auto-follow.
                self.lease.publish(
                    time.monotonic(),
                    (1500, 1500),
                    True
                )

                self.reason = 'WAIT_TARGET_NEUTRAL'
                return

            now = time.monotonic()
            _, x = max(candidates, key=lambda v: v[0])

            angle = pixel_angle(
                x,
                self.width,
                self.p['horizontal_fov_deg']
            )

            self.target = (
                x,
                angle,
                self.pd.calculate(angle, now)
            )
            self.target_at = now

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
            pwm = (1500, 1500)

            if ready and not self.stop_latched:
                base = command_pwm(
                    self.p['thrust_percentage']
                )[0]

                pwm = follow_command(
                    True,
                    base,
                    self.target[2]
                )

                self.reason = 'TRACK_TARGET'
                self.lease.publish(now, pwm, True)

            else:
                self.pd.reset()
                self.reason = 'WAIT_TARGET_NEUTRAL'
                self.lease.publish(
                    now,
                    (1500, 1500),
                    True
                )

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
                    f'angle_deg={status["target_angle_deg"]} '
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

            # Neutral when no fresh target is available.
            if (
                not self.navigation_ready(now)
                or self.stop_latched
            ):
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
    main()
