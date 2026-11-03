"""Standalone learned-shape following with unchanged Jetson actuator functions."""
from dataclasses import asdict, fields
import json
import math
import signal
import time
import numpy as np

from .core import Scan
from .follow import FollowConfig, Follower, fusion_rows, label_name, native_pwm
from .output import PairedOutput, telemetry_ready


def make_node():
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rcl_interfaces.msg import ParameterDescriptor
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Float32MultiArray, String, Empty
    from std_srvs.srv import Trigger, SetBool
    from yolo_msgs.msg import DetectionArray
    from mechaship_interfaces.srv import ThrottlePulseWidth
    from isv_water_trials.tank_core import guidance
    from isv_water_trials.actuator_config import command_pwm

    class ShapeFollower(Node):
        def __init__(self):
            super().__init__('shape_follow')
            defaults = asdict(FollowConfig())
            defaults['class_names'] = list(defaults['class_names'])
            defaults.update(enable_output=False, fusion_topic='/fusion/roi_center',
                            yolo_topic='/yolo/tracking', scan_topic='/scan', scan_frame='velodyne',
                            camera_frame='camera_frame', actuator_status_topic='/actuators/status',
                            score_threshold=.65, scan_yaw_deg=0., scan_x=0., scan_y=0.,
                            control_rate=10.)
            for name, value in defaults.items():
                self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
            self.p = {name: self.get_parameter(name).value for name in defaults}
            self.cfg = FollowConfig(**{item.name: self.p[item.name] for item in fields(FollowConfig)})
            if not 0 < self.p['score_threshold'] <= 1:
                raise ValueError('score_threshold must be in (0,1]')
            if not 1 <= self.p['control_rate'] <= 30:
                raise ValueError('control_rate must be 1..30 Hz')
            if not all(math.isfinite(self.p[key]) for key in ('scan_yaw_deg', 'scan_x', 'scan_y')):
                raise ValueError('Invalid scan transform')
            if self.p['enable_output'] and self.get_parameter('use_sim_time').value:
                raise ValueError('Hardware output is disabled with simulated ROS time')
            self.native_command_pwm = command_pwm
            if native_pwm((0., 0.), command_pwm) != (1500, 1500):
                raise ValueError('Installed native calibration does not match the 1500us bridge protocol')
            self.controller = Follower(self.cfg, guidance)
            self.scan = None
            self.yolo, self.yolo_stamp = [], -math.inf
            self.telemetry, self.telemetry_at = {}, -math.inf
            self.last_reason, self.previous_now = 'WAIT_DATA', -math.inf
            self.output, self.output_lock = None, None
            self.status_pub = self.create_publisher(String, '/shape_follow/status', 10)
            self.command_pub = self.create_publisher(String, '/shape_follow/command', 10)
            self.create_subscription(Float32MultiArray, self.p['fusion_topic'], self.on_fusion, qos_profile_sensor_data)
            self.create_subscription(DetectionArray, self.p['yolo_topic'], self.on_yolo, qos_profile_sensor_data)
            self.create_subscription(LaserScan, self.p['scan_topic'], self.on_scan, qos_profile_sensor_data)
            self.create_subscription(String, self.p['actuator_status_topic'], self.on_telemetry, qos_profile_sensor_data)
            self.create_subscription(Empty, '/water_trial/stop', self.external_stop, 10)
            self.create_service(Trigger, '/shape_follow/start', self.start)
            self.create_service(Trigger, '/shape_follow/stop', self.stop)
            self.create_service(Trigger, '/shape_follow/reset', self.reset)
            if self.p['enable_output']:
                import fcntl
                import os
                self.output_lock = open('/tmp/kabot_shape_output_'+os.environ.get('ROS_DOMAIN_ID', '0')+'.lock', 'a')
                fcntl.flock(self.output_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                self.arm_client = self.create_client(SetBool, '/actuators/arm')
                self.stop_client = self.create_client(Trigger, '/actuators/stop')
                self.pwm_clients = [self.create_client(ThrottlePulseWidth,
                    '/actuators/throttle_'+side+'/set_pulse_width') for side in ('left', 'right')]
                self.output = PairedOutput(
                    lambda value: self.arm_client.call_async(SetBool.Request(data=value)),
                    lambda left, right: [client.call_async(ThrottlePulseWidth.Request(pulse_width=pulse))
                                         for client, pulse in zip(self.pwm_clients, (left, right))],
                    lambda: self.stop_client.call_async(Trigger.Request()),
                    lambda: all(client.service_is_ready() for client in
                                (self.arm_client, self.stop_client, *self.pwm_clients)),
                    stop_ready=lambda: self.stop_client.service_is_ready())
            self.timer = self.create_timer(1/self.p['control_rate'], self.update)
            self.get_logger().info('Target '+self.cfg.target_label+'; class_id='+str(self.cfg.target_class_id)
                                   +'; native isv_water_trials guidance + command_pwm; '
                                   +('OUTPUT ENABLED; start service required' if self.output else 'PREVIEW'))

        def now(self):
            return self.get_clock().now().nanoseconds/1e9

        @staticmethod
        def stamp(header):
            return header.stamp.sec+header.stamp.nanosec/1e9

        def on_yolo(self, message):
            stamp = self.stamp(message.header)
            if (message.header.frame_id != self.p['camera_frame'] or stamp <= self.yolo_stamp
                    or not -.1 <= self.now()-stamp <= self.cfg.target_timeout):
                return
            accepted = []
            for detection in message.detections:
                if (label_name(detection.class_name) != self.cfg.target_label
                        or detection.class_id != self.cfg.target_class_id):
                    continue
                box = detection.bbox
                values = (detection.score, box.center.position.x, box.center.position.y, box.size.x, box.size.y)
                if (all(math.isfinite(value) for value in values)
                        and self.p['score_threshold'] <= detection.score <= 1
                        and box.size.x > 0 and box.size.y > 0):
                    accepted.append(values)
            self.yolo, self.yolo_stamp = accepted, stamp

        def on_fusion(self, message):
            now = self.now()
            if not -.1 <= now-self.yolo_stamp <= self.cfg.target_timeout:
                self.controller.observe([], now)
                return
            try:
                dimensions = message.layout.dim
                if dimensions and len(dimensions) != 2:
                    raise ValueError('Fusion layout must contain rows and cols')
                observations = fusion_rows(message.data, self.yolo_stamp,
                    rows=int(dimensions[0].size) if dimensions else None,
                    cols=int(dimensions[1].size) if dimensions else 8,
                    offset=int(message.layout.data_offset))
                # Fusion has no confidence field: verify each row against a fresh YOLO bbox.
                candidates = []
                for observation in observations:
                    if observation.class_id != self.cfg.target_class_id:
                        continue
                    matches = [box for box in self.yolo if math.hypot(observation.u-box[1], observation.v-box[2])
                               <= max(8., .35*min(box[3], box[4]))]
                    if len(matches) == 1:
                        candidates.append(observation)
                self.controller.observe(candidates, now)
            except (ValueError, TypeError, IndexError, OverflowError):
                self.controller.observe([], now)

        def on_scan(self, message):
            stamp = self.stamp(message.header)
            if (message.header.frame_id != self.p['scan_frame'] or not message.ranges
                    or not -.1 <= self.now()-stamp <= self.cfg.scan_timeout
                    or (self.scan is not None and stamp <= self.scan.stamp)
                    or not math.isfinite(message.angle_min) or not math.isfinite(message.angle_increment)
                    or message.angle_increment == 0 or not math.isfinite(message.range_min)
                    or not math.isfinite(message.range_max) or not 0 <= message.range_min < message.range_max):
                return
            self.scan = Scan(np.asarray(message.ranges), message.angle_min, message.angle_increment,
                             max(.2, message.range_min), message.range_max, stamp,
                             math.radians(self.p['scan_yaw_deg']), self.p['scan_x'], self.p['scan_y'])

        def on_telemetry(self, message):
            try:
                self.telemetry = json.loads(message.data)
                self.telemetry_at = time.monotonic()
            except (ValueError, TypeError):
                self.telemetry = {}

        def update(self):
            now = self.now()
            if now < self.previous_now-.1:
                self.controller.reset()
                self.scan, self.yolo_stamp = None, -math.inf
                if self.output:
                    self.output.request_stop('ROS time moved backwards')
            self.previous_now = now
            try:
                reason, percentages = self.controller.step(now, self.scan)
                intended = native_pwm(percentages, self.native_command_pwm)
            except (ValueError, RuntimeError, OverflowError):
                reason, percentages, intended = 'GUIDANCE_INVALID', (0., 0.), (1500, 1500)
            self.last_reason = reason
            if self.output:
                if reason not in ('FOLLOW_TARGET', 'ALIGN_TARGET', 'HOLD_DISTANCE') and self.output.active:
                    self.output.request_stop(reason)
                self.output.tick(time.monotonic(), intended, self.telemetry, time.monotonic()-self.telemetry_at)
            target = self.controller.target
            report = dict(stamp=now, state=reason, target_label=self.cfg.target_label,
                          target_class_id=self.cfg.target_class_id, stable_frames=self.controller.count,
                          target=asdict(target) if target else None, intended_pwm=list(intended),
                          left_percent=percentages[0], right_percent=percentages[1],
                          native_output='isv_water_trials.actuator_config.command_pwm',
                          output=dict(enabled=self.output is not None,
                                      active=bool(self.output and self.output.active),
                                      armed=bool(self.output and self.output.armed),
                                      stop_ack=bool(self.output and self.output.stop_ack),
                                      error=self.output.error if self.output else ''))
            text = String(data=json.dumps(report, allow_nan=False))
            self.status_pub.publish(text)
            self.command_pub.publish(text)

        def start(self, request, response):
            if self.output is None:
                response.success, response.message = False, 'PREVIEW: relaunch with enable_output:=true'
            elif (self.last_reason not in ('FOLLOW_TARGET', 'ALIGN_TARGET', 'HOLD_DISTANCE')
                  or not telemetry_ready(self.telemetry)
                  or time.monotonic()-self.telemetry_at > .5):
                response.success, response.message = False, 'Wait for stable target, scan and fresh AUTO telemetry'
            else:
                response.success = self.output.start()
                response.message = 'Start requested' if response.success else 'Stop/drain still pending'
            return response

        def stop(self, request, response):
            if self.output:
                self.output.request_stop('operator stop')
            response.success, response.message = True, 'Stop requested; inspect output.stop_ack'
            return response

        def reset(self, request, response):
            if self.output and (self.output.active or self.output.pending or self.output.stopping):
                response.success, response.message = False, 'Stop and wait for acknowledgment first'
            else:
                self.controller.reset()
                response.success, response.message = True, 'Target observations cleared'
            return response

        def external_stop(self, message):
            if self.output and (self.output.active or self.output.armed):
                self.output.request_stop('external /water_trial/stop')

    return ShapeFollower()


def main(args=None):
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    exiting = False

    def stop_signal(*_):
        nonlocal exiting
        exiting = True

    signals = [signal.SIGINT, signal.SIGTERM]
    if hasattr(signal, 'SIGHUP'):
        signals.append(signal.SIGHUP)
    previous = {item: signal.signal(item, stop_signal) for item in signals}
    node = None
    try:
        node = make_node()
        while rclpy.ok() and not exiting:
            rclpy.spin_once(node, timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.timer.cancel()
            if node.output and node.output.owned:
                node.output.request_stop('process exit')
                until = time.monotonic()+3
                while rclpy.ok() and time.monotonic() < until and not node.output.stop_ack:
                    node.output.tick(time.monotonic(), (1500, 1500), {}, math.inf)
                    rclpy.spin_once(node, timeout_sec=.02)
                if not node.output.stop_ack:
                    node.get_logger().error('STOP_UNCONFIRMED: MCU acknowledgment missing')
            node.destroy_node()
            if node.output_lock:
                node.output_lock.close()
        if rclpy.ok():
            rclpy.shutdown()
        for item, handler in previous.items():
            signal.signal(item, handler)


if __name__ == '__main__':
    main()
