"""ROS2 adapter for the kabot2026 sensor, tank SLAM and actuator interfaces."""
from collections import deque
from dataclasses import asdict, fields
import json
import math
from pathlib import Path
import signal
import time
import numpy as np

from .core import Detection, Mission, MissionConfig, Pose, Scan, fuse_detection, thrust_to_pwm
from .output import PairedOutput, OutputReadiness, telemetry_ready
from .output_runtime import acquire_output_lock
from .planner import Grid


def quaternion_yaw(q):
    values = (q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in values) or not .99 < sum(v*v for v in values) < 1.01:
        raise ValueError('Invalid orientation quaternion')
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.executors import ExternalShutdownException
    from sensor_msgs.msg import LaserScan, Image, CameraInfo
    from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
    from nav_msgs.msg import OccupancyGrid, Path as NavPath
    from std_msgs.msg import String, Empty
    from std_srvs.srv import Trigger, SetBool
    from mechaship_interfaces.srv import ThrottlePulseWidth
    from cv_bridge import CvBridge
    import cv2
    from .perception import Camera, detect_shapes, draw_detections

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    exiting = False
    def exit_signal(*_):
        nonlocal exiting
        exiting = True
    previous_signals = {sig: signal.signal(sig, exit_signal) for sig in (signal.SIGINT, signal.SIGTERM)}
    if hasattr(signal, 'SIGHUP'):
        previous_signals[signal.SIGHUP] = signal.signal(signal.SIGHUP, exit_signal)

    class ShapeNavigation(Node):
        def __init__(self):
            super().__init__('shape_navigation')
            defaults = dict(
                enable_output=False, detector_source='contour', target_color='red',
                score_threshold=.75, min_shape_area=250.,
                image_topic='/camera/image_raw', camera_info_topic='/camera/camera_info',
                scan_topic='/scan', pose_topic='/tank/pose', map_topic='/map',
                session_topic='/tank/session', yolo_topic='/yolo/detections',
                actuator_status_topic='/actuators/status', map_frame='tank_map', scan_frame='velodyne',
                camera_frame='camera_frame', require_session=True, session_timeout=3., map_timeout=5.,
                camera_fx=0., camera_cx=0., camera_width=640, camera_height=480,
                camera_yaw_deg=0., camera_x=0., camera_y=0.,
                scan_yaw_deg=0., scan_x=0., scan_y=0., pose_yaw_offset_deg=0.,
                sync_tolerance=.20, fusion_range_spread=.35,
                max_target_range=20., min_target_returns=3,
                output_limit=.25, log_directory='', control_rate=10.)
            defaults.update(asdict(MissionConfig()))
            for key, value in defaults.items():
                self.declare_parameter(key, value)
            self.p = {key: self.get_parameter(key).value for key in defaults}
            self.cfg = MissionConfig(**{f.name: self.p[f.name] for f in fields(MissionConfig)})
            self._validate()
            self.mission = Mission(self.cfg)
            self.camera = Camera(self.p['camera_fx'], self.p['camera_cx'],
                                 self.p['camera_width'], self.p['camera_height'])
            self.bridge = CvBridge()
            self.scans, self.poses = deque(maxlen=40), deque(maxlen=40)
            self.grid, self.map_at = None, -math.inf
            self.session_id, self.session_at, self.session_ready = None, -math.inf, False
            self.telemetry, self.telemetry_at = {}, -math.inf
            self.output_readiness = OutputReadiness()
            self.last_detections = []
            self.last_reason = 'WAIT_DATA'
            self.last_report = -math.inf
            self.last_image = -math.inf
            self.vision_reason = 'WAIT_CAMERA'
            self.timer_enabled = True
            self.log_file = None
            if self.p['log_directory']:
                directory = Path(self.p['log_directory']).expanduser()
                directory.mkdir(parents=True, exist_ok=True)
                self.log_file = (directory/('shape_'+time.strftime('%Y%m%d_%H%M%S')+'.jsonl')).open('x', encoding='utf-8')
            self.status_pub = self.create_publisher(String, '/shape_navigation/status', 10)
            self.command_pub = self.create_publisher(String, '/shape_navigation/command', 10)
            self.detections_pub = self.create_publisher(String, '/shape_navigation/detections', 10)
            self.path_pub = self.create_publisher(NavPath, '/shape_navigation/path', 1)
            self.goal_pub = self.create_publisher(PoseStamped, '/shape_navigation/goal', 1)
            self.image_pub = self.create_publisher(Image, '/shape_navigation/debug_image', 1)
            self.create_subscription(LaserScan, self.p['scan_topic'], self.on_scan, qos_profile_sensor_data)
            self.create_subscription(PoseWithCovarianceStamped, self.p['pose_topic'], self.on_pose, qos_profile_sensor_data)
            map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            self.config_pub = self.create_publisher(String, '/shape_navigation/config', map_qos)
            configuration = json.dumps(dict(kind='configuration', parameters=self.p,
                                            upstream_commit='6797c384d4ab1ea99d9f23dc918a448d25d0c89e'))
            self.config_pub.publish(String(data=configuration))
            if self.log_file:
                self.log_file.write(configuration+'\n'); self.log_file.flush()
            self.create_subscription(OccupancyGrid, self.p['map_topic'], self.on_map, map_qos)
            self.create_subscription(String, self.p['session_topic'], self.on_session, 10)
            self.create_subscription(String, self.p['actuator_status_topic'], self.on_actuator, qos_profile_sensor_data)
            self.create_subscription(CameraInfo, self.p['camera_info_topic'], self.on_camera, qos_profile_sensor_data)
            if self.p['detector_source'] == 'contour':
                self.create_subscription(Image, self.p['image_topic'], self.on_image, qos_profile_sensor_data)
            else:
                from yolo_msgs.msg import DetectionArray
                self.create_subscription(DetectionArray, self.p['yolo_topic'], self.on_yolo, qos_profile_sensor_data)
            self.create_service(Trigger, '/shape_navigation/start', self.start)
            self.create_service(Trigger, '/shape_navigation/stop', self.stop)
            self.create_service(Trigger, '/shape_navigation/reset', self.reset)
            self.create_subscription(Empty, '/water_trial/stop', self.external_stop, 10)
            self.output = None
            if self.p['enable_output']:
                # Avoid two command owners even if a second process is started.
                self.output_lock = acquire_output_lock()
                self.arm_client = self.create_client(SetBool, '/actuators/arm')
                self.stop_client = self.create_client(Trigger, '/actuators/stop')
                self.pwm_clients = [self.create_client(ThrottlePulseWidth,
                    '/actuators/throttle_'+side+'/set_pulse_width') for side in ('left', 'right')]
                self.output = PairedOutput(
                    lambda value: self.arm_client.call_async(SetBool.Request(data=value)),
                    lambda left, right: [client.call_async(ThrottlePulseWidth.Request(pulse_width=pulse))
                                         for client, pulse in zip(self.pwm_clients, (left, right))],
                    lambda: self.stop_client.call_async(Trigger.Request()),
                    lambda: all(c.service_is_ready() for c in [self.arm_client, self.stop_client, *self.pwm_clients]),
                    stop_ready=lambda: self.stop_client.service_is_ready())
            self.timer = self.create_timer(1/self.p['control_rate'], self.update)
            self.get_logger().info('Shape navigation ready: '+('hardware output enabled; call /shape_navigation/start' if self.output else 'PREVIEW'))

        def _validate(self):
            p = self.p
            if p['enable_output'] and self.get_parameter('use_sim_time').value:
                raise ValueError('Hardware output is disabled when use_sim_time is true')
            if p['detector_source'] not in ('contour', 'yolo'):
                raise ValueError('detector_source must be contour or yolo')
            if p['target_color'] not in ('red', 'green', 'blue', 'yellow', 'white', 'black'):
                raise ValueError('Invalid target_color')
            if not 0 < p['score_threshold'] <= 1 or not 0 < p['output_limit'] <= .3:
                raise ValueError('Invalid score/output limit')
            if not 5 <= p['control_rate'] <= 30:
                raise ValueError('control_rate must be 5..30 Hz for the MCU watchdog')
            for key in ('control_rate', 'map_timeout', 'session_timeout', 'sync_tolerance',
                        'max_target_range', 'fusion_range_spread', 'min_shape_area', 'min_target_returns'):
                if not math.isfinite(p[key]) or p[key] <= 0:
                    raise ValueError('Invalid '+key)
            for key in ('camera_fx', 'camera_cx', 'camera_yaw_deg', 'camera_x', 'camera_y',
                        'scan_yaw_deg', 'scan_x', 'scan_y', 'pose_yaw_offset_deg'):
                if not math.isfinite(p[key]):
                    raise ValueError('Invalid '+key)
            if p['camera_fx'] < 0:
                raise ValueError('camera_fx cannot be negative')
            if not p['target_label'] or p['target_label'] != p['target_label'].strip().lower():
                raise ValueError('target_label must be a nonempty lowercase class name')

        def now(self):
            return self.get_clock().now().nanoseconds/1e9

        def stamp(self, header):
            return header.stamp.sec+header.stamp.nanosec/1e9

        def fresh(self, stamp, limit=1.):
            return math.isfinite(stamp) and -.1 <= self.now()-stamp <= limit

        def on_scan(self, msg):
            stamp = self.stamp(msg.header)
            if (msg.header.frame_id != self.p['scan_frame'] or not self.fresh(stamp)
                    or not msg.ranges or not math.isfinite(msg.angle_min)
                    or not math.isfinite(msg.angle_increment) or msg.angle_increment == 0
                    or not math.isfinite(msg.range_min) or not math.isfinite(msg.range_max)
                    or not 0 <= msg.range_min < msg.range_max):
                return
            if self.scans and stamp <= self.scans[-1].stamp:
                return
            self.scans.append(Scan(np.array(msg.ranges), msg.angle_min, msg.angle_increment,
                                   max(.2, msg.range_min), msg.range_max, stamp,
                                   math.radians(self.p['scan_yaw_deg']), self.p['scan_x'], self.p['scan_y']))

        def on_pose(self, msg):
            stamp = self.stamp(msg.header)
            p = msg.pose.pose
            if (msg.header.frame_id != self.p['map_frame'] or not self.fresh(stamp)
                    or not all(math.isfinite(v) for v in (p.position.x, p.position.y))):
                return
            try:
                angle = quaternion_yaw(p.orientation)+math.radians(self.p['pose_yaw_offset_deg'])
            except ValueError:
                return
            if self.poses and stamp <= self.poses[-1].stamp:
                return
            self.poses.append(Pose(p.position.x, p.position.y, angle, stamp))

        def on_map(self, msg):
            if (msg.header.frame_id != self.p['map_frame'] or msg.info.width*msg.info.height != len(msg.data)
                    or not msg.data):
                return
            try:
                p = msg.info.origin
                self.grid = Grid(np.asarray(msg.data).reshape(msg.info.height, msg.info.width),
                                 msg.info.resolution, (p.position.x, p.position.y, quaternion_yaw(p.orientation)),
                                 self.cfg.footprint_radius+self.cfg.clearance)
                self.map_at = time.monotonic()
            except (ValueError, OverflowError) as exc:
                self.grid = None
                self.get_logger().warning('Map rejected: '+str(exc))

        def on_session(self, msg):
            try:
                payload = json.loads(msg.data)
                session_id = payload['session_id']
                if not isinstance(session_id, str) or not session_id:
                    raise ValueError('session_id missing')
                if self.session_id is not None and session_id != self.session_id:
                    self.mission.reset()
                    self.grid, self.map_at = None, -math.inf
                    self.scans.clear(); self.poses.clear()
                    self.last_image = -math.inf; self.last_detections = []
                    if self.output:
                        self.output.request_stop('SLAM session changed; reset target')
                self.session_id = session_id
                self.session_ready = payload.get('localization_ready') is True
                self.session_at = time.monotonic()
            except (ValueError, KeyError, TypeError):
                self.session_ready = False

        def on_actuator(self, msg):
            try:
                payload = json.loads(msg.data)
                if not isinstance(payload, dict):
                    raise ValueError('MCU telemetry must be an object')
                now = time.monotonic()
                if now-self.telemetry_at>.5:
                    self.output_readiness.observe(now,{},math.inf)
                self.output_readiness.observe(now,payload,0.)
                self.telemetry, self.telemetry_at = payload, now
            except (ValueError, TypeError):
                self.telemetry = {}
                self.output_readiness.observe(time.monotonic(),{},math.inf)

        def on_camera(self, msg):
            if msg.header.frame_id != self.p['camera_frame']:
                return
            try:
                self.camera.configure(msg.width, msg.height, msg.k, msg.d, msg.distortion_model)
                self.vision_reason = 'CALIBRATED'
            except (ValueError, TypeError) as exc:
                self.vision_reason = 'WAIT_CAMERA_CALIBRATION: '+str(exc)

        def on_image(self, msg):
            stamp = self.stamp(msg.header)
            if (msg.header.frame_id != self.p['camera_frame'] or not self.fresh(stamp)
                    or stamp <= self.last_image):
                return
            self.last_image = stamp
            try:
                frame = self.camera.rectify(self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'))
                detections = detect_shapes(frame, stamp, self.p['target_color'], self.p['min_shape_area'])
                self.vision_reason = 'OK'
                self.accept_detections(detections, stamp)
                if self.image_pub.get_subscription_count():
                    debug = self.bridge.cv2_to_imgmsg(draw_detections(frame, detections, self.cfg.target_label), 'bgr8')
                    debug.header = msg.header
                    self.image_pub.publish(debug)
            except (ValueError, RuntimeError, cv2.error) as exc:
                self.vision_reason = str(exc)

        def on_yolo(self, msg):
            stamp = self.stamp(msg.header)
            if (msg.header.frame_id != self.p['camera_frame'] or not self.fresh(stamp)
                    or stamp <= self.last_image):
                return
            self.last_image = stamp
            if not self.camera.valid(self.p['camera_width'], self.p['camera_height']):
                self.vision_reason = 'WAIT_CAMERA_CALIBRATION'
                return
            try:
                detections = [self.camera.rectify_detection(Detection(
                    d.class_name.strip().lower(), d.score, d.bbox.center.position.x, d.bbox.center.position.y,
                    d.bbox.size.x, d.bbox.size.y, stamp)) for d in msg.detections]
            except (ValueError, cv2.error) as exc:
                self.vision_reason = str(exc)
                return
            self.vision_reason = 'OK'
            self.accept_detections(detections, stamp)

        def accept_detections(self, detections, stamp):
            detections = [d for d in detections if isinstance(d.label, str) and d.label
                          and all(math.isfinite(v) for v in (d.score, d.cx, d.cy, d.width, d.height, d.stamp))
                          and 0 <= d.score <= 1 and d.width > 0 and d.height > 0]
            self.last_detections = [asdict(d) for d in detections]
            self.detections_pub.publish(String(data=json.dumps({'stamp': stamp, 'detections': self.last_detections})))
            if not self.scans or not self.poses:
                return
            scan = min(self.scans, key=lambda s: abs(s.stamp-stamp))
            pose = min(self.poses, key=lambda p: abs(p.stamp-stamp))
            candidates = []
            for detection in detections:
                if detection.label != self.cfg.target_label or not self.p['score_threshold'] <= detection.score <= 1:
                    continue
                point = fuse_detection(detection, scan, pose, self.camera.fx, self.camera.cx,
                    camera_yaw=math.radians(self.p['camera_yaw_deg']), camera_x=self.p['camera_x'], camera_y=self.p['camera_y'],
                    scan_yaw=scan.yaw, scan_x=scan.x, scan_y=scan.y, sync_tolerance=self.p['sync_tolerance'],
                    max_range=self.p['max_target_range'], min_returns=self.p['min_target_returns'],
                    range_spread=self.p['fusion_range_spread'])
                if point is not None:
                    candidates.append(point)
            if self.mission.target is not None and (self.mission.count >= self.cfg.stable_observations
                                                    or stamp-self.mission.last_seen <= self.cfg.target_timeout):
                candidates = [p for p in candidates if np.linalg.norm(p-self.mission.target) <= self.cfg.association_distance]
            # The same class on two bays is ambiguous: never guess by bbox size.
            if len(candidates) == 1:
                self.mission.observe(candidates[0], pose, stamp)

        def local_ready(self):
            return (not self.p['require_session'] or (self.session_ready and
                    time.monotonic()-self.session_at <= self.p['session_timeout']))

        def start(self, request, response):
            blocker = (self.output_readiness.check(time.monotonic(),self.telemetry,
                time.monotonic()-self.telemetry_at,self.output.ready()) if self.output else '')
            if self.output is None:
                response.success = False
                response.message = 'Preview only. Relaunch with enable_output:=true to use actuators.'
            elif (not self.local_ready() or self.last_reason not in ('SEARCH_SHAPE', 'APPROACH_SHAPE', 'ENTER_REGION', 'ALIGN_PATH')
                  or not telemetry_ready(self.telemetry) or time.monotonic()-self.telemetry_at > .5):
                response.success = False
                response.message = 'Wait for fresh scan, pose, map, localization and AUTO telemetry; '+self.last_reason
            elif self.mission.completed:
                response.success = False
                response.message = 'Mission complete; reset before starting again'
            elif blocker:
                response.success = False
                response.message = blocker
            else:
                response.success = self.output.start()
                response.message = 'Start requested; awaiting arm acknowledgment' if response.success else 'Stop/drain still pending'
            return response

        def stop(self, request, response):
            if self.output:
                self.output.request_stop()
            response.success = True
            response.message = 'Stop requested; inspect status output.stop_ack for MCU acknowledgment'
            return response

        def external_stop(self, msg):
            # The upstream stop service itself emits this topic. Ignore that
            # echo after stopping so it cannot trigger an endless stop loop.
            if self.output and (self.output.active or self.output.armed):
                self.output.request_stop('external /water_trial/stop')

        def reset(self, request, response):
            if self.output and (self.output.active or self.output.pending or self.output.stopping):
                response.success = False
                response.message = 'Stop and wait for acknowledgment before reset'
            else:
                self.mission.reset()
                response.success = True
                response.message = 'Target and mission reset'
            return response

        def update(self):
            if not self.timer_enabled:
                return
            now, monotonic = self.now(), time.monotonic()
            # A backward ROS-time jump (e.g. bag loop) invalidates all old goals.
            if hasattr(self, 'previous_now') and now < self.previous_now-.1:
                self.mission.reset(); self.scans.clear(); self.poses.clear()
                self.grid = None
                self.last_image = -math.inf; self.last_detections = []
                if self.output:
                    self.output.request_stop('ROS time moved backwards')
            self.previous_now = now
            pose = self.poses[-1] if self.poses else None
            scan = self.scans[-1] if self.scans else None
            grid = self.grid if monotonic-self.map_at <= self.p['map_timeout'] else None
            reason, command, path = self.mission.step(now, pose, scan, grid, localization_ready=self.local_ready())
            self.last_reason = reason
            intended_pwm = thrust_to_pwm(*command, self.p['output_limit'])
            if self.output:
                blocker = self.output_readiness.check(monotonic,self.telemetry,
                    monotonic-self.telemetry_at,self.output.ready())
                if blocker and self.output.active:
                    self.output.request_stop(blocker)
                if reason in ('WAIT_LOCALIZATION', 'WAIT_POSE', 'WAIT_SCAN', 'WAIT_MAP', 'SCAN_UNOBSERVED', 'REGION_REACHED'):
                    if self.output.active:
                        self.output.request_stop(reason)
                self.output.tick(monotonic, intended_pwm, self.telemetry, monotonic-self.telemetry_at)
            output = dict(enabled=self.output is not None, active=bool(self.output and self.output.active),
                          armed=bool(self.output and self.output.armed),
                          stop_ack=bool(self.output and self.output.stop_ack), error=self.output.error if self.output else '')
            goal = self.mission.goal()
            report = dict(stamp=now, state=reason, phase=self.mission.phase, output=output,
                          vision=self.vision_reason, target_label=self.cfg.target_label,
                          target=self.mission.target.tolist() if self.mission.target is not None else None,
                          goal=goal.tolist() if goal is not None else None, frame=self.p['map_frame'],
                          pose=asdict(pose) if pose else None, detections=self.last_detections,
                          intended_pwm=list(intended_pwm), forward=command[0], turn=command[1],
                          session=self.session_id, localization_ready=self.local_ready(),
                          target_age=now-self.mission.last_seen if math.isfinite(self.mission.last_seen) else None)
            self.command_pub.publish(String(data=json.dumps({k: report[k] for k in ('stamp', 'state', 'output', 'intended_pwm', 'forward', 'turn')})))
            if monotonic-self.last_report >= .2:
                data = json.dumps(report, allow_nan=False)
                self.status_pub.publish(String(data=data))
                if self.log_file:
                    self.log_file.write(data+'\n'); self.log_file.flush()
                self.last_report = monotonic
            header = PoseStamped().header
            header.frame_id = self.p['map_frame']; header.stamp = self.get_clock().now().to_msg()
            route = NavPath(); route.header = header
            for x, y in path:
                point = PoseStamped(); point.header = header
                point.pose.position.x = float(x); point.pose.position.y = float(y)
                point.pose.orientation.w = 1.
                route.poses.append(point)
            self.path_pub.publish(route)
            if goal is not None:
                target = PoseStamped(); target.header = header
                target.pose.position.x, target.pose.position.y = map(float, goal)
                angle = math.atan2(self.mission.direction[1], self.mission.direction[0])
                target.pose.orientation.z = math.sin(angle/2); target.pose.orientation.w = math.cos(angle/2)
                self.goal_pub.publish(target)

    node = None
    try:
        node = ShapeNavigation()
        while rclpy.ok() and not exiting:
            rclpy.spin_once(node, timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.timer_enabled = False
            node.timer.cancel()
            if node.output and node.output.owned:
                node.output.request_stop('process exit')
                until = time.monotonic()+3.
                while rclpy.ok() and time.monotonic() < until and not node.output.stop_ack:
                    node.output.tick(time.monotonic(), (1500, 1500), {}, math.inf)
                    rclpy.spin_once(node, timeout_sec=.02)
                if not node.output.stop_ack:
                    node.get_logger().error('STOP_UNCONFIRMED: MCU neutral acknowledgment missing')
            if node.log_file:
                node.log_file.close()
            node.destroy_node()
            if getattr(node,'output_lock',None):
                node.output_lock.close()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in previous_signals.items():
            signal.signal(sig, handler)


if __name__ == '__main__':
    main()
