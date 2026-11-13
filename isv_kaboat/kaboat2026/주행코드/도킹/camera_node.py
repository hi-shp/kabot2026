"""ROS2 camera-only docking perception. GUI connection is a separate step."""
import math
import os
from pathlib import Path
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Float32, Int32
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge

from .core import Calibration, Detection, MODEL_CLASSES, TargetSelector, bearing_deg, validate_model_names


class DockingCamera(Node):
    def __init__(self):
        super().__init__('docking_camera')
        defaults = dict(
            model_path='/home/isv/ros2_ws/weights/best3.pt', device='auto',
            image_topic='/camera/image_raw', camera_info_topic='/camera/camera_info',
            target_class_id=8, confidence_threshold=0.65, iou_threshold=0.5,
            lost_timeout=0.8, reacquire_timeout=3.0, center_gate_fraction=0.15,
            size_ratio_limit=2.0, image_timeout=1.0,
            camera_yaw_offset_deg=0.0, allow_fov_fallback=False, horizontal_fov_deg=0.0,
        )
        for name, value in defaults.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=name != 'target_class_id'))
        self.p = {name: self.get_parameter(name).value for name in defaults}
        for name in ('confidence_threshold', 'iou_threshold'):
            if not math.isfinite(self.p[name]) or not 0 < self.p[name] <= 1:
                raise ValueError(name + ' must be in (0,1]')
        if not math.isfinite(self.p['image_timeout']) or self.p['image_timeout'] <= 0:
            raise ValueError('image_timeout must be positive')
        if not math.isfinite(self.p['camera_yaw_offset_deg']):
            raise ValueError('camera_yaw_offset_deg must be finite')
        if self.p['allow_fov_fallback'] and not 0 < self.p['horizontal_fov_deg'] < 180:
            raise ValueError('explicit horizontal_fov_deg in (0,180) required for FOV fallback')
        self.selector = self.make_selector(self.p['target_class_id'])
        model_path = Path(self.p['model_path']).expanduser()
        if not model_path.is_file():
            raise FileNotFoundError('Trained model not found: ' + str(model_path))
        # Tracker dependencies must be installed explicitly, never downloaded at runtime.
        os.environ['YOLO_AUTOINSTALL'] = 'false'
        from ultralytics import YOLO
        import torch
        self.device = ('cuda:0' if torch.cuda.is_available() else 'cpu') if self.p['device'] == 'auto' else self.p['device']
        self.model = YOLO(str(model_path))
        validate_model_names(self.model.names)
        self.bridge = CvBridge()
        self.lock = threading.RLock()
        self.wake = threading.Event()
        self.shutdown_requested = threading.Event()
        self.latest = None
        self.calibration = None
        self.last_receive = -math.inf
        self.last_inference = None
        self.last_result_stamp = None
        self.last_debug = -math.inf
        self.generation = 0
        self.error = None
        self.previous_image_stamp = None
        self.pub_found = self.create_publisher(Bool, '/docking/camera/target_found', 1)
        self.pub_class = self.create_publisher(Int32, '/docking/camera/target_class_id', 1)
        self.pub_id = self.create_publisher(Int32, '/docking/camera/target_id', 1)
        self.pub_bearing = self.create_publisher(Float32, '/docking/camera/target_bearing_deg', 1)
        self.pub_center = self.create_publisher(PointStamped, '/docking/camera/target_center_px', 1)
        self.pub_image = self.create_publisher(Image, '/docking/camera/debug_image', 1)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.image_sub = self.create_subscription(Image, self.p['image_topic'], self.on_image, qos)
        self.info_sub = self.create_subscription(CameraInfo, self.p['camera_info_topic'], self.on_info, qos)
        self.add_on_set_parameters_callback(self.set_parameters_callback)
        self.watchdog = self.create_timer(0.1, self.check_stale)
        self.worker = threading.Thread(target=self.infer_loop, name='docking_camera_inference', daemon=False)
        self.worker.start()
        self.get_logger().info(f'Docking camera ready: {model_path}; {self.device}; image={self.p["image_topic"]}')

    def make_selector(self, class_id):
        return TargetSelector(class_id, self.p['lost_timeout'], self.p['reacquire_timeout'],
                              self.p['center_gate_fraction'], self.p['size_ratio_limit'])

    def set_parameters_callback(self, parameters):
        changes = [p for p in parameters if p.name == 'target_class_id']
        if not changes:
            return SetParametersResult(successful=True)
        try:
            selector = self.make_selector(changes[-1].value)
        except (ValueError, TypeError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        with self.lock:
            self.selector = selector
            self.p['target_class_id'] = selector.class_id
            self.generation += 1
            self.last_result_stamp = None
            self.publish_invalid()
        return SetParametersResult(successful=True)

    def source_fresh(self, stamp):
        age = (self.get_clock().now().nanoseconds - stamp) * 1e-9
        return -0.1 <= age <= self.p['image_timeout']

    def on_image(self, message):
        stamp = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
        with self.lock:
            if not self.source_fresh(stamp):
                return
            if self.previous_image_stamp is not None and stamp <= self.previous_image_stamp:
                return
            self.previous_image_stamp = stamp
            self.last_receive = time.monotonic()
            self.latest = (message, self.last_receive, stamp)
        self.wake.set()

    def on_info(self, message):
        roi = message.roi
        calibration = Calibration(message.width, message.height, tuple(message.k), tuple(message.d),
                                  message.distortion_model, bool(roi.x_offset or roi.y_offset or roi.width or roi.height),
                                  message.binning_x, message.binning_y)
        with self.lock:
            self.calibration = calibration

    def publish_invalid(self):
        if not self.context.ok():
            return
        self.pub_found.publish(Bool(data=False))
        self.pub_class.publish(Int32(data=self.selector.class_id))
        self.pub_id.publish(Int32(data=-1))

    def check_stale(self):
        with self.lock:
            if (self.last_result_stamp is None or not self.source_fresh(self.last_result_stamp)
                    or time.monotonic() - self.last_receive > self.p['image_timeout'] or self.error):
                self.publish_invalid()

    def infer_loop(self):
        try:
            while not self.shutdown_requested.is_set():
                self.wake.wait(0.1)
                self.wake.clear()
                with self.lock:
                    item, self.latest = self.latest, None
                    generation = self.generation
                if item is None:
                    continue
                message, _, stamp = item
                if not self.source_fresh(stamp):
                    continue
                frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8').copy()
                h, w = frame.shape[:2]
                results = self.model.track(frame, persist=True, tracker='bytetrack.yaml',
                    conf=self.p['confidence_threshold'], iou=self.p['iou_threshold'],
                    device=self.device, verbose=False)
                detections = self.parse_boxes(results[0].boxes)
                now = time.monotonic()
                with self.lock:
                    if self.shutdown_requested.is_set() or not self.context.ok():
                        break
                    if generation != self.generation or not self.source_fresh(stamp):
                        self.publish_invalid()
                        continue
                    target = self.selector.update(detections, now, w, h)
                    state = self.selector.status(now)
                    angle = None
                    reason = ''
                    if target is not None:
                        try:
                            angle = bearing_deg(*target.center, w, h, self.calibration,
                                self.p['camera_yaw_offset_deg'], self.p['allow_fov_fallback'], self.p['horizontal_fov_deg'])
                        except (ValueError, cv2.error) as exc:
                            reason = str(exc)
                            self.get_logger().warning(reason, throttle_duration_sec=5.0)
                    # center/header and bearing are emitted only for a fresh calibrated observation.
                    if angle is not None:
                        center = PointStamped()
                        center.header = message.header
                        center.point.x, center.point.y = map(float, target.center)
                        self.pub_center.publish(center)
                        self.pub_bearing.publish(Float32(data=float(angle)))
                        self.pub_class.publish(Int32(data=self.selector.class_id))
                        self.pub_id.publish(Int32(data=target.track_id))
                        self.pub_found.publish(Bool(data=True))
                    else:
                        self.publish_invalid()
                    self.last_result_stamp = stamp
                    fps = 0.0 if self.last_inference is None else 1.0 / max(1e-6, now - self.last_inference)
                    self.last_inference = now
                    if now - self.last_debug >= 0.1:
                        self.draw(frame, detections, target, state, angle, fps, reason)
                        debug = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                        debug.header = message.header
                        self.pub_image.publish(debug)
                        self.last_debug = now
        except Exception as exc:
            self.error = str(exc)
            self.get_logger().error('Docking inference stopped: ' + self.error)
            with self.lock:
                self.publish_invalid()

    @staticmethod
    def parse_boxes(boxes):
        if boxes is None or len(boxes) == 0:
            return []
        xyxy = boxes.xyxy.detach().cpu().numpy()
        classes = boxes.cls.detach().cpu().numpy()
        scores = boxes.conf.detach().cpu().numpy()
        ids = np.full(len(xyxy), -1) if boxes.id is None else boxes.id.detach().cpu().numpy()
        result = []
        for box, cid, score, tid in zip(xyxy, classes, scores, ids):
            if (not all(math.isfinite(float(v)) for v in (*box, cid, score, tid))
                    or cid != int(cid) or tid != int(tid) or not 0 <= int(cid) < len(MODEL_CLASSES)
                    or not 0 <= score <= 1 or box[2] <= box[0] or box[3] <= box[1]):
                continue
            result.append(Detection(int(cid), float(score), int(tid), tuple(map(float, box))))
        return result

    def draw(self, frame, detections, target, state, angle, fps, reason):
        h, w = frame.shape[:2]
        cv2.line(frame, (w // 2, 0), (w // 2, h - 1), (180, 180, 180), 1)
        for d in detections:
            selected = d is target
            color = (0, 255, 255) if selected else (200, 160, 80)
            x1, y1, x2, y2 = map(round, d.box)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3 if selected else 1)
            label = f'{d.class_id} {MODEL_CLASSES[d.class_id]} {d.confidence:.2f} ID:{d.track_id}'
            cv2.putText(frame, label, (max(0, x1), max(18, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, .45, color, 1)
            if selected:
                point = tuple(map(round, d.center))
                cv2.circle(frame, point, 5, color, -1)
                cv2.arrowedLine(frame, (w // 2, h - 1), point, color, 2)
        lines = [f'Target: {MODEL_CLASSES[self.selector.class_id]}  Class ID: {self.selector.class_id}',
                 f'Status: {state}  FPS: {fps:.1f}',
                 'Bearing: unavailable' if angle is None else f'Bearing: {angle:+.2f} deg (left +)',
                 'Center: unavailable' if target is None else f'Center: ({target.center[0]:.1f}, {target.center[1]:.1f})  ID: {target.track_id}  Conf: {target.confidence:.2f}']
        if reason:
            lines.append('Calibration required / see ROS log')
        for index, line in enumerate(lines):
            y = 20 + 20 * index
            cv2.putText(frame, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 0, 0), 3)
            cv2.putText(frame, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 255, 255), 1)

    def close(self):
        self.shutdown_requested.set()
        self.wake.set()
        self.worker.join()
        with self.lock:
            self.publish_invalid()
        del self.model


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DockingCamera()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
