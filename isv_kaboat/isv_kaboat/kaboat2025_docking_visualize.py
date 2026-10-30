#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageViewer(Node):
    def __init__(self):
        super().__init__(
            "image_viewer",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        self.bridge = CvBridge()

        # ---------- Parameters ----------
        self.image_topic     = str(self.declare_parameter("image_topic", "/camera1/image_raw").value)
        self.use_compressed  = bool(self.declare_parameter("use_compressed", False).value)  # True면 CompressedImage 구독
        self.window_name     = str(self.declare_parameter("window_name", "Docking Visualize").value)
        self.queue_depth     = int(self.declare_parameter("queue_depth", 5).value)
        self.use_best_effort = bool(self.declare_parameter("use_best_effort", True).value)
        self.resize_width    = int(self.declare_parameter("resize_width", 0).value)  # 0 = 원본
        self.draw_fps        = bool(self.declare_parameter("draw_fps", True).value)

        # QoS
        qos = QoSProfile(
            reliability=(QoSReliabilityPolicy.BEST_EFFORT if self.use_best_effort
                         else QoSReliabilityPolicy.RELIABLE),
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=self.queue_depth,
        )

        # ---------- State ----------
        self.last_t = None
        self.fps = 0.0
        self.frame_count = 0
        self.last_rx_time = None

        # ---------- Subscription ----------
        if self.use_compressed or self.image_topic.endswith("/compressed"):
            self.sub = self.create_subscription(CompressedImage, self.image_topic, self.cb_compressed, qos)
            self.get_logger().info(f"[SUB] CompressedImage: {self.image_topic} | QoS={'BEST_EFFORT' if self.use_best_effort else 'RELIABLE'}")
        else:
            self.sub = self.create_subscription(Image, self.image_topic, self.cb_raw, qos)
            self.get_logger().info(f"[SUB] Image: {self.image_topic} | QoS={'BEST_EFFORT' if self.use_best_effort else 'RELIABLE'}")

        # OpenCV window
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        # 키 입력 및 무프레임 경고 타이머
        self.create_timer(0.03, self._poll_key)      # ~33Hz 키 폴링
        self.create_timer(1.0, self._watchdog_tick)  # 1Hz 무프레임 감시

    # ---------- Callbacks ----------
    def _update_fps(self):
        t = time.time()
        if self.last_t is not None:
            dt = t - self.last_t
            if dt > 0:
                inst = 1.0 / dt
                self.fps = self.fps * 0.8 + inst * 0.2
        self.last_t = t
        self.last_rx_time = t

    def _draw_and_show(self, frame):
        # 리사이즈
        if self.resize_width > 0 and frame.shape[1] != self.resize_width:
            r = self.resize_width / float(frame.shape[1])
            frame = cv2.resize(frame, (self.resize_width, int(frame.shape[0] * r)), interpolation=cv2.INTER_AREA)

        # FPS 오버레이
        if self.draw_fps:
            cv2.putText(frame, f"FPS: {self.fps:4.1f}", (12, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)

        cv2.imshow(self.window_name, frame)
        cv2.waitKey(1)

    def cb_raw(self, msg: Image):
        try:
            # 가능한 인코딩 자동 처리
            enc = (msg.encoding or "bgr8").lower()
            if enc in ("bgr8", "bgra8"):
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            elif enc in ("rgb8", "rgba8"):
                rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
                frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            elif enc in ("mono8",):
                gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
                frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            else:
                # 알 수 없는 포맷이면 bgr8로 강제 시도
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"[cb_raw] decode failed: {e}")
            return

        self._update_fps()
        self._draw_and_show(frame)

    def cb_compressed(self, msg: CompressedImage):
        try:
            # CvBridge에 compressed 변환 함수가 있으면 사용
            try:
                frame = self.bridge.compressed_imgmsg_to_cv2(msg)
            except Exception:
                # 수동 디코드(fallback)
                np_arr = np.frombuffer(msg.data, np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if frame is None:
                    raise RuntimeError("cv2.imdecode returned None")
        except Exception as e:
            self.get_logger().warn(f"[cb_compressed] decode failed: {e}")
            return

        self._update_fps()
        self._draw_and_show(frame)

    # ---------- Helpers ----------
    def _poll_key(self):
        k = cv2.waitKey(1) & 0xFF
        if k in (ord('q'), 27):  # q / ESC
            self.get_logger().info("Quit requested.")
            rclpy.shutdown()

    def _watchdog_tick(self):
        # 3초 이상 프레임 없으면 경고
        if self.last_rx_time is None:
            self.get_logger().throttle(3000, "No frames received yet... "
                                             "Check topic name/type and QoS.")
        else:
            if time.time() - self.last_rx_time > 3.0:
                self.get_logger().warn("No frames for >3s. Check publisher and network/QoS.")

    def destroy_node(self):
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImageViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
