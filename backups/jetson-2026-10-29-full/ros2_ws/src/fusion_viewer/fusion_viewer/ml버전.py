import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml, os, math, traceback

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from yolo_msgs.msg import DetectionArray
from std_msgs.msg import Float32MultiArray, MultiArrayDimension


# ---------- PointCloud2 -> NumPy ----------
def pc2_to_xyz_numpy(msg: PointCloud2, logger=None) -> np.ndarray:
    import numpy as np
    if msg is None or len(msg.data) == 0:
        if logger:
            logger.warn("[PC2] msg None/empty")
        return np.empty((0, 3), dtype=np.float32)

    # 1) 빠른 경로: read_points_numpy 시도
    try:
        from sensor_msgs_py import point_cloud2 as pc2
        arr = pc2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        if isinstance(arr, np.ndarray) and arr.dtype.fields:
            x = arr["x"].astype(np.float32, copy=False)
            y = arr["y"].astype(np.float32, copy=False)
            z = arr["z"].astype(np.float32, copy=False)
            return np.stack((x, y, z), axis=1)
        return arr.astype(np.float32, copy=False).reshape(-1, 3)
    except Exception as e:
        if logger:
            logger.warn(f"[PC2] read_points_numpy failed, fallback: {e}")

    # 2) 수동 파싱: point_step/offset 그대로 사용해 x,y,z만 추출
    try:
        off = {f.name: f.offset for f in msg.fields}
        step = int(msg.point_step)
        n = int(len(msg.data) // step)
        buf = np.frombuffer(msg.data, dtype=np.uint8, count=n * step).reshape(n, step)
        x = buf[:, off["x"] : off["x"] + 4].view("<f4").reshape(-1)
        y = buf[:, off["y"] : off["y"] + 4].view("<f4").reshape(-1)
        z = buf[:, off["z"] : off["z"] + 4].view("<f4").reshape(-1)
        pts = np.stack((x, y, z), axis=1).astype(np.float32, copy=False)
        m = np.isfinite(pts).all(axis=1)
        return pts[m]
    except Exception as e:
        if logger:
            import traceback
            logger.error(f"[PC2] Fallback parse failed: {e}\n{traceback.format_exc()}")
        return np.empty((0, 3), dtype=np.float32)


# ---------- depth -> BGR colormap ----------
def depths_to_bgr(depth: np.ndarray, dmin: float = 1.0, dmax: float = 30.0) -> np.ndarray:
    if depth.size == 0 or dmax <= dmin:
        return np.zeros((0, 3), dtype=np.uint8)
    d = np.clip(depth, dmin, dmax)
    norm = (dmax - d) / max(1e-6, (dmax - dmin))  # 가까울수록 큰 값 → 빨강
    gray = (norm * 255.0).astype(np.uint8).reshape(-1, 1)
    color = cv2.applyColorMap(gray, cv2.COLORMAP_JET)
    return color.reshape(-1, 3)  # BGR


def _parse_track_id(det) -> float:
    """
    yolo_msgs/Detection의 det.id(문자열일 수 있음)를 float로 안전 변환.
    비어있으면 -1.0 반환.
    """
    tid_raw = getattr(det, "id", "")
    if tid_raw in ("", None):
        return -1.0
    try:
        return float(int(tid_raw))
    except Exception:
        try:
            return float(tid_raw)
        except Exception:
            return -1.0


class FusionViewer(Node):
    def __init__(self):
        super().__init__("fusion_viewer")

        # Params
        self.declare_parameter("image_topic", "/camera1/image_raw")
        # 기본값을 /yolo/tracking 으로 변경
        self.declare_parameter("detections_topic", "/yolo/tracking")
        self.declare_parameter("cloud_topic", "/pointcloud")
        self.declare_parameter("calib_file", "calib_example.yaml")
        self.declare_parameter("enable_projection", False)

        self.declare_parameter("depth_min", 1.0)
        self.declare_parameter("depth_max", 5.0)
        self.declare_parameter("reject_out_of_depth_range", False)

        self.declare_parameter("debug", True)
        self.declare_parameter("debug_every", 30)
        self.declare_parameter("draw_debug_text", True)
        self.declare_parameter("trace_very_verbose", False)

        # 이벤트 기반 뷰: 레이트 리밋/시간 매칭
        self.declare_parameter("min_draw_interval_ms", 50)
        self.declare_parameter("max_age_ms", 300)
        self.declare_parameter("force_projection_without_time_match", False)

        img_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        det_topic = self.get_parameter("detections_topic").get_parameter_value().string_value
        cloud_topic = self.get_parameter("cloud_topic").get_parameter_value().string_value
        calib_file = self.get_parameter("calib_file").get_parameter_value().string_value

        self.enable_projection = self.get_parameter("enable_projection").get_parameter_value().bool_value
        self.depth_min = float(self.get_parameter("depth_min").get_parameter_value().double_value)
        self.depth_max = float(self.get_parameter("depth_max").get_parameter_value().double_value)
        self.reject_out_of_depth = bool(
            self.get_parameter("reject_out_of_depth_range").get_parameter_value().bool_value
        )
        self.debug = bool(self.get_parameter("debug").get_parameter_value().bool_value)
        self.draw_debug_text = bool(self.get_parameter("draw_debug_text").get_parameter_value().bool_value)
        self.trace_very_verbose = bool(self.get_parameter("trace_very_verbose").get_parameter_value().bool_value)
        self.debug_every = int(self.get_parameter("debug_every").get_parameter_value().integer_value or 30)
        self.min_draw_ns = int(
            (self.get_parameter("min_draw_interval_ms").get_parameter_value().integer_value or 50) * 1e6
        )
        self.max_age_sec = float(
            self.get_parameter("max_age_ms").get_parameter_value().integer_value or 300
        ) / 1000.0
        self.force_no_time_match = bool(
            self.get_parameter("force_projection_without_time_match").get_parameter_value().bool_value
        )

        self.get_logger().info(f"[INIT] Image: {img_topic}")
        self.get_logger().info(f"[INIT] Detections: {det_topic}")
        self.get_logger().info(f"[INIT] Cloud: {cloud_topic}")
        self.get_logger().info(
            f"[INIT] Calib: {calib_file}  enable_projection={self.enable_projection}"
        )
        self.get_logger().info(
            f"[INIT] Depth color range: [{self.depth_min:.2f}, {self.depth_max:.2f}] m (reject_out_of_depth={self.reject_out_of_depth})"
        )
        self.get_logger().info(
            f"[INIT] Event: min_draw={self.min_draw_ns/1e6:.0f}ms, max_age={self.max_age_sec*1000:.0f}ms, force_time={self.force_no_time_match}"
        )
        self.get_logger().info(f"[INIT] Verbose trace: {self.trace_very_verbose}")

        self.bridge = CvBridge()
        self.last_image: Image = None
        self.last_dets: DetectionArray = None
        self.last_cloud: PointCloud2 = None

        # Sensor QoS (최신값만)
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST
        qos.durability = DurabilityPolicy.VOLATILE
        self.get_logger().info(
            f"[QoS] Using sensor profile: depth=1, BEST_EFFORT, KEEP_LAST, VOLATILE"
        )

        self.sub_img = self.create_subscription(Image, img_topic, self.cb_image, qos)
        self.sub_det = self.create_subscription(DetectionArray, det_topic, self.cb_dets, qos)
        self.sub_cloud = self.create_subscription(PointCloud2, cloud_topic, self.cb_cloud, qos)

        # 퍼블리셔 (u,v,classid,track_id,x,y,z,range)  ← 8열
        self.pub_fused = self.create_publisher(Float32MultiArray, "/fusion/roi_center", 10)

        # Calib
        self.K = None
        self.D = None
        self.T = None
        self._load_calib(calib_file)

        # State  ← 여기 들여쓰기 고쳤음!
        self.frame_idx = 0
        self._dbg = {}
        self._processing = False
        self._last_draw_ns = 0
        self._last_det_ns = 0
        self._cloud_recv_count = 0
        self._cloud_last_ns = 0

    # ----- Calib -----
    def _load_calib(self, calib_file):
        if not os.path.isabs(calib_file):
            from ament_index_python.packages import get_package_share_directory

            share = get_package_share_directory("fusion_viewer")
            cfg = os.path.join(share, "config", calib_file)
        else:
            cfg = calib_file

        if not os.path.exists(cfg):
            self.get_logger().warn(f"[CALIB] Calib file not found: {cfg}")
            return

        try:
            with open(cfg, "r") as f:
                data = yaml.safe_load(f)
            self.K = np.array(data["camera"]["K"], dtype=np.float32).reshape(3, 3)
            self.D = np.array(data["camera"]["D"], dtype=np.float32).reshape(-1)
            self.T = np.array(data["extrinsics"]["T"], dtype=np.float32).reshape(4, 4)
            self.get_logger().info("[CALIB] Calibration loaded.")
            self.get_logger().info(
                f"[CALIB] K[0,0..2]={self.K[0,0]:.3f},{self.K[0,1]:.3f},{self.K[0,2]:.3f}  cx={self.K[0,2]:.1f}, cy={self.K[1,2]:.1f}"
            )
            self.get_logger().info(f"[CALIB] T (first row) = {self.T[0, :]}")
            self.get_logger().info(f"[CALIB] T (t)        = {self.T[:3, 3]}")
        except Exception as e:
            self.get_logger().error(f"[CALIB] Exception loading calib: {e}\n{traceback.format_exc()}")

    # ----- Callbacks -----
    def cb_image(self, msg: Image):
        self.last_image = msg
        self.get_logger().info(
            f"[IMG] frame_id={msg.header.frame_id} stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"
        )
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_draw_ns < self.min_draw_ns:
            self.get_logger().info(
                f"[IMG] draw throttled ({(now_ns - self._last_draw_ns)/1e6:.1f}ms since last)"
            )
            return
        if self._processing:
            self.get_logger().info("[IMG] skip: already processing")
            return
        self._processing = True
        try:
            self._process_frame()
            self._last_draw_ns = now_ns
        finally:
            self._processing = False

    def cb_dets(self, msg: DetectionArray):
        self.last_dets = msg
        self._last_det_ns = self.get_clock().now().nanoseconds
        self.get_logger().info(
            f"[DET] detections={len(msg.detections)} frame_id={msg.header.frame_id} stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"
        )

    def cb_cloud(self, msg: PointCloud2):
        self.last_cloud = msg
        self._cloud_recv_count += 1
        self._cloud_last_ns = self.get_clock().now().nanoseconds
        self.get_logger().info(
            f"[PC] recv#{self._cloud_recv_count} frame_id={msg.header.frame_id} "
            f"stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d} "
            f"width={msg.width} height={msg.height}"
        )

    # ----- math/proj -----
    def project_points(self, xyz_cam: np.ndarray):
        self._dbg = {}
        try:
            if xyz_cam.size == 0:
                self._dbg["n_cam_in"] = 0
                self._dbg["n_zpos"] = 0
                return np.empty((0, 2), np.float32), np.empty((0,), np.float32)

            x, y, z = xyz_cam[:, 0], xyz_cam[:, 1], xyz_cam[:, 2]
            self._dbg["n_cam_in"] = int(xyz_cam.shape[0])
            self._dbg["z_min"] = float(np.min(z)) if z.size else None
            self._dbg["z_max"] = float(np.max(z)) if z.size else None

            zpos = z > 0.05  # 카메라 앞
            x, y, z = x[zpos], y[zpos], z[zpos]
            self._dbg["n_zpos"] = int(np.count_nonzero(zpos))

            if self.debug:
                self.get_logger().info(
                    f"[PROJ] xyz_cam in={self._dbg['n_cam_in']}, z>0.05={self._dbg['n_zpos']}, "
                    f"z_range=[{self._dbg['z_min']:.3f},{self._dbg['z_max']:.3f}]"
                )

            if x.size == 0:
                return np.empty((0, 2), np.float32), np.empty((0,), np.float32)

            fx, fy = self.K[0, 0], self.K[1, 1]
            cx, cy = self.K[0, 2], self.K[1, 2]
            u = fx * (x / z) + cx
            v = fy * (y / z) + cy
            pts = np.stack([u, v], axis=1).astype(np.float32)

            if self.trace_very_verbose:
                self.get_logger().info(f"[PROJ] sample uv: {pts[:5]}  sample z: {z[:5]}")

            return pts, z.astype(np.float32)
        except Exception as e:
            self.get_logger().error(f"[project_points] {e}\n{traceback.format_exc()}")
            return np.empty((0, 2), np.float32), np.empty((0,), np.float32)

    def _maybe_denormalize_bbox(self, cx, cy, w, h, W, H):
        if (w <= 2.0 and h <= 2.0) and (0.0 <= cx <= 1.0) and (0.0 <= cy <= 1.0):
            return cx * W, cy * H, w * W, h * H, True
        return cx, cy, w, h, False

    # ----- main -----
    def _process_frame(self):
        self.frame_idx += 1
        if self.last_image is None:
            self.get_logger().warn("[PROC] no image yet")
            return

        # Image
        try:
            img = self.bridge.imgmsg_to_cv2(self.last_image, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"[CvBridge] {e}")
            return
        H, W = img.shape[:2]
        self.get_logger().info(f"[PROC] frame#{self.frame_idx} image size={W}x{H}")

        # Draw detections
        det_total = 0
        det_drawn = 0
        if self.last_dets is not None:
            for det in self.last_dets.detections:
                cx = float(det.bbox.center.position.x)
                cy = float(det.bbox.center.position.y)
                w = float(det.bbox.size.x)
                h = float(det.bbox.size.y)
                cx, cy, w, h, denorm = self._maybe_denormalize_bbox(cx, cy, w, h, W, H)

                if not (math.isfinite(cx) and math.isfinite(cy) and math.isfinite(w) and math.isfinite(h)):
                    self.get_logger().warn(
                        f"[DET] non-finite bbox: cx={cx},cy={cy},w={w},h={h}"
                    )
                    continue
                if w <= 0.0 or h <= 0.0:
                    self.get_logger().warn(f"[DET] non-positive size: w={w}, h={h}")
                    continue

                x1 = int(round(cx - w / 2))
                y1 = int(round(cy - h / 2))
                x2 = int(round(cx + w / 2))
                y2 = int(round(cy + h / 2))
                if x2 < 0 or y2 < 0 or x1 >= W or y1 >= H:
                    det_total += 1
                    self.get_logger().info("[DET] bbox entirely off-screen")
                    continue

                x1c = max(0, min(W - 1, x1))
                y1c = max(0, min(H - 1, y1))
                x2c = max(0, min(W - 1, x2))
                y2c = max(0, min(H - 1, y2))
                if x2c <= x1c or y2c <= y1c:
                    det_total += 1
                    self.get_logger().info("[DET] bbox collapsed after clamping")
                    continue

                # 라벨에 tracking id 추가
                score = getattr(det, 'score', 0.0)
                tid = _parse_track_id(det)
                label = f"{det.class_name} {score:.2f}" + (f" #{int(tid)}" if tid >= 0 else "")
                label += (" (norm)" if denorm else "")
                cv2.rectangle(img, (x1c, y1c), (x2c, y2c), (0, 255, 0), 2)
                cv2.putText(
                    img,
                    label,
                    (x1c, max(0, y1c - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1,
                )
                det_total += 1
                det_drawn += 1
        self.get_logger().info(f"[DET] total={det_total}, drawn={det_drawn}")

        # LiDAR projection
        proj_reason = None
        n_raw = 0
        n_in = 0
        n_draw = 0

        if self.enable_projection:
            if self.K is None or self.T is None:
                proj_reason = "K/T missing"
                self.get_logger().warn(f"[PROJ] {proj_reason}")
            elif self.last_cloud is None:
                proj_reason = "no cloud yet"
                self.get_logger().warn(f"[PROJ] {proj_reason}")
            else:
                try:
                    img_t = self.last_image.header.stamp.sec + self.last_image.header.stamp.nanosec * 1e-9
                    pc_t = self.last_cloud.header.stamp.sec + self.last_cloud.header.stamp.nanosec * 1e-9
                    dt = abs(img_t - pc_t)
                    self.get_logger().info(
                        f"[TIME] dt(img-pc)={dt*1000:.1f}ms  (max_age={self.max_age_sec*1000:.1f}ms, force={self.force_no_time_match})"
                    )

                    if dt > self.max_age_sec and not self.force_no_time_match:
                        proj_reason = f"time_mismatch {dt*1000:.0f}ms"
                        self.get_logger().warn(f"[PROJ] {proj_reason}")
                    else:
                        # LiDAR 원시점 (LiDAR 프레임)
                        pts = pc2_to_xyz_numpy(self.last_cloud, logger=self.get_logger())
                        n_raw = int(pts.shape[0])
                        if n_raw == 0:
                            proj_reason = "cloud empty"
                            self.get_logger().warn(f"[PROJ] {proj_reason}")
                        else:
                            # 다운샘플
                            MAX_POINTS = 20000
                            if n_raw > MAX_POINTS:
                                step = n_raw // MAX_POINTS + 1
                                pts = pts[::step]
                                self.get_logger().info(
                                    f"[PC] downsampled: {n_raw} -> {pts.shape[0]} (step={step})"
                                )

                            # lidar->cam
                            R = self.T[:3, :3]
                            t = self.T[:3, 3]
                            if self.trace_very_verbose:
                                self.get_logger().info(
                                    f"[TF] t={t}, R00..02={R[0,0]:.3f},{R[0,1]:.3f},{R[0,2]:.3f}"
                                )
                            xyz_cam = (pts @ R.T) + t

                            # z-range before clipping
                            if xyz_cam.size > 0:
                                zmin = float(np.min(xyz_cam[:, 2]))
                                zmax = float(np.max(xyz_cam[:, 2]))
                                self.get_logger().info(
                                    f"[PROJ] xyz_cam z-range before clip: [{zmin:.3f}, {zmax:.3f}]"
                                )

                            # project
                            uv, depth = self.project_points(xyz_cam)

                            if uv.shape[0] == 0:
                                proj_reason = "no z>0"
                                self.get_logger().warn(f"[PROJ] {proj_reason}")
                            else:
                                # ----- 안전한 인덱싱: bool → int idx -----
                                u_all = uv[:, 0].astype(np.int32)
                                v_all = uv[:, 1].astype(np.int32)
                                m_inimg = (u_all >= 0) & (u_all < W) & (v_all >= 0) & (v_all < H)
                                if self.reject_out_of_depth:
                                    m_inimg &= (depth >= self.depth_min) & (depth <= self.depth_max)
                                idx_inimg = np.flatnonzero(m_inimg)

                                if idx_inimg.size == 0:
                                    proj_reason = "no in-image"
                                    self.get_logger().warn(
                                        f"[PROJ] {proj_reason}  uv_range: "
                                        f"u[{u_all.min()},{u_all.max()}], v[{v_all.min()},{v_all.max()}]"
                                    )
                                else:
                                    # 같은 인덱스를 모든 배열에 적용 (길이 일치 보장)
                                    u = u_all[idx_inimg]
                                    v = v_all[idx_inimg]
                                    depth = depth[idx_inimg]
                                    pts_inimg_lidar = pts[idx_inimg]  # (N,3) LiDAR frame

                                    n_in = int(u.shape[0])
                                    dmin = float(np.min(depth))
                                    dmax = float(np.max(depth))
                                    self.get_logger().info(
                                        f"[PROJ] in-image pts={n_in}, depth_range=[{dmin:.2f},{dmax:.2f}]"
                                    )

                                    # 점 시각화: 블록 채우기(원보다 안전)
                                    colors = depths_to_bgr(depth, self.depth_min, self.depth_max)
                                    block = 1  # 1=3x3, 2=5x5 ...
                                    for i in range(len(u)):
                                        px = int(u[i])
                                        py = int(v[i])
                                        if 0 <= px < W and 0 <= py < H:
                                            c = colors[i]
                                            x1b = max(0, px - block)
                                            x2b = min(W, px + block + 1)
                                            y1b = max(0, py - block)
                                            y2b = min(H, py + block + 1)
                                            img[y1b:y2b, x1b:x2b] = (int(c[0]), int(c[1]), int(c[2]))
                                    n_draw = int(depth.shape[0])

                                    # 각 ROI별 중앙값 + 퍼블리시 페이로드
                                    payload = []
                                    if det_drawn > 0 and self.last_dets is not None:
                                        for det in self.last_dets.detections:
                                            cx = float(det.bbox.center.position.x)
                                            cy = float(det.bbox.center.position.y)
                                            w = float(det.bbox.size.x)
                                            h = float(det.bbox.size.y)
                                            cx, cy, w, h, _ = self._maybe_denormalize_bbox(cx, cy, w, h, W, H)

                                            x1 = int(cx - w / 2)
                                            y1 = int(cy - h / 2)
                                            x2 = int(cx + w / 2)
                                            y2 = int(cy + h / 2)
                                            x1 = max(0, min(W - 1, x1))
                                            y1 = max(0, min(H - 1, y1))
                                            x2 = max(0, min(W - 1, x2))
                                            y2 = max(0, min(H - 1, y2))
                                            if x2 <= x1 or y2 <= y1:
                                                self.get_logger().info("[ROI] collapsed bbox after clamp")
                                                continue

                                            # ROI 내부 LiDAR 점만 선택
                                            roi_mask = (u >= x1) & (u <= x2) & (v >= y1) & (v <= y2)
                                            cnt = int(np.count_nonzero(roi_mask))
                                            if cnt > 0:
                                                # (1) range: 카메라 z 중앙값
                                                med = float(np.nanmedian(depth[roi_mask]))

                                                # (2) bbox 중심 픽셀(u,v)
                                                u0 = float((x1 + x2) / 2.0)
                                                v0 = float((y1 + y2) / 2.0)

                                                # (3) class_id (id와 분리)
                                                class_id = float(getattr(det, "class_id", -1))

                                                # (4) track_id
                                                track_id = _parse_track_id(det)

                                                # (5) (x,y,z): LiDAR 프레임 중앙값
                                                pts_roi_lidar = pts_inimg_lidar[roi_mask]  # (N,3)
                                                x_med = float(np.nanmedian(pts_roi_lidar[:, 0]))
                                                y_med = float(np.nanmedian(pts_roi_lidar[:, 1]))
                                                z_med = float(np.nanmedian(pts_roi_lidar[:, 2]))

                                                # 페이로드 한 행 추가: [u,v,classid,track_id,x,y,z,range]  ← 8열
                                                payload.append([u0, v0, class_id, track_id, x_med, y_med, z_med, med])

                                                # 화면 표시
                                                self.get_logger().info(
                                                    f"[ROI] pts_in_roi={cnt}, median={med:.2f} m"
                                                )
                                                cv2.putText(
                                                    img,
                                                    f"{med:.2f} m",
                                                    (x1, min(y2, H - 5)),
                                                    cv2.FONT_HERSHEY_SIMPLEX,
                                                    0.6,
                                                    (0, 200, 255),
                                                    2,
                                                )
                                            else:
                                                self.get_logger().info("[ROI] no lidar pts in roi")

                                    # ROI 결과 퍼블리시 (행렬 평탄화) 8열 레이아웃
                                    if len(payload) > 0:
                                        msg = Float32MultiArray()
                                        rows = len(payload)
                                        msg.layout.dim = [
                                            MultiArrayDimension(label="rows", size=rows, stride=rows * 8),
                                            MultiArrayDimension(label="cols", size=8, stride=8),
                                        ]
                                        flat = []
                                        for row in payload:
                                            flat.extend(row)
                                        msg.data = flat
                                        self.pub_fused.publish(msg)

                    # 주기적 요약 로그
                    if self.debug and (self.frame_idx % self.debug_every == 0):
                        reason = proj_reason if proj_reason else "ok"
                        self.get_logger().info(
                            f"[DBG#{self.frame_idx}] det_drawn={det_drawn} cloud_raw={n_raw} inimg={n_in} drawn={n_draw} reason={reason}"
                        )
                except Exception as e:
                    self.get_logger().error(f"[projection] {e}\n{traceback.format_exc()}")
                    proj_reason = "exception"

        # OSD
        if self.draw_debug_text:
            lines = [
                f"det: total={det_total}, drawn={det_drawn}",
                f"cloud_recv: {self._cloud_recv_count}",
            ]
            if self._cloud_last_ns:
                age_ms = (self.get_clock().now().nanoseconds - self._cloud_last_ns) / 1e6
                lines.append(f"cloud_age: {age_ms:.0f} ms")

            if self.enable_projection:
                if proj_reason:
                    lines.append(f"proj: {proj_reason}")
                lines.append(f"cloud: raw={n_raw}, inimg={n_in}, drawn={n_draw}")

                if self.last_cloud is not None:
                    img_t = self.last_image.header.stamp.sec + self.last_image.header.stamp.nanosec * 1e-9
                    pc_t = self.last_cloud.header.stamp.sec + self.last_cloud.header.stamp.nanosec * 1e-9
                    lines.append(f"dt(img-pc): {abs(img_t - pc_t) * 1000:.0f} ms")

            y = 18
            for s in lines:
                cv2.putText(img, s, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
                y += 20

        cv2.imshow("fusion_viewer", img)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = FusionViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

