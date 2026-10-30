import math
import time
import traceback
from typing import Dict, Tuple, List, Optional

import numpy as np
import pymap3d as pm
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    qos_profile_sensor_data,
)
from rclpy.parameter import Parameter

from sensor_msgs.msg import NavSatFix, PointCloud2
from std_msgs.msg import Float32MultiArray
from sensor_msgs_py import point_cloud2

from vectornav_msgs.msg import CommonGroup  # IMU(CommonGroup) 메시지

from simple_pid import PID
import yaml

# 프로젝트 모듈
from VO import (
    compute_vo_velocity_for_target_proj,          # 장애물 회피 (상대 조향각 + 제한 적용)
    compute_blocked_angle_intervals_from_xy,      # LiDAR XY 기반 블라인드 각도 계산
)
from relative_position import objs_to_rel_matrix_single  # 장애물 상대좌표 변환

from mechaship_interfaces.srv import ThrottlePulseWidth


class MotorControlNode(Node):
    def __init__(self) -> None:
        super().__init__(
            "pid_go",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        self.get_logger().info("----- start Hoping node -----")

        # -------------------- QoS: 센서용으로 통일 (Best Effort) --------------------
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # -------------------- 구독자 --------------------
        self.subscription_gps = self.create_subscription(
            NavSatFix, "/smc_2000/fix", self.gps_listener_callback, qos_profile
        )

        self.subscription_imu = self.create_subscription(
            CommonGroup, "/vectornav/raw/common", self.vectornav_callback, qos_profile
        )

        self.subscription_fusion = self.create_subscription(
            Float32MultiArray,
            "/fusion/roi_center",
            self.fusion_callback,
            qos_profile,
        )

        # ★ LiDAR 원시 포인트 (360°) 구독
        self.lidar_xy = np.empty((0, 2), dtype=np.float32)

        qos_profile_lidar = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.subscription_lidar = self.create_subscription(
            PointCloud2,
            "/ouster/points",
            self.lidar_callback,
            qos_profile_lidar,
        )

        # -------------------- 스로틀 서비스 클라이언트 --------------------
        self.set_throttle_handler_left = self.create_client(
            ThrottlePulseWidth, "/actuators/throttle_left/set_pulse_width"
        )
        self.set_throttle_handler_right = self.create_client(
            ThrottlePulseWidth, "/actuators/throttle_right/set_pulse_width"
        )
        self.set_throttle_handler_left.wait_for_service()
        self.set_throttle_handler_right.wait_for_service()

        # -------------------- 상태 변수 --------------------
        self.origin = [35.2316117, 129.08254347, 0.0]
        self.now_heading: Optional[float] = None  # ENU nav각 (0~360, 북=0, CW+)
        self.angle_to_goal: Optional[float] = None
        self.current_pos: Optional[Tuple[float, float]] = None  # (e, n)

        self.target_angle: Optional[float] = None  # 선체 기준 목표 조향각 (+:우, -:좌)
        self.last_time_pid = time.time()

        # PID: 각도 오차 제어
        self.angle_pid = PID(24.0 / 3, 0.0, 0.0)
        self.angle_pid.output_limits = (-400, 400)

        # -------------------- 목표점 리스트(ENU로 변환해 사용) --------------------
        self.goal_gps_coords = [
            (35.23163124, 129.08247798),
            (35.2315962, 129.08246026),
            (35.23157352, 129.08252906),
            (35.2316117, 129.08254347),
        ]
        self.goal_xy_coords: List[Tuple[float, float]] = []
        for lat, lon in self.goal_gps_coords:
            e, n = self.gps_enu_converter((lat, lon, 0.0))
            self.goal_xy_coords.append((e, n))

        self.status = 0
        self.now_goal = self.goal_xy_coords[0]

        # --- VO 요약 정보 저장용 ---
        self.last_class_str: str = "None"
        self.last_goal_rel_deg: float = 0.0
        self.last_vo_speed_cmd: float = 0.0
        self.last_vo_yaw_ship: float = 0.0

        # 최근 장애물 정보 (로그용)
        self.last_obs_range_m: float = 0.0
        self.last_obs_bearing_deg: float = 0.0

        # 모터 펄스 폭 제한
        self.MIN_PULSE_WIDTH = 1395
        self.MAX_PULSE_WIDTH = 1900

        # 정지/후진 로직
        self.stop_count = 0
        self.stop_status = 0

        # 트래킹 상태
        self.last_pose: Dict[int, Tuple[float, float, float]] = {}
        self.last_time: Dict[int, float] = {}
        self.last_vel: Dict[int, Tuple[float, float, float]] = {}

        self.last_objs: List[dict] = []

        # VO 설정 파일
        self.vo_config_path = "vo_config.yaml"

        # 속력/제어 매핑 파라미터
        self.v_allow_max: float = 3.5
        self.min_speed_mps: float = 0.2
        self.vo_update_period_s: float = 1.0
        self.obs_stale_threshold_s: float = 1.0
        self.ctrl_neutral: int = 1520
        self.ctrl_lo: int = 1520
        self.ctrl_hi: int = 1850
        self.ctrl_steer_clip: float = 300.0
        self.lidar_to_enu_deg: float = 0.0
        self.lidar_heading_offset_deg: float = 0.0

        self.max_steer_deg = 60.0

        # [LiDAR 블라인드 설정]
        # 1m 이내만 감지하여 차단
        self.blind_r_min: float = 0.0
        self.blind_r_max: float = 1.0  # 1m 이내
        self.blind_cluster_gap_deg: float = 3.0
        self.blind_cam_fov_deg: float = 90.0
        self.lidar_sample_step: int = 5

        # VO 대상 장애물 최대 거리
        self.vo_range_max_m: float = 5.0

        # 장애물 반경 테이블
        self.class_radius_map: Dict[int, float] = {}
        self.default_radius_m: float = 0.1

        # -------------------- vo_config.yaml 로드 --------------------
        try:
            with open(self.vo_config_path, "r", encoding="utf-8") as f:
                _cfg = yaml.safe_load(f) or {}
            _vo = _cfg.get("vo", {})
            self.v_allow_max = float(_vo.get("v_allow_max", self.v_allow_max))
            self.min_speed_mps = float(_vo.get("min_speed_mps", self.min_speed_mps))
            self.vo_update_period_s = float(
                _vo.get("update_period_s", self.vo_update_period_s)
            )
            self.obs_stale_threshold_s = float(
                _vo.get("stale_threshold_s", self.obs_stale_threshold_s)
            )
            self.lidar_to_enu_deg = float(
                _vo.get("lidar_to_enu_deg", self.lidar_to_enu_deg)
            )
            self.lidar_heading_offset_deg = float(
                _vo.get("lidar_heading_offset_deg", self.lidar_heading_offset_deg)
            )

            self.max_steer_deg = float(_vo.get("max_steer_deg", self.max_steer_deg))
            self.blind_r_min = float(_vo.get("blind_r_min", self.blind_r_min))
            self.blind_r_max = float(_vo.get("blind_r_max", self.blind_r_max))
            self.blind_cluster_gap_deg = float(
                _vo.get("blind_cluster_gap_deg", self.blind_cluster_gap_deg)
            )
            self.blind_cam_fov_deg = float(
                _vo.get("blind_cam_fov_deg", self.blind_cam_fov_deg)
            )
            self.lidar_sample_step = int(
                _vo.get("lidar_sample_step", self.lidar_sample_step)
            )
            self.vo_range_max_m = float(
                _vo.get("range_max_for_vo_m", self.vo_range_max_m)
            )
            _class_radius = _vo.get("class_radius_m", {})
            self.class_radius_map = {
                int(k): float(v) for k, v in _class_radius.items()
            }
            self.default_radius_m = float(
                _vo.get("default_radius_m", self.default_radius_m)
            )

            _ctrl = _cfg.get("control", {})
            self.ctrl_neutral = int(_ctrl.get("neutral_pulse", self.ctrl_neutral))
            self.ctrl_lo = int(_ctrl.get("forward_lo_pulse", self.ctrl_lo))
            self.ctrl_hi = int(_ctrl.get("forward_hi_pulse", self.ctrl_hi))
            self.ctrl_steer_clip = float(
                _ctrl.get("steer_clip_max", self.ctrl_steer_clip)
            )
        except Exception as e:
            self.get_logger().warn(
                f"[Config] vo_config.yaml load failed, using defaults: {e}"
            )

        self.forward_pw: int = 1540

        self.create_timer(self.vo_update_period_s, self.vo_step)

        self.get_logger().info("MotorControlNode initialized")
        self.get_logger().info(
            f"[Frame] ENU: 북=0°, 시계방향+, 라이다→ENU 회전={self.lidar_to_enu_deg:.1f}°"
        )
        self.get_logger().info(
            f"[BlindSector] LiDAR Range: {self.blind_r_min}~{self.blind_r_max}m (Excluding Front 90°)"
        )

    # -------------------- IMU 콜백 --------------------
    def vectornav_callback(self, msg: CommonGroup) -> None:
        yaw_deg = float(msg.yawpitchroll.x)
        self.now_heading = yaw_deg if yaw_deg >= 0.0 else 360.0 + yaw_deg

    # -------------------- GPS 콜백 --------------------
    def gps_listener_callback(self, gps: NavSatFix) -> None:
        e, n = self.gps_enu_converter(
            [gps.latitude, gps.longitude, gps.altitude]
        )
        self.current_pos = (e, n)

        goal_x, goal_y = self.now_goal
        delta_x = goal_x - e
        delta_y = goal_y - n

        angle_rad = math.atan2(delta_x, delta_y)
        self.angle_to_goal = (math.degrees(angle_rad) + 360.0) % 360.0

        distance = math.sqrt(delta_x * delta_x + delta_y * delta_y)
        if distance <= 1.0:
            self.stop_status = 1
            self.get_logger().info("===== Goal reached =====")
            self.status += 1
            if self.status >= len(self.goal_xy_coords):
                self.status = len(self.goal_xy_coords) - 1
            self.now_goal = self.goal_xy_coords[self.status]

    # -------------------- LiDAR 콜백 --------------------
    def lidar_callback(self, msg: PointCloud2) -> None:
        """
        360도 전체 포인트 중 1m 이내의 장애물 포인트만 수집
        """
        pts_xy = []
        try:
            sample_step = max(1, int(self.lidar_sample_step))  # 데이터가 너무 많으면 느려지므로 샘플링
            r_limit = self.blind_r_max  # 1.0m

            for idx, p in enumerate(
                point_cloud2.read_points(
                    msg, field_names=("x", "y", "z"), skip_nans=True
                )
            ):
                if idx % sample_step != 0:
                    continue
                x, y, z = p
                r = math.hypot(x, y)

                # 1m 이내에 있는 점들만 수집
                if self.blind_r_min <= r <= r_limit:
                    pts_xy.append((x, y))

            if pts_xy:
                self.lidar_xy = np.asarray(pts_xy, dtype=np.float32)
            else:
                self.lidar_xy = np.empty((0, 2), dtype=np.float32)

        except Exception as e:
            self.get_logger().warn(f"[LiDAR] parse failed: {e}")
            self.lidar_xy = np.empty((0, 2), dtype=np.float32)

    # -------------------- Fusion 콜백 --------------------
    def fusion_callback(self, msg: Float32MultiArray) -> None:
        data = list(msg.data) if msg.data else []
        if msg.layout and msg.layout.dim and len(msg.layout.dim) >= 2:
            try:
                rows = int(msg.layout.dim[0].size)
                cols = int(msg.layout.dim[1].size)
            except Exception:
                rows, cols = (0, 8)
        else:
            cols = 8
            rows = len(data) // cols if cols else 0

        if cols != 8:
            return

        objs = []
        t_now = self.get_clock().now().nanoseconds * 1e-9

        for i in range(rows):
            base = i * 8
            try:
                u, v, classid, track_id, x, y, z, rng = data[base : base + 8]
                tid = int(track_id)
                x, y, z = float(x), float(y), float(z)

                if tid in self.last_pose and tid in self.last_time:
                    px, py, pz = self.last_pose[tid]
                    dt = t_now - self.last_time[tid]
                    if dt <= 0.0:
                        dt = 1e-6
                    vx, vy, vz = (x - px) / dt, (y - py) / dt, (z - pz) / dt
                    self.last_vel[tid] = (vx, vy, vz)
                else:
                    self.last_vel[tid] = (0.0, 0.0, 0.0)

                self.last_pose[tid] = (x, y, z)
                self.last_time[tid] = t_now

                objs.append(
                    {
                        "u": float(u),
                        "v": float(v),
                        "class_id": int(classid),
                        "track_id": tid,
                        "x": x,
                        "y": y,
                        "z": z,
                        "range": float(rng),
                    }
                )
            except Exception:
                continue
        self.last_objs = objs

    # -------------------- VO + PID 주기 처리 --------------------
    def vo_step(self) -> None:
        if self.status != 0:
            return
        if self.now_heading is None or self.angle_to_goal is None:
            return
        if self.current_pos is None:
            self.get_logger().warn("VO skip: current_pos not ready (no GPS yet)")
            return

        try:
            t_now = self.get_clock().now().nanoseconds * 1e-9

            # -----------------------------------------------------------
            # Fusion 데이터 확인 (생략 가능)
            # -----------------------------------------------------------
            if not self.last_objs:
                pass

            # 1) 스테일/거리 필터링
            fresh_objs = []
            if self.last_objs:
                for o in self.last_objs:
                    tid = int(o.get("track_id", -1))
                    dt = t_now - self.last_time.get(tid, 0)
                    rng = float(o.get("range", 1e9))

                    if tid in self.last_time and dt <= self.obs_stale_threshold_s:
                        if rng <= self.vo_range_max_m:
                            fresh_objs.append(o)

            if not fresh_objs:
                vo_objs: List[dict] = []
                v_ship_obs_xy = np.zeros((0, 3), dtype=float)
                class_str = "None"
                self.last_obs_range_m = 0.0
                self.last_obs_bearing_deg = 0.0
            else:
                mat = objs_to_rel_matrix_single(
                    fresh_objs,
                    CLASS_RADIUS_M=self.class_radius_map,
                    DEFAULT_RADIUS_M=self.default_radius_m,
                )
                vo_objs = []
                v_rows = []

                min_r = 1e9
                min_bearing = 0.0

                for i, o in enumerate(fresh_objs):
                    x_surf = float(mat[i, 3])
                    y_surf = float(mat[i, 4])
                    z_surf = float(mat[i, 5])
                    vo_objs.append(
                        {
                            "x": x_surf,
                            "y": y_surf,
                            "z": z_surf,
                            "class_id": int(o["class_id"]),
                        }
                    )
                    tid = int(o["track_id"])
                    vx, vy, vz = self.last_vel.get(tid, (0.0, 0.0, 0.0))
                    v_rows.append([-vx, -vy, -vz])

                    r_xy = math.hypot(x_surf, y_surf)
                    th_deg = math.degrees(math.atan2(y_surf, x_surf))
                    if r_xy < min_r:
                        min_r = r_xy
                        min_bearing = th_deg

                self.last_obs_range_m = float(min_r)
                self.last_obs_bearing_deg = float(min_bearing)
                v_ship_obs_xy = (
                    np.asarray(v_rows, dtype=float)
                    if v_rows
                    else np.zeros((0, 3), dtype=float)
                )

                class_ids = sorted(
                    {int(o.get("class_id", -1)) for o in fresh_objs}
                )
                class_str = (
                    ", ".join(str(cid) for cid in class_ids)
                    if class_ids
                    else "None"
                )

            # -----------------------------------------------------------
            # 2) LiDAR 블라인드 구역 계산 및 로그 출력
            # -----------------------------------------------------------
            blocked_intervals: List[Tuple[float, float]] = []
            try:
                blocked_intervals = compute_blocked_angle_intervals_from_xy(
                    self.lidar_xy,
                    cam_fov_deg=self.blind_cam_fov_deg,  # 전방 90도는 LiDAR 블라인드 처리에서 제외 (Fusion이 담당)
                    r_min=self.blind_r_min,  # 0.0m
                    r_max=self.blind_r_max,  # 1.0m
                    cluster_gap_deg=self.blind_cluster_gap_deg,
                )

                # ★ 블라인드 로그
                if blocked_intervals:
                    interval_str = ", ".join(
                        [f"{s:.1f}~{e:.1f}" for s, e in blocked_intervals]
                    )
                    self.get_logger().info(
                        f"[Blind] Blocked Angles: {interval_str}"
                    )

            except Exception as e:
                self.get_logger().warn(f"Blind sector calc failed: {e}")
                blocked_intervals = []

            # 3) VO 계산 (Fusion 객체 + LiDAR 블라인드 구역)
            speed_cmd, relative_steering_cmd = compute_vo_velocity_for_target_proj(
                vo_objs,
                v_ship_obs_xy,
                heading_target_deg=float(self.angle_to_goal),
                config_path=self.vo_config_path,
                blocked_intervals=blocked_intervals,  # LiDAR 금지구역 전달
                ship_heading_deg=float(self.now_heading),
                max_steer_deg=self.max_steer_deg,
            )

            # 4) 목표 조향각 설정
            self.target_angle = relative_steering_cmd

            # 5) 속력 → 전진 스로틀
            self.forward_pw = self.speed_to_pulse(speed_cmd)

            # 로그용 데이터
            goal_rel_deg = (
                (self.angle_to_goal - self.now_heading + 540.0) % 360.0 - 180.0
            )

            self.last_class_str = class_str
            self.last_goal_rel_deg = goal_rel_deg
            self.last_vo_speed_cmd = speed_cmd
            self.last_vo_yaw_ship = relative_steering_cmd

            # 6) 모터 제어
            if self.stop_status == 1:
                self.back_motor()
                self.stop_count += 1
                if self.stop_count >= 4:
                    self.stop_count = 0
                    self.stop_status = 0
            else:
                self.pid_angle()

        except Exception as e:
            self.get_logger().error(
                f"[VO EXCEPTION] {e}\n{traceback.format_exc()}"
            )
            if self.now_heading is not None and self.angle_to_goal is not None:
                self.target_angle = (
                    (self.angle_to_goal - self.now_heading + 540.0)
                    % 360.0
                    - 180.0
                )
            self.pid_angle()

    # -------------------- 보조 함수 --------------------
    def pid_angle(self) -> None:
        if self.target_angle is None:
            return

        self.angle_pid.setpoint = 0.0
        current_time = time.time()
        dt = current_time - self.last_time_pid
        if dt <= 0.0:
            dt = 1e-3

        motor_cmd = self.angle_pid(-self.target_angle, dt)
        self.last_time_pid = current_time

        steer_mag = abs(motor_cmd)
        steer_dir = 1 if motor_cmd > 0 else 2

        self.drive_with_steer(self.forward_pw, steer_mag, steer_dir)

    def gps_enu_converter(self, gnss) -> Tuple[float, float]:
        e, n, _ = pm.geodetic2enu(
            gnss[0],
            gnss[1],
            gnss[2],
            self.origin[0],
            self.origin[1],
            self.origin[2],
        )
        return e, n

    def drive_with_steer(
        self, forward_pw: int, steer_percentage: float, direction: int
    ) -> None:
        throttle = ThrottlePulseWidth.Request()

        steer_clip = max(0.0, min(steer_percentage, self.ctrl_steer_clip))

        if direction == 1:
            left_pulse_width = forward_pw
            right_pulse_width = forward_pw - steer_clip
        else:
            left_pulse_width = forward_pw - steer_clip
            right_pulse_width = forward_pw

        left_pulse_width = self.constrain(
            left_pulse_width, self.MIN_PULSE_WIDTH, self.MAX_PULSE_WIDTH
        )
        right_pulse_width = self.constrain(
            right_pulse_width, self.MIN_PULSE_WIDTH, self.MAX_PULSE_WIDTH
        )

        left_delta_pw = int(left_pulse_width - self.ctrl_neutral)
        right_delta_pw = int(right_pulse_width - self.ctrl_neutral)

        if steer_percentage < 1e-3:
            steer_dir_str = "직진"
        else:
            steer_dir_str = "우회전" if direction == 1 else "좌회전"

        class_str = getattr(self, "last_class_str", "None")
        goal_rel_deg = getattr(self, "last_goal_rel_deg", 0.0)
        speed_cmd = getattr(self, "last_vo_speed_cmd", 0.0)
        vo_yaw_ship = getattr(self, "last_vo_yaw_ship", 0.0)
        obs_r = getattr(self, "last_obs_range_m", 0.0)
        obs_bearing = getattr(self, "last_obs_bearing_deg", 0.0)

        self.get_logger().info(
            f"Cls={class_str} | "
            f"GoalΔ={goal_rel_deg:.1f} | "
            f"V={speed_cmd:.2f} | "
            f"Steer={vo_yaw_ship:.1f} | "
            f"Act={steer_dir_str} | "
            f"L={left_delta_pw} R={right_delta_pw} | "
            f"Obs={obs_r:.1f}m/{obs_bearing:.1f}"
        )

        throttle.pulse_width = int(left_pulse_width)
        self.set_throttle_handler_left.call_async(throttle)

        throttle.pulse_width = int(right_pulse_width)
        self.set_throttle_handler_right.call_async(throttle)

    def speed_to_pulse(self, speed_mps: float) -> int:
        v = max(0.0, float(speed_mps))
        # 0 이하면 완전 정지, 그 외에는 최소 속력 이상으로 강제
        if v <= 0.0:
            return int(
                self.constrain(
                    float(self.ctrl_neutral),
                    self.MIN_PULSE_WIDTH,
                    self.MAX_PULSE_WIDTH,
                )
            )

        if v < float(self.min_speed_mps):
            v = float(self.min_speed_mps)

        v_max = max(0.5, float(self.v_allow_max))
        lo = float(self.ctrl_lo)
        hi = float(self.ctrl_hi)
        pw = lo + (hi - lo) * min(v / v_max, 1.0)
        return int(
            self.constrain(pw, self.MIN_PULSE_WIDTH, self.MAX_PULSE_WIDTH)
        )

    @staticmethod
    def constrain(val: float, lo: float, hi: float) -> float:
        return max(min(val, hi), lo)

    def stop_motor(self) -> None:
        left = ThrottlePulseWidth.Request()
        right = ThrottlePulseWidth.Request()
        left.pulse_width = self.ctrl_neutral
        right.pulse_width = self.ctrl_neutral
        self.set_throttle_handler_left.call_async(left)
        self.set_throttle_handler_right.call_async(right)
        self.get_logger().info("Motors stopped")

    def back_motor(self) -> None:
        throttle = ThrottlePulseWidth.Request()
        self.get_logger().info("back-------------------------")
        left_pw = self.constrain(1400, self.MIN_PULSE_WIDTH, self.MAX_PULSE_WIDTH)
        right_pw = self.constrain(1400, self.MIN_PULSE_WIDTH, self.MAX_PULSE_WIDTH)

        throttle.pulse_width = int(left_pw)
        self.set_throttle_handler_left.call_async(throttle)
        throttle.pulse_width = int(right_pw)
        self.set_throttle_handler_right.call_async(throttle)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
        node.stop_motor()
    finally:
        node.stop_motor()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
