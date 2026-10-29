#!/usr/bin/env python3
import math
import time
import traceback
from typing import Dict, Tuple, List, Optional
from simple_pid import PID
import numpy as np
import pymap3d as pm

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import Parameter

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32MultiArray
from vectornav_msgs.msg import CommonGroup
from mechaship_interfaces.srv import ThrottlePulseWidth

from isv_kaboat.first_nav import go_first_goal
from isv_kaboat.path_follower import PathFollower
from isv_kaboat.obstacle_isv import calculate_heading_by_risk


class MotorControlNode(Node):
    def __init__(self) -> None:
        super().__init__(
            "pid_go",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        self.get_logger().info("----- start Hoping node -----")

        # -------------------- QoS: 센서용으로 통일 --------------------
        qos_profile = qos_profile_sensor_data

        # -------------------- 구독자 --------------------
        self.subscription_gps = self.create_subscription(
            NavSatFix, "/smc_2000/fix", self.gps_listener_callback, qos_profile
        )

        self.subscription_imu = self.create_subscription(
            CommonGroup, "/vectornav/raw/common", self.vectornav_callback, qos_profile
        )

        self.subscription_fusion = self.create_subscription(
            Float32MultiArray, "/fusion/roi_center", self.fusion_callback, qos_profile
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
        self.origin = [35.2328965, 129.0798148, 0.0]  # ENU 원점
        self.now_heading: float | None = None
        self.current_pos: Tuple[float, float] | None = None
        self.target_angle: float | None = None       # 조타 PID 목표(선수 기준 ±180)
        self.angle_to_goal: float | None = None

        # === status 2 용 설정 ===
        # ★ fusion class_id 중에서 쫓아갈 타깃 id
        self.target_id: int = 6
        # ★ 마지막으로 타겟(class_id == target_id)을 본 시각 (sec)
        self.last_target_seen_time: float | None = None

        # === status 3 용 설정 ===
        #   - status 2와는 다른 target을 추종해서 1m 앞에서 멈춤
        #   - 그 후 CW/CCW로 선회
        self.dock_target_id: int = 7        # status 5에서 추종할 다른 타깃 class_id
        self.dock_distance_m: float = 1.0   # 타깃 1m 앞에서 도킹
        self.spin_direction: int = 1        # 1: CW, 2: CCW
        self.spin_duration_sec: float = 5.0 # 도킹 후 몇 초 동안 선회할지
        self.spin_start_time: Optional[float] = None

        self.last_time_pid = time.time()

        # PID: 각도 오차 제어
        self.angle_pid = PID(24.0 / 3, 0.0, 0.0)   # Kp=8, Ki,Kd=0(초기값)
        self.angle_pid.output_limits = (-400, 400)

        # 목표점 리스트(ENU로 변환해 사용)
        self.goal_gps_coords = [
            (35.2329550, 129.0799300),
            (35.2329100, 129.0798400),  # 경로 추정 전 첫 시작 좌표
            (35.2329350, 129.0798900),  # 정박할 좌표 추종 후
            (35.2329450, 129.0799100),  # 도킹 전 정지 포인트
            (35.2329500, 129.0799200),  # 탐색 전 정지 포인트
            (35.2329550, 129.0799300),  # 최종 골
        ]
        self.goal_xy_coords: List[Tuple[float, float]] = []
        for lat, lon in self.goal_gps_coords:
            e, n = self.gps_enu_converter((lat, lon, 0.0))
            self.goal_xy_coords.append((e, n))

        # 시작 status
        self.status = 6
        self.now_goal = self.goal_xy_coords[0]

        # 모터 펄스 폭 제한
        self.MIN_PULSE_WIDTH = 1395
        self.MAX_PULSE_WIDTH = 1900

        # 정지/후진 로직
        self.stop_count = 0
        self.stop_status = 0

        # 트래킹 상태(최근 물체 위치/시간/속도)
        self.last_pose: Dict[int, Tuple[float, float, float]] = {}
        self.last_time: Dict[int, float] = {}
        self.last_vel: Dict[int, Tuple[float, float]] = {}  # (vx, vy) in LiDAR/ship frame

        # 최신 Fusion 객체 캐시 → 주기 타이머에서 VO 사용
        self.last_objs: List[dict] = []

        self.path_follow = PathFollower(
            origin_latlonalt=tuple(self.origin),
            arrive_radius_m=1.0,
        )

        # 10 Hz 주기 처리: 상태 준비되면 VO + PID 수행
        self.create_timer(0.1, self.vo_step)

        self.get_logger().info("MotorControlNode initialized")

    # -------------------- IMU 콜백 --------------------
    def vectornav_callback(self, msg: CommonGroup) -> None:
        """VectorNav CommonGroup: yawpitchroll.x = yaw(deg)"""
        yaw_deg = float(msg.yawpitchroll.x)
        self.now_heading = yaw_deg if yaw_deg >= 0.0 else 360.0 + yaw_deg
        # self.get_logger().debug(f"[IMU] Heading={self.now_heading:.2f}°")

    # -------------------- GPS 콜백 --------------------
    def gps_listener_callback(self, gps: NavSatFix) -> None:
        e, n = self.gps_enu_converter([gps.latitude, gps.longitude, 0.0])
        self.current_pos = (e, n)

        goal_x, goal_y = self.now_goal
        delta_x = goal_x - e
        delta_y = goal_y - n

        # 북쪽 기준 절대 목표각(0~360, CW+)
        angle_rad = math.atan2(delta_x, delta_y)  # 주의: atan2(y, x)가 아님(북 기준)
        self.angle_to_goal = (math.degrees(angle_rad) + 360.0) % 360.0

        distance = math.sqrt(delta_x * delta_x + delta_y * delta_y)
        if distance <= 1.0:
            self.stop_status = 1
            self.get_logger().info("===== Goal reached =====")

    # -------------------- FUSION 콜백 --------------------
    def fusion_callback(self, msg: Float32MultiArray) -> None:
        """ /fusion/roi_center : [u,v,class_id,track_id,x,y,z,range]*rows """
        data = list(msg.data) if msg.data else []
        # cols/rows
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
            self.get_logger().warn(f"/fusion/roi_center expects 8 cols, got {cols}")
            return

        objs = []
        t_now = self.get_clock().now().nanoseconds * 1e-9

        for i in range(rows):
            base = i * 8
            try:
                u, v, classid, track_id, x, y, z, rng = data[base:base + 8]
                tid = int(track_id)
                x, y, z = float(x), float(y), float(z)

                # 속도 추정
                if tid in self.last_pose and tid in self.last_time:
                    px, py, pz = self.last_pose[tid]
                    dt = t_now - self.last_time[tid]
                    if dt <= 0.0:
                        dt = 1e-6
                    vx, vy, vz = (x - px) / dt, (y - py) / dt, (z - pz) / dt
                    self.last_vel[tid] = (vx, vy)
                else:
                    self.last_vel[tid] = (0.0, 0.0)

                self.last_pose[tid] = (x, y, z)
                self.last_time[tid] = t_now

                objs.append({
                    "u": float(u), "v": float(v),
                    "class_id": int(classid), "track_id": tid,
                    "x": x, "y": y, "z": z, "range": float(rng),
                })
            except Exception as e:
                self.get_logger().warn(f"[fusion] row {i} parse fail: {e}")
                continue

        # 최신 관측 캐시 (타이머에서 사용)
        self.last_objs = objs

    # -------------------- 주기 처리(VO + PID) ---------------------
    def vo_step(self) -> None:
        """10 Hz: 상태가 준비되면 VO 계산 → PID 조타"""

        # === status 0: 첫 목표점으로 직진 ===
        if self.status == 0:
            # 상태 준비 확인
            if self.now_heading is None or self.current_pos is None or self.now_goal is None:
                if self.now_heading is None:
                    self.get_logger().debug("status 0: now_heading None")
                if self.current_pos is None:
                    self.get_logger().warn("status 0: current_pos not ready (no GPS yet)")
                if self.now_goal is None:
                    self.get_logger().debug("status 0: now_goal None")
                return

            e, n = self.current_pos
            self.target_angle = go_first_goal(
                self.now_goal,
                e, n,
                self.now_heading
            )

            if self.target_angle > 180:
                self.target_angle = (360 - self.target_angle) * -1   # 0~180 → 0~-180

            self.get_logger().info(
                f"현재 target_angle = {self.target_angle:.2f}°"
                f" status = {self.status:.1f}°, "
            )

            if self.stop_status == 1:
                self.back_motor()
                self.get_logger().info("back_motor_on")

                self.stop_count += 1
                if self.stop_count == 4:
                    self.stop_count = 0
                    self.stop_status = 0
                    self.status += 1
                    self.get_logger().info("status+1")
                    if self.status >= len(self.goal_xy_coords):
                        self.status = 0
                    self.now_goal = self.goal_xy_coords[self.status]
            else:
                self.pid_angle2()

        # === status 1: PathFollower 기반 경로 추종 + 정지 시퀀스 ===
        elif (
            self.now_heading is not None
            and self.now_goal is not None
            and self.current_pos is not None
            and self.status == 1
        ):
            e, n = self.current_pos
            try:
                self.target_angle = self.path_follow.update(e, n, self.now_heading)  # ±180°
                self.get_logger().info(
                    f"현재 target_angle = {self.target_angle:.2f}°"
                    f" status = {self.status:.1f}°, "
                )
            except Exception as ex:
                self.get_logger().warn(f"PathFollower error: {ex}; fallback to goal")
                if self.angle_to_goal is None:
                    self.get_logger().warn("[status 1] angle_to_goal None, skip")
                    return
                self.target_angle = (
                    self.angle_to_goal - self.now_heading + 540
                ) % 360 - 180

            if self.path_follow.done():
                self.get_logger().info("All waypoints reached in status 1.")

            if self.stop_status == 1:
                # stop 시작 시간 기록이 없으면 지금 시각 저장
                if not hasattr(self, "stop_start_time") or self.stop_start_time is None:
                    self.stop_start_time = time.time()
                    self.get_logger().info("Stop sequence started")

                elapsed = time.time() - self.stop_start_time

                if elapsed < 4.0:
                    # 1단계: 4초 동안 역추진
                    self.back_motor()
                    self.get_logger().info("back_motor_on")

                elif elapsed < 8.0:
                    # 2단계: 모터 정지 후 추가로 4초간 대기
                    self.stop_motor()
                    self.get_logger().info("stop_motor")

                else:
                    # 시퀀스 종료
                    self.stop_status = 0
                    self.stop_start_time = None
                    self.get_logger().info("Stop sequence finished")
                    self.get_logger().info("status+1")
                    self.status += 1
                    if self.status >= len(self.goal_xy_coords):
                        self.status = 0
                    self.now_goal = self.goal_xy_coords[self.status]
            else:
                self.pid_angle2()

        # === status 2: go_first_goal 기반 이동 ===
        if self.status == 2:
            if self.now_heading is None or self.current_pos is None or self.now_goal is None:
                if self.now_heading is None:
                    self.get_logger().debug("status 2: now_heading None")
                if self.current_pos is None:
                    self.get_logger().warn("status 2: current_pos not ready (no GPS yet)")
                if self.now_goal is None:
                    self.get_logger().debug("status 2: now_goal None")
                return

            e, n = self.current_pos
            self.target_angle = go_first_goal(
                self.now_goal,
                e, n,
                self.now_heading
            )

            if self.target_angle > 180:
                self.target_angle = (360 - self.target_angle) * -1

            self.get_logger().info(
                f"현재 target_angle = {self.target_angle:.2f}°"
                f" status = {self.status:.1f}°, "
            )

            if self.stop_status == 1:
                self.back_motor()
                self.get_logger().info("back_motor_on")

                self.stop_count += 1
                if self.stop_count == 2:
                    self.stop_count = 0
                    self.stop_status = 0
                    self.status += 1
                    self.get_logger().info("status+1")
                    if self.status >= len(self.goal_xy_coords):
                        self.status = 0
                    self.now_goal = self.goal_xy_coords[self.status]
            else:
                self.pid_angle2()

        # === status 3: fusion target_id 물체를 따라가는 모드 ===
        elif (
            self.now_heading is not None
            and self.current_pos is not None
            and self.status == 3
        ):
            now_t = time.time()

            # 1) fusion 데이터에서 target_id 물체를 찾아 타깃 각 계산
            ok = self.compute_target_angle_from_fusion()

            # target_angle이 아직 None이면 로그 찍다가 죽지 않게 방어
            if not ok or self.target_angle is None:
                # 타겟이 안 보이거나 아직 각도 계산이 안 됐으면 일단 정지
                self.get_logger().debug(
                    f"[status 3] target_angle not ready (ok={ok}), stop motors."
                )
                self.stop_motor()
                return

            # 여기까지 왔으면 target_angle은 숫자라고 가정 가능
            self.get_logger().info(
                f"현재 target_angle = {self.target_angle:.2f}°"
                f" status = {self.status:.1f}°, "
            )

            # 2) 너무 가까워졌거나(=stop_status==1) 도킹 완료로 판단된 경우: 후진/정리
            if self.stop_status == 1:
                self.back_motor()
                self.stop_count += 1
                if self.stop_count == 4:
                    self.stop_count = 0
                    self.stop_status = 0
                    self.status += 1
                    if self.status >= len(self.goal_xy_coords):
                        self.status = 0  # 마지막 지점에서 유지(또는 0으로 루프)
                    self.now_goal = self.goal_xy_coords[self.status]
            else:
                # 3) 타깃 각도에 맞춰 PID 조타
                self.pid_angle()

        # === status 4: go_first_goal 기반 이동 ===
        if self.status == 4:
            if self.now_heading is None or self.current_pos is None or self.now_goal is None:
                if self.now_heading is None:
                    self.get_logger().debug("status 4: now_heading None")
                if self.current_pos is None:
                    self.get_logger().warn("status 4: current_pos not ready (no GPS yet)")
                if self.now_goal is None:
                    self.get_logger().debug("status 4: now_goal None")
                return

            e, n = self.current_pos
            self.target_angle = go_first_goal(
                self.now_goal,
                e, n,
                self.now_heading
            )

            if self.target_angle > 180:
                self.target_angle = (360 - self.target_angle) * -1

            self.get_logger().info(
                f"현재 target_angle = {self.target_angle:.2f}°"
                f" status = {self.status:.1f}°, "
            )

            if self.stop_status == 1:
                self.back_motor()
                self.get_logger().info("back_motor_on")

                self.stop_count += 1
                if self.stop_count == 2:
                    self.stop_count = 0
                    self.stop_status = 0
                    self.status += 1
                    self.get_logger().info("status+1")
                    if self.status >= len(self.goal_xy_coords):
                        self.status = 0
                    self.now_goal = self.goal_xy_coords[self.status]
            else:
                self.pid_angle2()

        # === status 5: dock_target_id 추종 후 1m 앞에서 선회 ===
        elif (
            self.now_heading is not None
            and self.current_pos is not None
            and self.status == 5
        ):
            dock_candidates = [
                o for o in self.last_objs
                if o["class_id"] == self.dock_target_id
            ]
            if not dock_candidates:
                self.get_logger().debug(
                    f"[status 5] no object with dock_target_id={self.dock_target_id}, stop motors."
                )
                self.stop_motor()
                return

            # 배 기준에서 가장 가까운 타겟 선택
            target = min(dock_candidates, key=lambda o: math.hypot(o["x"], o["y"]))
            tx = target["x"]
            ty = target["y"]

            distance = math.hypot(tx, ty)

            # # 1) 아직 도킹 거리(1m) 밖이면 → 타겟을 향해 조타하며 접근
            # if distance > self.dock_distance_m:
            #     angle_body_deg = math.degrees(math.atan2(ty, tx))
            #     self.target_angle = -angle_body_deg

            #     self.get_logger().info(
            #         f"[status 5] tracking dock target_id={self.dock_target_id}, "
            #         f"dist={distance:.2f} m, body_angle={angle_body_deg:.1f}°, "
            #         f"cmd_target_angle={self.target_angle:.1f}°"
            #     )

            #     # 타겟 쪽으로 조타하며 접근
            #     self.pid_angle()
            #     self.spin_start_time = None
            # 1) 아직 도킹 거리(1m) 밖이면 → 타겟을 향해 조타하며 접근
            if distance > self.dock_distance_m:
                angle_body_deg = math.degrees(math.atan2(ty, tx))  # 배 기준 타깃 각도

                # ===== 여기부터: CW/CCW에 따라 좌/우측으로 비껴서 접근 =====
                side_offset_deg = 30.0  # 옆으로 얼마나 비껴 들어갈지(필요하면 조절)

                if self.spin_direction == 1:
                    # CW로 돌 거면, 타깃의 왼쪽으로 접근
                    # -> 타깃을 약간 오른쪽(−각도)에 두도록 헤딩 설정
                    desired_body_deg = angle_body_deg - side_offset_deg
                else:
                    # CCW로 돌 거면, 타깃의 오른쪽으로 접근
                    # -> 타깃을 약간 왼쪽(+각도)에 두도록 헤딩 설정
                    desired_body_deg = angle_body_deg + side_offset_deg

                # PID 입력용 선수 기준 목표각 (0이 정면, +는 좌현, -는 우현)
                self.target_angle = -desired_body_deg

                self.get_logger().info(
                    f"[status 5] tracking dock target_id={self.dock_target_id}, "
                    f"dist={distance:.2f} m, "
                    f"raw_body_angle={angle_body_deg:.1f}°, "
                    f"desired_body_angle={desired_body_deg:.1f}°, "
                    f"cmd_target_angle={self.target_angle:.1f}° "
                    f"(spin_dir={self.spin_direction})"
                )

                # 타겟 옆으로 비껴 들어가며 접근
                self.pid_angle()
                self.spin_start_time = None





            else:
                if self.spin_start_time is None:
                    self.spin_start_time = time.time()
                    self.get_logger().info(
                        f"[status 5] dock distance reached (~{distance:.2f} m), "
                        f"start spinning direction={self.spin_direction} "
                        f"for {self.spin_duration_sec:.1f} s"
                    )

                elapsed = time.time() - self.spin_start_time

                if elapsed < self.spin_duration_sec:
                    spin_percent = 10.0
                    self.turn_angle(spin_percent, self.spin_direction)
                else:
                    self.stop_motor()
                    self.spin_start_time = None

                    self.status += 1
                    if self.status >= len(self.goal_xy_coords):
                        self.status = 0

                    self.now_goal = self.goal_xy_coords[self.status]
                    self.get_logger().info(
                        f"[status 5] spin finished, move to next status={self.status}"
                    )

        # === status 6: 3D LiDAR 위험도 기반 회피 + 목표 추종 ===
        elif (
            self.now_heading is not None
            and self.now_goal is not None
            and self.current_pos is not None
            and self.status == 6
        ):
            if self.angle_to_goal is None:
                self.get_logger().warn("[status 6] angle_to_goal not ready")
                return

            obstacles = self.last_objs if hasattr(self, "last_objs") else []

            try:
                desired_bearing_abs, err_signed = calculate_heading_by_risk(
                    obstacles=obstacles,
                    now_heading_abs=float(self.now_heading),
                    angle_to_goal_abs=float(self.angle_to_goal),
                    angle_increment=1.0,
                    base_clearance_deg=8.0,
                    clearance_gain=18.0,
                    min_consider_dist=0.0,
                    max_consider_dist=1.5,
                    distance_weight=10.0,
                    heading_weight=0.07,
                    goal_weight=0.0,
                )
                self.target_angle = err_signed

                self.get_logger().info(
                    f"status={self.status:.1f}°, "
                    f"target_angle={self.target_angle:.1f}° (obstacles={len(obstacles)})"
                )

            except Exception as ex:
                self.get_logger().warn(f"[status 6] risk solver error: {ex}; fallback to goal")
                if self.angle_to_goal is None:
                    self.get_logger().warn("[status 6] angle_to_goal None on fallback")
                    return
                self.target_angle = (self.angle_to_goal - self.now_heading + 540) % 360 - 180

            if self.stop_status == 1:
                self.back_motor()
                self.stop_count += 1
                if self.stop_count == 4:
                    self.stop_count = 0
                    self.stop_status = 0
                    self.status += 1
                    if self.status >= len(self.goal_xy_coords):
                        self.status = 0
                    self.now_goal = self.goal_xy_coords[self.status]
            else:
                self.pid_angle()

        # === status 7: 3D LiDAR 위험도 기반 회피 + 목표 추종(또 다른 구간) ===
        elif (
            self.now_heading is not None
            and self.now_goal is not None
            and self.current_pos is not None
            and self.status == 7
        ):
            if self.angle_to_goal is None:
                self.get_logger().warn("[status 7] angle_to_goal not ready")
                return

            obstacles = self.last_objs if hasattr(self, "last_objs") else []

            try:
                desired_bearing_abs, err_signed = calculate_heading_by_risk(
                    obstacles=obstacles,
                    now_heading_abs=float(self.now_heading),
                    angle_to_goal_abs=float(self.angle_to_goal),
                    angle_increment=1.0,
                    base_clearance_deg=8.0,
                    clearance_gain=18.0,
                    min_consider_dist=1.0,
                    max_consider_dist=1.5,
                    distance_weight=10.0,
                    heading_weight=0.07,
                    goal_weight=0.0,
                )
                self.target_angle = err_signed

                self.get_logger().info(
                    f"status={self.status:.1f}°, "
                    f"target_angle={self.target_angle:.1f}° (obstacles={len(obstacles)})"
                )

            except Exception as ex:
                self.get_logger().warn(f"[status 7] risk solver error: {ex}; fallback to goal")
                if self.angle_to_goal is None:
                    self.get_logger().warn("[status 7] angle_to_goal None on fallback")
                    return
                self.target_angle = (self.angle_to_goal - self.now_heading + 540) % 360 - 180

            if self.stop_status == 1:
                self.back_motor()
                self.stop_count += 1
                if self.stop_count == 4:
                    self.stop_count = 0
                    self.stop_status = 0
                    self.status += 1
                    if self.status >= len(self.goal_xy_coords):
                        self.status = 0
                    self.now_goal = self.goal_xy_coords[self.status]
            else:
                self.pid_angle()

    # -------------------- 보조 함수 --------------------
    def compute_target_angle_from_fusion(self) -> bool:
        """
        self.last_objs 에서 class_id == self.target_id 인 객체를 찾고,
        그 중 배에서 가장 가까운 물체를 골라 선수 기준 각도를 self.target_angle로 설정.
        성공하면 True, 못 찾으면 False 리턴.
        """
        if not self.last_objs:
            self.get_logger().debug("[fusion] no objects in last_objs")
            return False

        # target_id와 같은 class만 추리기
        target_candidates = [o for o in self.last_objs if o["class_id"] == self.target_id]
        if not target_candidates:
            self.get_logger().debug(f"[fusion] no object with class_id={self.target_id}")
            return False

        # 여러 개면 배 기준에서 가장 가까운 것을 선택
        target = min(target_candidates, key=lambda o: math.hypot(o["x"], o["y"]))

        tx = target["x"]
        ty = target["y"]

        # ★ 타겟을 봤으니, 지금 시간을 기록
        self.last_target_seen_time = time.time()

        # 좌표계 가정:
        #  - x > 0 : 선수 방향(앞)
        #  - y > 0 : 좌현(왼쪽), y < 0 : 우현(오른쪽)
        angle_body_deg = math.degrees(math.atan2(ty, tx))

        # PID 로직: 타깃이 왼쪽(+각도)이면 좌현으로 돌아야 하므로 부호 반대로
        self.target_angle = -angle_body_deg

        self.get_logger().info(
            f"[fusion] target_id={self.target_id}, "
            f"body_angle={angle_body_deg:.1f}°, "
            f"cmd_target_angle={self.target_angle:.1f}°"
        )
        return True

    # -------------------- 저속용 제어 --------------------
    def pid_angle(self) -> None:
        """선수 기준 목표각(self.target_angle)으로 PID 조타"""
        if self.target_angle is None:
            return

        self.angle_pid.setpoint = 0.0
        current_time = time.time()
        dt = current_time - self.last_time_pid
        if dt <= 0.0:
            dt = 1e-3
        motor_cmd = self.angle_pid(abs(self.target_angle), dt)
        self.last_time_pid = current_time

        if self.target_angle > 0:
            self.turn_angle(abs(motor_cmd), 1)  # CW
        else:
            self.turn_angle(abs(motor_cmd), 2)  # CCW

    def turn_angle(self, percentage: float, direction: int) -> None:
        left_req = ThrottlePulseWidth.Request()
        right_req = ThrottlePulseWidth.Request()

        if direction == 1:  # CW
            left_pw = 1550
            right_pw = 1550 - 4 * percentage
        else:               # CCW
            left_pw = 1550 - 4 * percentage
            right_pw = 1550

        left_pw = self.constrain(left_pw, self.MIN_PULSE_WIDTH, self.MAX_PULSE_WIDTH)
        right_pw = self.constrain(right_pw, self.MIN_PULSE_WIDTH, self.MAX_PULSE_WIDTH)

        left_req.pulse_width = int(left_pw)
        right_req.pulse_width = int(right_pw)
        self.set_throttle_handler_left.call_async(left_req)
        self.set_throttle_handler_right.call_async(right_req)

    # -------------------- 고속용 제어 --------------------
    def pid_angle2(self) -> None:
        """선수 기준 목표각(self.target_angle)으로 PID 조타"""
        if self.target_angle is None:
            return

        self.angle_pid.setpoint = 0.0
        current_time = time.time()
        dt = current_time - self.last_time_pid
        if dt <= 0.0:
            dt = 1e-3
        motor_cmd = self.angle_pid(abs(self.target_angle), dt)
        self.last_time_pid = current_time

        if self.target_angle > 0:
            self.turn_angle2(abs(motor_cmd), 1)  # CW
        else:
            self.turn_angle2(abs(motor_cmd), 2)  # CCW

    def turn_angle2(self, percentage: float, direction: int) -> None:
        left_req = ThrottlePulseWidth.Request()
        right_req = ThrottlePulseWidth.Request()

        if direction == 1:  # CW
            left_pw = 1600
            right_pw = 1600 - percentage
        else:               # CCW
            left_pw = 1600 - percentage
            right_pw = 1600

        left_pw = self.constrain(left_pw, self.MIN_PULSE_WIDTH, self.MAX_PULSE_WIDTH)
        right_pw = self.constrain(right_pw, self.MIN_PULSE_WIDTH, self.MAX_PULSE_WIDTH)

        left_req.pulse_width = int(left_pw)
        right_req.pulse_width = int(right_pw)
        self.set_throttle_handler_left.call_async(left_req)
        self.set_throttle_handler_right.call_async(right_req)

    # -------------------- 유틸 --------------------
    def gps_enu_converter(self, gnss) -> Tuple[float, float]:
        e, n, _ = pm.geodetic2enu(
            gnss[0], gnss[1], gnss[2], self.origin[0], self.origin[1], self.origin[2]
        )
        return e, n

    @staticmethod
    def angle_difference(a: float, b: float) -> float:
        return (b - a + 180.0) % 360.0 - 180.0

    @staticmethod
    def constrain(val: float, lo: float, hi: float) -> float:
        return max(min(val, hi), lo)

    def stop_motor(self) -> None:
        left = ThrottlePulseWidth.Request()
        right = ThrottlePulseWidth.Request()
        left.pulse_width = 1500
        right.pulse_width = 1500
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
