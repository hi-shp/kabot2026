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
from isv_kaboat.path_follower2 import PathFollower2
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
        self.now_heading: float | None = None
        self.current_pos: Tuple[float, float] | None = None
        self.target_angle: float | None = None       # 조타 PID 목표(선수 기준 ±180)
        self.angle_to_goal: float | None = None

        # === status 2 용 설정 ===
        # ★ fusion class_id 중에서 쫓아갈 타깃 id
        self.target_id: int = 0
        self.dock_ok_distance_m: float = 0.5
        # ★ 마지막으로 타겟(class_id == target_id)을 본 시각 (sec)
        self.last_target_seen_time: float | None = None



        self.last_time_pid = time.time()

        # PID: 각도 오차 제어
        self.angle_pid = PID(24.0 / 3, 0.0, 0.0)   # Kp=8, Ki,Kd=0(초기값)
        self.angle_pid.output_limits = (-400, 400)

        # 목표점 리스트(ENU로 변환해 사용)
        self.origin = [35.2316955, 129.0827215, 0.0]  # ENU 원점
        self.goal_gps_coords = [

            (35.2316194, 129.0827744),  # 추종 후 정박할 좌표 
            (35.2316836, 129.0827230),  # 도킹 전 정지 포인트
            (35.2329450, 129.0799100),  # 회피 전 정지 포인트
            (35.2329500, 129.0799200),  # 최종 골

        ]
        self.goal_xy_coords: List[Tuple[float, float]] = []
        for lat, lon in self.goal_gps_coords:
            e, n = self.gps_enu_converter((lat, lon, 0.0))
            self.goal_xy_coords.append((e, n))

        # 시작 status###################################################################################################################################################################
            
        self.status = 0

        # 0 경로 추정    1  도킹  전 정지포인트           2 도킹           3  경로          4 회피       #######################################################################################
        self.now_goal = self.goal_xy_coords[0]

        # 모터 펄스 폭 제한
        self.MIN_PULSE_WIDTH = 1350
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
            arrive_radius_m=0.3,
        )
        
        self.path_follow2 = PathFollower2(
            origin_latlonalt=tuple(self.origin),
            arrive_radius_m=0.3,
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
        if distance <= 0.3:
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


        # === status 0: PathFollower 기반 경로 추종 + 정지 시퀀스 ===
        if (
            self.now_heading is not None
            and self.now_goal is not None
            and self.current_pos is not None
            and self.status == 0
        ):
            e, n = self.current_pos

            # 1) 먼저 PathFollower가 끝났는지 확인
            if self.path_follow.done():
                self.get_logger().info("All waypoints reached in status 0.")

                if self.angle_to_goal is None:
                    self.get_logger().warn("[status 0] angle_to_goal None, skip")
                    return

                # ✅ PathFollower 종료 → now_goal 방향으로 조향각 설정
                self.target_angle = (
                    self.angle_to_goal - self.now_heading + 540.0
                ) % 360.0 - 180.0

                self.get_logger().info(
                    f"[status 0] path done → goal heading target_angle = {self.target_angle:.2f}°"
                )

            else:
                # 2) 아직 path follower 진행 중일 때만 update() 호출
                try:
                    # PathFollower가 주는 상대각(±180°) 사용
                    self.target_angle = self.path_follow.update(e, n, self.now_heading)
                    self.get_logger().info(
                        f"[status 0] path follow: target_angle = {self.target_angle:.2f}°"
                    )
                except Exception as ex:
                    self.get_logger().warn(f"PathFollower error: {ex}; fallback to goal")
                    if self.angle_to_goal is None:
                        self.get_logger().warn("[status 0] angle_to_goal None, skip")
                        return
                    self.target_angle = (
                        self.angle_to_goal - self.now_heading + 540.0
                    ) % 360.0 - 180.0

            # === stop 시퀀스 처리 ===
            if self.stop_status == 1:
                # stop 시작 시간 기록이 없으면 지금 시각 저장
                if not hasattr(self, "stop_start_time") or self.stop_start_time is None:
                    self.stop_start_time = time.time()
                    self.get_logger().info("Stop sequence started")

                elapsed = time.time() - self.stop_start_time

                if elapsed < 1.0:
                    # 1단계: 1초 동안 역추진
                    self.back_motor()
                    self.get_logger().info("back_motor_on")

                elif elapsed < 6.0:
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




        # === status 1: go_first_goal 기반 이동 ===
        elif self.status == 1:
            if self.now_heading is None or self.current_pos is None or self.now_goal is None:
                if self.now_heading is None:
                    self.get_logger().debug("status 1: now_heading None")
                if self.current_pos is None:
                    self.get_logger().warn("status 1: current_pos not ready (no GPS yet)")
                if self.now_goal is None:
                    self.get_logger().debug("status 1: now_goal None")
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
                if self.stop_count == 1:
                    self.stop_count = 0
                    self.stop_status = 0
                    self.status += 1
                    self.get_logger().info("status+1")
                    if self.status >= len(self.goal_xy_coords):
                        self.status = 0
                    self.now_goal = self.goal_xy_coords[self.status]
            else:
                self.pid_angle2()





        # === status 2: fusion target_id 물체를 따라가는 모드 ===
        elif (
            self.now_heading is not None
            and self.current_pos is not None
            and self.status == 2
        ):
            # 1) fusion 객체 목록 가져오기
            objs = getattr(self, "last_objs", [])
            if not objs:
                self.get_logger().debug("[status 2] no objects in last_objs, stop motors.")
                self.stop_motor()
                return

            # 2) target_id 와 같은 class_id 만 추리기
            target_candidates = [o for o in objs if o["class_id"] == self.target_id]
            if not target_candidates:
                self.get_logger().debug(
                    f"[status 2] no object with class_id={self.target_id}, stop motors."
                )
                self.stop_motor()
                return

            # 3) 여러 개면 배 기준에서 가장 가까운 것 선택 (range 최소)
            target = min(target_candidates, key=lambda o: o["range"])

            # 픽셀 좌표 / 거리 꺼내기
            u = float(target["u"])
            v = float(target["v"])          # 지금은 안 쓰지만, 나중에 pitch 제어 등에 쓸 수 있음
            dist = float(target["range"])   # 배 기준 거리(m)로 사용



            # --- 여기서부터 u 를 이용해서 self.target_angle 계산 ---

            # 카메라 해상도 (예: 640, 필요하면 파라미터/멤버로 빼도 됨)
            screen_width = 640  

            # 기존 코드:
            # angle_offset = (
            #     ((self.target_x - (screen_width / 2)) / (screen_width / 2)) * 90 * 0.3
            # )
            # self.target_angle = -angle_offset

            # 위 로직을 그대로 u에 적용
            angle_offset = (
                ((u - (screen_width / 2.0)) / (screen_width / 2.0)) * 90.0 
            )
            self.target_angle = -angle_offset  # 화면 중심 기준으로 좌우 각도

            self.get_logger().info(
                f"[status 2] fusion target_id={self.target_id}, "
                f"cmd_target_angle={self.target_angle:.1f}° "
                f"dist={dist:.2f} m"
            )

            # target_angle 이 숫자인 상태라고 가정 가능하지만,
            # 방어적으로 한 번 더 체크
            if self.target_angle is None:
                self.get_logger().debug("[status 2] target_angle is None, stop motors.")
                self.stop_motor()
                return

            # ★ 도킹 거리 안에 들어왔으면 stop_status = 1 로 플래그 세팅
            dock_dist = self.dock_ok_distance_m  # 또는 별도 상수 사용 가능
            if dist <= dock_dist:
                if self.stop_status == 0:
                    self.get_logger().info(
                        f"[status 2] target within dock distance "
                        f"({dist:.2f} m <= {dock_dist:.2f} m), set stop_status = 1"
                    )
                self.stop_status = 1


            # 4) 너무 가까워졌거나(=stop_status==1) 도킹 완료로 판단된 경우: 후진/정리
            if self.stop_status == 1:
                self.back_motor()
                self.stop_count += 1
                if self.stop_count == 4:
                    self.stop_count = 0
                    self.stop_status = 0
                    self.status += 1
                    if self.status >= len(self.goal_xy_coords):
                        self.status = 0  # 마지막 지점에서 루프
                    self.now_goal = self.goal_xy_coords[self.status]
                    self.get_logger().info(
                        f"[status 2] dock finish, move to next status={self.status}"
                    )
            else:
                # 5) 타깃 각도에 맞춰 PID 조타
                self.pid_angle()


        # === status 3: PathFollower 기반 경로 추종 + 정지 시퀀스 ===
        elif (
            self.now_heading is not None
            and self.now_goal is not None
            and self.current_pos is not None
            and self.status == 3
        ):
            e, n = self.current_pos

            # 1) 먼저 PathFollower가 끝났는지 확인
            if self.path_follow2.done():
                self.get_logger().info("All waypoints reached in status 3.")

                if self.angle_to_goal is None:
                    self.get_logger().warn("[status 3] angle_to_goal None, skip")
                    return

                # ✅ PathFollower 종료 → now_goal 방향으로 조향각 설정
                self.target_angle = (
                    self.angle_to_goal - self.now_heading + 540.0
                ) % 360.0 - 180.0

                self.get_logger().info(
                    f"[status 3] path done → goal heading target_angle = {self.target_angle:.2f}°"
                )

            else:
                # 2) 아직 path follower 진행 중일 때만 update() 호출
                try:
                    # PathFollower가 주는 상대각(±180°) 사용
                    self.target_angle = self.path_follow2.update(e, n, self.now_heading)
                    self.get_logger().info(
                        f"[status 3] path follow: target_angle = {self.target_angle:.2f}°"
                    )
                except Exception as ex:
                    self.get_logger().warn(f"PathFollower error: {ex}; fallback to goal")
                    if self.angle_to_goal is None:
                        self.get_logger().warn("[status 3] angle_to_goal None, skip")
                        return
                    self.target_angle = (
                        self.angle_to_goal - self.now_heading + 540.0
                    ) % 360.0 - 180.0

            # === stop 시퀀스 처리 ===
            if self.stop_status == 1:
                # stop 시작 시간 기록이 없으면 지금 시각 저장
                if not hasattr(self, "stop_start_time") or self.stop_start_time is None:
                    self.stop_start_time = time.time()
                    self.get_logger().info("Stop sequence started")

                elapsed = time.time() - self.stop_start_time

                if elapsed < 1.0:
                    # 1단계: 1초 동안 역추진
                    self.back_motor()
                    self.get_logger().info("back_motor_on")

                elif elapsed < 2.0:
                    # 2단계: 모터 정지 후 추가로 1초간 대기
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
      
        # === status4: 3D LiDAR 위험도 기반 회피 + 목표 추종 ===
        elif (
            self.now_heading is not None
            and self.now_goal is not None
            and self.current_pos is not None
            and self.status == 4
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
                if self.stop_count == 1:
                    self.stop_count = 0
                    self.stop_status = 0
                    self.status += 1
                    if self.status >= len(self.goal_xy_coords):
                        self.status = 0
                    self.now_goal = self.goal_xy_coords[self.status]
            else:
                self.pid_angle()







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
            left_pw = 1600
            right_pw = 1600 - 2 * percentage
        else:               # CCW
            left_pw = 1600 - 2 * percentage
            right_pw = 1600

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
            left_pw = 1750
            right_pw = 1750 - percentage
        else:               # CCW
            left_pw = 1750 - percentage
            right_pw = 1750

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
