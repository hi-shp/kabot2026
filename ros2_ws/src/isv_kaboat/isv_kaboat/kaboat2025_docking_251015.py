#!/usr/bin/env python3
import math, time, re, traceback
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from vectornav_msgs.msg import CommonGroup            # IMU (yawpitchroll.x: yaw [deg])
from std_msgs.msg import Float32MultiArray            # /fusion/roi_center [u,v,class,track,x,y,z,range]*N
from mechaship_interfaces.srv import ThrottlePulseWidth

# ========================= 사용자 설정(라벨 한 줄로 타겟 변경) =========================
# 예: "red circle", "red triangle", "red cross",
#     "green circle", "green triangle", "green cross",
#     "blue circle", "blue triangle", "blue cross"
TARGET_LABEL_DEFAULT = "red circle"

# 라벨 → class_id 매핑(예시). 반드시 실제 모델의 class_id로 수정하세요.
# 안전하게 9개 모두 채워 두세요.
DEFAULT_CLASS_MAP = {
    "red circle":    10,
    "red triangle":  11,
    "red cross":     12,
    "green circle":  20,
    "green triangle":21,
    "green cross":   22,
    "blue circle":   30,
    "blue triangle": 31,
    "blue cross":    32,
}

# ========================= 유틸 =========================
def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

def deg_norm180(a):
    return (a + 180.0) % 360.0 - 180.0

def _normalize_label(s: str) -> str:
    # 공백 정리 + 소문자화
    s = s.strip().lower()
    s = re.sub(r"\s+", " ", s)
    return s

# ========================= 상태 =========================
class VisionState:
    SEARCH    = 0   # 타겟 미검출: 저속 전진하며 탐색
    APPROACH  = 1   # 충분히 멀다 → 중속 접근
    ALIGN     = 2   # 근접 → 정렬(조향 우선, 저속)
    DOCK      = 3   # 도킹 구간 → 초저속·미세조향
    HOLD      = 4   # 정지(도킹 완료)

# ====================== 메인 노드 =======================
class VisionDockNode(Node):
    def __init__(self):
        super().__init__(
            "vision_line_dock",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        self.get_logger().info("=== Vision-only Line Dock Controller (Single-Target, fused-data) ===")

        # ---------- 파라미터(ros2 param로 오버라이드 가능) ----------
        # 화면/시야
        self.screen_width  = int(self.declare_parameter("screen_width", 640).value)
        self.hfov_deg      = float(self.declare_parameter("hfov_deg", 90.0).value)     # 수평 FOV(도)
        self.vision_gain   = float(self.declare_parameter("vision_gain", 0.30).value)  # 각도 스케일

        # 거리 임계치( /fusion/roi_center 의 range 사용 )
        self.dist_approach = float(self.declare_parameter("approach_distance_m", 3.5).value)
        self.dist_align    = float(self.declare_parameter("align_distance_m",   1.6).value)
        self.dist_dock     = float(self.declare_parameter("dock_distance_m",    0.9).value)
        self.dist_hold     = float(self.declare_parameter("hold_distance_m",    0.7).value)  # 도킹 정지 임계

        # 속도(퍼센트 → PWM 매핑에 사용)
        self.speed_search   = float(self.declare_parameter("speed_search",   12.0).value)
        self.speed_approach = float(self.declare_parameter("speed_approach", 28.0).value)
        self.speed_align    = float(self.declare_parameter("speed_align",    16.0).value)
        self.speed_dock     = float(self.declare_parameter("speed_dock",      8.0).value)

        # 조향 게인
        self.steer_kp       = float(self.declare_parameter("steer_kp", 2.0).value)   # [PWM%/deg] 비율
        self.steer_max_pct  = float(self.declare_parameter("steer_max_pct", 45.0).value)  # 좌우 최대 디퍼런셜(%)

        # 비전 타임아웃(유실 시 SEARCH로 복귀)
        self.vision_timeout = float(self.declare_parameter("vision_timeout", 1.0).value)

        # 스러스터 PWM 범위(기존 인터페이스와 동일)
        self.MIN_PW = int(self.declare_parameter("min_pulse_width", 1395).value)
        self.MAX_PW = int(self.declare_parameter("max_pulse_width", 1900).value)
        self.STOP_PW = 1500

        # ---------- 단일 타겟 라벨/클래스 설정 ----------
        # 라벨 문자열은 코드 상수(TARGET_LABEL_DEFAULT) 또는 파라미터 target_label로 지정
        label_param = str(self.declare_parameter("target_label", TARGET_LABEL_DEFAULT).value)
        self.target_label = _normalize_label(label_param)

        # 매핑은 기본 dict + 개별 파라미터로 덮어쓰기
        # 각 라벨당 파라미터명 예: class_id_red_circle, class_id_green_triangle, ...
        self.class_map = dict(DEFAULT_CLASS_MAP)
        for label in list(self.class_map.keys()):
            pname = f"class_id_{label.replace(' ', '_')}"
            val = int(self.declare_parameter(pname, self.class_map[label]).value)
            self.class_map[label] = val

        # 필요 시 직접 class_id를 지정해 매핑을 무시할 수 있음 (-1이면 비활성)
        self.target_class_id_override = int(self.declare_parameter("target_class_id", -1).value)

        # 유효성 검사
        if self.target_label not in self.class_map:
            self.get_logger().warn(
                f"target_label='{self.target_label}' 가 등록되지 않았습니다. "
                f"가능한 값: {list(self.class_map.keys())}"
            )

        self.target_class_id = (
            self.target_class_id_override
            if self.target_class_id_override >= 0
            else self.class_map.get(self.target_label, -1)
        )
        self.get_logger().info(
            f"Target = '{self.target_label}'  → class_id = {self.target_class_id}"
        )

        # ---------- 내부 상태 ----------
        self.state = VisionState.SEARCH
        self.yaw_deg = None

        # 최근 검출(단일 타겟)
        self.last_seen_t = 0.0
        self.mid_u = None    # 화면상 목표 x(픽셀)
        self.min_range = None

        # dong 수신 패턴과 동일: 최근 물체 포즈/시간/속도 및 최신 객체 캐시
        self.last_pose = {}   # tid -> (x,y,z)
        self.last_time = {}   # tid -> t
        self.last_vel  = {}   # tid -> (vx,vy)
        self.last_objs = []   # [{u,v,class_id,track_id,x,y,z,range}, ...]

        # ---------- 통신 설정 (센서 QoS 사용) ----------
        self.create_subscription(CommonGroup, '/vectornav/raw/common', self.imu_cb, qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray, '/fusion/roi_center', self.fusion_cb, qos_profile_sensor_data)

        self.cli_left  = self.create_client(ThrottlePulseWidth, "/actuators/throttle_left/set_pulse_width")
        self.cli_right = self.create_client(ThrottlePulseWidth, "/actuators/throttle_right/set_pulse_width")
        self.cli_left.wait_for_service()
        self.cli_right.wait_for_service()

        self.create_timer(0.1, self.loop)  # 10 Hz

    # -------------------- 콜백 --------------------
    def imu_cb(self, msg: CommonGroup):
        yaw = float(msg.yawpitchroll.x)  # deg
        self.yaw_deg = yaw + 360.0 if yaw < 0 else yaw

    def fusion_cb(self, msg: Float32MultiArray):
        """
        data row = [u, v, class_id, track_id, x, y, z, range]
        u: 픽셀 x(가정), range: m(가정).
        dong 코드의 수신/파싱 패턴을 적용해 rows/cols를 해석하고,
        track_id 기반 속도를 추정하여 self.last_vel, self.last_objs를 갱신합니다.
        이후 타겟 class_id의 '가장 가까운 1개'를 선택해 mid_u/min_range를 설정합니다.
        """
        try:
            data = list(msg.data) if msg.data else []
            # 레이아웃에서 rows/cols 읽기 (dong 방식과 동일)
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
                self.get_logger().warn(f"/fusion/roi_center expects 8 cols but got {cols}")
                return

            # 단일 타겟 class_id 유효성
            if self.target_class_id < 0:
                return

            objs = []
            t_now = self.get_clock().now().nanoseconds * 1e-9

            for i in range(rows):
                base = i * 8
                try:
                    u, v, cid, tid, x, y, z, rng = data[base:base + 8]
                    tid = int(tid)
                    x, y, z = float(x), float(y), float(z)

                    # 속도 추정 (dong과 동일 로직)
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
                        "class_id": int(cid), "track_id": tid,
                        "x": x, "y": y, "z": z, "range": float(rng),
                    })
                except Exception as e:
                    self.get_logger().warn(f"[fusion] row {i} parse fail: {e}")
                    continue

            # 최신 관측 캐시
            self.last_objs = objs

            # --- 단일 타겟 선택(가장 가까운 1개) ---
            nearest = None
            for o in objs:
                if int(o["class_id"]) != self.target_class_id:
                    continue
                rng = float(o["range"])
                if rng <= 0.0 or not math.isfinite(rng):
                    continue
                if (nearest is None) or (rng < nearest["range"]):
                    nearest = {"u": float(o["u"]), "range": rng}

            # 목표 mid_u 및 거리 산정
            self.mid_u = None
            self.min_range = None
            if nearest is not None:
                self.mid_u = nearest["u"]
                self.min_range = nearest["range"]
                self.last_seen_t = time.time()

        except Exception as e:
            self.get_logger().error(f"[fusion_cb EXCEPTION] {e}\n{traceback.format_exc()}")

    # -------------------- 주기 루프 --------------------
    def loop(self):
        now = time.time()
        seen_recent = (now - self.last_seen_t) <= self.vision_timeout

        # 상태 전이
        if not seen_recent or self.mid_u is None or self.min_range is None:
            # 비전 유실 → 탐색
            self._set_state(VisionState.SEARCH)
        else:
            d = self.min_range
            if d > self.dist_approach:
                self._set_state(VisionState.APPROACH)
            elif d > self.dist_align:
                self._set_state(VisionState.APPROACH)
            elif d > self.dist_dock:
                self._set_state(VisionState.ALIGN)
            elif d > self.dist_hold:
                self._set_state(VisionState.DOCK)
            else:
                self._set_state(VisionState.HOLD)

        # 명령 산출
        if self.state == VisionState.SEARCH:
            # 화면 중앙으로 가정(조향=0), 저속 전진하며 표식 탐색
            steer_pct = 0.0
            speed_pct = self.speed_search

        elif self.state == VisionState.APPROACH:
            steer_pct = self._steer_from_mid()
            speed_pct = self.speed_approach

        elif self.state == VisionState.ALIGN:
            steer_pct = self._steer_from_mid()  # 근접 시 조향을 더 우선: 게인이 커도 됨
            speed_pct = self.speed_align

        elif self.state == VisionState.DOCK:
            steer_pct = self._steer_from_mid()
            speed_pct = self.speed_dock

        else:  # HOLD
            self._stop()
            self.get_logger().info("HOLD: docked.")
            return

        # 스러스터 구동(차동 구동: 전진 + 조향)
        self._drive(speed_pct, steer_pct)

        # 디버깅
        self.get_logger().info(
            f"target='{self.target_label}' (id={self.target_class_id}) "
            f"state={self._sname(self.state)} mid_u={None if self.mid_u is None else round(self.mid_u,1)} "
            f"range={None if self.min_range is None else round(self.min_range,2)}m "
            f"speed={round(speed_pct,1)}% steer={round(steer_pct,1)}%"
        )

    # -------------------- 조향 계산 --------------------
    def _steer_from_mid(self):
        """
        화면 중심 대비 mid_u 오차로 조향 비율[%] 계산.
        +값: 우현(오른쪽) 가정. 실제 선체 기준 필요 시 부호 반전하세요.
        """
        if self.mid_u is None:
            return 0.0
        half = self.screen_width / 2.0
        norm = (self.mid_u - half) / half                 # -1 ~ +1
        angle_err_deg = - norm * self.hfov_deg * self.vision_gain  # 각도 오차(도)
        steer = self.steer_kp * angle_err_deg
        return clamp(steer, -self.steer_max_pct, +self.steer_max_pct)

    # -------------------- 구동/정지 --------------------
    def _drive(self, speed_pct: float, steer_pct: float):
        """
        전진 + 차동조향을 PWM으로 매핑.
        - 기준: STOP_PW=1500
        - 전진: 1500 ~ MAX_PW (선형 매핑)
        - 좌/우 디퍼런셜: steer_pct로 좌우에 ± 가산
        """
        speed_pct = clamp(speed_pct, 0.0, 100.0)
        steer_pct = clamp(steer_pct, -100.0, 100.0)

        # 속도 매핑: 1500 → 정지, 1900 근처 → 최대전진. 여유있게 1500~1880 사용
        base = self.STOP_PW + int((1880 - self.STOP_PW) * (speed_pct / 100.0))

        # 조향 매핑: 최대 ±(기본폭의 45%) 만큼 좌우 차등
        steer_span = int((base - self.STOP_PW) * (abs(steer_pct) / 100.0))
        if steer_pct >= 0:   # 우현: 좌를 더 세게, 우를 덜 세게
            left  = base + steer_span
            right = base - steer_span
        else:                # 좌현: 우를 더 세게, 좌를 덜 세게
            left  = base - steer_span
            right = base + steer_span

        left  = clamp(left,  self.MIN_PW, self.MAX_PW)
        right = clamp(right, self.MIN_PW, self.MAX_PW)
        self._set_thrusters(left, right)

    def _stop(self):
        self._set_thrusters(self.STOP_PW, self.STOP_PW)

    def _set_thrusters(self, left_pw: int, right_pw: int):
        req = ThrottlePulseWidth.Request()
        req.pulse_width = int(left_pw)
        self.cli_left.call_async(req)

        req = ThrottlePulseWidth.Request()
        req.pulse_width = int(right_pw)
        self.cli_right.call_async(req)

    # -------------------- 상태 관리 --------------------
    def _set_state(self, s):
        if s != self.state:
            self.get_logger().info(f"[STATE] {self._sname(self.state)} → {self._sname(s)}")
            self.state = s

    @staticmethod
    def _sname(s):
        return {
            VisionState.SEARCH: "SEARCH",
            VisionState.APPROACH: "APPROACH",
            VisionState.ALIGN: "ALIGN",
            VisionState.DOCK: "DOCK",
            VisionState.HOLD: "HOLD",
        }[s]

# ====================== 엔트리포인트 ======================
def main(args=None):
    rclpy.init(args=args)
    node = VisionDockNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
