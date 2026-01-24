#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data

from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from vision_msgs.msg import Detection2DArray


def norm_text(s: str) -> str:
    return (s or "").strip().lower()


class Course2(Node):
    def __init__(self):
        super().__init__("course2")

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter("detections_topic", "/detections")
        self.declare_parameter("imu_topic", "/imu")

        self.declare_parameter("key_topic", "/actuator/key/degree")
        self.declare_parameter("thruster_topic", "/actuator/thruster")  # 너 환경에 맞게 수정

        self.declare_parameter("tracking_target", "buoy")
        self.declare_parameter("screen_width", 640.0)
        self.declare_parameter("angle_conversion_factor", 90.0)

        # 단순 조건: -30도 이하
        self.declare_parameter("trigger_angle_deg", -30.0)

        # 출력값
        self.declare_parameter("key_neutral_deg", 90.0)
        self.declare_parameter("key_circle_deg", 60.0)
        self.declare_parameter("thruster_straight", 20.0)
        self.declare_parameter("thruster_circle", 20.0)

        # IMU 0도 2번 통과
        self.declare_parameter("yaw_zero_tol", 5.0)
        self.declare_parameter("zero_pass_need", 2)

        self.declare_parameter("control_hz", 10.0)

        # -----------------------------
        # Load params
        # -----------------------------
        self.detections_topic = self.get_parameter("detections_topic").value
        self.imu_topic = self.get_parameter("imu_topic").value
        self.key_topic = self.get_parameter("key_topic").value
        self.thruster_topic = self.get_parameter("thruster_topic").value

        self.tracking_target = self.get_parameter("tracking_target").value
        self.screen_width = float(self.get_parameter("screen_width").value)
        self.angle_conversion_factor = float(self.get_parameter("angle_conversion_factor").value)

        self.trigger_angle_deg = float(self.get_parameter("trigger_angle_deg").value)

        self.key_neutral_deg = float(self.get_parameter("key_neutral_deg").value)
        self.key_circle_deg = float(self.get_parameter("key_circle_deg").value)
        self.thruster_straight = float(self.get_parameter("thruster_straight").value)
        self.thruster_circle = float(self.get_parameter("thruster_circle").value)

        self.yaw_zero_tol = float(self.get_parameter("yaw_zero_tol").value)
        self.zero_pass_need = int(self.get_parameter("zero_pass_need").value)

        control_hz = float(self.get_parameter("control_hz").value)
        self.dt = 1.0 / max(control_hz, 1e-6)

        # -----------------------------
        # Publishers
        # -----------------------------
        self.key_pub = self.create_publisher(Float64, self.key_topic, 10)
        self.thruster_pub = self.create_publisher(Float64, self.thruster_topic, 10)

        # -----------------------------
        # Subscribers
        # -----------------------------
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            self.detections_topic,
            self.detection_cb,
            qos_profile_sensor_data,
        )

        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.imu_sub = self.create_subscription(Imu, self.imu_topic, self.imu_cb, imu_qos)

        # -----------------------------
        # State
        # -----------------------------
        self.STATE_STRAIGHT = 0
        self.STATE_CIRCLE = 1
        self.STATE_DONE = 2
        self.state = self.STATE_STRAIGHT

        # Vision
        self.latest_vision_angle = None
        self.target_seen = False  # 타겟이 인식된 프레임인지

        # IMU yaw
        self.yaw_deg = None
        self.zero_pass_count = 0
        self._was_in_zero_zone = False

        # Timer
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(
            f"[course2] start | trigger: vision_angle <= {self.trigger_angle_deg} deg | "
            f"det:{self.detections_topic} imu:{self.imu_topic}"
        )

    # -----------------------------
    # IMU yaw
    # -----------------------------
    def normalize_180(self, deg: float) -> float:
        return (deg + 180.0) % 360.0 - 180.0

    def quat_to_yaw_deg(self, q) -> float:
        x, y, z, w = q.x, q.y, q.z, q.w
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return math.degrees(yaw)

    def imu_cb(self, msg: Imu):
        q = msg.orientation
        if q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 0.0:
            return

        yaw = self.normalize_180(self.quat_to_yaw_deg(q))
        self.yaw_deg = yaw

        # 원운동 상태에서만 0도 통과 카운트
        if self.state != self.STATE_CIRCLE:
            self._was_in_zero_zone = (abs(yaw) <= self.yaw_zero_tol)
            return

        in_zero_zone = abs(yaw) <= self.yaw_zero_tol

        # 엣지 검출: 밖 -> 안
        if (not self._was_in_zero_zone) and in_zero_zone:
            self.zero_pass_count += 1
            self.get_logger().info(f"[IMU] ZERO PASS {self.zero_pass_count}/{self.zero_pass_need} (yaw={yaw:+.2f})")

            if self.zero_pass_count >= self.zero_pass_need:
                self.get_logger().info("[IMU] ✅ circle done -> DONE")
                self.state = self.STATE_DONE

        self._was_in_zero_zone = in_zero_zone

    # -----------------------------
    # Detection
    # -----------------------------
    def _extract_classid_score(self, det):
        # 네 프로젝트 구현으로 교체 가능
        try:
            if len(det.results) == 0:
                return None, None
            r0 = det.results[0]
            class_id = int(r0.hypothesis.class_id)
            score = float(r0.hypothesis.score)
            return class_id, score
        except Exception:
            return None, None

    def get_object_name(self, class_id: int) -> str:
        # 네 label map으로 교체
        return str(class_id)

    def detection_cb(self, msg: Detection2DArray):
        target = None
        tgt_norm = norm_text(self.tracking_target)

        for det in msg.detections:
            class_id, score = self._extract_classid_score(det)
            if class_id is None:
                continue
            name = self.get_object_name(class_id)
            if norm_text(name) == tgt_norm:
                target = det
                break

        if target is None:
            self.target_seen = False
            self.latest_vision_angle = None
            return

        self.target_seen = True

        cx = float(target.bbox.center.position.x)
        angle_offset = ((cx - (self.screen_width / 2.0)) / (self.screen_width / 2.0)) * self.angle_conversion_factor
        vision_target_angle = -angle_offset  # 왼쪽 -, 오른쪽 +

        self.latest_vision_angle = vision_target_angle

    # -----------------------------
    # Control
    # -----------------------------
    def publish_key(self, deg: float):
        m = Float64()
        m.data = float(deg)
        self.key_pub.publish(m)

    def publish_thruster(self, val: float):
        m = Float64()
        m.data = float(val)
        self.thruster_pub.publish(m)

    def enter_circle_mode(self):
        self.state = self.STATE_CIRCLE
        self.zero_pass_count = 0
        # 시작 순간 0도 근처면 즉시 카운트되는 걸 막기 위한 초기화
        if self.yaw_deg is not None:
            self._was_in_zero_zone = (abs(self.yaw_deg) <= self.yaw_zero_tol)
        else:
            self._was_in_zero_zone = False
        self.get_logger().info("[STATE] -> CIRCLE")

    def control_loop(self):
        # DONE이면 1회 처리 후 STRAIGHT 복귀
        if self.state == self.STATE_DONE:
            self.publish_key(self.key_neutral_deg)
            self.publish_thruster(self.thruster_straight)
            self.state = self.STATE_STRAIGHT
            return

        # --- 상태 전이 로직 (단순) ---
        # 타겟이 보이고, vision_angle <= -30이면 CIRCLE
        if self.state == self.STATE_STRAIGHT:
            if self.target_seen and (self.latest_vision_angle is not None) and (self.latest_vision_angle <= self.trigger_angle_deg):
                self.enter_circle_mode()

        # CIRCLE 상태 중에는 IMU가 DONE으로 바꿀 때까지 계속 원운동
        # (vision_angle이 다시 커져도 원운동 유지하고 싶지 않으면 아래 조건을 추가하면 됨)

        # --- 출력 ---
        if self.state == self.STATE_STRAIGHT:
            self.publish_key(self.key_neutral_deg)
            self.publish_thruster(self.thruster_straight)

        elif self.state == self.STATE_CIRCLE:
            self.publish_key(self.key_circle_deg)
            self.publish_thruster(self.thruster_circle)


def main(args=None):
    rclpy.init(args=args)
    node = Course2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
