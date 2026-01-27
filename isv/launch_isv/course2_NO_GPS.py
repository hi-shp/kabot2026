import os
import yaml
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from tf_transformations import euler_from_quaternion
from mechaship_interfaces.msg import RgbwLedColor

def constrain(v, lo, hi):
    if math.isnan(v): return lo + (hi - lo) / 2.0
    return lo if v < lo else hi if v > hi else v

def norm_text(s: str) -> str:
    
    """대소문자/연속 공백 무시 비교용"""
    return " ".join(str(s).strip().lower().split())


class course2(Node):
    def __init__(self):
        super().__init__("course2")

        # ✅ 같은 폴더의 isv_params.yaml 자동 로드
        this_dir = os.path.dirname(os.path.abspath(__file__))
        self.yaml_path = os.path.join(this_dir, "isv_params.yaml")
        self._load_params_from_yaml()
        # ---------------- Publishers ----------------
        self.key_publisher = self.create_publisher(Float64, "/actuator/key/degree", 10)
        self.thruster_publisher = self.create_publisher(Float64, "/actuator/thruster/percentage", 10)
        self.led_pub = self.create_publisher(RgbwLedColor, "/actuator/rgbwled/color", 10)
        # ---------------- Subscribers ----------------
        self.imu_sub = self.create_subscription(Imu, "/imu", self.imu_callback, qos_profile_sensor_data)
        self.detection_subscriber = self.create_subscription(Detection2DArray, "/detections", self.detection_callback, qos_profile_sensor_data)
        # ---------------- State / Defaults ----------------
        self.latest_detection_msg = None
        self.latest_det_msg = None
        self.is_hoping_started = False
        self.last_target_time = None
        self.current_servo_deg = self.servo_neutral_deg
        self.turn_step_deg = 20.0  # 호핑에서 사용할 step
        self.current_yaw_deg = 0.0
        self.imu_target_angle = 0.0
        # yaw 0도 2번 카운트 
        self.yaw_zero_count = 0
        self.yaw_zero_latched = False
        self.yaw_zero_tol = 0
        # ✅ 순차 진행 상태
        self.phase = "HOPING"  # HOPING -> DETECTION -> DONE
        # LED 기본값
        self.led_default_white = 20
        self.led_on_brightness = 80
        self.current_led = self.make_led(white=self.led_default_white)
        self.publish_led(self.current_led)
        # ---------------- Timer -----------------------------------------------
        # ✅ timer 등 설정
        self.create_timer(self.timer_period, self.timer_callback)
        # ---- info -------------------------------------------------------------
        self.get_logger().info("Loaded params (from YAML):")
        self.get_logger().info(f"- yaml_path: {self.yaml_path}")
        self.get_logger().info(f"- hoping_target: '{self.hoping_target}'")
        self.get_logger().info(f"- detection_target: '{self.detection_target}'")
        self.get_logger().info(f"- labels(count): {len(self.available_objects)}")
        self.get_logger().info(f"- vision: screen_width={self.screen_width}, angle_conversion_factor={self.angle_conversion_factor}")
        self.get_logger().info(f"- phase: {self.phase}")
        self.get_logger().info(f"[START] hoping_target='{self.hoping_target}' | detection_target='{self.detection_target}'")
        #------------- YAML Load -----------------------------------------------
    def _load_params_from_yaml(self):
        script_dir = os.path.dirname(os.path.realpath(__file__))
        yaml_path = os.path.join(script_dir, "isv_params.yaml")
        with open(yaml_path, "r") as file:
            params = yaml.safe_load(file)

        self.available_objects = []
        self.hoping_target = ""
        self.detection_target = ""
        vision = params.get("vision", {})
        self.available_objects = list(vision.get("available_objects", []))
        self.hoping_target = str(vision.get("hoping_target", "")).strip()
        self.detection_target = str(vision.get("detection_target", "")).strip()
        self.screen_width = int(vision.get("screen_width", self.screen_width))
        self.angle_conversion_factor = float(vision.get("angle_conversion_factor", self.angle_conversion_factor))
        servo = params.get("servo", {})
        self.servo_min_deg = float(servo.get("min_deg", 45.0))
        self.servo_max_deg = float(servo.get("max_deg", 135.0))
        self.servo_neutral_deg = float(servo.get("neutral_deg", 90.0))
        node_settings = params.get("node_settings", {})
        self.timer_period= float(node_settings.get("timer_priod", 0.5))
        self.default_thruster = float(self.thruster_cfg.get("state1", 10.0))

    # ---------------- LED Utils ----------------
    def make_led(self, red=0, green=0, blue=0, white=0) -> RgbwLedColor:
        msg = RgbwLedColor()
        msg.red = int(constrain(red, 0, 255))
        msg.green = int(constrain(green, 0, 255))
        msg.blue = int(constrain(blue, 0, 255))
        msg.white = int(constrain(white, 0, 255))
        return msg

    def publish_led(self, msg: RgbwLedColor):
        self.current_led = msg
        self.led_pub.publish(msg)
    
    def led_by_target_name(self, name: str) -> RgbwLedColor:
        """라벨 이름으로 LED 메시지 생성"""
        n = norm_text(name)
        b = int(self.led_on_brightness)
        if n.startswith("blue "):
            return self.make_led(blue=b)
        if n.startswith("green "):
            return self.make_led(green=b)
        if n.startswith("red "):
            return self.make_led(red=b)
        return self.make_led(white=self.led_default_white)
    # ---------------- IMU (yaw only) ----------------
    def normalize_180(self, angle_deg: float) -> float:
        a = (angle_deg + 180.0) % 360.0 - 180.0
    
    def imu_callback(self, msg: Imu):
        yaw_deg = self.normalize_180(euler_from_quaternion(msg.orientation))
        self.current_yaw_deg = yaw_deg

        # ✅ HOPING 단계에서만 "yaw 0도 2번"을 감시해서 DETECTION 단계로 전환
        if self.phase == "HOPING":
            if abs(yaw_deg) <= self.yaw_zero_tol:
                if not self.yaw_zero_latched:
                    self.yaw_zero_count += 1
                    self.yaw_zero_latched = True
                    self.get_logger().info(f"[IMU] yaw≈0 detected count={self.yaw_zero_count}")

                    if self.yaw_zero_count >= 2:
                        self.phase = "DETECTION"
                        self.imu_target_angle = 0.0  # 요구: imu 목표각 0
                        self.get_logger().info("[PHASE] HOPING -> DETECTION (imu_target_angle=0.0)")

                        # 호핑 조향은 중립으로 정리
                        self.current_servo_deg = self.servo_neutral_deg
                        self.key_publisher.publish(Float64(data=float(self.current_servo_deg)))
            else:
                self.yaw_zero_latched = False

        # ✅ 목표각이 -90이 된 순간부터: (현재 - 목표) 오차를 중립 90에서 "빼서" 서보 제어
        if float(self.imu_target_angle) == -90.0:
            self.control_to_target_yaw_minus90()

    def control_to_target_yaw_minus90(self):
        current = float(self.current_yaw_deg)
        current = -current
        target = float(self.imu_target_angle)  # -90.0

        # ✅ 사용자 요청: 오차 = 현재 - 목표
        #    (단, wrap으로 -180~180 범위로 안전하게)
        error = current - target

        # ✅ 사용자 요청: 중립 90에서 오차를 뺌
        servo_cmd = self.servo_neutral_deg - error

        # 서보 범위 제한
        servo_cmd = constrain(servo_cmd, self.servo_min_deg, self.servo_max_deg)

        self.current_servo_deg = servo_cmd
        self.key_publisher.publish(Float64(data=float(servo_cmd)))
     
        self.get_logger().info(
                f"[STEER(-90)] yaw={current:.1f} target={target:.1f} err(cur-tgt)={error:.1f} -> servo={servo_cmd:.1f}"
            )

    # ---------------- Detection ----------------
    def detection_callback(self, msg: Detection2DArray):
        self.latest_det_msg = msg
        self.latest_detection_msg = msg

    def get_object_name(self, class_id: int) -> str:
        if 0 <= class_id < len(self.available_objects):
            return str(self.available_objects[class_id])
        return "unknown"

    def _extract_classid_score(self, det):
        if not det.results:
            return None, None

        res0 = det.results[0]
        class_id = None
        score = None

        if hasattr(res0, "hypothesis") and res0.hypothesis is not None:
            hyp = res0.hypothesis
            if hasattr(hyp, "class_id"):
                try:
                    class_id = int(hyp.class_id)
                except Exception:
                    class_id = None
            if hasattr(hyp, "score"):
                try:
                    score = float(hyp.score)
                except Exception:
                    score = None

        if score is None and hasattr(res0, "score"):
            try:
                score = float(res0.score)
            except Exception:
                score = None

        return class_id, score

    # ✅ HOPING 단계에서만 호출될 함수
    def hoping_detection(self, msg: Detection2DArray):
        target = None
        tgt_norm = norm_text(self.hoping_target)

        for det in msg.detections:
            class_id, score = self._extract_classid_score(det)
            if class_id is None:
                continue
            name = self.get_object_name(class_id)
            if norm_text(name) == tgt_norm:
                target = det
                break

        # 타겟 못 찾으면 직진(중립)
        if target is None:
            self.current_servo_deg = self.servo_neutral_deg
            self.key_publisher.publish(Float64(data=float(self.current_servo_deg)))
            return

        # 처음 인식되는 순간에만 start_hoping
        if not self.is_hoping_started:
            self.is_hoping_started = True
            self.get_logger().info("start_hoping")

        # 카메라 중앙=0도 기준 각도 추정
        cx = float(target.bbox.center.position.x)
        vision_target_angle = ((cx - (self.screen_width / 2.0)) / (self.screen_width / 2.0)) * self.angle_conversion_factor

        # ✅ hoping_target의 각도 로그 출력
        self.get_logger().info(
            f"[HOPING] target='{self.hoping_target}' cx={cx:.1f} -> angle={vision_target_angle:.2f} deg"
        )

        # ✅ 좌로 -30도 이하이면 20도씩 꺾고, 아니면 직진
        if vision_target_angle <= -30.0:
            self.current_servo_deg = 70.0
        else:
            self.current_servo_deg = self.servo_neutral_deg

        self.current_servo_deg = constrain(self.current_servo_deg, self.servo_min_deg, self.servo_max_deg)
        self.key_publisher.publish(Float64(data=float(self.current_servo_deg)))

    # ✅ DETECTION 단계에서만 호출될 함수
    def target_detection(self, msg: Detection2DArray):
        if abs(float(self.imu_target_angle)) > 1e-6:
            return False
        tgt_norm = norm_text(self.detection_target)
        for det in msg.detections:
            class_id, score = self._extract_classid_score(det)
            if class_id is None:
                continue
            name = self.get_object_name(class_id)
            if norm_text(name) == tgt_norm:
                roi_center_x = float(det.bbox.center.position.x)
                detection_availible_error = 10.0
                if abs(roi_center_x - (self.screen_width / 2.0)) <= detection_availible_error:
                    self.imu_target_angle = -90.0
                    led_msg = self.led_by_target_name(name)
                    self.publish_led(led_msg)
                    self.get_logger().info(
                        f"[DETECTED & CENTERED] {name} "
                        f"(roi_x={roi_center_x:.1f}) -> imu_target_angle = {self.imu_target_angle}"
                    )
                    return True
                else:
                    self.get_logger().info(
                        f"[DETECTED but NOT CENTERED] {name} "
                        f"(roi_x={roi_center_x:.1f}, center={(self.screen_width / 2.0)})")
            return False

    # ---------------- Timer ----------------
    def timer_callback(self):
        self.cmd_thruster = self.default_thruster
        if self.latest_det_msg is None:
            return

        # ✅ 순차 실행: HOPING 단계에서는 호핑만
        if self.phase == "HOPING":
            self.hoping_detection(self.latest_det_msg)
            return

        # ✅ DETECTION 단계에서는 detection만
        if self.phase == "DETECTION":
            detected = self.target_detection(self.latest_det_msg)
            if detected:
                self.phase = "DONE"
                self.get_logger().info("[PHASE] DETECTION -> DONE (now steering to -90 using imu error)")
            return

        # DONE이면 IMU callback이 계속 -90 제어 수행
        if self.phase == "DONE":
            return


    def main(args=None):
        rclpy.init(args=args)
        node = course2()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
if __name__ == "__main__":
    main()