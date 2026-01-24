#!/usr/bin/env python3
import os
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Float64

from mechaship_interfaces.msg import RgbwLedColor


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def norm_text(s: str) -> str:
    """대소문자/연속 공백 무시 비교용"""
    return " ".join(str(s).strip().lower().split())


class VisionServoTestNode(Node):
    """
    isv_params.yaml을 읽어서:
      - /detections에서 tracking_target(예: Blue Circle)을 찾고
      - bbox center.x로 vision_target_angle 계산
      - /actuator/key/degree 로 서보 각 publish
      - /actuator/thruster/percentage 로 추력 publish(옵션)
      - /actuator/rgbwled/color 로 LED 제어

    타겟이 없으면(일정 시간 이후) 중립각/기본 LED로 복귀.
    """

    def __init__(self):
        super().__init__("vision_servo_test")
        self.get_logger().info("=== VisionServoTestNode (YAML-based + LED) ===")

        # ---- YAML 로드 ----
        self._load_params_from_yaml()

        # ---- Subscriber (/detections) ----
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            self.detections_topic,
            self.detection_cb,
            qos_profile_sensor_data,
        )

        # ---- Publishers ----
        self.servo_pub = self.create_publisher(Float64, self.servo_topic, 10)
        self.thruster_pub = self.create_publisher(Float64, self.thruster_topic, 10)

        # ✅ LED Publisher
        self.led_pub = self.create_publisher(RgbwLedColor, self.led_topic, 10)

        # ---- state ----
        self.last_target_time = None
        self.last_servo_cmd = self.servo_neutral_deg
        self.current_led = self.make_led(white=self.led_default_white)

        # 타겟 끊겼을 때 중립/hold 처리
        self.create_timer(0.05, self.timer_cb)

        # ---- info ----
        self.get_logger().info("Loaded params:")
        self.get_logger().info(f"- yaml_path: {self.yaml_path}")
        self.get_logger().info(f"- detections_topic: {self.detections_topic}")
        self.get_logger().info(f"- tracking_target: '{self.tracking_target}'")
        self.get_logger().info(f"- labels(count): {len(self.available_objects)}")
        self.get_logger().info(
            f"- servo_topic: {self.servo_topic} neutral={self.servo_neutral_deg} "
            f"range=[{self.servo_min_deg},{self.servo_max_deg}]"
        )
        self.get_logger().info(
            f"- vision: screen_width={self.screen_width}, angle_factor={self.angle_conversion_factor}, steer_gain={self.steer_gain}"
        )
        self.get_logger().info(
            f"- no_target_mode={self.no_target_mode}, hold_timeout_sec={self.hold_timeout_sec}"
        )
        self.get_logger().info(
            f"- enable_thruster={self.enable_thruster}, thruster_percent={self.thruster_percent}"
        )
        self.get_logger().info(
            f"- led_topic: {self.led_topic}, led_default_white={self.led_default_white}, led_on_brightness={self.led_on_brightness}"
        )

        # 시작 LED 기본값 publish
        self.publish_led(self.current_led)

    # ---------------- LED Utils ----------------
    def make_led(self, red=0, green=0, blue=0, white=0) -> RgbwLedColor:
        msg = RgbwLedColor()
        msg.red = int(clamp(red, 0, 255))
        msg.green = int(clamp(green, 0, 255))
        msg.blue = int(clamp(blue, 0, 255))
        msg.white = int(clamp(white, 0, 255))
        return msg

    def publish_led(self, msg: RgbwLedColor):
        """메시지 그대로 publish + 내부 상태 갱신"""
        self.current_led = msg
        self.led_pub.publish(msg)

    def set_led(self, red=0, green=0, blue=0, white=0):
        """숫자 인자로 LED 설정"""
        self.publish_led(self.make_led(red, green, blue, white))

    # ---------------- YAML ----------------
    def _load_params_from_yaml(self):
        script_dir = os.path.dirname(os.path.realpath(__file__))
        self.yaml_path = os.path.join(script_dir, "isv_params.yaml")

        try:
            with open(self.yaml_path, "r") as f:
                params = yaml.safe_load(f)
        except FileNotFoundError:
            self.get_logger().fatal(f"FATAL: isv_params.yaml not found: {self.yaml_path}")
            raise
        except Exception as e:
            self.get_logger().fatal(f"FATAL: failed to read YAML: {e}")
            raise

        # ---- servo ----
        servo = params["servo"]
        self.servo_min_deg = float(servo["min_deg"])
        self.servo_max_deg = float(servo["max_deg"])
        self.servo_neutral_deg = float(servo["neutral_deg"])

        # ---- vision ----
        vision = params["vision"]
        self.screen_width = float(vision["screen_width"])
        self.angle_conversion_factor = float(vision["angle_conversion_factor"])

        # ---- vision targets ----
        vt = params["vision_targets"]
        self.available_objects = list(vt["available_objects"])
        self.tracking_target = str(vt["tracking_target"])

        # ---- topics ----
        self.detections_topic = "/detections"
        self.servo_topic = "/actuator/key/degree"
        self.thruster_topic = "/actuator/thruster/percentage"
        self.led_topic = "/actuator/rgbwled/color"

        # ---- options ----
        self.steer_gain = 1.0
        self.no_target_mode = "neutral"  # "neutral" or "hold"
        self.hold_timeout_sec = 0.3

        self.enable_thruster = True
        self.thruster_percent = 0.0

        self.led_default_white = 20
        self.led_on_brightness = 80

    # ---------------- Detections ----------------
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

    def detection_cb(self, msg: Detection2DArray):
        target = None
        target_score = None
        target_class_id = None
        target_name = None

        tgt_norm = norm_text(self.tracking_target)

        for det in msg.detections:
            class_id, score = self._extract_classid_score(det)
            if class_id is None:
                continue

            name = self.get_object_name(class_id)

            if norm_text(name) == tgt_norm:
                target = det
                target_class_id = class_id
                target_score = score
                target_name = name
                break

        if target is None:
            # 타겟 못 찾으면 timer_cb에서 neutral/hold + LED 복귀
            return

        # bbox center.x 기반 각도 계산
        cx = float(target.bbox.center.position.x)
        angle_offset = ((cx - (self.screen_width / 2.0)) / (self.screen_width / 2.0)) * self.angle_conversion_factor
        vision_target_angle = -angle_offset

        # 서보 명령 생성
        servo_cmd = self.servo_neutral_deg - (self.steer_gain * vision_target_angle)
        servo_cmd = clamp(servo_cmd, self.servo_min_deg, self.servo_max_deg)

        self.last_servo_cmd = servo_cmd
        self.last_target_time = self.get_clock().now()

        # publish servo
        self.servo_pub.publish(Float64(data=float(servo_cmd)))

        # ✅ 타겟 잡히면 LED 색상 켬 (메시지 직접 publish)
        led_msg = self.led_by_target_name(target_name)
        self.publish_led(led_msg)

        score_str = f"{target_score:.2f}" if isinstance(target_score, (float, int)) else "NA"
        self.get_logger().info(
            f"[TARGET] '{self.tracking_target}' matched (id={target_class_id}, score={score_str}) "
            f"cx={cx:.1f} angle={vision_target_angle:.2f}deg -> servo={servo_cmd:.1f}deg"
        )

    def timer_cb(self):
        # 타겟이 없을 때 안전 처리 (서보 + LED 복귀)
        target_recent = False
        if self.last_target_time is not None:
            dt = (self.get_clock().now() - self.last_target_time).nanoseconds / 1e9
            target_recent = (dt <= self.hold_timeout_sec)

        # ---- servo fallback ----
        if self.no_target_mode == "hold" and target_recent:
            self.servo_pub.publish(Float64(data=float(self.last_servo_cmd)))
        else:
            self.servo_pub.publish(Float64(data=float(self.servo_neutral_deg)))

        # ---- LED fallback ----
        if not target_recent:
            # 기본 white로 복귀
            if not (self.current_led.white == self.led_default_white and
                    self.current_led.red == 0 and self.current_led.green == 0 and self.current_led.blue == 0):
                self.set_led(white=self.led_default_white)

        # ---- thruster publish (옵션) ----
        if self.enable_thruster:
            self.thruster_pub.publish(Float64(data=float(self.thruster_percent)))

    def destroy_node(self):
        # 종료 시 안전하게 LED/서보/추력 정리
        try:
            self.servo_pub.publish(Float64(data=float(self.servo_neutral_deg)))
            self.thruster_pub.publish(Float64(data=0.0))
            self.publish_led(self.make_led(white=self.led_default_white))
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = VisionServoTestNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.get_logger().info("Shutdown: set servo neutral, thruster 0, LED default")
            except Exception:
                pass
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()