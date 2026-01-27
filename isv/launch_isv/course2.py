import os
import yaml
import math
from math import degrees
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import NavSatFix,Imu
from std_msgs.msg import Float64
from mechaship_interfaces.msg import RgbwLedColor
from tf_transformations import euler_from_quaternion


def constrain(v, lo, hi):
    if math.isnan(v): return lo + (hi - lo) / 2.0
    return lo if v < lo else hi if v > hi else v

def norm_text(s: str) -> str:
    return " ".join(str(s).strip().lower().split())


class course2(Node):
    def __init__(self):
        super().__init__("course2")
        # ---------------- Publishers ----------------
        self.key_publisher = self.create_publisher(Float64, "/actuator/key/degree", 10)
        self.thruster_publisher = self.create_publisher(Float64, "/actuator/thruster/percentage", 10)
        self.led_pub = self.create_publisher(RgbwLedColor, "/actuator/rgbwled/color", 10)
        self.curr_yaw_publisher = self.create_publisher(Float64, "/current_yaw", 10)
        self.goal_publisher = self.create_publisher(NavSatFix, "/waypoint/goal", 10)
        self.dist_publisher = self.create_publisher(Float64, "/waypoint/distance", 10)
        # ---------------- Subscribers ----------------
        self.imu_sub = self.create_subscription(Imu, "/imu", self.imu_callback, qos_profile_sensor_data)
        self.detection_sub = self.create_subscription(Detection2DArray, "/detections", self.detection_callback, qos_profile_sensor_data)
        self.gps_sub = self.create_subscription(NavSatFix, "/gps/fix", self.gps_listener_callback, qos_profile_sensor_data)
        # ---------------- State / Defaults ----------------
        self.latest_detection_msg = None
        self.latest_det_msg = None
        self.is_hoping_started = False
        # 서보 기본
        self.current_servo_deg = self.servo_neutral_deg
        self.turn_step_deg = 20.0  # 호핑에서 사용할 step
        # IMU 현재 yaw / 목표각
        self.initial_yaw_abs = None
        self.current_yaw_rel = 0.0
        self.imu_target_angle = 0.0
        # yaw 0도 2번 카운트 (연속 중복 카운트 방지)
        self.yaw_zero_count = 0
        self.yaw_zero_latched = False
        self.yaw_zero_tol = 0.0  
        #GPS 
        self.origin = None
        self.origin_set = False
        self.wp_index = 1
        self.current_goal_enu = None
        self.dist_to_goal_m = None
        # LED 기본값
        self.led_default_white = 20
        self.led_on_brightness = 80
        self.current_led = self.make_led(white=self.led_default_white)
        self.publish_led(self.current_led)
        #timer 등록
        self.timer_handle = self.create_timer(self.timer_period, self.timer)
        # ✅ 순차 진행 상태
        self.phase = "HOPING"  # HOPING -> DETECTION -> DONE
        # ---- info ----
        self.get_logger().info("Loaded params (from YAML):")
        self.get_logger().info(f"- yaml_path: {self.yaml_path}")
        self.get_logger().info(f"- hoping_target: '{self.hoping_target}'")
        self.get_logger().info(f"- detection_target: '{self.detection_target}'")
        self.get_logger().info(f"- labels(count): {len(self.available_objects)}")
        self.get_logger().info(f"- vision: screen_width={self.screen_width}, angle_conversion_factor={self.angle_conversion_factor}")
        self.get_logger().info(f"- phase: {self.phase}")
        self.get_logger().info(f"[START] hoping_target='{self.hoping_target}' | detection_target='{self.detection_target}'")   
    # ---------------- YAML Load ----------------
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
        self.screen_width = int(vision.get("screen_width", 640))
        servo = params.get("servo", {})
        self.servo_min_deg = float(servo.get("min_deg", 45.0))
        self.servo_max_deg = float(servo.get("max_deg", 135.0))
        self.servo_neutral_deg = float(servo.get("neutral_deg", 90.0))
        self.angle_conversion_factor = float(vision.get("angle_conversion_factor", 90))
        self.thruster_cfg = params["state"]
        self.default_thruster = float(self.thruster_cfg.get("state1", 10.0))
        nav = params["navigation"]
        self.waypoints = nav["waypoints"]
        self.arrival_radius = nav.get("arrival_radius", 0.5)
        node_settings = params.get("node_settings", {})
        self.timer_period= node_settings.get("timer_period", 0.5)

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
        if a == -180.0:
            a = 180.0
        return a

    def imu_callback(self, msg: Imu):
        q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        _, _, yaw_rad = euler_from_quaternion(q)
        current_yaw_abs = -yaw_rad 
        if self.initial_yaw_abs is None:
            self.initial_yaw_abs = current_yaw_abs
        rel_yaw_deg = self.normalize_180(degrees(current_yaw_abs - self.initial_yaw_abs))
        self.current_yaw_rel = rel_yaw_deg
        self.curr_yaw_publisher.publish(Float64(data=float(rel_yaw_deg)))
        # ✅ HOPING 단계에서만 "yaw 0도 2번"을 감시해서 DETECTION 단계로 전환
        if self.phase == "HOPING":
            if abs(rel_yaw_deg) <= 0.1:
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

        if float(self.imu_target_angle) == -90.0:
            self.control_to_target_yaw_minus90()
    #------------------------------GPS---------------------------------------------
    def gps_listener_callback(self, gps: NavSatFix):
        if math.isnan(gps.latitude) or math.isnan(gps.longitude) is None:
            return
        if not self.origin_set:
            self.origin = [gps.latitude, gps.longitude, gps.altitude]
            self.origin_set = True
            self.get_logger().info(f"시작 위치: {self.origin[:2]}")
            self.update_current_goal()
        curr_e, curr_n = self.gps_enu_converter([gps.latitude, gps.longitude, gps.altitude])
        
        if self.current_goal_enu is not None:
            goal_e, goal_n = self.current_goal_enu
            dx, dy = goal_e - curr_e, goal_n - curr_n
            self.dist_to_goal_m = math.hypot(dx, dy)
            target_ang_abs = degrees(math.atan2(dx, dy))
            target_ang_rel = self.normalize_180(target_ang_abs - degrees(self.initial_yaw_abs))
            self.goal_rel_deg = self.normalize_180(target_ang_rel - self.current_yaw_rel)
    def gps_enu_converter(self, lla):
        if self.origin is None:
            return 0.0, 0.0
        lat, lon, _ = lla
        lat0, lon0, _ = self.origin
        R = 6378137.0
        dlat = math.radians(lat - lat0)
        dlon = math.radians(lon - lon0)
        latm = math.radians((lat + lat0) * 0.5)
        return dlon * R * math.cos(latm), dlat * R
    def gps_distance(self, lat1, lon1, lat2, lon2):
        R = 6371000.0
        lat1 = math.radians(lat1)
        lon1 = math.radians(lon1)
        lat2 = math.radians(lat2)
        lon2 = math.radians(lon2)

        dlat = lat2 - lat1
        dlon = lon2 - lon1

        a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return R * c
    def update_current_goal(self):
        if self.wp_index < len(self.waypoints):
            target_lat, target_lon = self.waypoints[self.wp_index]
            self.current_goal_enu = self.gps_enu_converter([target_lat, target_lon, 0.0])
            goal_msg = NavSatFix()
            goal_msg.latitude = target_lat
            goal_msg.longitude = target_lon
            self.goal_publisher.publish(goal_msg)
            self.get_logger().info(f"웨이포인트 목표: {self.wp_index+1}/{len(self.waypoints)}")
        else:
            self.current_goal_enu = None
            self.arrived_all = True
            self.get_logger().info("모든 웨이포인트 도착")
    def control_to_target_yaw_minus90(self):
        current = float(self.current_yaw_rel)
        current = -current
        target = float(self.imu_target_angle)  # -90.0
        error = current - target
        servo_cmd = self.servo_neutral_deg - error
        servo_cmd = constrain(servo_cmd, self.servo_min_deg, self.servo_max_deg)
        self.current_servo_deg = servo_cmd
        self.key_publisher.publish(Float64(data=float(servo_cmd)))        
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
        if msg is None or not msg.detections:
            return False

        target = None
        tgt_norm = norm_text(self.hoping_target)

        # 1) 목표 객체 찾기
        for det in msg.detections:
            class_id, score = self._extract_classid_score(det)
            if class_id is None:
                continue
            name = self.get_object_name(class_id)
            if norm_text(name) == tgt_norm:
                target = det
                break
        if target is None:
            return False
        
        if not self.is_hoping_started:
            self.is_hoping_started = True
            self.get_logger().info("start_hoping")

        # 3) ROI 중심 x 추출 (메시지 타입 안전 처리)
        center = target.bbox.center
        if hasattr(center, "x"):  # geometry_msgs/Pose2D 형태
            cx = float(center.x)
        elif hasattr(center, "position") and hasattr(center.position, "x"):  # 다른 타입 대비
            cx = float(center.position.x)
        else:
            self.get_logger().warn("[HOPING] bbox.center has unknown structure (no x/position.x)")
            return False
        vision_target_angle = ((cx - (self.screen_width / 2.0)) / (self.screen_width / 2.0)) * self.angle_conversion_factor
        self.get_logger().info(f"[HOPING] target='{self.hoping_target}' cx={cx:.1f} -> angle={vision_target_angle:.2f} deg" )
        if vision_target_angle <= -30.0:
            self.current_servo_deg = 70.0
        else:
            self.current_servo_deg = self.servo_neutral_deg

        self.current_servo_deg = constrain(self.current_servo_deg, self.servo_min_deg, self.servo_max_deg)
        self.key_publisher.publish(Float64(data=float(self.current_servo_deg)))
        return True

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
# ---------------- Timer ----------------------------------------------------------------------
    def timer(self):
        state_key = f"state{self.wp_index}"
        self.cmd_thruster = float(self.thruster_cfg.get(state_key, self.default_thruster))
        self.thruster_publisher.publish(Float64(self.cmd_thruster))
        if self.latest_det_msg is None:
            return

        if self.phase == "HOPING":
            self.hoping_detection(self.latest_det_msg)
            return

        if self.phase == "DETECTION":
            detected = self.target_detection(self.latest_det_msg)
            if detected:
                self.phase = "DONE"
                self.get_logger().info("[PHASE] DETECTION -> DONE (now steering to -90 using imu error)")
            return  
        
        if self.phase == "DONE":
            if self.dist_to_goal_m is None:
                return
            self.dist_publisher.publish(Float64(data=float(self.dist_to_goal_m)))
            if self.dist_to_goal_m <= self.gps_reach_threshold:
                self.phase = "NEXT"
                self.get_logger().info(
                    f"[PHASE] DONE -> NEXT (reached by ENU dist: {self.dist_to_goal_m:.2f} m)"
                )
            return
def main(args=None):
        rclpy.init(args=args)
        node = course2()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()