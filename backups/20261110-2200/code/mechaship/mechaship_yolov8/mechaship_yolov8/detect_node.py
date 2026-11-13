import time
from os.path import join

# OpenCV
from cv_bridge import CvBridge

# ROS 2
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.lifecycle.publisher import LifecyclePublisher
from rclpy.qos import qos_profile_sensor_data

# ROS 2 Messages and Services
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

# YOLO
from ultralytics import YOLO

DEBUG = True


class DetectNode(LifecycleNode):
    def __init__(self) -> None:
        super().__init__(
            "detect_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

    # 디버그 메세지 출력 함수
    def log_debug(self, message: str) -> None:
        if DEBUG:
            self.get_logger().info(message)

    # <on_configure> 노드를 생성할 때 / 변수 선언
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.log_debug(f"Configuring {self.get_name()}")

        # 파라미터 가져오기
        self.image_topic = (
            self.get_parameter_or("image_topic", "/image_raw/compressed")
            .get_parameter_value()
            .string_value
        )

        self.model = (
            self.get_parameter_or("model_params.model", "yolov8n.pt")
            .get_parameter_value()
            .string_value
        )

        self.img_size = (
            self.get_parameter_or("model_params.img_size", 640)
            .get_parameter_value()
            .integer_value
        )

        self.conf = (
            self.get_parameter_or("model_params.conf", 0.5)
            .get_parameter_value()
            .double_value
        )

        self.iou = (
            self.get_parameter_or("model_params.iou", 0.8)
            .get_parameter_value()
            .double_value
        )

        self.enable = (
            self.get_parameter_or("model_params.enable", True)
            .get_parameter_value()
            .bool_value
        )

        self.log_debug(f"image_topic : {self.image_topic}")
        self.log_debug(f"model : {self.model}")
        self.log_debug(f"img_size : {self.img_size}")
        self.log_debug(f"conf : {self.conf}")
        self.log_debug(f"iou : {self.iou}")
        self.log_debug(f"enable : {self.enable}")

        # 객체 인식 결과 Publisher 생성
        self.detection_publisher: LifecyclePublisher = self.create_lifecycle_publisher(
            Detection2DArray, "detections", qos_profile_sensor_data
        )

        # 객체 인식 실행 여부 Subscription 생성
        self.detection_enable_subscription = self.create_subscription(
            Bool,
            "detection_enable",
            self.detection_enable_callback,
            qos_profile_sensor_data,
        )

        # CvBridge 불러오기
        self.br = CvBridge()

        self.compressed_image = CompressedImage()
        self.timer = self.create_timer(1, self.timer_callback)

        # 모든 요소(publisher, subscriber 등) 상태 전환 >> <on_configure>
        super().on_configure(state)

        # 상태 전환 결과 반환 (SUCCESS)
        return TransitionCallbackReturn.SUCCESS

    # 객체 인식 실행 여부 Subscription callback
    def detection_enable_callback(self, enable_msg: Bool) -> None:
        # self.enable 변수 업데이트
        self.enable = enable_msg.data

    # <on_activate> 노드를 실행할 때 / 시스템 구성
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.log_debug(f"Activating {self.get_name()}")

        # YOLO 모델 불러오기
        self.log_debug("--> Load model")
        model_full_path = join(
            get_package_share_directory(__package__), "model", self.model
        )
        self.yolo = YOLO(model_full_path, task="detect")
        self.log_debug("done")

        # 이미지 Subscription 생성
        self.image_subscription = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        # 모든 요소(publisher, subscriber 등) 상태 전환 >> <on_activate>
        super().on_activate(state)

        # 상태 전환 결과 반환 (SUCCESS)
        return TransitionCallbackReturn.SUCCESS

    # <on_deactivate> 노드를 중지할 때 / 시스템 정지
    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.log_debug(f"Deactivating {self.get_name()}")

        # 이미지 Subscription 반환하기
        self.destroy_subscription(self.image_subscription)
        self.image_subscription = None

        del self.yolo

        # 모든 요소(publisher, subscriber 등) 상태 전환 >> <on_deactivate>
        super().on_deactivate(state)

        # 상태 전환 결과 반환 (SUCCESS)
        return TransitionCallbackReturn.SUCCESS

    # <on_cleanup> 노드를 초기화할 때 / 시스템 초기화
    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.log_debug(f"Cleaning up {self.get_name()}")

        # 객체 인식 결과 Publisher 반환하기
        self.destroy_publisher(self.detection_publisher)

        # 객체 인식 실행 여부 Subscription 반환하기
        self.destroy_subscription(self.detection_enable_subscription)
        self.detection_enable_subscription = None

        # 모든 요소(publisher, subscriber 등) 상태 전환 >> <on_cleanup>
        super().on_cleanup(state)

        # 상태 전환 결과 반환 (SUCCESS)
        return TransitionCallbackReturn.SUCCESS

    # <on_shutdown> 노드를 삭제할 때
    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.log_debug(f"Shutting down {self.get_name()}")

        # 모든 요소(publisher, subscriber 등) 상태 전환 >> <on_shutdown>
        super().on_shutdown(state)

        # 상태 전환 결과 반환 (SUCCESS)
        return TransitionCallbackReturn.SUCCESS

    # 이미지 Subscription callback
    def image_callback(self, compressed_image: CompressedImage) -> None:
        self.compressed_image = compressed_image

    def timer_callback(self):
        # self.enable이 True 일때만 실행
        if not self.enable:
            return

        # 객체 인식 결과 Publisher가 activate 상태일때만 실행
        if not self.detection_publisher.is_activated:
            self.get_logger().warn("Publisher is not activated")
            return

        if self.compressed_image.data == []:
            return

        # compressed image를 OpenCV 이미지로 변환
        origin_image = self.br.compressed_imgmsg_to_cv2(self.compressed_image)

        # 객체 탐지 실행
        self.log_debug("--> Running model")
        start_time = time.time()
        results = self.yolo.predict(
            source=origin_image,
            stream=False,
            conf=self.conf,
            iou=self.iou,
            verbose=False,
            visualize=False,
        )
        finish_time = time.time()

        self.log_debug(f"detect Time: {(finish_time - start_time)}")

        # Detection2DArray에 탐지 결과 담기
        detections_msg = Detection2DArray()
        detections_msg.header = self.compressed_image.header

        for box_data in results[0].boxes:
            detection = Detection2D()
            detection.header = self.compressed_image.header

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(int(box_data.cls))
            hypothesis.hypothesis.score = float(box_data.conf)
            detection.results = [hypothesis]

            box = box_data.xywh[0]
            detection.bbox.center.position.x = float(box[0])
            detection.bbox.center.position.y = float(box[1])
            detection.bbox.size_x = float(box[2])
            detection.bbox.size_y = float(box[3])

            detection.id = self.yolo.names[int(box_data.cls)]

            detections_msg.detections.append(detection)

        if not self.detection_publisher.is_activated:
            self.get_logger().warn("Publisher is not activated")

        # Detection2DArray Publish
        self.detection_publisher.publish(detections_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DetectNode()
    node.trigger_configure()
    node.trigger_activate()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")

    except Exception as e:
        node.get_logger().error(f"Exception: {e}")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
