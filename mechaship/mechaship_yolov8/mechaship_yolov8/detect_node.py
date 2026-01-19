import time
from os.path import join

# RKNN
IS_SBC = True
try:
    from rknnlite.api import RKNNLite  # type: ignore
    from utils.rknn_helper import RKNNHelper
except ImportError:
    IS_SBC = False

# ROS 2
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.lifecycle import Node, Publisher, State, TransitionCallbackReturn
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data

# ROS 2 Messages and Services
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class DetectNode(Node):
    def __init__(self) -> None:
        super().__init__(
            "detect_node", automatically_declare_parameters_from_overrides=True
        )

    # <on_configure> 노드를 생성할 때 / 변수 선언
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().debug(f"Configuring {self.get_name()}")

        if not IS_SBC:
            self.get_logger().fatal("SBC에서 실행해주시기 바랍니다.")
            return TransitionCallbackReturn.ERROR

        # 파라미터 가져오기
        self.image_topic = (
            self.get_parameter_or(
                "image_topic",
                Parameter(
                    "image_topic", Parameter.Type.STRING, "/image_raw/compressed"
                ),
            )
            .get_parameter_value()
            .string_value
        )

        self.rknn_model = (
            self.get_parameter_or(
                "model_params.rknn_model",
                Parameter(
                    "model_params.rknn_model", Parameter.Type.STRING, "yolov8.rknn"
                ),
            )
            .get_parameter_value()
            .string_value
        )

        self.label = (
            self.get_parameter_or(
                "model_params.label",
                Parameter("model_params.label", Parameter.Type.STRING, "labels.txt"),
            )
            .get_parameter_value()
            .string_value
        )

        self.img_size = (
            self.get_parameter_or(
                "model_params.img_size",
                Parameter("model_params.img_size", Parameter.Type.INTEGER, 640),
            )
            .get_parameter_value()
            .integer_value
        )

        _conf = (
            self.get_parameter_or(
                "model_params.conf",
                Parameter("model_params.conf", Parameter.Type.DOUBLE, 0.5),
            )
            .get_parameter_value()
            .double_value
        )

        _iou = (
            self.get_parameter_or(
                "model_params.iou",
                Parameter("model_params.iou", Parameter.Type.DOUBLE, 0.8),
            )
            .get_parameter_value()
            .double_value
        )

        _fps = (
            self.get_parameter_or(
                "model_params.fps",
                Parameter("model_params.fps", Parameter.Type.INTEGER, 10),
            )
            .get_parameter_value()
            .integer_value
        )

        self.enable = (
            self.get_parameter_or(
                "model_params.enable",
                Parameter("model_params.enable", Parameter.Type.BOOL, True),
            )
            .get_parameter_value()
            .bool_value
        )

        self.get_logger().info(f"image_topic : {self.image_topic}")
        self.get_logger().info(f"rknn_model : {self.rknn_model}")
        self.get_logger().info(f"label : {self.label}")
        self.get_logger().info(f"img_size : {self.img_size}")
        self.get_logger().info(f"conf : {_conf}")
        self.get_logger().info(f"iou : {_iou}")
        self.get_logger().info(f"fps : {_fps}")
        self.get_logger().info(f"enable : {self.enable}")

        # RKNN 핼퍼 불러오기 (rknn_helper.py)
        self.rknn_helper = RKNNHelper(_conf, _iou, self.img_size)

        # 객체 인식 결과 Publisher 생성
        self.detection_publisher: Publisher = self.create_lifecycle_publisher(
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
        self.timer = self.create_timer(1 / _fps, self.timer_callback)

        # 상태 전환 결과 반환 (SUCCESS)
        return TransitionCallbackReturn.SUCCESS

    # 객체 인식 실행 여부 Subscription callback
    def detection_enable_callback(self, enable_msg: Bool) -> None:
        # self.enable 변수 업데이트
        self.enable = enable_msg.data

    # <on_activate> 노드를 실행할 때 / 시스템 구성
    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().debug(f"Activating {self.get_name()}")

        # RKNN Lite 불러오기
        self.model = RKNNLite()

        # RKNN model 불러오기
        self.get_logger().info("--> Load RKNN model")
        model_full_path = join(
            get_package_share_directory(__package__), "model", self.rknn_model
        )
        ret = self.model.load_rknn(model_full_path)
        if ret != 0:  # 실패할 경우 예외 발생
            raise RuntimeError("Load RKNN model failed!")
        self.get_logger().info("done")

        # 런타임 환경 초기화
        self.get_logger().info("--> Init runtime environment")
        ret = self.model.init_runtime(core_mask=RKNNLite.NPU_CORE_0)
        if ret != 0:  # 실패할 경우 예외 발생
            raise RuntimeError("Init runtime environment failed!")
        self.get_logger().info("done")

        # 라벨 파일을 읽어 리스트로 변환
        label_full_path = join(
            get_package_share_directory(__package__), "model", self.label
        )
        with open(label_full_path, "r") as f:
            self.classes = [line.strip() for line in f.readlines()]

        # 이미지 Subscription 생성
        self.image_subscription = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        # 상태 전환 결과 반환
        return super().on_activate(state)

    # <on_deactivate> 노드를 중지할 때 / 시스템 정지
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().debug(f"Deactivating {self.get_name()}")

        # RKNN 반환하기
        self.model.release()

        # 이미지 Subscription 반환하기
        self.destroy_subscription(self.image_subscription)
        self.image_subscription = None

        # 상태 전환 결과 반환
        return super().on_deactivate(state)

    # <on_cleanup> 노드를 초기화할 때 / 시스템 초기화
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().debug(f"Cleaning up {self.get_name()}")

        self.destroy_timer(self.timer)

        # 객체 인식 결과 Publisher 반환하기
        self.destroy_publisher(self.detection_publisher)

        # 객체 인식 실행 여부 Subscription 반환하기
        self.destroy_subscription(self.detection_enable_subscription)
        self.detection_enable_subscription = None

        # 상태 전환 결과 반환 (SUCCESS)
        return TransitionCallbackReturn.SUCCESS

    # <on_shutdown> 노드를 삭제할 때
    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().debug(f"Shutting down {self.get_name()}")

        self.destroy_timer(self.timer)

        # 객체 인식 결과 Publisher 반환하기
        self.destroy_publisher(self.detection_publisher)

        # 객체 인식 실행 여부 Subscription 반환하기
        self.destroy_subscription(self.detection_enable_subscription)
        self.detection_enable_subscription = None

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

        if not self.compressed_image.data:
            return

        # compressed image를 OpenCV 이미지로 변환
        origin_image = self.br.compressed_imgmsg_to_cv2(self.compressed_image, "rgb8")

        # 비율에 맞춰 이미지 리사이즈
        padded_image = self.rknn_helper.resize_with_padding(origin_image)

        # RKNN에 사용할 수 있는 구조로 변경 (4dim)
        input_image = self.rknn_helper.expand_dims(padded_image)

        # 객체 탐지 실행
        self.get_logger().debug("--> Running model")
        start_time = time.time()
        outputs = self.model.inference(inputs=[input_image])
        finish_time = time.time()

        boxes, classes, scores = self.rknn_helper.post_process(outputs)

        # 탐지된 객체가 없을 경우 종료
        if boxes is None or classes is None or scores is None:
            return

        self.get_logger().debug(f"detect Time: {(finish_time - start_time)}")

        # Detection2DArray에 탐지 결과 담기
        detections_msg = Detection2DArray()
        detections_msg.header = self.compressed_image.header

        for box, class_id, score in zip(boxes, classes, scores):
            detection = Detection2D()
            detection.header = self.compressed_image.header

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(int(class_id))
            hypothesis.hypothesis.score = float(score)
            detection.results = [hypothesis]

            x1, y1, x2, y2 = box
            x = int((x1 + x2) / 2)
            y = int((y1 + y2) / 2)
            w = int(x2 - x1)
            h = int(y2 - y1)

            detection.bbox.center.position.x = float(x)
            detection.bbox.center.position.y = float(y)
            detection.bbox.size_x = float(w)
            detection.bbox.size_y = float(h)

            detection.id = str(self.classes[int(class_id)])

            detections_msg.detections.append(detection)

        # Detection2DArray Publish
        self.detection_publisher.publish(detections_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DetectNode()

    try:
        if node.trigger_configure() == TransitionCallbackReturn.SUCCESS:
            node.trigger_activate()

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
