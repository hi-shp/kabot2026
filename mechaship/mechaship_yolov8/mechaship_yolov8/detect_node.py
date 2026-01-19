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

# ROS 2 Messages
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class DetectNode(Node):
    def __init__(self) -> None:
        super().__init__(
            "detect_node",
            automatically_declare_parameters_from_overrides=True,
        )

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        if not IS_SBC:
            self.get_logger().fatal("SBC에서 실행해주시기 바랍니다.")
            return TransitionCallbackReturn.ERROR

        self.image_topic = (
            self.get_parameter_or(
                "image_topic",
                Parameter("image_topic", Parameter.Type.STRING, "/image_raw/compressed"),
            )
            .get_parameter_value()
            .string_value
        )

        self.rknn_model = (
            self.get_parameter_or(
                "model_params.rknn_model",
                Parameter("model_params.rknn_model", Parameter.Type.STRING, "yolov8.rknn"),
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

        conf = (
            self.get_parameter_or(
                "model_params.conf",
                Parameter("model_params.conf", Parameter.Type.DOUBLE, 0.5),
            )
            .get_parameter_value()
            .double_value
        )

        iou = (
            self.get_parameter_or(
                "model_params.iou",
                Parameter("model_params.iou", Parameter.Type.DOUBLE, 0.8),
            )
            .get_parameter_value()
            .double_value
        )

        fps = (
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

        self.rknn_helper = RKNNHelper(conf, iou, self.img_size)

        self.detection_publisher: Publisher = self.create_lifecycle_publisher(
            Detection2DArray, "detections", qos_profile_sensor_data
        )

        self.detection_enable_subscription = self.create_subscription(
            Bool,
            "detection_enable",
            self.detection_enable_callback,
            qos_profile_sensor_data,
        )

        self.br = CvBridge()
        self.compressed_image = CompressedImage()
        self.timer = self.create_timer(1 / fps, self.timer_callback)

        return TransitionCallbackReturn.SUCCESS

    def detection_enable_callback(self, msg: Bool) -> None:
        self.enable = msg.data

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.model = RKNNLite()

        model_path = join(
            get_package_share_directory(__package__), "model", self.rknn_model
        )
        if self.model.load_rknn(model_path) != 0:
            raise RuntimeError("RKNN load failed")

        if self.model.init_runtime(core_mask=RKNNLite.NPU_CORE_0) != 0:
            raise RuntimeError("RKNN runtime init failed")

        label_path = join(
            get_package_share_directory(__package__), "model", self.label
        )
        with open(label_path, "r") as f:
            self.classes = [line.strip() for line in f.readlines()]

        self.image_subscription = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.model.release()
        self.destroy_subscription(self.image_subscription)
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.destroy_timer(self.timer)
        self.destroy_publisher(self.detection_publisher)
        self.destroy_subscription(self.detection_enable_subscription)
        return TransitionCallbackReturn.SUCCESS

    def image_callback(self, msg: CompressedImage) -> None:
        self.compressed_image = msg

    def timer_callback(self):
        if not self.enable:
            return

        if not self.detection_publisher.is_activated:
            return

        if not self.compressed_image.data:
            return

        origin_image = self.br.compressed_imgmsg_to_cv2(
            self.compressed_image, "rgb8"
        )

        padded_image = self.rknn_helper.resize_with_padding(origin_image)
        input_image = self.rknn_helper.expand_dims(padded_image)

        outputs = self.model.inference(inputs=[input_image])
        boxes, classes, scores = self.rknn_helper.post_process(outputs)

        if boxes is None:
            return

        detections_msg = Detection2DArray()
        detections_msg.header = self.compressed_image.header

        for box, class_id, score in zip(boxes, classes, scores):
            x1, y1, x2, y2 = box
            x = (x1 + x2) / 2
            y = (y1 + y2) / 2
            w = x2 - x1
            h = y2 - y1

            # ★ 터미널 출력
            self.get_logger().info(
                f"[DETECT] label={self.classes[int(class_id)]} "
                f"center=({x:.1f}, {y:.1f}) size=({w:.1f}, {h:.1f}) score={score:.2f}"
            )

            detection = Detection2D()
            detection.header = self.compressed_image.header

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(int(class_id))
            hypothesis.hypothesis.score = float(score)
            detection.results = [hypothesis]

            detection.bbox.center.position.x = float(x)
            detection.bbox.center.position.y = float(y)
            detection.bbox.size_x = float(w)
            detection.bbox.size_y = float(h)
            detection.id = self.classes[int(class_id)]

            detections_msg.detections.append(detection)

        self.detection_publisher.publish(detections_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DetectNode()

    try:
        if node.trigger_configure() == TransitionCallbackReturn.SUCCESS:
            node.trigger_activate()
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
