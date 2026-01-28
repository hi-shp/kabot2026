import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

class VisualizeNode(Node):
    FONT = cv2.FONT_HERSHEY_SIMPLEX
    COLOR = (0, 255, 0)
    THICKNESS = 2

    def __init__(self) -> None:
        super().__init__(
            "visualize_node", automatically_declare_parameters_from_overrides=True
        )

        # 파라미터 가져오기
        self.image_topic = (
            self.get_parameter_or(
                "image_topic",
                Parameter("image_topic", Parameter.Type.STRING, "/image_raw/compressed"),
            )
            .get_parameter_value()
            .string_value
        )

        self.detections_topic = (
            self.get_parameter_or(
                "detections_topic",
                Parameter("detections_topic", Parameter.Type.STRING, "/detections"),
            )
            .get_parameter_value()
            .string_value
        )

        self.preview = (
            self.get_parameter_or(
                "preview",
                Parameter("preview", Parameter.Type.BOOL, True),
            )
            .get_parameter_value()
            .bool_value
        )

        self.get_logger().info(f"image_topic : {self.image_topic}")
        self.get_logger().info(f"detections_topic : {self.detections_topic}")
        self.get_logger().info(f"preview : {self.preview}")

        # 객체 인식 결과 이미지 Publisher 생성
        self.processed_image_publisher = self.create_publisher(
            Image, "processed_image", qos_profile_sensor_data
        )
        self.processed_image_compressed_publisher = self.create_publisher(
            CompressedImage, "processed_image/compressed", qos_profile_sensor_data
        )

        # 이미지 Subscription 생성
        self.image_subscription = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        # 객체 인식 결과 Subscription 생성
        self.detection_subscription = self.create_subscription(
            Detection2DArray,
            self.detections_topic,
            self.detection_callback,
            qos_profile_sensor_data,
        )

        # CvBridge 불러오기
        self.br = CvBridge()

        # 객체 인식 결과 및 초기화용 메시지 생성
        self.detections = Detection2DArray()

    # 객체 인식 결과 Subscription callback
    def detection_callback(self, detections_msg: Detection2DArray) -> None:
        self.detections = detections_msg

    # 이미지 Subscription callback
    def image_callback(self, compressed_image: CompressedImage) -> None:
        # 1. compressed image를 OpenCV 이미지로 변환
        origin_image = self.br.compressed_imgmsg_to_cv2(compressed_image, "bgr8")

        # 2. 타임스탬프 비교를 통한 유효성 검사 (이미지 시점 vs 마지막 감지 시점)
        # 0.5초(500ms) 이상 차이가 나면 오래된 정보로 판단하여 그리지 않음
        img_time = compressed_image.header.stamp.sec + (compressed_image.header.stamp.nanosec * 1e-9)
        det_time = self.detections.header.stamp.sec + (self.detections.header.stamp.nanosec * 1e-9)
        
        time_diff = abs(img_time - det_time)

        if time_diff < 0.5:
            for detection in self.detections.detections:
                detection: Detection2D  # type hint
                cx = detection.bbox.center.position.x
                cy = detection.bbox.center.position.y
                sx = detection.bbox.size_x
                sy = detection.bbox.size_y

                min_pt = (round(cx - sx / 2.0), round(cy - sy / 2.0))
                max_pt = (round(cx + sx / 2.0), round(cy + sy / 2.0))

                cv2.rectangle(origin_image, min_pt, max_pt, self.COLOR, self.THICKNESS)

                if len(detection.results) > 0:
                    hypo: ObjectHypothesisWithPose = detection.results[0]
                    label = "{} {:.3f}".format(detection.id, hypo.hypothesis.score)
                    pos = (min_pt[0] + self.THICKNESS, max_pt[1] - self.THICKNESS - 1)
                    cv2.putText(
                        origin_image, label, pos, self.FONT, 0.75, self.COLOR, 1, cv2.LINE_AA
                    )
        
        # 3. 미리보기 화면 표시
        if self.preview:
            cv2.imshow("Processed Image", origin_image)
            cv2.waitKey(1)

        # 4. 결과 Publish
        processed_image_msg = self.br.cv2_to_imgmsg(origin_image, "bgr8")
        processed_image_msg.header = compressed_image.header

        processed_image_compressed_msg = self.br.cv2_to_compressed_imgmsg(origin_image, "jpeg")
        processed_image_compressed_msg.header = compressed_image.header

        self.processed_image_publisher.publish(processed_image_msg)
        self.processed_image_compressed_publisher.publish(processed_image_compressed_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VisualizeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
    except Exception as e:
        node.get_logger().error(f"Exception: {e}")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()