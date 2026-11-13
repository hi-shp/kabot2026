#!/usr/bin/env python3
# cam_auto_capture.py
import time
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import numpy as np
import cv2

# ====== 설정 ======
CAMERA_TOPIC = "/camera1/image_raw"      # 실제 카메라 토픽명
CAMERA_IS_COMPRESSED = False             # CompressedImage이면 True
CAPTURE_INTERVAL = 1.0                   # 자동 저장 간격 (초)
# ==================

SAVE_DIR = Path.home() / "camera_screenshot"


class CamAutoCapture(Node):
    def __init__(self) -> None:
        super().__init__("cam_auto_capture")

        SAVE_DIR.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f"[auto-cap] save dir: {SAVE_DIR}")

        self.bridge = CvBridge()
        self.last_raw: Optional[np.ndarray] = None
        self.last_comp: Optional[bytes] = None

        # --- Subscriber ---
        if CAMERA_IS_COMPRESSED:
            self.create_subscription(
                CompressedImage, CAMERA_TOPIC, self._on_comp, qos_profile_sensor_data
            )
            self.get_logger().info(f"[auto-cap] subscribe: {CAMERA_TOPIC} (CompressedImage)")
        else:
            self.create_subscription(
                Image, CAMERA_TOPIC, self._on_raw, qos_profile_sensor_data
            )
            self.get_logger().info(f"[auto-cap] subscribe: {CAMERA_TOPIC} (Image)")

        # --- 자동 캡처 타이머 ---
        self.create_timer(CAPTURE_INTERVAL, self._auto_capture)
        self.get_logger().info(f"[auto-cap] auto capture every {CAPTURE_INTERVAL} sec enabled.")

    # ---------- callbacks ----------
    def _on_raw(self, msg: Image) -> None:
        try:
            self.last_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"[auto-cap] raw convert error: {e}")

    def _on_comp(self, msg: CompressedImage) -> None:
        try:
            self.last_comp = bytes(msg.data)
        except Exception as e:
            self.get_logger().warn(f"[auto-cap] comp read error: {e}")

    # ---------- auto save ----------
    def _auto_capture(self) -> None:
        ts = time.strftime("%Y%m%d_%H%M%S")

        # Compressed 이미지 처리
        if CAMERA_IS_COMPRESSED and self.last_comp:
            out = SAVE_DIR / f"cap_{ts}.jpg"
            try:
                with open(out, "wb") as f:
                    f.write(self.last_comp)
                self.get_logger().info(f"[auto-cap] saved (jpg) → {out}")
            except Exception as e:
                self.get_logger().warn(f"[auto-cap] save error: {e}")
            return

        # Raw 이미지 처리
        if self.last_raw is not None:
            out = SAVE_DIR / f"cap_{ts}.png"
            try:
                if cv2.imwrite(str(out), self.last_raw):
                    self.get_logger().info(f"[auto-cap] saved (png) → {out}")
                else:
                    self.get_logger().warn(f"[auto-cap] cv2.imwrite failed: {out}")
            except Exception as e:
                self.get_logger().warn(f"[auto-cap] save error: {e}")
            return

        self.get_logger().warn("[auto-cap] no frame yet; nothing to save.")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CamAutoCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

