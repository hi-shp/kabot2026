#!/usr/bin/env python3
# cam_manual_capture.py
import os, sys, time, select
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import numpy as np
import cv2

# ====== 설정 (필요하면 이 두 줄만 바꾸세요) ======
CAMERA_TOPIC = "/camera1/image_raw"  # 실제 카메라 토픽명
CAMERA_IS_COMPRESSED = False  # 토픽 타입이 CompressedImage면 True
# ==============================================

SAVE_DIR = Path.home() / "camera_screenshot"
HOTKEY = "a"  # 이 키를 누르면 저장

# (리눅스 TTY 전용) 키보드 즉시 입력용
try:
    import termios, tty
except Exception:
    termios = None
    tty = None


class CamManualCapture(Node):
    def __init__(self) -> None:
        super().__init__("cam_manual_capture")
        SAVE_DIR.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f"[cap] save dir: {SAVE_DIR}")

        self.bridge = CvBridge()
        self.last_raw: Optional[np.ndarray] = None
        self.last_comp: Optional[bytes] = None

        if CAMERA_IS_COMPRESSED:
            self.create_subscription(
                CompressedImage, CAMERA_TOPIC, self._on_comp, qos_profile_sensor_data
            )
            self.get_logger().info(f"[cap] subscribe: {CAMERA_TOPIC} (CompressedImage)")
        else:
            self.create_subscription(
                Image, CAMERA_TOPIC, self._on_raw, qos_profile_sensor_data
            )
            self.get_logger().info(f"[cap] subscribe: {CAMERA_TOPIC} (Image)")

        # 키보드 폴링 타이머
        self._kb_enabled = self._enable_keyboard_nonblock()
        if self._kb_enabled:
            self.get_logger().info(f"[cap] press '{HOTKEY}' to save a screenshot")
        else:
            self.get_logger().warn("[cap] keyboard input unavailable (no TTY?).")

        self.create_timer(0.05, self._poll_keyboard)  # 20 Hz

    # ---------- callbacks ----------
    def _on_raw(self, msg: Image) -> None:
        try:
            self.last_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"[cap] raw convert error: {e}")

    def _on_comp(self, msg: CompressedImage) -> None:
        try:
            self.last_comp = bytes(msg.data)  # 그대로 저장 가능
        except Exception as e:
            self.get_logger().warn(f"[cap] comp read error: {e}")

    # ---------- keyboard ----------
    def _enable_keyboard_nonblock(self) -> bool:
        if termios is None or tty is None:
            return False
        try:
            if sys.stdin.isatty():
                self._kb_fd = sys.stdin.fileno()
                self._kb_old = termios.tcgetattr(self._kb_fd)
                tty.setcbreak(self._kb_fd)  # 엔터 없이 1글자씩 읽기
                return True
        except Exception:
            pass
        return False

    def _restore_keyboard(self) -> None:
        if hasattr(self, "_kb_fd") and hasattr(self, "_kb_old"):
            try:
                termios.tcsetattr(self._kb_fd, termios.TCSADRAIN, self._kb_old)
            except Exception:
                pass

    def _poll_keyboard(self) -> None:
        if not self._kb_enabled:
            return
        try:
            r, _, _ = select.select([sys.stdin], [], [], 0)
            if r:
                ch = sys.stdin.read(1)
                if ch.lower() == HOTKEY.lower():
                    self._save_now()
        except Exception:
            pass

    # ---------- save ----------
    def _save_now(self) -> None:
        ts = time.strftime("%Y%m%d_%H%M%S")
        if CAMERA_IS_COMPRESSED and self.last_comp:
            # 압축 이미지는 그대로 jpg 저장
            out = SAVE_DIR / f"cap_{ts}.jpg"
            try:
                with open(out, "wb") as f:
                    f.write(self.last_comp)
                self.get_logger().info(f"[cap] saved (jpg) → {out}")
            except Exception as e:
                self.get_logger().warn(f"[cap] save error: {e}")
            return

        if self.last_raw is not None:
            out = SAVE_DIR / f"cap_{ts}.png"
            try:
                ok = cv2.imwrite(str(out), self.last_raw)
                if ok:
                    self.get_logger().info(f"[cap] saved (png) → {out}")
                else:
                    self.get_logger().warn(f"[cap] cv2.imwrite failed: {out}")
            except Exception as e:
                self.get_logger().warn(f"[cap] save error: {e}")
            return

        self.get_logger().warn("[cap] no frame yet; nothing to save.")

    def destroy_node(self) -> None:
        self._restore_keyboard()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CamManualCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
