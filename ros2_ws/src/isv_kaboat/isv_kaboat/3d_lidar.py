#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class LidarNear1mVisualizerNode(Node):
    def __init__(self):
        super().__init__("lidar_near1m_visualizer")

        # ----- 파라미터 / 기본 설정 -----
        self.declare_parameter("input_topic", "/ouster/points")
        self.declare_parameter("output_topic", "/lidar/near1m")
        self.declare_parameter("radius_max", 1.0)

        self.input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        self.output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self.radius_max = float(self.get_parameter("radius_max").get_parameter_value().double_value)

        self.get_logger().info(
            f"[Init] input={self.input_topic}  output={self.output_topic}  "
            f"radius_max={self.radius_max:.2f} m"
        )

        # ----- QoS: LiDAR용 BEST_EFFORT -----
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ----- ROS I/O -----
        self.sub = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.cloud_cb,
            sensor_qos,
        )
        self.pub_near = self.create_publisher(
            PointCloud2,
            self.output_topic,
            sensor_qos,
        )

        self.msg_count = 0

    # =============== 콜백 ===============
    def cloud_cb(self, msg: PointCloud2):
        # 0) x,y,z만 추출
        xyz_all = self._extract_xyz(msg)
        if xyz_all is None or xyz_all.size == 0:
            return

        # 1) 반경 r <= radius_max 인 포인트만 필터링
        #    여기서 r은 수평거리만 보고 싶으면 sqrt(x^2 + y^2),
        #    3D 거리로 보고 싶으면 sqrt(x^2 + y^2 + z^2) 중 골라쓰면 됨.
        x = xyz_all[:, 0]
        y = xyz_all[:, 1]
        z = xyz_all[:, 2]

        # 수평 거리 기준 (x,y) 기준 1m
        r = np.sqrt(x * x + y * y)
        mask = r <= self.radius_max

        xyz_near = xyz_all[mask] if mask.any() else np.zeros((0, 3), dtype=np.float32)

        # 2) PointCloud2로 변환해서 퍼블리시
        pc2_msg = self._xyz_to_pc2(xyz_near, msg.header)
        self.pub_near.publish(pc2_msg)

        # 3) 디버깅 로그
        self.msg_count += 1
        if self.msg_count <= 5 or self.msg_count % 30 == 0:
            self.get_logger().info(
                f"[near1m] #{self.msg_count} all={xyz_all.shape[0]}  "
                f"near={xyz_near.shape[0]} (r<={self.radius_max:.2f}m)"
            )

    # =============== 유틸 ===============
    @staticmethod
    def _field_offset(fields, name, default=None):
        for f in fields:
            if f.name == name:
                return f.offset
        return default

    def _extract_xyz(self, msg: PointCloud2):
        """PointCloud2 → (N,3) float32 (x,y,z만)."""
        try:
            n = msg.width * msg.height
            if n == 0:
                return None

            x_off = self._field_offset(msg.fields, "x")
            y_off = self._field_offset(msg.fields, "y")
            z_off = self._field_offset(msg.fields, "z")

            if x_off is None or y_off is None or z_off is None:
                self.get_logger().error("x/y/z field missing in PointCloud2")
                return None

            dtype = np.dtype(
                {
                    "names": ["x", "y", "z"],
                    "formats": ["<f4", "<f4", "<f4"],
                    "offsets": [x_off, y_off, z_off],
                    "itemsize": msg.point_step,
                }
            )
            arr = np.frombuffer(msg.data, dtype=dtype, count=n)

            xyz = np.empty((n, 3), dtype=np.float32)
            xyz[:, 0] = arr["x"]
            xyz[:, 1] = arr["y"]
            xyz[:, 2] = arr["z"]

            # NaN 제거
            xyz = xyz[np.isfinite(xyz).all(axis=1)]
            return xyz if xyz.size else None

        except Exception as e:
            self.get_logger().error(f"_extract_xyz error: {e}")
            return None

    @staticmethod
    def _xyz_to_pc2(points_xyz: np.ndarray, header: Header) -> PointCloud2:
        """(N,3) float32 → PointCloud2 (x,y,z만)."""
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = int(points_xyz.shape[0])

        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = points_xyz.astype(np.float32, copy=False).tobytes()
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = LidarNear1mVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
