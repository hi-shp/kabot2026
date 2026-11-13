#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 PointCloud2: FOV(-25°~+25°, elevation) → Voxel(centroid) → SOR
VSCode에서 바로 실행 가능. intensity는 전혀 사용하지 않음. Open3D 시각화 없음.

- Input  : /ouster/points
- Output :
    /o3d_voxel/raw          (FOV 필터만 적용된 원본 xyz)
    /o3d_voxel/downsampled  (Voxel → SOR 결과 xyz)
- QoS    : BEST_EFFORT (sensor stream friendly)
- Steps  : [NEW] 수직 FOV 필터(기하 기반 elevation) → Voxel grid (centroid) → SOR
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import open3d as o3d
import math

# ====== 기본 설정 ======
DEFAULT_INPUT_TOPIC    = "/ouster/points"

DEFAULT_PUB_RAW_TOPIC  = "/o3d_voxel/raw"
DEFAULT_PUB_DS_TOPIC   = "/o3d_voxel/downsampled"

# 수직 FOV(도): -25° ~ +25° 만 통과
DEFAULT_FOV_MIN_DEG    = -25.0
DEFAULT_FOV_MAX_DEG    =  25.0

# LiDAR 수직축 (일반적으로 z-up)
# up_axis 옵션: 'z' | 'y' | 'x'
DEFAULT_UP_AXIS        = 'z'

# 다운샘플링
DEFAULT_VOXEL_SIZE     = 0.05     # m

# 이상치 제거: "none" | "sor" | "ror" (기본 SOR)
DEFAULT_OUTLIER_METHOD   = "sor"
DEFAULT_SOR_NEIGHBORS    = 20
DEFAULT_SOR_STD_RATIO    = 2.0
DEFAULT_ROR_RADIUS       = 0.20   # m (사용 안함)
DEFAULT_ROR_MIN_NEIGHBOR = 4      # (사용 안함)

DEFAULT_PUBLISH_RAW    = True


class VoxelCentroidNode(Node):
    def __init__(self):
        super().__init__('voxel_centroid_downsampler_fov_geom')

        # ---- 실행 기본값 ----
        self.input_topic   = DEFAULT_INPUT_TOPIC

        self.pub_raw_topic = DEFAULT_PUB_RAW_TOPIC
        self.pub_ds_topic  = DEFAULT_PUB_DS_TOPIC
        self.publish_raw   = bool(DEFAULT_PUBLISH_RAW)

        self.fov_min_deg   = float(DEFAULT_FOV_MIN_DEG)
        self.fov_max_deg   = float(DEFAULT_FOV_MAX_DEG)
        self.up_axis       = DEFAULT_UP_AXIS.lower()  # 'z','y','x'

        self.voxel_size    = float(DEFAULT_VOXEL_SIZE)

        self.outlier_method    = DEFAULT_OUTLIER_METHOD.lower()
        self.sor_nb_neighbors  = int(DEFAULT_SOR_NEIGHBORS)
        self.sor_std_ratio     = float(DEFAULT_SOR_STD_RATIO)
        self.ror_radius        = float(DEFAULT_ROR_RADIUS)
        self.ror_min_neighbors = int(DEFAULT_ROR_MIN_NEIGHBOR)

        self.get_logger().info(
            "[Init] input=%s | FOV=[%.1f, %.1f]deg (up=%s) | voxel=%.3f | outlier=%s | publish_raw=%s"
            % (self.input_topic, self.fov_min_deg, self.fov_max_deg, self.up_axis,
               self.voxel_size, self.outlier_method.upper(), self.publish_raw)
        )

        # ---- QoS ----
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ---- ROS I/O ----
        self.sub = self.create_subscription(PointCloud2, self.input_topic, self.cloud_cb, sensor_qos)
        self.pub_ds  = self.create_publisher(PointCloud2, self.pub_ds_topic,  sensor_qos)
        self.pub_raw = self.create_publisher(PointCloud2, self.pub_raw_topic, sensor_qos) if self.publish_raw else None

        self.msg_count = 0

    # ================= Callback =================
    def cloud_cb(self, msg: PointCloud2):
        # 0) x,y,z만 추출
        xyz_all = self._extract_xyz(msg)
        if xyz_all is None or xyz_all.size == 0:
            return

        # 1) FOV 필터(기하 기반 elevation)
        xyz_fov = self._filter_by_vertical_fov_geom(xyz_all, self.fov_min_deg, self.fov_max_deg, self.up_axis)

        # 2) Voxel centroid
        xyz_voxel = self._voxel_downsample_centroid(xyz_fov, self.voxel_size)

        # 3) Open3D 이상치 제거 (기본 SOR)
        if self.outlier_method in ('sor', 'ror'):
            xyz_ds = self._remove_outliers_open3d(xyz_voxel, self.outlier_method)
        else:
            xyz_ds = xyz_voxel

        # Publish
        if self.pub_raw is not None:
            # "raw"는 FOV 필터 후의 원본 모양 (downsample 전)
            self.pub_raw.publish(self._xyz_to_pc2(xyz_fov, msg.header))
        self.pub_ds.publish(self._xyz_to_pc2(xyz_ds, msg.header))

        # 로그
        self.msg_count += 1
        if self.msg_count <= 5 or self.msg_count % 30 == 0:
            ratio = (xyz_fov.shape[0] / max(1, xyz_all.shape[0])) * 100.0
            self.get_logger().info(
                f"[recv] #{self.msg_count} all={xyz_all.shape[0]}  FOV={xyz_fov.shape[0]} ({ratio:.1f}%)  "
                f"voxel={xyz_voxel.shape[0]}  ds={xyz_ds.shape[0]}  "
                f"voxel={self.voxel_size:.3f}  outlier={self.outlier_method}"
            )

    # ================= Utils ====================
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
            x_off = self._field_offset(msg.fields, 'x')
            y_off = self._field_offset(msg.fields, 'y')
            z_off = self._field_offset(msg.fields, 'z')
            if x_off is None or y_off is None or z_off is None:
                self.get_logger().error("x/y/z field missing")
                return None

            dtype = np.dtype({
                'names':   ['x','y','z'],
                'formats': ['<f4','<f4','<f4'],
                'offsets': [x_off, y_off, z_off],
                'itemsize': msg.point_step
            })
            arr = np.frombuffer(msg.data, dtype=dtype, count=n)

            xyz = np.empty((n,3), dtype=np.float32)
            xyz[:,0] = arr['x']
            xyz[:,1] = arr['y']
            xyz[:,2] = arr['z']
            xyz = xyz[np.isfinite(xyz).all(axis=1)]
            return xyz if xyz.size else None
        except Exception as e:
            self.get_logger().error(f"extract_xyz error: {e}")
            return None

    def _filter_by_vertical_fov_geom(self, xyz: np.ndarray,
                                     deg_min: float, deg_max: float,
                                     up_axis: str = 'z') -> np.ndarray:
        """
        좌표로 직접 고각(elevation) 계산하여 [-25°, +25°]만 통과.
        - up_axis='z' : elev = atan2(z, sqrt(x^2 + y^2))
          up_axis='y' : elev = atan2(y, sqrt(x^2 + z^2))
          up_axis='x' : elev = atan2(x, sqrt(y^2 + z^2))
        """
        if xyz.shape[0] == 0:
            return xyz

        ax = up_axis.lower()
        if ax == 'z':
            rho = np.sqrt(np.maximum(0.0, xyz[:,0]**2 + xyz[:,1]**2))
            elev_deg = np.degrees(np.arctan2(xyz[:,2], rho))
        elif ax == 'y':
            rho = np.sqrt(np.maximum(0.0, xyz[:,0]**2 + xyz[:,2]**2))
            elev_deg = np.degrees(np.arctan2(xyz[:,1], rho))
        elif ax == 'x':
            rho = np.sqrt(np.maximum(0.0, xyz[:,1]**2 + xyz[:,2]**2))
            elev_deg = np.degrees(np.arctan2(xyz[:,0], rho))
        else:
            # 잘못된 입력 시 z-up으로 처리
            rho = np.sqrt(np.maximum(0.0, xyz[:,0]**2 + xyz[:,1]**2))
            elev_deg = np.degrees(np.arctan2(xyz[:,2], rho))

        mask = (elev_deg >= deg_min) & (elev_deg <= deg_max)
        sel = xyz[mask] if mask.any() else np.zeros((0,3), dtype=np.float32)

        # 초기 몇 프레임 디버그 로그(범위 확인)
        if self.msg_count < 10:
            if mask.any():
                self.get_logger().info(
                    f"[FOV(elev-{ax})] kept {sel.shape[0]}/{xyz.shape[0]} "
                    f"({sel.shape[0]/max(1,xyz.shape[0]):.0%}), "
                    f"range=[{elev_deg[mask].min():.1f},{elev_deg[mask].max():.1f}] deg"
                )
            else:
                self.get_logger().warn("[FOV(elev)] no points in range; returning empty cloud")
        return sel

    def _remove_outliers_open3d(self, xyz: np.ndarray, method: str) -> np.ndarray:
        """Open3D: SOR/ROR 이상치 제거. (기본 SOR)"""
        try:
            pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz.astype(np.float64)))
            if method == 'sor':
                _, ind = pcd.remove_statistical_outlier(
                    nb_neighbors=self.sor_nb_neighbors,
                    std_ratio=self.sor_std_ratio
                )
            elif method == 'ror':
                _, ind = pcd.remove_radius_outlier(
                    nb_points=self.ror_min_neighbors,
                    radius=self.ror_radius
                )
            else:
                return xyz
            if len(ind) == 0:
                return np.zeros((0,3), dtype=np.float32)
            return xyz[ind].astype(np.float32, copy=False)
        except Exception as e:
            self.get_logger().error(f"outlier removal error ({method}): {e}")
            return xyz

    @staticmethod
    def _voxel_downsample_centroid(xyz: np.ndarray, voxel: float) -> np.ndarray:
        """Voxel-grid 다운샘플: 각 voxel의 XYZ 평균(centroid)."""
        if voxel <= 0.0 or xyz.shape[0] == 0:
            return xyz
        vids = np.floor(xyz / voxel).astype(np.int64)  # (N,3)
        key  = vids.view(np.dtype((np.void, vids.dtype.itemsize * vids.shape[1]))).ravel()
        _, inv = np.unique(key, return_inverse=True)
        counts = np.bincount(inv)
        sum_x  = np.bincount(inv, weights=xyz[:,0])
        sum_y  = np.bincount(inv, weights=xyz[:,1])
        sum_z  = np.bincount(inv, weights=xyz[:,2])
        centroids = np.column_stack((sum_x, sum_y, sum_z)) / counts[:,None]
        return centroids.astype(np.float32, copy=False)

    @staticmethod
    def _xyz_to_pc2(points_xyz: np.ndarray, header: Header) -> PointCloud2:
        """(N,3) float32 → PointCloud2 (x,y,z만)."""
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width  = int(points_xyz.shape[0])
        msg.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step   = msg.point_step * msg.width
        msg.is_dense   = True
        msg.data = points_xyz.astype(np.float32, copy=False).tobytes()
        return msg


def main():
    # VSCode에서 python 파일만 실행해도 ROS2 노드로 동작
    rclpy.init()
    node = VoxelCentroidNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
