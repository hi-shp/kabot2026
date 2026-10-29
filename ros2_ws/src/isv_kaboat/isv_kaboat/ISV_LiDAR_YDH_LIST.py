#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 PointCloud2: FOV(-15°~+15°, elevation) → Range(<=2.5m) → Voxel(centroid) → SOR → DBSCAN(밀도기반, 거리-적응 ε)
- 서로 다른 객체는 서로 다른 색상(rgb)으로 RViz에서 시각화
- 각 클러스터에 대해 AABB 바운딩박스를 MarkerArray로 퍼블리시

Topics:
  Input :
    /ouster/points
  Output:
    /o3d_voxel/raw           (FOV+Range 필터)
    /o3d_voxel/downsampled   (Voxel→SOR 결과)
    /o3d_voxel/clusters      (DBSCAN 라벨 색상 포함 PointCloud2)
    /o3d_voxel/bboxes        (클러스터 바운딩박스 MarkerArray)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
import open3d as o3d
import struct

# ================= 기본 설정 =================
DEFAULT_INPUT_TOPIC   = "/ouster/points"
DEFAULT_PUB_RAW_TOPIC = "/o3d_voxel/raw"
DEFAULT_PUB_DS_TOPIC  = "/o3d_voxel/downsampled"
DEFAULT_PUB_CL_TOPIC  = "/o3d_voxel/clusters"
DEFAULT_PUB_BB_TOPIC  = "/o3d_voxel/bboxes"

# FOV / Range / Downsample
DEFAULT_FOV_MIN_DEG = -15.0
DEFAULT_FOV_MAX_DEG =  15.0
DEFAULT_UP_AXIS     = 'z'
DEFAULT_MAX_RANGE_M = 2.5
DEFAULT_VOXEL_SIZE  = 0.03

# 이상치 제거(SOR)
DEFAULT_OUTLIER_METHOD = "sor"
DEFAULT_SOR_NEIGHBORS  = 30
DEFAULT_SOR_STD_RATIO  = 1.5

DEFAULT_PUBLISH_RAW = True

# ===== DBSCAN(밀도기반) 설정 =====
# base eps는 가까운 구간에 해당, 멀수록 eps를 증가시킴
DEFAULT_DBSCAN_EPS        = 0.13
DEFAULT_DBSCAN_MIN_POINTS = 8
DEFAULT_DBSCAN_PRINT      = False
# 거리 적응 계수: eps_r = base_eps * (1 + eps_range_alpha * r_mid)
DEFAULT_EPS_RANGE_ALPHA   = 0.10

# ===== 바운딩박스 필터(점 개수 기준) =====
DEFAULT_MIN_POINTS_FOR_BBOX = 50
DEFAULT_MAX_POINTS_FOR_BBOX = 300


class VoxelCentroidNode(Node):
    def __init__(self):
        super().__init__('voxel_centroid_dbscan_bbox')

        # 토픽/처리 파라미터
        self.input_topic   = DEFAULT_INPUT_TOPIC
        self.pub_raw_topic = DEFAULT_PUB_RAW_TOPIC
        self.pub_ds_topic  = DEFAULT_PUB_DS_TOPIC
        self.pub_cl_topic  = DEFAULT_PUB_CL_TOPIC
        self.pub_bb_topic  = DEFAULT_PUB_BB_TOPIC
        self.publish_raw   = bool(DEFAULT_PUBLISH_RAW)

        self.fov_min_deg = float(DEFAULT_FOV_MIN_DEG)
        self.fov_max_deg = float(DEFAULT_FOV_MAX_DEG)
        self.up_axis     = DEFAULT_UP_AXIS.lower()
        self.max_range_m = float(DEFAULT_MAX_RANGE_M)
        self.voxel_size  = float(DEFAULT_VOXEL_SIZE)

        self.outlier_method   = DEFAULT_OUTLIER_METHOD.lower()
        self.sor_nb_neighbors = int(DEFAULT_SOR_NEIGHBORS)
        self.sor_std_ratio    = float(DEFAULT_SOR_STD_RATIO)

        # DBSCAN
        self.db_eps        = float(DEFAULT_DBSCAN_EPS)
        self.db_min_points = int(DEFAULT_DBSCAN_MIN_POINTS)
        self.db_print      = bool(DEFAULT_DBSCAN_PRINT)
        self.eps_range_alpha = float(DEFAULT_EPS_RANGE_ALPHA)

        # 바운딩박스 점 개수 필터
        self.min_points_bbox = int(DEFAULT_MIN_POINTS_FOR_BBOX)
        self.max_points_bbox = int(DEFAULT_MAX_POINTS_FOR_BBOX)

        # 라벨 안정화를 위한 이전 프레임 중심(노이즈 제외)
        self.prev_centers = None  # shape: (K_prev, 3)

        self.get_logger().info(
            f"[Init] input={self.input_topic}, FOV=[{self.fov_min_deg},{self.fov_max_deg}]deg, "
            f"max_range={self.max_range_m}m, voxel={self.voxel_size}, outlier={self.outlier_method}, "
            f"DBSCAN(base_eps={self.db_eps}, min_pts={self.db_min_points}, eps_range_alpha={self.eps_range_alpha}), "
            f"BBox(min_pts={self.min_points_bbox}, max_pts={self.max_points_bbox})"
        )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.sub     = self.create_subscription(PointCloud2, self.input_topic, self.cloud_cb, qos)
        self.pub_ds  = self.create_publisher(PointCloud2, self.pub_ds_topic, qos)
        self.pub_raw = self.create_publisher(PointCloud2, self.pub_raw_topic, qos)
        self.pub_cl  = self.create_publisher(PointCloud2, self.pub_cl_topic, qos)
        self.pub_bb  = self.create_publisher(MarkerArray, self.pub_bb_topic, qos)

        self.msg_count = 0
        self.marker_seq = 0

    # ================= Callback =================
    def cloud_cb(self, msg: PointCloud2):
        # 0) 추출
        xyz_all = self._extract_xyz(msg)
        if xyz_all is None or xyz_all.size == 0:
            self._publish_empty_markers(msg.header)
            return

        # 1) FOV 필터
        xyz_fov = self._filter_by_vertical_fov(xyz_all, self.fov_min_deg, self.fov_max_deg, self.up_axis)

        # 2) 거리 필터
        xyz_fr = self._filter_by_max_range(xyz_fov, self.max_range_m)

        # 3) Voxel centroid
        xyz_voxel = self._voxel_downsample_centroid(xyz_fr, self.voxel_size)

        # 4) SOR
        xyz_ds = self._remove_outliers_sor(xyz_voxel) if self.outlier_method == "sor" else xyz_voxel

        # 5) DBSCAN (거리-적응 ε)
        if xyz_ds.shape[0] > 0:
            labels = self._dbscan_open3d_range_adaptive(
                xyz_ds,
                base_eps=self.db_eps,
                min_points=self.db_min_points,
                print_progress=self.db_print
            )

            # 현재 프레임의 클러스터(노이즈 제외) 중심 계산
            centers, valid_ids = self._centers_from_labels(xyz_ds, labels)

            # 프레임 간 라벨 안정화
            if self.prev_centers is not None and self.prev_centers.shape[0] > 0 and centers.shape[0] > 0:
                labels = self._remap_dbscan_labels_by_centers(labels, centers, self.prev_centers, valid_ids)
                centers, _ = self._centers_from_labels(xyz_ds, labels)

            self.prev_centers = centers.copy() if centers.shape[0] > 0 else None

            # 포인트 publish (rgb)
            rgb_float = self._dbscan_labels_to_rgb32(labels)
            xyzrgb = np.column_stack((xyz_ds.astype(np.float32), rgb_float))
            self.pub_cl.publish(self._xyzrgb_to_pc2(xyzrgb, msg.header))

            # 바운딩박스 publish (MarkerArray)
            self._publish_bboxes_from_labels(xyz_ds, labels, msg.header)
        else:
            self.prev_centers = None
            self.pub_cl.publish(self._xyzrgb_to_pc2(np.zeros((0,4), dtype=np.float32), msg.header))
            self._publish_empty_markers(msg.header)

        # 퍼블리시
        if self.publish_raw:
            self.pub_raw.publish(self._xyz_to_pc2(xyz_fr, msg.header))
        self.pub_ds.publish(self._xyz_to_pc2(xyz_ds, msg.header))

        # 로그
        self.msg_count += 1
        if self.msg_count <= 5 or self.msg_count % 30 == 0:
            self.get_logger().info(
                f"[recv #{self.msg_count}] all={xyz_all.shape[0]}  FOV={xyz_fov.shape[0]}  "
                f"R<={self.max_range_m:.1f}m={xyz_fr.shape[0]}  DS={xyz_ds.shape[0]}  "
                f"DBSCAN(base_eps={self.db_eps}, min={self.db_min_points}, alpha={self.eps_range_alpha})"
            )

    # ==================== Utils ====================
    def _extract_xyz(self, msg: PointCloud2):
        try:
            n = msg.width * msg.height
            if n == 0:
                return None
            dtype = np.dtype({
                'names': ['x', 'y', 'z'],
                'formats': ['<f4', '<f4', '<f4'],
                'offsets': [0, 4, 8],  # 센서 커스텀인 경우 수정 필요
                'itemsize': msg.point_step
            })
            arr = np.frombuffer(msg.data, dtype=dtype, count=n)
            xyz = np.vstack((arr['x'], arr['y'], arr['z'])).T
            return xyz[np.isfinite(xyz).all(axis=1)]
        except Exception as e:
            self.get_logger().error(f"extract_xyz error: {e}")
            return None

    def _filter_by_vertical_fov(self, xyz, deg_min, deg_max, up_axis='z'):
        if xyz.shape[0] == 0:
            return xyz
        ax = up_axis.lower()
        if ax == 'z':
            elev = np.degrees(np.arctan2(xyz[:,2], np.sqrt(xyz[:,0]**2 + xyz[:,1]**2)))
        elif ax == 'y':
            elev = np.degrees(np.arctan2(xyz[:,1], np.sqrt(xyz[:,0]**2 + xyz[:,2]**2)))
        else:
            elev = np.degrees(np.arctan2(xyz[:,0], np.sqrt(xyz[:,1]**2 + xyz[:,2]**2)))
        mask = (elev >= deg_min) & (elev <= deg_max)
        return xyz[mask]

    def _filter_by_max_range(self, xyz, max_r):
        if xyz.shape[0] == 0:
            return xyz
        mask = np.einsum('ij,ij->i', xyz, xyz) <= max_r**2
        return xyz[mask]

    def _remove_outliers_sor(self, xyz):
        try:
            pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz.astype(np.float64)))
            _, ind = pcd.remove_statistical_outlier(
                nb_neighbors=self.sor_nb_neighbors,
                std_ratio=self.sor_std_ratio
            )
            return xyz[ind] if len(ind) > 0 else np.zeros((0,3), dtype=np.float32)
        except Exception as e:
            self.get_logger().error(f"SOR error: {e}")
            return xyz

    def _voxel_downsample_centroid(self, xyz, voxel):
        if voxel <= 0.0 or xyz.shape[0] == 0:
            return xyz
        vids = np.floor(xyz / voxel).astype(np.int64)
        key  = vids.view(np.dtype((np.void, vids.dtype.itemsize * vids.shape[1]))).ravel()
        _, inv = np.unique(key, return_inverse=True)
        counts = np.bincount(inv)
        sums = np.array([np.bincount(inv, weights=xyz[:, i]) for i in range(3)]).T
        return (sums / counts[:, None]).astype(np.float32)

    # =============== DBSCAN (거리-적응 ε) ===============
    def _dbscan_open3d_range_adaptive(self, X: np.ndarray, base_eps: float, min_points: int, print_progress: bool = False):
        """
        거리 r에 따라 eps를 키워주는 pragmatic 방식:
        - r을 구간(bin)으로 나누고,
        - 각 구간에서 eps = base_eps * (1 + eps_range_alpha * r_mid)로 DBSCAN 실행,
        - 라벨을 이어붙여 최종 labels 반환.
        """
        r = np.linalg.norm(X, axis=1)

        # 구간 자동 설정: [0, 0.5*max_r], (0.5*max_r, max_r], > max_r
        max_r = max(1e-6, float(self.max_range_m))
        bins = [0.0, 0.5*max_r, max_r, np.inf]

        labels = -np.ones(X.shape[0], dtype=np.int32)
        next_label = 0

        for b in range(len(bins) - 1):
            m = (r >= bins[b]) & (r < bins[b+1])
            if not np.any(m):
                continue
            Xb = X[m]
            # 해당 구간 중심 거리
            r_mid = 0.5 * (bins[b] + bins[b+1])
            eps = base_eps * (1.0 + self.eps_range_alpha * max(0.0, r_mid))

            pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(Xb.astype(np.float64)))
            lab = np.asarray(
                pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=print_progress),
                dtype=np.int32
            )

            valid = lab >= 0
            if np.any(valid):
                unique = np.unique(lab[valid])
                # 구간 내 라벨 → 글로벌 라벨로 remap
                remap = {int(u): (next_label + i) for i, u in enumerate(unique)}
                out = -np.ones_like(lab)
                for u in unique:
                    out[lab == u] = remap[int(u)]
                labels[m] = out
                next_label += len(unique)
            # 노이즈(-1)는 그대로 유지

        return labels

    def _centers_from_labels(self, X: np.ndarray, labels: np.ndarray):
        if labels.size == 0 or X.shape[0] == 0:
            return np.zeros((0,3), dtype=np.float32), np.array([], dtype=np.int32)
        unique = np.array(sorted([l for l in np.unique(labels) if l >= 0]), dtype=np.int32)
        if unique.size == 0:
            return np.zeros((0,3), dtype=np.float32), np.array([], dtype=np.int32)
        centers = []
        for cid in unique:
            pts = X[labels == cid]
            centers.append(pts.mean(axis=0))
        centers = np.vstack(centers).astype(np.float32)
        return centers, unique

    def _remap_dbscan_labels_by_centers(self, labels, centers, prev_centers, valid_ids):
        K_prev = prev_centers.shape[0]
        K_now  = centers.shape[0]
        if K_prev == 0 or K_now == 0:
            return labels

        dist = np.linalg.norm(centers[:, None, :] - prev_centers[None, :, :], axis=2)
        mapping_now_to_prev = np.full(K_now, -1, dtype=np.int32)
        used_prev = set()
        for _ in range(min(K_now, K_prev)):
            i, j = divmod(dist.argmin(), K_prev)
            while j in used_prev:
                dist[i, j] = np.inf
                i, j = divmod(dist.argmin(), K_prev)
            mapping_now_to_prev[i] = j
            used_prev.add(j)
            dist[i, :] = np.inf
            dist[:, j] = np.inf

        id_to_prev = { int(valid_ids[i]) : int(mapping_now_to_prev[i]) for i in range(K_now) if mapping_now_to_prev[i] != -1 }

        remapped = labels.copy()
        for idx in np.where(labels >= 0)[0]:
            old_id = int(labels[idx])
            if old_id in id_to_prev:
                remapped[idx] = id_to_prev[old_id]
            else:
                remapped[idx] = K_prev + (old_id % 10)  # 새 클러스터는 뒤쪽 번호
        return remapped

    # --------- RViz rgb(float32) 유틸 ----------
    def _pack_rgb_float32(self, r, g, b):
        return np.frombuffer(struct.pack('I', (int(r) << 16) | (int(g) << 8) | int(b)), dtype=np.float32)[0]

    def _dbscan_labels_to_rgb32(self, labels: np.ndarray) -> np.ndarray:
        if labels is None or labels.size == 0:
            return np.zeros((0,1), dtype=np.float32)
        palette = np.array([
            [230,  25,  75], [ 60, 180,  75], [255, 225,  25], [  0, 130, 200],
            [245, 130,  48], [145,  30, 180], [ 70, 240, 240], [240,  50, 230],
            [210, 245,  60], [250, 190, 190], [  0, 128, 128], [230, 190, 255],
            [170, 110,  40], [255, 250, 200], [128,   0,   0], [170, 255, 195],
            [128, 128,   0], [255, 215, 180], [  0,   0, 128], [128, 128, 128],
        ], dtype=np.uint8)
        noise_color = np.array([160,160,160], dtype=np.uint8)

        max_label = labels[labels >= 0].max() if np.any(labels >= 0) else -1
        rgb = np.zeros((labels.shape[0], 1), dtype=np.float32)
        if max_label >= 0:
            for k in range(max_label + 1):
                r, g, b = palette[k % len(palette)]
                rgb[labels == k, 0] = self._pack_rgb_float32(r, g, b)
        rgb[labels == -1, 0] = self._pack_rgb_float32(*noise_color.tolist())
        return rgb

    # --------- PointCloud2 빌더 ----------
    def _xyz_to_pc2(self, xyz, header):
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = int(xyz.shape[0])
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step   = msg.point_step * msg.width
        msg.is_dense   = True
        msg.data = xyz.astype(np.float32).tobytes()
        return msg

    def _xyzrgb_to_pc2(self, xyzrgb, header):
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = int(xyzrgb.shape[0])
        msg.fields = [
            PointField(name='x',   offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',   offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',   offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step   = msg.point_step * msg.width
        msg.is_dense   = True
        msg.data = xyzrgb.astype(np.float32).tobytes()
        return msg

    # --------- Bounding Box MarkerArray 퍼블리시 ----------
    def _publish_bboxes_from_labels(self, X: np.ndarray, labels: np.ndarray, header: Header):
        """
        labels(길이 N, -1=노이즈)와 점 X(N,3)에서 각 클러스터의 AABB를 계산해 MarkerArray로 퍼블리시.
        MIN/MAX 점 개수 기준 필터 적용.
        """
        ma = MarkerArray()

        # 유효 라벨들
        unique = [int(l) for l in np.unique(labels) if l >= 0]
        marker_id = 0

        for cid in unique:
            idx = np.where(labels == cid)[0]
            count = idx.size
            if count < self.min_points_bbox or count > self.max_points_bbox:
                continue

            pts = X[idx]
            mins = pts.min(axis=0)
            maxs = pts.max(axis=0)
            center = (mins + maxs) / 2.0
            size   = np.maximum(maxs - mins, 1e-3)  # 0 방지

            m = Marker()
            m.header = header
            m.ns = "o3d_bboxes"
            m.id = marker_id
            marker_id += 1
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = float(center[0])
            m.pose.position.y = float(center[1])
            m.pose.position.z = float(center[2])
            # AABB이므로 회전 없음
            m.pose.orientation.x = 0.0
            m.pose.orientation.y = 0.0
            m.pose.orientation.z = 0.0
            m.pose.orientation.w = 1.0
            m.scale.x = float(size[0])
            m.scale.y = float(size[1])
            m.scale.z = float(size[2])
            # 파란색 + 반투명
            m.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.35)
            m.lifetime.sec = 0   # 0이면 영속, 프레임마다 덮어씀
            m.frame_locked = False

            ma.markers.append(m)

        if len(ma.markers) == 0:
            # 이전 것 삭제
            del_all = Marker()
            del_all.header = header
            del_all.ns = "o3d_bboxes"
            del_all.id = 0
            del_all.action = Marker.DELETEALL
            ma.markers.append(del_all)

        self.pub_bb.publish(ma)

    def _publish_empty_markers(self, header: Header):
        ma = MarkerArray()
        del_all = Marker()
        del_all.header = header
        del_all.ns = "o3d_bboxes"
        del_all.id = 0
        del_all.action = Marker.DELETEALL
        ma.markers.append(del_all)
        self.pub_bb.publish(ma)


def main():
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
