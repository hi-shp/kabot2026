"""User pipeline adapted to Velodyne and the installed NumPy/SciPy runtime."""
import json
import math
import time
from collections import deque

import numpy as np
from scipy.spatial import cKDTree
from scipy.optimize import linear_sum_assignment


def extract_xyz(msg):
    fields = {f.name: f for f in msg.fields}
    if not all(k in fields and fields[k].datatype == 7 and fields[k].count == 1 for k in ('x', 'y', 'z')):
        raise ValueError('PointCloud2 requires FLOAT32 x/y/z fields')
    if msg.width * msg.height == 0:
        return np.empty((0, 3), dtype=np.float32)
    dtype = np.dtype(dict(names=['x', 'y', 'z'], formats=[('>' if msg.is_bigendian else '<')+'f4']*3,
                         offsets=[fields[k].offset for k in ('x', 'y', 'z')], itemsize=msg.point_step))
    arr = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                     strides=(msg.row_step, msg.point_step))
    xyz = np.column_stack([arr[k].ravel() for k in ('x', 'y', 'z')])
    return xyz[np.isfinite(xyz).all(axis=1)]


def filter_points(xyz, low, high, max_range):
    if not len(xyz):
        return xyz, xyz
    elevation = np.degrees(np.arctan2(xyz[:, 2], np.hypot(xyz[:, 0], xyz[:, 1])))
    fov = xyz[(elevation >= low) & (elevation <= high)]
    return fov, fov[np.linalg.norm(fov, axis=1) <= max_range]


def voxel_centroids(xyz, size):
    if not len(xyz):
        return xyz.copy()
    _, inverse = np.unique(np.floor(xyz / size).astype(np.int64), axis=0, return_inverse=True)
    counts = np.bincount(inverse)
    return np.column_stack([np.bincount(inverse, weights=xyz[:, i])/counts for i in range(3)]).astype(np.float32)


def statistical_outliers(xyz, neighbors, std_ratio):
    if len(xyz) < 3:
        return xyz.copy()
    distances, _ = cKDTree(xyz).query(xyz, k=min(len(xyz), neighbors+1))
    means = distances[:, 1:].mean(axis=1)
    return xyz[means <= means.mean()+std_ratio*means.std()]


def dbscan(xyz, eps, min_points):
    labels = np.full(len(xyz), -1, dtype=np.int32)
    if not len(xyz):
        return labels
    tree = cKDTree(xyz)
    visited = np.zeros(len(xyz), dtype=bool)
    cluster = 0
    for i in range(len(xyz)):
        if visited[i]:
            continue
        visited[i] = True
        neighbors = tree.query_ball_point(xyz[i], eps)
        if len(neighbors) < min_points:
            continue
        labels[i] = cluster
        queue = deque(neighbors)
        queued = set(neighbors)
        while queue:
            j = queue.popleft()
            if not visited[j]:
                visited[j] = True
                group = tree.query_ball_point(xyz[j], eps)
                if len(group) >= min_points:
                    for k in group:
                        if k not in queued:
                            queued.add(k)
                            queue.append(k)
            if labels[j] < 0:
                labels[j] = cluster
        cluster += 1
    return labels


def adaptive_dbscan(xyz, max_range, base_eps, alpha, min_points):
    labels = np.full(len(xyz), -1, dtype=np.int32)
    distances = np.linalg.norm(xyz, axis=1)
    next_id = 0
    for low, high, final in [(0., max_range/2, False), (max_range/2, max_range, True)]:
        mask = (distances >= low) & ((distances <= high) if final else (distances < high))
        part = dbscan(xyz[mask], base_eps*(1+alpha*(low+high)/2), min_points)
        part[part >= 0] += next_id
        labels[mask] = part
        if np.any(part >= 0):
            next_id = int(part.max())+1
    return labels


def navigation_slice(xyz, minimum_elevation=0., maximum_elevation=2.):
    """Match the backup VLP16 +1 degree horizontal ring for 2D navigation.

    Full YDH FOV/SOR/DBSCAN clouds and boxes remain unchanged. Only the planar
    scan projection excludes upper/lower layers, which otherwise flatten ground
    returns into collision obstacles. No generic ground plane is assumed.
    """
    if not len(xyz):
        return xyz.copy()
    elevation = np.degrees(np.arctan2(xyz[:, 2], np.hypot(xyz[:, 0], xyz[:, 1])))
    return xyz[(elevation >= minimum_elevation) & (elevation <= maximum_elevation)]


def project_ranges(xyz, resolution, minimum, maximum):
    count = int(math.ceil(2*math.pi/resolution))
    increment = 2*math.pi/count
    ranges = np.full(count, np.inf, dtype=np.float32)
    if len(xyz):
        distances = np.hypot(xyz[:, 0], xyz[:, 1])
        valid = (distances >= minimum) & (distances <= maximum)
        angles = np.arctan2(xyz[valid, 1], xyz[valid, 0])
        indices = np.floor((angles+math.pi)/increment).astype(int) % count
        np.minimum.at(ranges, indices, distances[valid])
    return ranges, increment


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import PointCloud2, PointField, LaserScan
    from std_msgs.msg import String
    from visualization_msgs.msg import Marker, MarkerArray

    class Preprocessor(Node):
        def __init__(self):
            super().__init__('lidar_preprocess')
            defaults = dict(input_topic='/velodyne_points', scan_topic='/scan_preprocessed',
                fov_min_deg=-15., fov_max_deg=15., max_range_m=2.5, min_range_m=.1,
                voxel_size=.03, sor_neighbors=30, sor_std_ratio=1.5,
                dbscan_eps=.13, dbscan_min_points=8, eps_range_alpha=.10,
                min_points_for_bbox=50, max_points_for_bbox=300,
                scan_resolution=.007, protected_radius=.2, max_input_age=.5,
                scan_elevation_min_deg=0., scan_elevation_max_deg=2.)
            for k, v in defaults.items():
                self.declare_parameter(k, v)
            self.p = {k: self.get_parameter(k).value for k in defaults}
            p = self.p
            if not (-90 <= p['fov_min_deg'] < p['fov_max_deg'] <= 90):
                raise ValueError('Invalid vertical FOV')
            for key in ('max_range_m', 'min_range_m', 'voxel_size', 'sor_std_ratio',
                        'dbscan_eps', 'scan_resolution', 'max_input_age'):
                if not math.isfinite(p[key]) or p[key] <= 0:
                    raise ValueError('Invalid '+key)
            if not (-90 <= p['scan_elevation_min_deg'] < p['scan_elevation_max_deg'] <= 90):
                raise ValueError('Invalid navigation scan elevation slice')
            if not p['min_range_m'] < p['max_range_m'] or not 0 <= p['protected_radius'] < p['max_range_m']:
                raise ValueError('Invalid distance limits')
            if p['sor_neighbors'] < 2 or p['dbscan_min_points'] < 2 or p['eps_range_alpha'] < 0:
                raise ValueError('Invalid clustering parameters')
            if not 1 <= p['min_points_for_bbox'] <= p['max_points_for_bbox']:
                raise ValueError('Invalid bbox point limits')
            qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=1)
            self.raw_pub = self.create_publisher(PointCloud2, '/o3d_voxel/raw', qos)
            self.ds_pub = self.create_publisher(PointCloud2, '/o3d_voxel/downsampled', qos)
            self.cl_pub = self.create_publisher(PointCloud2, '/o3d_voxel/clusters', qos)
            self.bb_pub = self.create_publisher(MarkerArray, '/o3d_voxel/bboxes', qos)
            self.scan_pub = self.create_publisher(LaserScan, p['scan_topic'], qos)
            self.status_pub = self.create_publisher(String, '/lidar_preprocess/status', 10)
            self.subscription = self.create_subscription(PointCloud2, p['input_topic'], self.receive, qos)
            self.previous = {}
            self.next_id = 0
            self.log_at = -math.inf
            self.get_logger().info('PREPROCESS_CONFIG '+json.dumps(p))

        def cloud(self, xyz, header, colors=None):
            msg = PointCloud2()
            msg.header = header
            msg.height, msg.width = 1, len(xyz)
            names = ['x', 'y', 'z'] + (['rgb'] if colors is not None else [])
            msg.fields = [PointField(name=n, offset=4*i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate(names)]
            msg.point_step = 4*len(names)
            msg.row_step = msg.point_step*msg.width
            msg.is_bigendian, msg.is_dense = False, True
            data = xyz if colors is None else np.column_stack((xyz, colors))
            msg.data = np.asarray(data, dtype='<f4').tobytes()
            return msg

        def stable_labels(self, xyz, labels):
            ids = np.unique(labels[labels >= 0])
            centers = {int(i): xyz[labels == i].mean(axis=0) for i in ids}
            mapping = {}
            if centers and self.previous:
                current_ids, prior_ids = list(centers), list(self.previous)
                distance = np.linalg.norm(np.array([centers[i] for i in current_ids])[:, None, :]
                    - np.array([self.previous[i] for i in prior_ids])[None, :, :], axis=2)
                rows, cols = linear_sum_assignment(distance)
                for i, j in zip(rows, cols):
                    if distance[i, j] <= .4:
                        mapping[current_ids[i]] = prior_ids[j]
            for i in centers:
                if i not in mapping:
                    mapping[i] = self.next_id
                    self.next_id += 1
            self.previous = {mapping[i]: center for i, center in centers.items()}
            return np.array([mapping.get(int(i), -1) for i in labels], dtype=np.int32)

        def receive(self, msg):
            started = time.monotonic()
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            age = self.get_clock().now().nanoseconds/1e9-stamp
            if not -.1 <= age <= self.p['max_input_age']:
                return
            try:
                all_points = extract_xyz(msg)
                fov, raw = filter_points(all_points, self.p['fov_min_deg'], self.p['fov_max_deg'], self.p['max_range_m'])
                voxel = voxel_centroids(raw, self.p['voxel_size'])
                ds = statistical_outliers(voxel, self.p['sor_neighbors'], self.p['sor_std_ratio'])
                labels = adaptive_dbscan(ds, self.p['max_range_m'], self.p['dbscan_eps'],
                    self.p['eps_range_alpha'], self.p['dbscan_min_points'])
                labels = self.stable_labels(ds, labels)
                self.raw_pub.publish(self.cloud(raw, msg.header))
                self.ds_pub.publish(self.cloud(ds, msg.header))
                palette = np.array([0xe6194b, 0x3cb44b, 0xffe119, 0x0082c8, 0xf58230, 0x911eb4], dtype='<u4')
                colors = np.full(len(labels), 0xa0a0a0, dtype='<u4')
                valid = labels >= 0
                colors[valid] = palette[labels[valid] % len(palette)]
                self.cl_pub.publish(self.cloud(ds, msg.header, colors.view('<f4')))
                boxes = MarkerArray()
                clear = Marker()
                clear.header, clear.action = msg.header, Marker.DELETEALL
                boxes.markers.append(clear)
                for cid in np.unique(labels[labels >= 0]):
                    group = ds[labels == cid]
                    if not self.p['min_points_for_bbox'] <= len(group) <= self.p['max_points_for_bbox']:
                        continue
                    lower, upper = group.min(axis=0), group.max(axis=0)
                    center, size = (lower+upper)/2, np.maximum(upper-lower, .001)
                    box = Marker()
                    box.header, box.ns, box.id = msg.header, 'o3d_bboxes', int(cid)
                    box.type, box.action = Marker.CUBE, Marker.ADD
                    box.pose.position.x, box.pose.position.y, box.pose.position.z = map(float, center)
                    box.pose.orientation.w = 1.
                    box.scale.x, box.scale.y, box.scale.z = map(float, size)
                    box.color.b, box.color.a = 1., .35
                    box.lifetime.nanosec = 500000000
                    boxes.markers.append(box)
                self.bb_pub.publish(boxes)
                # DBSCAN noise remains in navigation; bbox count filters are display-only.
                # Preserve raw returns inside the configured hull guard despite SOR.
                protected = raw[np.hypot(raw[:, 0], raw[:, 1]) < self.p['protected_radius']]
                navigation = navigation_slice(ds, self.p['scan_elevation_min_deg'], self.p['scan_elevation_max_deg'])
                scan_points = np.concatenate((navigation, protected))
                ranges, increment = project_ranges(scan_points, self.p['scan_resolution'],
                    self.p['min_range_m'], self.p['max_range_m'])
                scan = LaserScan()
                scan.header = msg.header
                scan.angle_min, scan.angle_increment = -math.pi, increment
                scan.angle_max = scan.angle_min+(len(ranges)-1)*increment
                scan.range_min, scan.range_max = self.p['min_range_m'], self.p['max_range_m']
                scan.ranges = ranges.tolist()
                self.scan_pub.publish(scan)
                status = dict(input=len(all_points), fov=len(fov), range_filtered=len(raw), voxel=len(voxel),
                    sor=len(ds), navigation_points=len(navigation), clusters=len(np.unique(labels[labels >= 0])), protected=len(protected),
                    finite_scan=int(np.count_nonzero(np.isfinite(ranges))),
                    processing_ms=round((time.monotonic()-started)*1000, 2), input_age=round(age, 3))
                self.status_pub.publish(String(data=json.dumps(status)))
                if started-self.log_at >= 1:
                    self.log_at = started
                    self.get_logger().info('PREPROCESS '+json.dumps(status))
            except (ValueError, RuntimeError, IndexError) as exc:
                self.get_logger().error('PREPROCESS_ERROR '+str(exc))

    rclpy.init()
    node = Preprocessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
