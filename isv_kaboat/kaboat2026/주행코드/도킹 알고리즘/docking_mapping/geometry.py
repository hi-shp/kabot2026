"""WGS84 geodetic <-> fixed-origin ENU and explicit sensor extrinsics."""
import numpy as np
from pyproj import Transformer
from scipy.spatial.transform import Rotation


class ENU:
    def __init__(self, origin):
        self.origin = np.asarray(origin, dtype=float)
        if (self.origin.shape != (3,) or not np.isfinite(self.origin).all()
                or abs(self.origin[0]) > 90 or abs(self.origin[1]) > 180):
            raise ValueError('Invalid latitude, longitude, ellipsoid altitude origin')
        self.forward = Transformer.from_crs(4979, 4978, always_xy=True)
        self.inverse = Transformer.from_crs(4978, 4979, always_xy=True)
        lat, lon, alt = self.origin
        self.ecef0 = np.array(self.forward.transform(lon, lat, alt))
        p, l = np.radians([lat, lon])
        self.r = np.array([[-np.sin(l), np.cos(l), 0],
                           [-np.sin(p)*np.cos(l), -np.sin(p)*np.sin(l), np.cos(p)],
                           [np.cos(p)*np.cos(l), np.cos(p)*np.sin(l), np.sin(p)]])

    def to_enu(self, lla):
        values = np.asarray(lla, dtype=float)
        if (values.shape[-1] != 3 or not np.isfinite(values).all()
                or np.any(np.abs(values[..., 0]) > 90) or np.any(np.abs(values[..., 1]) > 180)):
            raise ValueError('Invalid WGS84 coordinates')
        xyz = np.stack(self.forward.transform(values[..., 1], values[..., 0], values[..., 2]), axis=-1)
        return (xyz - self.ecef0) @ self.r.T

    def to_lla(self, xyz):
        xyz = np.asarray(xyz, dtype=float)
        if xyz.shape[-1] != 3 or not np.isfinite(xyz).all():
            raise ValueError('Invalid ENU coordinates')
        ecef = xyz @ self.r + self.ecef0
        lon, lat, alt = self.inverse.transform(ecef[..., 0], ecef[..., 1], ecef[..., 2])
        return np.stack([lat, lon, alt], axis=-1)


def transform(rotation=None, translation=(0, 0, 0)):
    result = np.eye(4)
    if rotation is not None:
        result[:3, :3] = rotation
    result[:3, 3] = translation
    return result


def apply(points, matrix):
    return np.asarray(points) @ matrix[:3, :3].T + matrix[:3, 3]


def rigid(matrix):
    matrix = np.asarray(matrix, dtype=float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0, 0, 0, 1])
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(matrix[:3, :3]), 1)):
        raise ValueError('Invalid SE(3) transform')
    return matrix


def boat_pose(enu_gps, quaternion, config):
    """R_ENU_base = R_true_magnetic * R_ENU_imu * R_imu_base.

    VectorNav use_enu=true exposes RFU_TO_ENU, not FLU_TO_ENU.
    R_imu_base must explicitly account for RFU and physical mounting.
    Declination positive east means true heading = magnetic heading + declination.
    """
    if not config['extrinsics_verified']:
        raise ValueError('WAIT_EXTRINSICS')
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or not .9 < np.linalg.norm(q) < 1.1:
        raise ValueError('Invalid IMU quaternion')
    reference = config['heading_reference']
    if reference not in ('true', 'magnetic', 'relative'):
        raise ValueError('WAIT_HEADING_REFERENCE')
    declination = 0.
    if reference == 'magnetic':
        declination = config['magnetic_declination_deg']
        if declination is None or not np.isfinite(declination):
            raise ValueError('WAIT_DECLINATION')
    world_imu = Rotation.from_euler('z', -declination, degrees=True).as_matrix() @ Rotation.from_quat(q).as_matrix()
    imu_base = Rotation.from_euler('xyz', config['imu_from_base_rpy_deg'], degrees=True).as_matrix()
    world_base = world_imu @ imu_base
    antenna = np.asarray(config['gps_antenna_in_base_m'], dtype=float)
    return transform(world_base, np.asarray(enu_gps) - world_base @ antenna)


def lidar_extrinsic(config):
    return transform(Rotation.from_euler('xyz', config['lidar_in_base_rpy_deg'], degrees=True).as_matrix(),
                     config['lidar_in_base_xyz_m'])


def voxel(points, size, limit=None):
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    if not len(points):
        return points
    keys, inverse = np.unique(np.floor(points / size).astype(np.int64), axis=0, return_inverse=True)
    sums = np.zeros((len(keys), 3))
    np.add.at(sums, inverse, points)
    result = sums / np.bincount(inverse)[:, None]
    if limit and len(result) > limit:
        result = result[np.linspace(0, len(result)-1, limit, dtype=int)]
    return result
