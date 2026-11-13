"""Trimmed ICP and covariance-weighted GICP using NumPy/SciPy on ARM64.

    The returned transform maps raw source into target coordinates (absolute T).
    correction = T @ inverse(initial); never multiply initial a second time.
"""
from dataclasses import dataclass
import math
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
from .geometry import apply, rigid, transform


@dataclass
class Result:
    matrix: np.ndarray
    accepted: bool
    reason: str
    rmse: float = math.inf
    inlier_ratio: float = 0.
    converged: bool = False
    information_ratio: float = 0.
    iterations: int = 0

    def metrics(self):
        return dict(accepted=self.accepted, reason=self.reason, rmse=self.rmse,
                    inlier_ratio=self.inlier_ratio, converged=self.converged,
                    information_ratio=self.information_ratio, iterations=self.iterations)


def surface_covariances(points, neighbors=15):
    _, ids = cKDTree(points).query(points, k=min(neighbors, len(points)))
    samples = points[ids]
    centered = samples - samples.mean(axis=1, keepdims=True)
    covariance = np.einsum('nki,nkj->nij', centered, centered) / samples.shape[1]
    eigenvalues, vectors = np.linalg.eigh(covariance)
    # GICP local plane covariance: small variance normal to surface.
    diagonal = np.tile([.01, 1., 1.], (len(points), 1))
    regularized = (vectors * diagonal[:, None, :]) @ vectors.transpose(0, 2, 1)
    return regularized, vectors[:, :, 0]


def align(source, target, initial, config):
    initial = rigid(initial).copy()
    source, target = np.asarray(source, float), np.asarray(target, float)
    if (source.ndim != 2 or target.ndim != 2 or source.shape[1:] != (3,)
            or target.shape[1:] != (3,) or not np.isfinite(source).all() or not np.isfinite(target).all()
            or min(len(source), len(target)) < config['min_correspondences']):
        return Result(initial, False, 'INSUFFICIENT_POINTS')
    method = config['method']
    if method not in ('icp', 'gicp'):
        raise ValueError('method must be icp or gicp')
    tree = cKDTree(target)
    target_cov, normals = surface_covariances(target, config['covariance_neighbors'])
    source_cov = surface_covariances(source, config['covariance_neighbors'])[0] if method == 'gicp' else None
    current = initial.copy()
    converged = False
    for iteration in range(config['max_iterations']):
        world = apply(source, current)
        distances, indices = tree.query(world)
        valid = np.flatnonzero(distances <= config['max_correspondence_m'])
        if len(valid) < config['min_correspondences']:
            return Result(current, False, 'NO_OVERLAP', iterations=iteration+1)
        count = max(config['min_correspondences'], int(len(valid) * config['trim_fraction']))
        valid = valid[np.argsort(distances[valid])[:count]]
        src, dst = world[valid], target[indices[valid]]
        if method == 'icp':
            a, b = src.mean(axis=0), dst.mean(axis=0)
            u, _, vt = np.linalg.svd((src-a).T @ (dst-b))
            rotation = vt.T @ u.T
            if np.linalg.det(rotation) < 0:
                vt[-1] *= -1
                rotation = vt.T @ u.T
            delta = transform(rotation, b - rotation @ a)
        else:
            rotated_cov = current[:3, :3] @ source_cov[valid] @ current[:3, :3].T
            covariance = target_cov[indices[valid]] + rotated_cov
            whitening = np.linalg.inv(np.linalg.cholesky(covariance))
            pivot = src.mean(axis=0)
            def residual(value):
                rotated = (src-pivot) @ Rotation.from_rotvec(value[3:]).as_matrix().T + pivot + value[:3]
                return np.einsum('nij,nj->ni', whitening, rotated-dst).ravel()
            fit = least_squares(residual, np.zeros(6), loss='soft_l1', f_scale=.1,
                                max_nfev=config['inner_iterations'], ftol=1e-6, xtol=1e-6, gtol=1e-6)
            if not np.isfinite(fit.x).all():
                return Result(current, False, 'OPTIMIZER_FAILED', iterations=iteration+1)
            rotation = Rotation.from_rotvec(fit.x[3:]).as_matrix()
            delta = transform(rotation, pivot + fit.x[:3] - rotation @ pivot)
        next_matrix = delta @ current
        movement = np.linalg.norm(next_matrix[:3, 3] - current[:3, 3])
        turn = Rotation.from_matrix(delta[:3, :3]).magnitude()
        current = rigid(next_matrix)
        if movement <= config['translation_tolerance_m'] and turn <= math.radians(config['rotation_tolerance_deg']):
            converged = True
            break
    world = apply(source, current)
    distances, ids = tree.query(world)
    valid = distances <= config['max_correspondence_m']
    ratio = float(np.mean(valid))
    if np.count_nonzero(valid) < config['min_correspondences']:
        return Result(current, False, 'NO_OVERLAP', inlier_ratio=ratio, iterations=iteration+1)
    # RMSE includes all distance-gated correspondences, not only trimmed pairs.
    rmse = float(np.sqrt(np.mean(distances[valid]**2)))
    points = world[valid]
    local = points - points.mean(axis=0)
    scale = max(.1, float(np.sqrt(np.mean(np.sum(local**2, axis=1)))))
    ns = normals[ids[valid]]
    jacobian = np.concatenate([ns, np.cross(local/scale, ns)], axis=1)
    eigen = np.linalg.eigvalsh(jacobian.T @ jacobian / len(points))
    information_ratio = float(max(0, eigen[0]) / max(1e-12, eigen[-1]))
    correction_translation = float(np.linalg.norm(current[:3, 3] - initial[:3, 3]))
    correction_rotation = math.degrees(Rotation.from_matrix(current[:3, :3] @ initial[:3, :3].T).magnitude())
    reason = 'CONVERGED'
    if not converged: reason = 'NOT_CONVERGED'
    elif rmse > config['max_rmse_m']: reason = 'HIGH_RMSE'
    elif ratio < config['min_inlier_ratio']: reason = 'LOW_INLIER_RATIO'
    elif information_ratio < config['min_information_ratio']: reason = 'DEGENERATE_GEOMETRY'
    elif correction_translation > config['max_translation_correction_m']: reason = 'POSITION_JUMP'
    elif correction_rotation > config['max_rotation_correction_deg']: reason = 'ROTATION_JUMP'
    return Result(current, reason == 'CONVERGED', reason, rmse, ratio, converged, information_ratio, iteration+1)
