"""LiDAR gap/wall geometry helpers. ROS2 is not required to import this file.

Coordinate convention: x forward, y left, angles counterclockwise (+left).
These helpers assume a horizontal 2D scan in the vessel's frame after applying
only a configured yaw offset. Sensor translation must be calibrated separately.
"""
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class Entrance:
    bearing_deg: float  # +left from boat bow
    distance_m: float
    width_m: float
    right_edge_deg: float
    left_edge_deg: float
    center_x: float
    center_y: float


@dataclass(frozen=True)
class Walls:
    left_m: float
    right_m: float
    center_y_ahead: float
    heading_right_deg: float
    front_m: float


def to_points(ranges, angle_min, angle_increment, lidar_yaw_deg=0.0,
              valid_min=0.20, valid_max=3.0):
    """Return angle/r/x/y arrays with NaN for invalid beams; preserve ordering."""
    r = np.asarray(ranges, dtype=float)
    angles = angle_min + np.arange(len(r), dtype=float) * angle_increment
    angles = (angles + math.radians(lidar_yaw_deg) + math.pi) % (2*math.pi) - math.pi
    valid = (np.isfinite(r) & (r >= valid_min) & (r <= valid_max))
    a = np.where(valid, angles, np.nan)
    x, y = np.full(r.shape, np.nan), np.full(r.shape, np.nan)
    x[valid] = r[valid] * np.cos(angles[valid])
    y[valid] = r[valid] * np.sin(angles[valid])
    return a, np.where(valid, r, np.nan), x, y


def find_entrances(angles, ranges, x, y, min_width=1.0, max_width=2.6,
                   depth_alignment=0.55, cluster_jump=0.24,
                   min_segment_points=3, fov_deg=80.0):
    """Conservative gap candidates BETWEEN two detected obstacle clusters.

    A single distant back wall does not establish a traversable opening.
    Fail closed if the posts/wall ends cannot be observed. This detector is
    topology-specific and must be calibrated using recorded /scan bag data.
    """
    if len(angles) == 0:
        return []
    mask = (np.isfinite(ranges) & np.isfinite(angles) &
            (np.abs(np.degrees(angles)) <= fov_deg) & (x > 0.20))
    idx = np.flatnonzero(mask)
    if len(idx) == 0:
        return []
    # Splits at missing beam(s), range discontinuity, or physical separation.
    runs = []
    current = [int(idx[0])]
    for j in idx[1:]:
        j = int(j)
        prev = current[-1]
        apart = math.hypot(x[j] - x[prev], y[j] - y[prev])
        if j != prev + 1 or apart > cluster_jump:
            if len(current) >= min_segment_points:
                runs.append(current)
            current = [j]
        else:
            current.append(j)
    if len(current) >= min_segment_points:
        runs.append(current)
    if len(runs) < 2:
        return []
    # LaserScan may have clockwise beam order (angle_increment negative).
    runs.sort(key=lambda arr: float(np.nanmean(angles[arr])))
    candidates = []
    for low, high in zip(runs[:-1], runs[1:]):
        i = low[-1] if angles[low[0]] <= angles[low[-1]] else low[0]
        j = high[0] if angles[high[0]] <= angles[high[-1]] else high[-1]
        a1, a2 = float(angles[i]), float(angles[j])
        if a2 - a1 < math.radians(5.0):
            continue
        p1 = np.array([x[i], y[i]])
        p2 = np.array([x[j], y[j]])
        width = float(np.linalg.norm(p2 - p1))
        # Real doorway edges need similar forward depth, not one post + back wall.
        if not (min_width <= width <= max_width):
            continue
        if abs(p1[0] - p2[0]) > depth_alignment:
            continue
        center = (p1 + p2) / 2
        if center[0] < 0.3:
            continue
        candidates.append(Entrance(
            bearing_deg=math.degrees(math.atan2(center[1], center[0])),
            distance_m=float(np.linalg.norm(center)),
            width_m=width,
            right_edge_deg=math.degrees(a1),
            left_edge_deg=math.degrees(a2),
            center_x=float(center[0]),
            center_y=float(center[1])
        ))
    return candidates


def choose_target_entrance(entrances, target_lidar_bearing_deg,
                           max_center_diff_deg=15.0,
                           ray_margin_deg=2.0, ambiguity_margin_deg=5.0):
    """Pick a unique gap whose angular interval contains the detected target.

    Returns None on ambiguity, not a guessed adjacent bay.
    """
    eligible = [e for e in entrances
                if e.right_edge_deg - ray_margin_deg <= target_lidar_bearing_deg
                <= e.left_edge_deg + ray_margin_deg
                and abs(e.bearing_deg - target_lidar_bearing_deg) <= max_center_diff_deg]
    if not eligible:
        return None
    ranked = sorted(eligible, key=lambda e: abs(e.bearing_deg - target_lidar_bearing_deg))
    if len(ranked) > 1:
        delta = (abs(ranked[1].bearing_deg - target_lidar_bearing_deg)
                 - abs(ranked[0].bearing_deg - target_lidar_bearing_deg))
        if delta < ambiguity_margin_deg:
            return None
    return ranked[0]


def estimate_walls(angles, ranges, x, y, *, lookahead_m=0.80,
                   min_line_span=0.25, wall_half_range=1.65,
                   front_sector_deg=12.0):
    """Estimate two near-parallel side walls with y(x) robust linear fits.

    Returns None if side walls or front barrier are not reliably measured.
    """
    if len(angles) == 0:
        return None
    degrees = np.degrees(angles)
    valid = np.isfinite(ranges) & np.isfinite(x) & (x > -0.25) & (x < 2.4)

    def fit_side(side):
        m = (valid & (degrees * side >= 32.0) & (degrees * side <= 120.0)
             & (y * side >= 0.0) & (y * side <= wall_half_range))
        xx = x[m]
        yy = y[m]
        if xx.size < 6 or np.ptp(xx) < min_line_span:
            return None
        # First fit; trim outliers around wall candidate and refit.
        slope, intercept = np.polyfit(xx, yy, 1)
        residual = np.abs(yy - (slope*xx + intercept))
        kept = residual < 0.17
        if kept.sum() < 6 or np.ptp(xx[kept]) < min_line_span:
            return None
        slope, intercept = np.polyfit(xx[kept], yy[kept], 1)
        if abs(slope) > 1.0:  # Walls cannot tilt >45 deg relative to bow
            return None
        return slope, intercept

    left = fit_side(+1)
    right = fit_side(-1)
    if left is None or right is None:
        return None
    lx = left[0] * lookahead_m + left[1]
    rx = right[0] * lookahead_m + right[1]
    if lx <= 0 or rx >= 0 or lx - rx < 0.55:
        return None
    in_front = (np.isfinite(ranges) & (np.abs(degrees) <= front_sector_deg)
                & (x > 0))
    front = float(np.nanmin(ranges[in_front])) if np.any(in_front) else math.inf
    # +y means left; right-turn target angle is negative of it.
    mid_y = float((lx + rx) / 2)
    heading_right = -math.degrees(math.atan2(mid_y, lookahead_m))
    return Walls(float(lx), float(-rx), mid_y, heading_right, front)
