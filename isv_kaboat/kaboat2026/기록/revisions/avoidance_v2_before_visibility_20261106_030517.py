"""V2 waypoint avoidance: cluster obstacle returns, then select a gap center."""
from dataclasses import dataclass
import math

import numpy as np

from .core import wrap
from .lidar_preprocess import dbscan
from .local_dock import motion_clear
from .route_mission import RouteMission


@dataclass(frozen=True)
class Gap:
    right: float
    left: float
    right_point: np.ndarray
    left_point: np.ndarray

    @property
    def center(self):
        return (self.right_point + self.left_point) / 2

    @property
    def width(self):
        return float(np.linalg.norm(self.left_point - self.right_point))

    @property
    def angle(self):
        x, y = self.center
        return math.atan2(y, x)


def choose_gap(scan, desired, config, radius):
    fov = math.radians(config.avoidance_fov_deg)
    limit = min(scan.range_max, config.avoidance_range)
    points = scan.points(scan.yaw, scan.x, scan.y)
    distances = np.linalg.norm(points, axis=1)
    closest = float(distances.min()) if len(points) else None
    diagnostics = dict(closest=closest, radius=radius, clusters=0, candidates=0,
                       free=0, rejected_collision=0, rejected_no_numeric_range=0)
    report = dict(algorithm="V2_DBSCAN_GAP", reason="NO_SAFE_GAP", gaps=[],
                  diagnostics=diagnostics,
                  obstacles=points[::max(1, math.ceil(len(points) / 250))].tolist())
    if closest is not None and closest < radius:
        report["reason"] = "OBSTACLE_STOP"
        return None, report

    ranges = np.asarray(scan.ranges, dtype=float)
    rays = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment + scan.yaw
    valid = (np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)) | np.isposinf(ranges)

    def clear(angle):
        delta = np.arctan2(np.sin(rays - angle), np.cos(rays - angle))
        observed = np.abs(delta) <= max(math.radians(1.5), abs(scan.angle_increment))
        if not np.any(valid & observed):
            diagnostics["rejected_no_numeric_range"] += 1
            return False
        c, s = math.cos(angle), math.sin(angle)
        along = points[:, 0] * c + points[:, 1] * s
        across = points[:, 1] * c - points[:, 0] * s
        horizon = min(config.avoidance_horizon, limit)
        if np.any(np.hypot(along - np.clip(along, 0, horizon), across) < radius):
            diagnostics["rejected_collision"] += 1
            return False
        return True

    angles = np.arctan2(points[:, 1], points[:, 0])
    front = (np.abs(angles) <= fov) & (distances <= limit)
    front_points, front_angles = points[front], angles[front]
    labels = dbscan(front_points, eps=0.2, min_points=3)
    clusters = []
    for label in np.unique(labels[labels >= 0]):
        indices = np.flatnonzero(labels == label)
        right = indices[np.argmin(front_angles[indices])]
        left = indices[np.argmax(front_angles[indices])]
        clusters.append((front_angles[right], front_angles[left],
                         front_points[right], front_points[left]))
    diagnostics["clusters"] = len(clusters)

    # Positive angles are port/CCW in the current boat frame.
    right_angle = -fov
    right_point = limit * np.array([math.cos(fov), -math.sin(fov)])
    end_point = limit * np.array([math.cos(fov), math.sin(fov)])
    gaps = []
    boundaries = sorted(clusters, key=lambda item: item[0])
    boundaries.append((fov, fov, end_point, end_point))
    for near_angle, far_angle, near_point, far_point in boundaries:
        if near_angle - right_angle > math.radians(10):
            gaps.append(Gap(right_angle, near_angle, right_point, near_point))
        if far_angle > right_angle:
            right_angle, right_point = far_angle, far_point

    # Merge nearby gap headings only when their combined center is traversable.
    merged = []
    for gap in gaps:
        if merged and abs(gap.angle - merged[-1].angle) < math.radians(10):
            previous = merged[-1]
            combined = Gap(previous.right, gap.left, previous.right_point, gap.left_point)
            if clear(combined.angle):
                merged[-1] = combined
                continue
        merged.append(gap)

    direct_angle = float(np.clip(desired, -fov, fov))
    direct_clear = clear(direct_angle)
    minimum_width = max(0.6, 2 * radius)
    safe = [gap for gap in merged if gap.width >= minimum_width and clear(gap.angle)]
    diagnostics["candidates"] = len(merged)
    diagnostics["free"] = len(safe)
    report["gaps"] = [
        dict(left_deg=math.degrees(gap.left), right_deg=math.degrees(gap.right),
             width_m=gap.width, center_deg=math.degrees(gap.angle))
        for gap in safe
    ]
    report["direct_clear"] = direct_clear
    report["selected_direct"] = not clusters and direct_clear
    if report["selected_direct"]:
        angle, distance = direct_angle, limit
    elif safe:
        chosen = min(safe, key=lambda gap: abs(float(wrap(gap.angle - desired))))
        angle, distance = chosen.angle, float(np.linalg.norm(chosen.center))
    else:
        return None, report
    report.update(reason="CLEAR", selected_angle_deg=math.degrees(angle),
                  selected_distance=distance)
    return angle, report


class RouteMissionV2(RouteMission):
    def use_selected_corridor(self, desired):
        # V2 never turns toward an unchecked goal outside the scan sector.
        return True

    def select_corridor(self, scan, desired, radius):
        return choose_gap(scan, desired, self.route_config, radius)

    REVERSE_HORIZON = 0.8
    REVERSE_PWM = 1350
    REVERSE_NEUTRAL_DELAY = 0.25
    REVERSE_LIMIT = 3.0
    FORWARD_CONFIRM = 0.6

    def reset(self):
        super().reset()
        self.recovery_started = None
        self.forward_clear_since = None

    @property
    def motion_states(self):
        return super().motion_states + ("RECOVERY_REVERSE", "RECOVERY_WAIT")

    def step(self, now, scan):
        state, percentages = super().step(now, scan)
        forward = state in ("TO_WP1", "WP1_TO_WP2", "AVOID_OBSTACLE")
        if not self.executing or not (forward or state == "NO_SAFE_GAP"):
            self.recovery_started = self.forward_clear_since = None
            return state, percentages

        if forward:
            if self.recovery_started is None:
                return state, percentages
            if self.forward_clear_since is None or scan.stamp < self.forward_clear_since:
                self.forward_clear_since = scan.stamp
            if scan.stamp - self.forward_clear_since >= self.FORWARD_CONFIRM:
                self.recovery_started = self.forward_clear_since = None
                return state, percentages
            return self.recovery_neutral("CONFIRM_FORWARD")

        self.forward_clear_since = None
        rear = self.raw_scan
        radius = self.config.footprint_radius + self.config.clearance
        if (rear is None or not -.1 <= now - rear.stamp <= self.config.scan_timeout
                or not motion_clear(rear, np.array([-self.REVERSE_HORIZON, 0.]),
                                    radius, horizon=self.REVERSE_HORIZON)):
            self.avoidance["recovery"] = "REAR_BLOCKED_OR_UNOBSERVED"
            return state, percentages

        if self.recovery_started is None:
            self.recovery_started = now
        elapsed = now - self.recovery_started
        if elapsed >= self.REVERSE_LIMIT:
            self.avoidance["recovery"] = "REVERSE_LIMIT_REACHED"
            return state, percentages
        if elapsed < self.REVERSE_NEUTRAL_DELAY:
            return self.recovery_neutral("BEFORE_REVERSE")
        self.command_kind = "RECOVERY_REVERSE"
        self.intended_pwm = (self.REVERSE_PWM, self.REVERSE_PWM)
        self.avoidance["recovery"] = "REVERSING"
        return "RECOVERY_REVERSE", self.percentages()

    def recovery_neutral(self, reason):
        self.command_kind = "NEUTRAL"
        self.intended_pwm = (1500, 1500)
        self.avoidance["recovery"] = reason
        return "RECOVERY_WAIT", (0., 0.)
