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


def corridor_observed(points, angle, horizon, radius):
    # A no-return ray can mean water, a missed wall, or a filtered return.
    # Require a measured return beyond the corridor in all three angular bands.
    relative = np.arctan2(points[:, 1], points[:, 0]) - angle
    relative = np.arctan2(np.sin(relative), np.cos(relative))
    far = np.linalg.norm(points, axis=1) >= horizon + radius
    half = math.atan2(radius, horizon)
    edges = np.linspace(-half, half, 4)
    return all(np.any(far & (relative >= low) & (relative < high))
               for low, high in zip(edges[:-1], edges[1:]))


def choose_gap(scan, desired, config, radius, raw_scan=None):
    fov = math.radians(config.avoidance_fov_deg)
    limit = min(scan.range_max, config.avoidance_range)
    points = scan.points(scan.yaw, scan.x, scan.y)
    distances = np.linalg.norm(points, axis=1)
    horizon = min(config.avoidance_horizon, limit)
    raw_points = (raw_scan.points(raw_scan.yaw, raw_scan.x, raw_scan.y)
                  if raw_scan is not None else np.empty((0, 2)))
    raw_near = raw_points[np.linalg.norm(raw_points, axis=1) < horizon + radius]
    guard_points = np.concatenate((points, raw_near))
    closest = float(np.linalg.norm(guard_points, axis=1).min()) if len(guard_points) else None
    diagnostics = dict(closest=closest, radius=radius, clusters=0, candidates=0,
                       free=0, rejected_collision=0, rejected_unobserved=0)
    report = dict(algorithm="V2_DBSCAN_GAP", reason="NO_SAFE_GAP", gaps=[],
                  diagnostics=diagnostics,
                  obstacles=guard_points[::max(1, math.ceil(len(guard_points) / 250))].tolist())
    if closest is not None and closest < radius:
        report["reason"] = "OBSTACLE_STOP"
        return None, report

    def clear(angle):
        if not corridor_observed(raw_points, angle, horizon, radius):
            diagnostics["rejected_unobserved"] += 1
            return False
        c, s = math.cos(angle), math.sin(angle)
        along = guard_points[:, 0] * c + guard_points[:, 1] * s
        across = guard_points[:, 1] * c - guard_points[:, 0] * s
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
        raw = self.raw_scan
        if raw is not None and abs(scan.stamp - raw.stamp) > self.config.sync_tolerance:
            raw = None
        return choose_gap(scan, desired, self.route_config, radius, raw_scan=raw)

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
        if self.raw_scan is not None and not -.1 <= now - self.raw_scan.stamp <= self.config.scan_timeout:
            self.raw_scan = None
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
                                    radius, horizon=self.REVERSE_HORIZON)
                or not corridor_observed(rear.points(rear.yaw, rear.x, rear.y),
                                         math.pi, self.REVERSE_HORIZON, radius)):
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
