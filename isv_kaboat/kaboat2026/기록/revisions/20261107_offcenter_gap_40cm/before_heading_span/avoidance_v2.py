"""V2 waypoint avoidance: cluster obstacle returns, then select a gap center."""
from dataclasses import dataclass
import math

import numpy as np

from .core import wrap
from .lidar_preprocess import dbscan
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


@dataclass(frozen=True)
class CheckedGap:
    """An original opening with a checked representative heading.

    For a rescued opening the displayed bounds are the tested free heading run.
    Keep the original opening width and reference range for unchanged PWM lookup.
    """
    opening: Gap
    right: float
    left: float
    angle: float
    rescued: bool = False

    @property
    def width(self):
        return self.opening.width

    @property
    def center(self):
        if not self.rescued:
            return self.opening.center
        distance = float(np.linalg.norm(self.opening.center))
        return distance * np.array([math.cos(self.angle), math.sin(self.angle)])


def checked_gap_runs(gap, evaluate):
    """Keep a clear original center; otherwise look inside the same opening.

    Every sampled heading uses the exact same horizon, hull radius, raw points,
    and scan-validity checks as the original center. No safety limit is relaxed.
    """
    if evaluate(gap.angle)[0]:
        return [CheckedGap(gap, gap.right, gap.left, gap.angle)]
    count = max(2, math.ceil((gap.left-gap.right)/math.radians(1.)))
    angles = np.linspace(gap.right, gap.left, count+1)
    valid = np.array([evaluate(float(angle))[0] for angle in angles], dtype=bool)
    runs = np.split(np.flatnonzero(valid), np.flatnonzero(np.diff(np.flatnonzero(valid)) > 1)+1)
    result = []
    for run in runs:
        if len(run) < 2:
            continue
        right, left = float(angles[run[0]]), float(angles[run[-1]])
        # Retain the existing >10 degree minimum candidate opening rule.
        if left-right <= math.radians(10):
            continue
        midpoint = (right+left)/2
        index = min(run, key=lambda i: abs(float(angles[i])-midpoint))
        result.append(CheckedGap(gap, right, left, float(angles[index]), True))
    return result


def corridor_samples_valid(scan, angle, horizon, radius):
    """Allow +inf/no returns; reject missing sectors and invalid range samples.

    A far reflecting wall is not required to authorize a direction. Range data
    still has to cover the hull's angular envelope, and NaN/zero/-inf are never
    evidence of a clear corridor.
    """
    ranges = np.asarray(scan.ranges, dtype=float)
    angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment + scan.yaw
    relative = np.arctan2(np.sin(angles-angle), np.cos(angles-angle))
    half = math.atan2(radius + math.hypot(scan.x, scan.y), max(.2, horizon))
    sector = np.abs(relative) <= half
    valid = ((np.isfinite(ranges) & (ranges >= scan.range_min)
              & (ranges <= scan.range_max)) | np.isposinf(ranges))
    if np.count_nonzero(sector) < 3 or not np.all(valid[sector]):
        return False
    samples = np.sort(relative[sector])
    return bool(np.max(np.diff(np.r_[-half, samples, half]))
                <= max(.04, abs(scan.angle_increment)*2.1))


def score_weights(config):
    values=(config.v2_wp_weight,config.v2_clearance_weight,config.v2_direction_weight)
    total=sum(values)
    return dict(zip(('wp','clearance','direction'),(float(v/total) for v in values)))


def score_candidate(angle, desired, extra_clearance, config, previous_angle=None):
    """Normalized costs of an already collision-checked candidate; lower wins."""
    if extra_clearance<0 or math.isnan(extra_clearance):
        raise ValueError('A colliding candidate must not be scored')
    wp_error=abs(float(wrap(angle-desired)))
    change=abs(float(wrap(angle-previous_angle))) if previous_angle is not None else 0.
    wp_cost=wp_error/math.pi
    clearance_cost=1.-min(1.,extra_clearance/config.v2_preferred_clearance)
    direction_cost=change/math.pi
    weights=score_weights(config)
    terms=dict(weighted_wp=weights['wp']*wp_cost,
               weighted_clearance=weights['clearance']*clearance_cost,
               weighted_direction=weights['direction']*direction_cost)
    return dict(wp_error_deg=math.degrees(wp_error),
        direction_change_deg=math.degrees(change) if previous_angle is not None else None,
        clearance_m=float(extra_clearance) if math.isfinite(extra_clearance) else None,
        wp_cost=wp_cost,clearance_cost=clearance_cost,direction_cost=direction_cost,
        total_score=sum(terms.values()),**terms)


def choose_gap(scan, desired, config, radius, raw_scan=None, previous_angle=None):
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
                       free=0, rejected_collision=0, rejected_invalid_scan=0)
    report = dict(algorithm="V2_DBSCAN_GAP", reason="NO_SAFE_GAP", gaps=[],
                  selection_policy='normalized_wp_clearance_direction',
                  score_weights=score_weights(config),candidate_scores=[],selected_score=None,
                  previous_direction_deg=math.degrees(previous_angle) if previous_angle is not None else None,
                  preferred_clearance_m=config.v2_preferred_clearance,selected_gap_index=None,
                  diagnostics=diagnostics,
                  obstacles=guard_points[::max(1, math.ceil(len(guard_points) / 250))].tolist())
    if closest is not None and closest < radius:
        report["reason"] = "OBSTACLE_STOP"
        return None, report

    evaluations={}
    def evaluate(angle):
        key=round(float(angle),12)
        if key in evaluations:return evaluations[key]
        if (not corridor_samples_valid(scan, angle, horizon, radius)
                or (raw_scan is not None
                    and not corridor_samples_valid(raw_scan, angle, horizon, radius))):
            diagnostics["rejected_invalid_scan"] += 1
            evaluations[key]=(False,0.);return evaluations[key]
        c, s = math.cos(angle), math.sin(angle)
        along = guard_points[:, 0] * c + guard_points[:, 1] * s
        across = guard_points[:, 1] * c - guard_points[:, 0] * s
        distances=np.hypot(along-np.clip(along,0,horizon),across)
        margin=float(distances.min())-radius if len(distances) else math.inf
        if margin<0:
            diagnostics["rejected_collision"] += 1
            evaluations[key]=(False,margin);return evaluations[key]
        evaluations[key]=(True,margin);return evaluations[key]

    def clear(angle):return evaluate(angle)[0]

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
    minimum_width = 2 * radius
    safe = []
    searched = 0
    for gap in merged:
        if gap.width < minimum_width:
            continue
        if not clear(gap.angle):
            searched += 1
        safe.extend(checked_gap_runs(gap, evaluate))
    diagnostics["candidates"] = len(merged)
    diagnostics["free"] = len(safe)
    diagnostics["searched_offcenter"] = searched
    diagnostics["rescued_gaps"] = sum(gap.rescued for gap in safe)
    report["gaps"] = [
        dict(left_deg=math.degrees(gap.left), right_deg=math.degrees(gap.right),
             width_m=gap.width, center_deg=math.degrees(gap.angle),
             representative='checked_free_run' if gap.rescued else 'original_center')
        for gap in safe
    ]
    report["direct_clear"] = direct_clear
    # Preserve the direct-WP policy in open water. Otherwise rank the same
    # collision-checked gap centers as V2, without blending steering angles.
    direct=not clusters and direct_clear
    candidates=([(direct_angle,limit,None,'WP_DIRECT')] if direct else
        [(gap.angle,float(np.linalg.norm(gap.center)),i,'G'+str(i))
         for i,gap in enumerate(safe,1)])
    if not candidates:
        return None, report
    rows=[]
    for angle,distance,index,name in candidates:
        row=score_candidate(angle,desired,evaluate(angle)[1],config,previous_angle)
        row.update(candidate_id=name,gap_index=index,angle_deg=math.degrees(angle),
                   distance_m=distance,selected=False)
        rows.append(row)
    chosen_index=min(range(len(rows)),key=lambda i:(round(rows[i]['total_score'],12),
        rows[i]['wp_cost'],rows[i]['clearance_cost'],rows[i]['direction_cost'],i))
    winner=rows[chosen_index];winner['selected']=True
    angle,distance,index,_=candidates[chosen_index]
    diagnostics['scored_candidates']=len(rows)
    report.update(candidate_scores=rows,selected_direct=direct,
        selected_gap_index=index,selected_score=winner['total_score'])
    for gap_index,gap_report in enumerate(report['gaps'],1):
        gap_report['selected']=gap_index==index
        gap_report['score']=next((row for row in rows if row['gap_index']==gap_index),None)
    report.update(reason="CLEAR", selected_angle_deg=math.degrees(angle),
                  selected_distance=distance)
    return angle, report


class RouteMissionV2(RouteMission):
    def reset(self):
        super().reset()
        self.previous_world_direction=None
        self.previous_direction_stamp=None
        self.previous_direction_stage=None

    def use_selected_corridor(self, desired):
        # V2 never turns toward an unchecked goal outside the scan sector.
        return True

    def select_corridor(self, scan, desired, radius):
        raw = self.raw_scan
        if raw is not None and abs(scan.stamp - raw.stamp) > self.config.sync_tolerance:
            raw = None
        previous=None
        if (self.previous_world_direction is not None and self.heading is not None
                and self.previous_direction_stage==self.stage
                and self.previous_direction_stamp is not None
                and 0<=scan.stamp-self.previous_direction_stamp<=self.config.scan_timeout):
            previous=float(wrap(self.previous_world_direction-self.heading))
        return choose_gap(scan,desired,self.route_config,radius,raw_scan=raw,previous_angle=previous)

    def step(self, now, scan):
        if self.raw_scan is not None and not -.1 <= now - self.raw_scan.stamp <= self.config.scan_timeout:
            self.raw_scan = None
        # No rear-visibility-dependent recovery. A blocked forward route stays
        # at the parent's neutral NO_SAFE_GAP / OBSTACLE_STOP output.
        result=super().step(now, scan)
        angle=self.avoidance.get('selected_angle_deg')
        if (result[0] in ('TO_WP1','WP1_TO_WP2','AVOID_OBSTACLE')
                and self.command_kind in ('HEADING','AVOIDANCE')
                and isinstance(angle,(int,float)) and math.isfinite(angle)
                and self.heading is not None and scan is not None):
            # A held scan must not advance the reference repeatedly as the IMU turns.
            if self.previous_direction_stamp is None or scan.stamp>self.previous_direction_stamp:
                self.previous_world_direction=float(wrap(self.heading+math.radians(angle)))
                self.previous_direction_stamp=scan.stamp
                self.previous_direction_stage=self.stage
        else:
            self.previous_world_direction=None
            self.previous_direction_stamp=None
            self.previous_direction_stage=None
        return result
