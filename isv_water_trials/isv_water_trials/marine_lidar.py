"""All-layer buoy evidence in GPS/IMU coordinates, independent of SLAM.

Input XYZ is the self-masked LiDAR cloud. VectorNav angles are NED/FRD,
LiDAR and vessel axes are forward/left/up. No return is not free-space proof.
"""
from collections import deque
from dataclasses import dataclass, field
import math
import time

import numpy as np
from scipy.spatial import cKDTree


@dataclass(frozen=True)
class MarineConfig:
    enabled: bool = True
    sensor_height_m: float = .30
    height_uncertainty_m: float = .10
    auto_waterline: bool = True
    min_range_m: float = .40
    max_range_m: float = 25.
    min_height_m: float = .035
    small_height_m: float = .01
    small_confirmation_scans: int = 5
    small_confirmation_span_s: float = .35
    small_max_range_m: float = 8.
    small_water_uncertainty_m: float = .025
    tall_height_m: float = .25
    max_height_m: float = 2.5
    voxel_m: float = .20
    confirmation_scans: int = 3
    confirmation_span_s: float = .18
    observation_window_s: float = .9
    association_m: float = .45
    memory_s: float = 6.
    far_memory_s: float = 1.2
    near_memory_range_m: float = 4.
    max_cells: int = 800
    max_input_points: int = 200000
    cloud_max_age_s: float = .5
    imu_max_age_s: float = .15
    max_hz: float = 10.
    mount_roll_deg: float = 0.
    mount_pitch_deg: float = 0.
    imu_mount_yaw_deg: float = 0.
    # Optional measured bow extent, from vessel centre. Zero uses hull length/2.
    bow_extent_m: float = 0.
    lowest_beam_deg: float = -15.
    connector_gap_m: float = .8
    connector_alignment_m: float = .12
    connector_max_segments: int = 32

    def __post_init__(self):
        for name, value in vars(self).items():
            if not isinstance(value, (bool, int, float)) or not math.isfinite(value):
                raise ValueError('Invalid marine LiDAR setting: '+name)
            if name in ('enabled', 'auto_waterline'):
                if not isinstance(value, bool):
                    raise ValueError('Expected boolean: '+name)
            elif isinstance(value, bool):
                raise ValueError('Expected number: '+name)
        if not .05 <= self.sensor_height_m <= 3 or not 0 <= self.height_uncertainty_m < self.sensor_height_m:
            raise ValueError('Invalid sensor height/uncertainty')
        if not 0 < self.min_range_m < self.max_range_m <= 100:
            raise ValueError('Invalid marine LiDAR range')
        if not 0 < self.min_height_m < self.tall_height_m < self.max_height_m:
            raise ValueError('Invalid marine LiDAR height thresholds')
        if not .05 <= self.voxel_m <= .5 or not .05 <= self.association_m <= 1:
            raise ValueError('Invalid voxel/association size')
        for name in ('confirmation_scans', 'small_confirmation_scans', 'max_cells', 'max_input_points', 'connector_max_segments'):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                raise ValueError('Expected positive integer: '+name)
        if not 2 <= self.confirmation_scans <= 8 or not 1 <= self.max_cells <= 4000:
            raise ValueError('Invalid evidence limits')
        if not 0 < self.confirmation_span_s < self.observation_window_s <= 3:
            raise ValueError('Invalid confirmation window')
        if not 0 < self.small_height_m < self.min_height_m:
            raise ValueError('Invalid small-obstacle height')
        if not self.confirmation_scans <= self.small_confirmation_scans <= 7:
            raise ValueError('Invalid small-obstacle observation count')
        if not self.confirmation_span_s <= self.small_confirmation_span_s < self.observation_window_s:
            raise ValueError('Invalid small-obstacle confirmation window')
        if not self.min_range_m < self.small_max_range_m <= 100:
            raise ValueError('Invalid small-obstacle range')
        if not 0 < self.small_water_uncertainty_m <= .05:
            raise ValueError('Invalid small-obstacle water uncertainty')
        if not 0 < self.far_memory_s <= self.memory_s <= 15 or not 1 <= self.max_hz <= 20:
            raise ValueError('Invalid memory/rate limit')
        if min(self.near_memory_range_m, self.cloud_max_age_s, self.imu_max_age_s) <= 0:
            raise ValueError('Invalid freshness/range setting')
        if not -45 <= self.lowest_beam_deg < 0 or self.bow_extent_m < 0:
            raise ValueError('Invalid beam/bow geometry')
        if not .2 <= self.connector_gap_m <= 1. or not .03 <= self.connector_alignment_m <= .2 or self.connector_max_segments > 64:
            raise ValueError('Invalid low-connector interpolation limits')


def rotation(roll, pitch, yaw=0.):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


def lidar_origin(pose, boat):
    yaw = pose['yaw']
    arm = np.asarray(boat['lidar_lever_arm_m'], dtype=float)
    return np.asarray(pose['position']) + rotation(0., 0., yaw)[:2, :2] @ arm


class AttitudeHistory:
    def __init__(self):
        self.samples = deque(maxlen=256)

    def add(self, stamp, roll_deg, pitch_deg):
        if not all(math.isfinite(v) for v in (stamp, roll_deg, pitch_deg)):
            return
        if self.samples and stamp < self.samples[-1][0]:
            self.samples.clear()
        if self.samples and stamp == self.samples[-1][0]:
            return
        self.samples.append((stamp, math.radians(roll_deg), math.radians(pitch_deg)))

    def at(self, stamp, max_age):
        if not self.samples:
            return None
        samples = np.asarray(self.samples)
        if stamp < samples[0, 0]-max_age or stamp > samples[-1, 0]+max_age:
            return None
        index = min(len(samples)-1, int(np.searchsorted(samples[:, 0], stamp)))
        before = max(0, index-1)
        if min(abs(samples[index, 0]-stamp), abs(samples[before, 0]-stamp)) > max_age:
            return None
        angles = np.unwrap(samples[:, 1:], axis=0)
        result = [float(np.interp(stamp, samples[:, 0], angles[:, i])) for i in range(2)]
        dt = samples[index, 0]-samples[before, 0]
        rate = float(np.max(np.abs(angles[index]-angles[before]))/dt) if dt > 0 else 0.
        return (*result, rate)


@dataclass
class Evidence:
    point: np.ndarray
    radius: float
    last: float
    observations: deque = field(default_factory=lambda: deque(maxlen=8))
    confirmed: bool = False
    height_m: float = 0.


class MarineObstacles:
    def __init__(self, settings, boat):
        self.cfg = settings
        self.boat = boat
        self.tracks = []
        self.stamp = None
        self.pose = None
        self.water_z = -settings.sensor_height_m
        self.water_uncertainty = settings.height_uncertainty_m
        self.water_stamp = None
        self.water_proposals = deque(maxlen=3)
        self.status = {}

    def reset(self):
        self.tracks.clear()
        self.stamp = None
        self.pose = None
        self.water_z = -self.cfg.sensor_height_m
        self.water_uncertainty = self.cfg.height_uncertainty_m
        self.water_stamp = None
        self.water_proposals.clear()

    def waterline(self, points, stamp):
        """Fit only a distributed horizontal surface near the mounting prior.

        A compact buoy cannot establish the waterline. Three independent clouds
        are required before replacing the uncertain installation height.
        """
        cfg = self.cfg
        if not cfg.auto_waterline or len(points) < 80:
            return
        prior = -cfg.sensor_height_m
        band = max(.08, cfg.height_uncertainty_m)+.08
        # Only the water-height estimate is sampled. Obstacle detection uses
        # every return, including one-beam low buoys.
        sampled = points[::max(1, math.ceil(len(points)/6000))]
        subset = sampled[(np.abs(sampled[:, 2]-prior) <= band)]
        if len(subset) < 80:
            return
        hist, edges = np.histogram(subset[:, 2], bins=np.arange(prior-band, prior+band+.03, .03))
        peak = int(np.argmax(hist))
        z = (edges[peak]+edges[peak+1])/2
        surface = subset[np.abs(subset[:, 2]-z) <= .045]
        cell_keys = np.floor(surface[:, :2]/.3).astype(np.int64)
        cells = np.unique(cell_keys[:, 0]*100001+cell_keys[:, 1])
        sectors = np.unique(np.floor((np.arctan2(surface[:, 1], surface[:, 0])+math.pi)/(math.pi/4)))
        radial = np.linalg.norm(surface[:, :2], axis=1)
        if len(surface) < 60 or len(cells) < 30 or len(sectors) < 4 or np.ptp(radial) < .6:
            return
        level = float(np.median(surface[:, 2]))
        roughness = float(np.median(np.abs(surface[:, 2]-level)))
        if self.water_proposals and stamp-self.water_proposals[-1][0] > .5:
            self.water_proposals.clear()
        self.water_proposals.append((stamp, level, roughness))
        if len(self.water_proposals) < 3 or np.ptp([p[1] for p in self.water_proposals]) > .06:
            return
        target = float(np.median([p[1] for p in self.water_proposals]))
        step = .03 if self.water_stamp is not None else band
        self.water_z += float(np.clip(target-self.water_z, -step, step))
        self.water_uncertainty = max(.015, min(.045, 2.5*roughness))
        self.water_stamp = stamp

    def update(self, xyz, stamp, pose, attitude):
        started = time.perf_counter()
        cfg = self.cfg
        if self.stamp is not None and stamp == self.stamp:
            return self.snapshot(stamp, pose)
        if self.stamp is not None and stamp < self.stamp:
            self.reset()
        if self.pose is not None:
            jump = np.linalg.norm(np.asarray(pose['position'])-self.pose['position'])
            if jump > 1.5+5.*max(0., stamp-self.stamp):
                self.tracks.clear()
        self.stamp, self.pose = stamp, dict(pose)
        self.status = dict(input_points=len(xyz), source='all_layers', reason='ok')
        points = np.asarray(xyz, dtype=float).reshape(-1, 3)
        if len(points) > cfg.max_input_points:
            self.status['reason'] = 'cloud_point_budget_exceeded'
            return self.snapshot(stamp, pose)
        good = np.isfinite(points).all(axis=1)
        radial = np.linalg.norm(points[:, :2], axis=1)
        points = points[good & (radial >= cfg.min_range_m) & (radial <= cfg.max_range_m)]
        mount = rotation(math.radians(cfg.mount_roll_deg), math.radians(cfg.mount_pitch_deg),
                         math.radians(self.boat['lidar_yaw_deg']))
        body = points @ mount.T
        hull = body[:, :2]+np.asarray(self.boat['lidar_lever_arm_m'])
        bow = cfg.bow_extent_m or self.boat['length_m']/2
        inside = ((hull[:, 0] >= -self.boat['length_m']/2) & (hull[:, 0] <= bow) &
                  (np.abs(hull[:, 1]) <= self.boat['width_m']/2))
        body = body[~inside]
        roll, pitch, tilt_rate = attitude
        imu_mount = rotation(0., 0., math.radians(cfg.imu_mount_yaw_deg))
        # NED/FRD -> FLU preserves roll sign and reverses pitch sign.
        leveling = imu_mount @ rotation(roll, -pitch) @ imu_mount.T
        level = body @ leveling.T
        self.waterline(level, stamp)
        uncertainty = self.water_uncertainty
        if self.water_stamp is not None:
            uncertainty = min(max(cfg.height_uncertainty_m, uncertainty),
                              uncertainty+.005*max(0., stamp-self.water_stamp-2.))
        height = level[:, 2]-self.water_z
        radial = np.linalg.norm(level[:, :2], axis=1)
        # Header-time leveling cannot fully correct roll/pitch during one spin.
        error = uncertainty+.01+radial*np.sin(min(.2, math.radians(.15)+tilt_rate*.05))
        strong = height >= cfg.min_height_m+error
        calibrated = (cfg.height_uncertainty_m <= cfg.small_water_uncertainty_m or
                      (self.water_stamp is not None and stamp-self.water_stamp <= 2.))
        small = (calibrated & (uncertainty <= cfg.small_water_uncertainty_m) &
                 (radial <= cfg.small_max_range_m) & (height >= cfg.small_height_m+error))
        keep = (strong | small) & (height <= cfg.max_height_m)
        candidates, heights = level[keep], height[keep]
        self.status.update(water_z_m=float(self.water_z), water_uncertainty_m=float(uncertainty),
                           waterline_source='observed' if self.water_stamp is not None else 'installation_prior',
                           height_candidate_points=len(candidates), hull_points_removed=int(inside.sum()),
                           small_candidate_points=int((keep & ~strong).sum()))
        self.status['height_histogram'] = np.histogram(height, [-10., 0., .05, .10, .20, .60, 10.])[0].tolist()
        self.status['low_height_threshold_3m'] = float(cfg.min_height_m+uncertainty+.01+
            3.*np.sin(min(.2, math.radians(.15)+tilt_rate*.05)))
        detections = self.detections(candidates, heights, pose, ~strong[keep])
        self.track(detections, stamp, pose)
        result = self.snapshot(stamp, pose)
        self.status['processing_ms'] = round((time.perf_counter()-started)*1000, 3)
        result['status']['processing_ms'] = self.status['processing_ms']
        return result

    def detections(self, points, heights, pose, small):
        if not len(points):
            return []
        cfg = self.cfg
        raw_keys = np.floor(points[:, :2]/cfg.voxel_m).astype(np.int64)
        _, indices, inverse = np.unique(raw_keys[:, 0]*100001+raw_keys[:, 1], return_index=True, return_inverse=True)
        keys = raw_keys[indices]
        count = np.bincount(inverse)
        strong_count = np.bincount(inverse, weights=~small, minlength=len(keys))
        centers = np.column_stack([np.bincount(inverse, weights=points[:, i])/count for i in range(2)])
        high = np.full(len(keys), -np.inf)
        low = np.full(len(keys), np.inf)
        np.maximum.at(high, inverse, heights)
        np.minimum.at(low, inverse, heights)
        radius = np.zeros(len(keys))
        np.maximum.at(radius, inverse, np.linalg.norm(points[:, :2]-centers[inverse], axis=1))
        # Count distinct horizontal beams. Two vertical layers at one azimuth
        # must not masquerade as the width evidence of a small buoy.
        beam = np.floor((np.arctan2(points[:, 1], points[:, 0])+math.pi)/math.radians(.1)).astype(int)
        distinct = np.unique(inverse*3601+beam)
        beams = np.bincount(distinct//3601, minlength=len(keys))
        flat_keys = {tuple(keys[i]) for i in range(len(keys))
                     if high[i]-low[i] < .06 and high[i] < cfg.tall_height_m+.15}
        offsets = [(dx, dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1) if dx or dy]
        # Reject an entire connected flat sheet, including its thin edge.
        # Otherwise a wave sheet's boundary can look like a persistent rope.
        sheet_keys = {key for key in flat_keys
                      if sum((key[0]+dx, key[1]+dy) in flat_keys for dx, dy in offsets) >= 7}
        pending = list(sheet_keys)
        while pending:
            x, y = pending.pop()
            for dx, dy in offsets:
                key = (x+dx, y+dy)
                if key in flat_keys and key not in sheet_keys:
                    sheet_keys.add(key)
                    pending.append(key)
        yaw = rotation(0., 0., pose['yaw'])[:2, :2]
        world = centers @ yaw.T+lidar_origin(pose, self.boat)
        result = []
        for i in np.argsort(np.linalg.norm(centers, axis=1)):
            if tuple(keys[i]) in sheet_keys:
                continue
            result.append((world[i], float(radius[i])+.04, int(beams[i]),
                           float(high[i]), float(high[i]-low[i]), strong_count[i] == 0))
            if len(result) >= cfg.max_cells:
                break
        self.status['candidate_cells'] = len(result)
        self.status['water_sheet_cells'] = len(sheet_keys)
        return result

    def track(self, detections, stamp, pose):
        cfg = self.cfg
        origin = lidar_origin(pose, self.boat)
        self.tracks = [t for t in self.tracks if 0 <= stamp-t.last <= self.ttl(t, origin)]
        tree = cKDTree([t.point for t in self.tracks]) if self.tracks else None
        matches = tree.query_ball_point([d[0] for d in detections], cfg.association_m) if tree is not None and detections else [[] for _ in detections]
        used = set()
        for (point, radius, beams, height, vertical_span, small), candidates in zip(detections, matches):
            candidates = [i for i in candidates if i not in used]
            index = min(candidates, key=lambda i: math.hypot(self.tracks[i].point[0]-point[0], self.tracks[i].point[1]-point[1])) if candidates else None
            if index is None:
                evidence = Evidence(point.copy(), radius, stamp)
                self.tracks.append(evidence)
            else:
                used.add(index)
                evidence = self.tracks[index]
            history = deque((o for o in evidence.observations if stamp-o[0] <= cfg.observation_window_s), maxlen=8)
            history.append((stamp, point.copy(), beams, small))
            evidence.observations = history
            positions = np.asarray([o[1] for o in history])
            center = positions.sum(axis=0)/len(positions)
            scatter = float(np.max(np.linalg.norm(positions-center, axis=1)))
            strong_history = [o for o in history if not o[3]]
            wide = sum(o[2] >= 2 for o in strong_history) >= cfg.confirmation_scans
            required = cfg.confirmation_scans if wide else max(4, cfg.confirmation_scans)
            persistent = (len(strong_history) >= required and
                          stamp-strong_history[0][0] >= cfg.confirmation_span_s and scatter <= .30)
            small_wide = sum(o[2] >= 2 for o in history) >= cfg.small_confirmation_scans
            small_required = cfg.small_confirmation_scans if small_wide else cfg.small_confirmation_scans+1
            small_persistent = (len(history) >= small_required and
                                stamp-history[0][0] >= cfg.small_confirmation_span_s and scatter <= .30)
            tall = height >= cfg.tall_height_m and beams >= 2 and vertical_span >= .08
            evidence.confirmed = (evidence.confirmed and scatter <= .30) or tall or persistent or small_persistent
            evidence.point = point.copy()
            evidence.radius = radius
            evidence.last = stamp
            evidence.height_m = height
        # Prefer confirmed/nearby objects when the sea produces excess clutter.
        self.tracks.sort(key=lambda t: (not t.confirmed, math.hypot(t.point[0]-origin[0], t.point[1]-origin[1]), -t.last))
        self.tracks = self.tracks[:cfg.max_cells]

    def ttl(self, evidence, origin):
        distance = math.hypot(evidence.point[0]-origin[0], evidence.point[1]-origin[1])
        near = max(self.cfg.near_memory_range_m,
                   self.cfg.sensor_height_m/math.tan(math.radians(abs(self.cfg.lowest_beam_deg)))+1.)
        if evidence.height_m < self.cfg.tall_height_m:
            near = max(near, min(8., self.cfg.max_range_m))
        return self.cfg.memory_s if evidence.confirmed and distance <= near else self.cfg.far_memory_s

    def connector_points(self, evidence, stamp):
        """Bridge short collinear gaps supported by at least three low tracks."""
        low = [t for t in evidence if 0 < t.height_m < self.cfg.tall_height_m]
        if len(low) < 3:
            return [], 0
        centers = np.asarray([t.point for t in low])
        tree = cKDTree(centers)
        neighborhoods = [set(indices) for indices in tree.query_ball_point(centers, self.cfg.connector_gap_m)]
        pairs = sorted(tree.query_pairs(self.cfg.connector_gap_m),
                       key=lambda pair: np.linalg.norm(centers[pair[0]]-centers[pair[1]]))
        interpolated = []
        segments = 0
        for i, j in pairs:
            delta = centers[j]-centers[i]
            length = float(np.linalg.norm(delta))
            if length <= low[i].radius+low[j].radius+.15:
                continue
            direction = delta/length
            neighbors = neighborhoods[i] | neighborhoods[j]
            neighbors.difference_update((i, j))
            if not neighbors:
                continue
            offsets = centers[list(neighbors)]-centers[i]
            along = offsets @ direction
            aligned = abs(offsets[:, 0]*direction[1]-offsets[:, 1]*direction[0]) <= self.cfg.connector_alignment_m
            # Consecutive support points only. An already observed interval
            # needs no interpolation and must not create overlapping segments.
            if np.any(aligned & (along > .1) & (along < length-.1)):
                continue
            if not np.any(aligned & ((along < -.1) | (along > length+.1))):
                continue
            # Synthetic gap samples never become observations or refresh memory.
            last = min(low[i].last, low[j].last)
            ttl = min(self.ttl(low[i], lidar_origin(self.pose, self.boat)),
                      self.ttl(low[j], lidar_origin(self.pose, self.boat)))
            if stamp-last > ttl:
                continue
            radius = max(.05, min(.12, low[i].radius, low[j].radius))
            count = max(2, math.ceil(length/.12))
            for fraction in np.arange(1, count)/count:
                point = centers[i]+fraction*delta
                interpolated.append([*point, radius, last, ttl])
            segments += 1
            if segments >= self.cfg.connector_max_segments:
                break
        return interpolated, segments

    def snapshot(self, stamp, pose):
        origin = lidar_origin(pose, self.boat)
        points = []
        confirmed = []
        for evidence in self.tracks:
            age = stamp-evidence.last
            ttl = self.ttl(evidence, origin)
            if evidence.confirmed and 0 <= age <= ttl:
                points.append([*evidence.point, evidence.radius, evidence.last, ttl])
                confirmed.append(evidence)
        bridges, segments = self.connector_points(confirmed, stamp)
        status = dict(self.status, confirmed_cells=len(points), tracked_cells=len(self.tracks),
                      confirmed_low_cells=sum(0 < t.height_m < self.cfg.tall_height_m for t in confirmed),
                      connector_segments=segments, connector_fill_cells=len(bridges),
                      blind_water_range_m=round(self.cfg.sensor_height_m/math.tan(math.radians(abs(self.cfg.lowest_beam_deg))), 2))
        return dict(stamp=stamp, points=points, connector_points=bridges, status=status)


def merge_obstacles(scan, pose, result, boat, settings):
    """Project confirmed world discs into the existing 720-bin planning scan."""
    if not settings.enabled or pose is None or not result:
        return scan
    age = scan['stamp']-result['stamp']
    # Individual last-observed times bound memory even across skipped clouds.
    if not -.05 <= age <= settings.memory_s:
        return scan
    discs = np.asarray(result['points']+result.get('connector_points', []), dtype=float).reshape(-1, 5)
    if not len(discs):
        return scan
    measured_age = scan['stamp']-discs[:, 3]
    discs = discs[(measured_age >= -.05) & (measured_age <= discs[:, 4])]
    ranges = np.asarray(scan['ranges']).copy()
    angles = np.asarray(scan['angles'])
    if len(angles) < 2:
        return scan
    delta = discs[:, :2]-lidar_origin(pose, boat)
    distances = np.linalg.norm(delta, axis=1)
    bearings = np.arctan2(delta[:, 1], delta[:, 0])-pose['yaw']-math.radians(boat['lidar_yaw_deg'])
    fused = 0
    increment = abs(angles[1]-angles[0])
    for disc, distance, bearing in zip(discs, distances, bearings):
        radius = disc[2]+min(.25, .04*max(0., scan['stamp']-disc[3]))
        if distance-radius >= scan['range_max']:
            continue
        center = int(round((bearing-angles[0])/increment)) % len(angles)
        if distance <= radius:
            slots = np.arange(len(angles))
        else:
            span = math.asin(radius/distance)+increment/2
            count = int(math.ceil(span/increment))
            slots = (center+np.arange(-count, count+1)) % len(angles)
            difference = np.arctan2(np.sin(angles[slots]-bearing), np.cos(angles[slots]-bearing))
            slots = slots[(np.abs(difference) <= span) | (slots == center)]
        value = max(scan['range_min']+1e-4, distance-radius)
        ranges[slots] = np.fmin(ranges[slots], value)
        fused += 1
    return dict(scan, ranges=ranges, marine_cells=fused)
