"""V3: minimize inverse LiDAR range plus goal-heading error over V2 safe gaps."""
import math
import os
import time

import numpy as np

from kaboat_gps_hopping.gps_hopping import GPSHoppingConfig, GPSHoppingMission
from isv_shape_navigation.core import wrap
from isv_shape_navigation.supplied_pwm import heading_pwm
from .avoidance_v2 import RouteMissionV2, choose_gap


# Edit these values to tune V3. Range is metres; heading error is degrees.
DISTANCE_WEIGHT = 1.0
ANGLE_WEIGHT = 50.0
DISTANCE_EPSILON = 1e-3


def lidar_range_at_angle(scan, angle):
    """Return the measured range at a candidate bearing, or the scan limit."""
    ranges = np.asarray(scan.ranges, dtype=float)
    if not len(ranges):
        raise ValueError("Empty LiDAR scan")
    bearings = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment + scan.yaw
    errors = np.abs(np.arctan2(np.sin(bearings-angle), np.cos(bearings-angle)))
    index = int(np.argmin(errors))
    value = float(ranges[index])
    if math.isinf(value) and value > 0:
        return float(scan.range_max)
    if not math.isfinite(value) or not scan.range_min <= value <= scan.range_max:
        raise ValueError("Invalid LiDAR range at candidate bearing")
    return value


def score_candidate(angle, desired, distance):
    """J = distance_weight/(LiDAR distance + 1e-3) + angle_weight*error_deg."""
    if not math.isfinite(distance) or distance < 0:
        raise ValueError("Invalid LiDAR distance")
    error_deg = math.degrees(abs(wrap(angle-desired)))
    distance_cost = DISTANCE_WEIGHT / (distance + DISTANCE_EPSILON)
    angle_cost = ANGLE_WEIGHT * error_deg
    return dict(lidar_distance_m=distance, goal_angle_error_deg=error_deg,
                weighted_distance=distance_cost, weighted_angle=angle_cost,
                total_score=distance_cost+angle_cost)


def choose_gap_v3(scan, desired, config, radius, raw_scan=None):
    """Keep V2 collision/coverage filtering; replace only its ranking function."""
    _, report = choose_gap(scan, desired, config, radius, raw_scan=raw_scan)
    report.update(algorithm="V3_INVERSE_RANGE_GOAL_ANGLE",
                  selection_policy="inverse_lidar_range_plus_goal_angle",
                  score_weights=dict(distance=DISTANCE_WEIGHT, angle=ANGLE_WEIGHT),
                  distance_epsilon=DISTANCE_EPSILON)
    candidates = report['candidate_scores']
    if not candidates:
        return None, report
    for row in candidates:
        angle = math.radians(row['angle_deg'])
        row.update(score_candidate(angle, desired, lidar_range_at_angle(scan, angle)))
        row['selected'] = False
    winner = min(candidates, key=lambda row: (row['total_score'],
                  row['goal_angle_error_deg'], -row['lidar_distance_m'], row['candidate_id']))
    winner['selected'] = True
    index = winner['gap_index']
    report.update(selected_angle_deg=winner['angle_deg'],
                  selected_distance=winner['distance_m'],
                  selected_gap_index=index,
                  selected_direct=winner['candidate_id'] == 'WP_DIRECT',
                  selected_score=winner['total_score'])
    for gap_index, gap in enumerate(report['gaps'], 1):
        gap['selected'] = gap_index == index
        gap['score'] = next((row for row in candidates if row['gap_index'] == gap_index), None)
    return math.radians(winner['angle_deg']), report


def best_effort_heading(scan, desired, fov_deg):
    """Keep moving toward the clearest observed bearing if no corridor passes."""
    ranges = np.asarray(scan.ranges, dtype=float)
    bearings = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment + scan.yaw
    bearings = np.arctan2(np.sin(bearings), np.cos(bearings))
    valid = ((np.isfinite(ranges) & (ranges >= scan.range_min) &
              (ranges <= scan.range_max)) | np.isposinf(ranges))
    valid &= np.abs(bearings) <= math.radians(fov_deg)
    if not np.any(valid):
        return None
    distances = np.where(np.isposinf(ranges[valid]), scan.range_max, ranges[valid])
    headings = bearings[valid]
    errors = np.abs(np.arctan2(np.sin(headings-desired), np.cos(headings-desired)))
    index = int(np.lexsort((errors, -distances))[0])
    return float(headings[index]), float(distances[index])


class RouteMissionV3(RouteMissionV2):
    output_turn_gain = 1.5
    waypoint_reverse_seconds = 1.0
    waypoint_reverse_pwm = (1300, 1300)

    def reset(self):
        super().reset()
        self.reverse_started = None
        self.wp2_reverse_done = False
        gps = self.gps_config
        self.hopper = GPSHoppingMission(self.config, self.guidance, GPSHoppingConfig(
            goal_configured=True,
            goal_latitude=self.route_config.wp_latitudes[1],
            goal_longitude=self.route_config.wp_longitudes[1],
            gps_arrival_radius=self.route_config.wp_radius,
            gps_arrival_hold=gps.gps_arrival_hold,
            gps_timeout=gps.gps_timeout,
            max_gps_sigma=gps.max_gps_sigma,
            max_gps_leg=gps.max_gps_leg,
            heading_timeout=gps.heading_timeout,
            gps_percent=gps.gps_percent,
            align_tolerance_deg=gps.align_tolerance_deg))
        if os.environ.get('KABOAT_V3_RESUME_WP2') == '1':
            self.stage = 'GPS_HOP_WP2'
            self.phase = 'GPS_TRANSIT'
            self.wp_index = 1

    @property
    def motion_states(self):
        # The ROS adapter snapshots this tuple at startup. Keep short sensor
        # gaps armed with neutral PWM so a recovered scan resumes immediately.
        return (*super().motion_states, 'GPS_TRANSIT', 'ALIGN_GPS',
                'GPS_ARRIVAL_HOLD', 'GPS_HOLD', 'WAIT_GPS', 'GPS_UNCERTAIN',
                'WAIT_HEADING', 'WAIT_SCAN', 'WAIT_SYNC',
                'WP1_REVERSE', 'WP2_REVERSE')

    def reverse_at_waypoint(self, now, scan):
        if not self.executing:
            self.reverse_started = None
            self.intended_pwm = (1500, 1500)
            self.command_kind = 'NEUTRAL'
            return self.stage, self.percentages()
        if self.reverse_started is None:
            self.reverse_started = now
        if now - self.reverse_started < self.waypoint_reverse_seconds:
            self.intended_pwm = self.waypoint_reverse_pwm
            self.command_kind = 'WAYPOINT_REVERSE'
            return self.stage, self.percentages()
        if self.stage == 'WP1_REVERSE':
            self.stage = 'GPS_HOP_WP2'
        else:
            self.wp2_reverse_done = True
            self.stage = 'GPS_HOP_WP2'
        self.reverse_started = None
        return self.step(now, scan)

    def step(self, now, scan):
        if self.stage in ('WP1_REVERSE', 'WP2_REVERSE'):
            return self.reverse_at_waypoint(now, scan)
        if self.stage == 'TO_WP1':
            if (self.executing and self.fix is not None and self.geo_state(now) is None
                    and self.distance_to_wp <= self.route_config.wp_radius):
                # WP1 is a transfer point, so hand off to WP2 without a dwell.
                self.arrival_since = self.fix.stamp - self.route_config.arrival_hold
            result = super().step(now, scan)
            if self.stage == 'WP1_TO_WP2':
                self.stage = 'WP1_REVERSE'
                return self.reverse_at_waypoint(now, scan)
            return result

        self.hopper.fix = self.fix
        self.hopper.heading = self.heading
        self.hopper.heading_stamp = self.heading_stamp
        self.wp_index = 1
        self.geo_state(now)
        state, _ = self.hopper.step(now)
        if (self.executing and not self.wp2_reverse_done
                and state in ('GPS_ARRIVAL_HOLD', 'GPS_HOLD')):
            self.stage = 'WP2_REVERSE'
            return self.reverse_at_waypoint(now, scan)
        self.phase = state
        self.distance_to_wp = self.hopper.gps_distance
        self.avoidance = {}
        self.hold_remaining = (max(0., self.gps_config.gps_arrival_hold -
                               (self.fix.stamp - self.hopper.gps_inside_since))
                               if self.fix is not None and self.hopper.gps_inside_since is not None else 0.)
        if state == 'GPS_HOLD':
            self.announce_waypoint(2)
        self.intended_pwm = (heading_pwm(self.distance_to_wp, -self.heading_error_deg,
                             forward_boost=self.route_config.forward_pwm_boost)
                             if state in ('GPS_TRANSIT', 'ALIGN_GPS') else (1500, 1500))
        self.command_kind = 'GPS_HOPPING' if self.intended_pwm != (1500, 1500) else 'NEUTRAL'
        return state, self.percentages()

    def select_corridor(self, scan, desired, radius):
        started=time.perf_counter()
        raw = self.raw_scan
        if raw is not None and abs(scan.stamp-raw.stamp) > self.config.sync_tolerance:
            raw = None
        angle, report = choose_gap_v3(scan, desired, self.route_config, radius, raw_scan=raw)
        report['planning_ms']=(time.perf_counter()-started)*1000
        if angle is not None:
            return angle, report
        fallback = best_effort_heading(scan, desired, self.route_config.avoidance_fov_deg)
        if fallback is None:
            return angle, report
        angle, distance = fallback
        report.update(reason='BEST_EFFORT', fallback_reason=report['reason'],
                      selected_angle_deg=math.degrees(angle), selected_distance=distance,
                      selected_direct=False, direct_clear=False)
        return angle, report
