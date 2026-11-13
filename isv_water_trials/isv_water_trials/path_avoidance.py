"""Persistent avoidance paths and measured reverse/pivot manoeuvres."""
import math
import numpy as np
from scipy.spatial import cKDTree
from .outdoor_follow import maneuver_space, point_follow_command


def path_length(path):
    return float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())


class StableAvoidancePath:
    def __init__(self):
        self.path = None
        self.goal = None
        self.changed_at = 0.

    def select(self, result, position, goal, hits, config, yaw, now):
        position = np.asarray(position, dtype=float)
        goal = np.asarray(goal, dtype=float)
        candidate = result.get('smooth') or []
        previous = None
        if self.path is not None and np.linalg.norm(goal-self.goal) < .05 and now >= self.changed_at:
            segments = np.diff(self.path, axis=0)
            fractions = np.clip(np.sum((position-self.path[:-1])*segments, axis=1) /
                                np.maximum(np.sum(segments*segments, axis=1), 1e-12), 0., 1.)
            projected = self.path[:-1]+fractions[:, None]*segments
            index = int(np.argmin(np.linalg.norm(projected-position, axis=1)))
            previous = np.vstack([position, projected[index], self.path[index+1:]])
            points = np.asarray(hits, dtype=float).reshape(-1, 2)
            points = points[np.isfinite(points).all(axis=1)]
            delta = points-position
            c, s = math.cos(yaw), math.sin(yaw)
            own = ((abs(delta[:, 0]*c+delta[:, 1]*s) < config['length_m']/2) &
                   (abs(-delta[:, 0]*s+delta[:, 1]*c) < config['width_m']/2))
            points = points[~own]
            if len(points):
                tree = cKDTree(points)
                radius = config['width_m']/2+float(config.get('obstacle_margin_m', 0.))
                arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(previous, axis=0), axis=1))]
                sample_arc = np.arange(0., arc[-1]+.08, .08)
                sampled = np.c_[np.interp(sample_arc, arc, previous[:, 0]),
                                np.interp(sample_arc, arc, previous[:, 1])]
                if np.any(tree.query(np.vstack([previous, sampled]))[0] < radius-1e-9):
                    previous = None
        retain = previous is not None and len(previous) >= 2
        if retain and candidate:
            old_length = path_length(previous)
            saving = old_length-path_length(np.asarray(candidate))
            retain = now-self.changed_at < 2. or saving < max(1., old_length*.2)
        if retain:
            previous = previous[np.r_[True, np.linalg.norm(np.diff(previous, axis=0), axis=1) > 1e-6]]
            result = dict(result, raw=previous.tolist(), smooth=previous.tolist(),
                          path_retained=True, status='기존 회피 방향 유지')
        elif candidate:
            self.changed_at = now
        self.path = np.asarray(result['smooth']) if len(result.get('smooth', [])) >= 2 else None
        self.goal = goal.copy()
        return result


def retreat_space(scan, config):
    """Check a straight 0.8 m reverse sweep and measured forward clearance."""
    result = dict(retreat_clear=False, front_clearance_m=None)
    if scan is None:
        return result
    ranges = np.asarray(scan.get('ranges', []), dtype=float)
    angles = np.asarray(scan.get('angles', []), dtype=float)
    if len(ranges) < 180 or len(angles) != len(ranges):
        return result
    valid = (ranges > scan.get('range_min', .1)) & (np.isfinite(ranges) | np.isposinf(ranges))
    angles = angles+math.radians(config.get('lidar_yaw_deg', 0.))
    hit = valid & np.isfinite(ranges)
    points = ranges[hit, None]*np.c_[np.cos(angles[hit]), np.sin(angles[hit])]
    points += np.asarray(config.get('lidar_lever_arm_m', [0., 0.]))
    half = np.array([config['length_m']/2, config['width_m']/2])
    points = points[~np.all(abs(points) < half, axis=1)]
    ahead = points[(points[:, 0] >= half[0]) & (abs(points[:, 1]) <= half[1]+.15)]
    if len(ahead):
        result['front_clearance_m'] = max(0., float(ahead[:, 0].min()-half[0]))
    rear = abs(np.arctan2(np.sin(angles-math.pi), np.cos(angles-math.pi))) < math.radians(40)
    coverage = bool(np.any(rear) and np.mean(valid[rear]) >= .8)
    margin = float(config.get('obstacle_margin_m', 0.))
    swept = ((points[:, 0] >= -half[0]-.8-margin) & (points[:, 0] <= half[0]+margin) &
             (abs(points[:, 1]) <= half[1]+margin))
    result['retreat_clear'] = coverage and not bool(swept.any())
    return result


class PathAvoidanceController:
    def __init__(self):
        self.key = None
        self.mode = 'track'
        self.previous_error = None
        self.started_at = 0.
        self.start_position = None
        self.start_yaw = 0.
        self.aligned_since = None
        self.recovering = False
        self.search_yaw = None
        self.search_sign = 1.

    def begin_recovery(self, state, error, scan, config, space, now):
        self.recovering = True
        self.mode = 'reverse' if space['retreat_clear'] else 'pivot'
        self.started_at = now
        self.start_position = np.asarray(state['position']).copy()
        self.start_yaw = float(state['yaw'])
        self.aligned_since = None
        if abs(error) > math.radians(10):
            self.search_sign = math.copysign(1., error)
        else:
            # When the goal is straight ahead but blocked, scan the roomier side.
            angles = np.asarray(scan['angles'])+math.radians(config.get('lidar_yaw_deg', 0.))
            angles = np.arctan2(np.sin(angles), np.cos(angles))
            ranges = np.asarray(scan['ranges'])
            valid = (ranges > scan.get('range_min', .1)) & ~np.isnan(ranges)
            left = valid & (angles > .15) & (angles < math.pi/2)
            right = valid & (angles < -.15) & (angles > -math.pi/2)
            if left.any() and right.any():
                difference = np.median(np.minimum(ranges[left], 4.))-np.median(np.minimum(ranges[right], 4.))
                if abs(difference) > .2:
                    self.search_sign = math.copysign(1., difference)
        self.search_yaw = self.start_yaw+self.search_sign*math.radians(35)

    def recover_command(self, state, error, rate, scan, config, space, percent, now, blocked, buoy_contact):
        cap = min(float(percent)/100., .30)
        front = space['front_clearance_m']
        if self.mode == 'reverse':
            heading = np.array([math.cos(self.start_yaw), math.sin(self.start_yaw)])
            travelled = float((self.start_position-np.asarray(state['position'])) @ heading)
            if space['retreat_clear'] and travelled < .6 and now-self.started_at < 2.5:
                return [-cap, -cap], math.degrees(error), dict(maneuver='막힌 경로 후진',
                    maneuver_phase='reverse', retreat_m=travelled, front_clearance_m=front,
                    maneuver_limit_percent=cap*100)
            self.mode = 'pivot'
            self.aligned_since = None
        # Lock a real turn target even if the blocked GPS goal is dead ahead.
        pivot_error = (math.atan2(math.sin(self.search_yaw-state['yaw']),
                                 math.cos(self.search_yaw-state['yaw'])) if blocked else error)
        aligned = abs(pivot_error) <= math.radians(8 if blocked else 18) and abs(rate) <= math.radians(8)
        if aligned:
            if self.aligned_since is None:
                self.aligned_since = now
            if now-self.aligned_since >= .35:
                if blocked:
                    self.begin_recovery(state, error, scan, config, space, now)
                    return self.recover_command(state, error, rate, scan, config, space, percent, now, blocked, buoy_contact)
                self.recovering = False
                self.mode = 'track'
                self.aligned_since = None
                return None
        else:
            self.aligned_since = None
        pivot_blocked = maneuver_space(scan, config, pivot_error)['rotation_blocked']
        turn = cap*float(np.clip((pivot_error-.9*rate)/math.radians(60), -1., 1.))
        command = [0., 0.] if pivot_blocked and not buoy_contact else [-turn, turn]
        return command, math.degrees(pivot_error), dict(maneuver='제자리 회전 · 경로 재탐색',
            maneuver_phase='pivot', blocked_path=blocked, front_clearance_m=front,
            maneuver_limit_percent=cap*100)

    def command(self, state, target, scan, config, percent, now, key, blocked=False, buoy_contact=False):
        if key != self.key:
            self.__init__()
            self.key = key
        nominal, _, error_deg = point_follow_command(dict(state, rotation_blocked=False), target, percent)
        # Contact permission concerns obstacle geometry, not missing sensor data.
        if scan is None:
            self.mode = 'track'
            self.recovering = False
            self.aligned_since = None
            return [0., 0.], error_deg, dict(maneuver='라이다 수신 대기',
                maneuver_phase='waiting', front_clearance_m=None)
        ranges = np.asarray(scan.get('ranges', []), dtype=float)
        angles = np.asarray(scan.get('angles', []), dtype=float)
        valid = (ranges > scan.get('range_min', .1)) & (np.isfinite(ranges) | np.isposinf(ranges))
        if (len(ranges) < 180 or len(ranges) != len(angles) or
                not np.isfinite(angles).all() or np.mean(valid) < .8):
            self.mode = 'track'
            self.recovering = False
            self.aligned_since = None
            return [0., 0.], error_deg, dict(maneuver='라이다 유효 관측 대기',
                maneuver_phase='waiting', front_clearance_m=None)
        error = math.radians(error_deg)
        velocity = state.get('velocity')
        speed = float(velocity[0]) if velocity is not None else 0.
        rate = float(velocity[2]) if velocity is not None else float(state.get('yaw_rate_rad_s', 0.))
        space = maneuver_space(scan, config, error)
        space.update(retreat_space(scan, config))
        front = space['front_clearance_m']
        if blocked or self.recovering or space['rotation_blocked']:
            if not self.recovering:
                self.begin_recovery(state, error, scan, config, space, now)
            recovery = self.recover_command(state, error, rate, scan, config, space, percent, now, blocked, buoy_contact)
            if recovery is not None:
                return recovery
        near = front is not None and front < max(1., max(0., speed)*.8+.4)
        flipped = (self.previous_error is not None and self.previous_error*error < 0 and
                   abs(error-self.previous_error) >= math.radians(50) and abs(error) >= math.radians(25))
        self.previous_error = error
        large_turn = abs(error) >= math.radians(55)
        constrained_turn = space['rotation_blocked'] and abs(error) >= math.radians(15)
        need_space = (near or space['rotation_blocked']) and (large_turn or flipped or constrained_turn or blocked)
        need_space = need_space or (large_turn and speed > .35)
        if self.mode == 'track' and (large_turn or need_space or blocked):
            self.mode = 'reverse' if need_space and space['retreat_clear'] else 'pivot'
            self.started_at = now
            self.start_position = np.asarray(state['position']).copy()
            self.start_yaw = state['yaw']
            self.aligned_since = None
        cap = min(float(percent)/100., .30)
        travelled = 0.
        if self.mode == 'reverse':
            heading = np.array([math.cos(self.start_yaw), math.sin(self.start_yaw)])
            travelled = float((self.start_position-np.asarray(state['position'])) @ heading)
            if not space['retreat_clear'] or travelled >= .6 or now-self.started_at >= 2.5:
                self.mode = 'pivot'
                self.aligned_since = None
            else:
                return [-cap, -cap], error_deg, dict(maneuver='후진 공간 확보', maneuver_phase='reverse',
                    retreat_m=travelled, front_clearance_m=front, maneuver_limit_percent=cap*100)
        if self.mode == 'pivot':
            aligned = abs(error) <= math.radians(18) and abs(rate) <= math.radians(8)
            if aligned and not blocked:
                if self.aligned_since is None:
                    self.aligned_since = now
                if now-self.aligned_since >= .35:
                    self.mode = 'track'
            else:
                self.aligned_since = None
            if self.mode == 'pivot':
                turn = cap*float(np.clip((error-.9*rate)/math.radians(60), -1., 1.))
                command = [0., 0.] if space['rotation_blocked'] or scan is None else [-turn, turn]
                return command, error_deg, dict(maneuver='제자리 방향 정렬', maneuver_phase='pivot',
                    front_clearance_m=front, maneuver_limit_percent=cap*100)
        if blocked:
            return [0., 0.], error_deg, dict(maneuver='회피 경로 갱신', maneuver_phase='waiting', front_clearance_m=front)
        # Reduce surge before entering a sharp turn, keeping differential control.
        surge = sum(nominal)/2
        turn = (nominal[1]-nominal[0])/2
        surge *= float(np.clip((55.-abs(error_deg))/30., 0., 1.))
        command = [surge-turn, surge+turn]
        if space['rotation_blocked'] and abs(turn) > 1e-6:
            command = [0., 0.]
        return command, error_deg, dict(maneuver='회피 경로 추종', maneuver_phase='track', front_clearance_m=front)
