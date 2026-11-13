#!/usr/bin/env python3
"""GPS stages with the supplied paired-thruster controller.
Status 0: approach start. Status 1: V3 avoidance through three GPS goals. Status 2: existing GPS hopping.
Status 3: existing V3 avoidance to the docking start, then two seconds reverse.
Status 4: V3 avoidance toward the camera target, falling back to that shape's GPS.
Status 5: visit each buoy-ring GPS once with V3 avoidance, then enter status 6.
Status 6: approach obstacle-avoidance start GPS, two seconds reverse, five seconds neutral.
Status 7: existing V3 avoidance toward the assigned goal GPS.

Requires the kaboat2026 runtime overlay.
"""

import json
import math
import os
import signal
import sys
import time
from buoy_search_circle import buoy_circle_points


# Variable values live in the sibling configuration file, not in the driving algorithm.
def load_mission_config(path=None):
    from pathlib import Path
    import runpy
    filename = Path(path) if path is not None else Path(__file__).resolve().with_name('mission_config.py')
    values = runpy.run_path(str(filename))
    for name in ('ROS_CONFIG', 'COMMON_CONFIG', 'STATUS_CONFIG'):
        if not isinstance(values.get(name), dict):
            raise ValueError(f'{filename}: {name} must be a dictionary')
    return values


_mission_config = load_mission_config()
ROS_CONFIG = _mission_config['ROS_CONFIG']
COMMON_CONFIG = _mission_config['COMMON_CONFIG']
STATUS_CONFIG = _mission_config['STATUS_CONFIG']

def validate_status_config(config):
    if set(config) != set(range(8)):
        raise ValueError('STATUS_CONFIG must contain statuses 0 through 7')
    for status, values in config.items():
        if status in (1, 2, 3, 4, 5, 7):
            weights = [values['avoidance_distance_weight'], values['avoidance_angle_weight']]
            if (not all(isinstance(v, (int, float)) and not isinstance(v, bool)
                        and math.isfinite(v) and v >= 0 for v in weights)
                    or sum(weights) <= 0):
                raise ValueError(f'Status {status}: avoidance weights must be nonnegative with positive sum')
        thrust = values['thrust_percentage']
        if (isinstance(thrust, bool) or not isinstance(thrust, (int, float))
                or not math.isfinite(thrust) or not 0 <= thrust <= 100):
            raise ValueError(f'Status {status}: thrust_percentage must be 0..100')
        if status == 4:
            arrival = values['arrival_radius_m']
            if (type(arrival) not in (int, float) or not math.isfinite(arrival) or arrival <= 0):
                raise ValueError('Status 4: arrival_radius_m must be positive and finite')
        if status == 5:
            radius = values['orbit_radius_m']
            count = values['orbit_point_count']
            arrival = values['orbit_arrival_radius_m']
            directions = values['orbit_direction_by_buoy']
            if (type(count) is not int or count < 3
                    or not math.isfinite(radius) or radius <= 0
                    or not math.isfinite(arrival) or not 0 < arrival < radius * math.sin(math.pi / count)):
                raise ValueError('Status 5: orbit arrival zones must be positive and must not overlap')
            if (directions.keys() != values['gps_by_buoy'].keys()
                    or any(direction not in ('cw', 'ccw') for direction in directions.values())):
                raise ValueError('Status 5: assign cw or ccw to every configured buoy')
        if status in (2, 3, 6):
            pwm = values['reverse_pwm']
            if (not isinstance(pwm, (tuple, list)) or len(pwm) != 2
                    or not all(type(v) is int and 1100 <= v <= 1500 for v in pwm)):
                raise ValueError(f'Status {status}: reverse_pwm must contain two integers, 1100..1500')
            for key in ('reverse_seconds', 'wait_seconds') if status != 3 else ('reverse_seconds',):
                value = values[key]
                if (isinstance(value, bool) or not isinstance(value, (int, float))
                        or not math.isfinite(value) or value < 0):
                    raise ValueError(f'Status {status}: {key} must be finite and nonnegative')
    return config


def apply_course_avoidance_weights(selected, report, distance_weight, angle_weight):
    """Rerank the existing V3 candidates only; keep its collision filtering/fallback."""
    report['score_weights'] = dict(distance=distance_weight, angle=angle_weight)
    candidates = report.get('candidate_scores', [])
    if not candidates:
        return selected, report
    epsilon = report['distance_epsilon']
    for row in candidates:
        distance_cost = distance_weight / (row['lidar_distance_m'] + epsilon)
        angle_cost = angle_weight * row['goal_angle_error_deg']
        row.update(weighted_distance=distance_cost, weighted_angle=angle_cost,
                   total_score=distance_cost + angle_cost, selected=False)
    winner = min(candidates, key=lambda row: (row['total_score'],
        row['goal_angle_error_deg'], -row['lidar_distance_m'], row['candidate_id']))
    winner['selected'] = True
    index = winner['gap_index']
    report.update(selected_angle_deg=winner['angle_deg'], selected_distance=winner['distance_m'],
                  selected_gap_index=index, selected_direct=winner['candidate_id'] == 'WP_DIRECT',
                  selected_score=winner['total_score'])
    for gap_index, gap in enumerate(report['gaps'], 1):
        gap['selected'] = gap_index == index
        gap['score'] = next((row for row in candidates if row['gap_index'] == gap_index), None)
    return math.radians(winner['angle_deg']), report


def mission_log_line(status, telemetry, telemetry_age, target_v=None):
    """Display command percentages and reported MCU acknowledgments; never control output."""
    missions = {0: '경로 시작점 접근', 1: '경로 추종', 2: '호핑·장애물 회피',
                3: '도킹 시작점·장애물 회피', 4: '도킹', 5: '부표 탐색',
                6: '회피 시작점 접근', 7: '장애물 회피 주행'}
    stage = status['status']
    title = f"[status {stage} | {missions.get(stage, '미정')}]"
    if stage == 1:
        title += f" [경로 {status['route_point']}/3]"
    if stage == 7:
        title += (' [최종 목표]' if status.get('avoidance_final_goal')
                  else f" [중간 지점 {status.get('avoidance_point')}]")
    fmt = lambda value, places=2: f'{value:+.{places}f}' if isinstance(value, (int, float)) and math.isfinite(value) else '대기'
    pwm = status['intended_pwm']
    outputs = f"명령 좌={fmt((pwm[0]-1500)/4, 1)}% 우={fmt((pwm[1]-1500)/4, 1)}%"
    state = status['state']
    fresh = (isinstance(telemetry, dict) and telemetry.get('receiving') is True
             and math.isfinite(telemetry_age) and 0 <= telemetry_age <= .5)
    def acknowledged(expected):
        if not fresh:
            return '미확인(수신 지연)'
        if telemetry.get('left') == expected[0] and telemetry.get('right') == expected[1]:
            return '확인'
        return '대기'
    target = ''
    if stage == 4:
        detected = status.get('docking_mode') == 'CAMERA'
        target = f"대상={status['docking_target']} 검출={'예' if detected else '아니오'} | "
    elif stage == 5:
        target = f"탐색={status['search_target']} 검출=미사용(GPS 전용) | "
    reverse = state.endswith('_REVERSE')
    neutral = tuple(pwm) == (1500, 1500)
    if reverse:
        body = f"후진 명령 | {outputs} | 후진 MCU={acknowledged(pwm)}"
    elif neutral and (status.get('final_target_deg') is None or state.endswith('_WAIT')):
        body = f"중립 명령 | {outputs} | 중립 MCU={acknowledged((1500, 1500))}"
        if state.startswith('WAIT_') or state == 'OUTPUT_STOPPED':
            body += f" | 사유={state}"
    elif stage == 4 and status.get('docking_mode') == 'CAMERA':
        body = (f"카메라 u={fmt(status.get('target_x'), 1)} v={fmt(target_v, 1)} | "
                f"회피 최종각={fmt(status.get('final_target_deg'))}° | {outputs}")
    else:
        label = '회피 최종각' if stage in (1, 2, 3, 4, 5, 7) else 'GPS 목표각'
        body = f"{label}={fmt(status.get('final_target_deg'))}° | {outputs}"
    if not status.get('active') or not status.get('armed'):
        body += ' | 출력=대기'
    if status.get('output_error'):
        body += f" | 오류={status['output_error']}"
    return f'{title} {target}{body}'


def docking_gps_values(raw):
    if not isinstance(raw, dict):
        raise ValueError('Docking GPS must map model class names to coordinate pairs')
    result = {}
    for name, pair in raw.items():
        if not isinstance(name, str) or not isinstance(pair, (list, tuple)) or len(pair) != 2:
            raise ValueError('Invalid docking GPS entry')
        if pair[0] is None and pair[1] is None:
            result[name] = None
        elif (all(isinstance(v, (int, float)) and math.isfinite(v) for v in pair)
              and -85 <= pair[0] <= 85 and -180 <= pair[1] <= 180):
            result[name] = tuple(pair)
        else:
            raise ValueError('Enter both latitude and longitude for ' + name)
    return result


def select_docking_target(camera_angle, gps_angle, gps_distance):
    if camera_angle is not None and math.isfinite(camera_angle):
        return 'CAMERA', None, camera_angle
    if (gps_angle is not None and gps_distance is not None
            and math.isfinite(gps_angle) and math.isfinite(gps_distance)):
        return 'GPS', gps_distance, gps_angle
    return 'WAIT', None, None


def gps_variable_defaults():
    config = validate_status_config(STATUS_CONFIG)
    route_points = config[1]['gps_points']
    if len(route_points) != 3:
        raise ValueError('Status 1 requires three gps_points')
    values = docking_gps_values(dict(start=config[0]['gps'], path1=route_points[0],
        path2=route_points[1], path3=route_points[2], hop=config[2]['gps'], dock_start=config[3]['gps'],
        avoidance_start=config[6]['gps'], avoidance_goal=config[7]['gps']))
    if values['start'] is None:
        raise ValueError("Enter STATUS_CONFIG[0]['gps']")
    route = [values[name] for name in ('path1', 'path2', 'path3')]
    if any(p is not None for p in route) and any(p is None for p in route):
        raise ValueError("Enter all three STATUS_CONFIG[1]['gps_points']")
    intermediate = config[7].get('intermediate_gps', [])
    if not isinstance(intermediate, (list, tuple)):
        raise ValueError('Status 7 intermediate_gps must be an ordered list')
    checked = docking_gps_values({str(i): pair for i, pair in enumerate(intermediate)})
    if any(pair is None for pair in checked.values()):
        raise ValueError('Enter both coordinates for each status 7 intermediate point, or remove it')
    return dict(goal_latitude=values['start'][0], goal_longitude=values['start'][1],
                route_goals_json=json.dumps(route if route[0] is not None else []),
                hop_goal_json=json.dumps(values['hop'] or []),
                dock_start_goal_json=json.dumps(values['dock_start'] or []),
                avoidance_start_goal_json=json.dumps(values['avoidance_start'] or []),
                avoidance_goal_json=json.dumps(values['avoidance_goal'] or []),
                avoidance_intermediate_json=json.dumps(list(checked.values())))


def pixel_angle(x, width, fov_deg):
    if not all(math.isfinite(v) for v in (x, width, fov_deg)):
        raise ValueError('Nonfinite camera geometry')
    if width <= 0 or not 0 <= x < width or not 0 < fov_deg < 180:
        raise ValueError('Invalid camera geometry')
    focal = width / 2 / math.tan(math.radians(fov_deg / 2))
    return -math.degrees(math.atan2(x - width / 2, focal))


class PixelPD:
    def __init__(self, kp=8.0, kd=0.0, turn_limit=400.0):
        self.kp, self.kd, self.limit = kp, kd, turn_limit
        self.reset()

    def reset(self):
        self.previous = None

    def calculate(self, error_deg, now):
        derivative = 0.0
        if self.previous is not None:
            old_error, old_time = self.previous
            dt = now - old_time
            if 0.01 <= dt <= 0.5:
                derivative = (error_deg - old_error) / dt
        self.previous = error_deg, now
        return max(
            -self.limit,
            min(
                self.limit,
                self.kp * error_deg + self.kd * derivative
            )
        )


def advance_status(current_status, distance_m, radius_m=COMMON_CONFIG['arrival_radius_m']):
    if (current_status == 0 and math.isfinite(distance_m)
            and 0 <= distance_m <= radius_m):
        return 1
    return current_status


class GPSRouteProgress:
    """Status 0 approach, then status 1 with three ordered GPS goals."""

    def __init__(self, start_goal, route_goals, hop_goal=None, dock_start_goal=None,
                 avoidance_start_goal=None, avoidance_goal=None,
                 arrival_radius_m=COMMON_CONFIG['arrival_radius_m'], avoidance_intermediate=()):
        if len(route_goals) not in (0, 3):
            raise ValueError('Status 1 requires exactly three GPS goals')
        for latitude, longitude in [start_goal, *route_goals,
                                    *([hop_goal] if hop_goal else []),
                                    *([dock_start_goal] if dock_start_goal else []),
                                    *([avoidance_start_goal] if avoidance_start_goal else []),
                                    *([avoidance_goal] if avoidance_goal else []), *avoidance_intermediate]:
            if not (math.isfinite(latitude) and math.isfinite(longitude)
                    and -85 <= latitude <= 85 and -180 <= longitude <= 180):
                raise ValueError('Invalid GPS goal')
        self.start_goal = tuple(start_goal)
        self.route_goals = tuple(tuple(goal) for goal in route_goals)
        self.hop_goal = tuple(hop_goal) if hop_goal else None
        self.dock_start_goal = tuple(dock_start_goal) if dock_start_goal else None
        self.avoidance_start_goal = tuple(avoidance_start_goal) if avoidance_start_goal else None
        self.avoidance_goal = tuple(avoidance_goal) if avoidance_goal else None
        self.avoidance_intermediate = tuple(tuple(point) for point in avoidance_intermediate)
        self.avoidance_index = 0
        if not math.isfinite(arrival_radius_m) or arrival_radius_m <= 0:
            raise ValueError('arrival_radius_m must be positive')
        self.arrival_radius_m = arrival_radius_m
        self.status = 0
        self.route_index = 0
        self.complete = False

    @property
    def goal(self):
        if self.status == 0:
            return self.start_goal
        if self.status == 1:
            return self.route_goals[self.route_index] if self.route_goals else None
        if self.status == 2:
            return self.hop_goal
        if self.status == 3:
            return self.dock_start_goal
        if self.status == 6:
            return self.avoidance_start_goal
        if self.status == 7:
            if self.avoidance_index < len(self.avoidance_intermediate):
                return self.avoidance_intermediate[self.avoidance_index]
            return self.avoidance_goal
        return None

    def advance_avoidance(self, distance_m):
        if (self.status == 7 and self.avoidance_index < len(self.avoidance_intermediate)
                and math.isfinite(distance_m) and 0 <= distance_m <= self.arrival_radius_m):
            self.avoidance_index += 1
            return True
        return False

    def finish_hopping(self):
        if self.status == 2:
            self.status = 3

    def finish_dock_start(self):
        if self.status == 3:
            self.status = 4

    def finish_docking(self):
        if self.status == 4:
            self.status = 5

    def finish_search(self):
        if self.status == 5:
            self.status = 6

    def finish_avoidance_start(self):
        if self.status == 6:
            self.status = 7

    def advance(self, distance_m):
        if (self.complete or self.goal is None or not math.isfinite(distance_m)
                or not 0 <= distance_m <= self.arrival_radius_m):
            return False
        if self.status == 0:
            self.status = 1
        elif self.route_index < 2:
            self.route_index += 1
        else:
            self.complete = True
            self.status = 2
        return True


class BuoyOrbit:
    def __init__(self, center, direction, radius, count, arrival_radius):
        self.ring = buoy_circle_points(*center, count, radius_m=radius)
        self.direction = 1 if direction == 'cw' else -1
        self.arrival_radius = arrival_radius
        self.waypoints = []
        self.index = 0

    @property
    def goal(self):
        return self.waypoints[self.index] if self.index < len(self.waypoints) else None

    def start(self, distance_to):
        start = min(range(len(self.ring)), key=lambda i: distance_to(self.ring[i]))
        count = len(self.ring)
        self.waypoints = [self.ring[(start + i * self.direction) % count]
                          for i in range(count)]

    def advance(self, distance):
        if self.goal is not None and distance <= self.arrival_radius:
            self.index += 1
            return True
        return False


class HoppingArrivalSequence:
    """Existing V3 reverse pulse, followed by the requested five-second neutral wait."""

    def __init__(self, reverse_seconds=1.0, wait_seconds=5.0, reverse_pwm=(1300, 1300)):
        self.phase = 'APPROACH'
        self.started = None
        self.reverse_done = False
        self.reverse_seconds = reverse_seconds
        self.wait_seconds = wait_seconds
        self.reverse_pwm = tuple(reverse_pwm)

    def step(self, now, arrived, executing):
        if not executing:
            self.started = None
            return (1500, 1500)
        if self.phase == 'APPROACH' and arrived and not self.reverse_done:
            self.phase, self.started = 'REVERSE', now
        if self.phase == 'REVERSE':
            if self.started is None:
                self.started = now
            if now - self.started < self.reverse_seconds:
                return self.reverse_pwm
            self.reverse_done = True
            self.phase, self.started = 'WAIT', now
        if self.phase == 'WAIT':
            if self.started is None:
                self.started = now
            if now - self.started < self.wait_seconds:
                return (1500, 1500)
            self.phase, self.started = 'COMPLETE', None
        return None


class DockStartArrivalSequence:
    def __init__(self, reverse_seconds=2.0, reverse_pwm=(1300, 1300),
                 arrival_radius_m=COMMON_CONFIG['arrival_radius_m']):
        self.phase = 'APPROACH'
        self.started = None
        self.reverse_seconds = reverse_seconds
        self.reverse_pwm = tuple(reverse_pwm)
        self.arrival_radius_m = arrival_radius_m

    def step(self, now, distance_m, executing):
        if not executing:
            self.started = None
            return (1500, 1500)
        if self.phase == 'APPROACH' and 0 <= distance_m <= self.arrival_radius_m:
            self.phase, self.started = 'REVERSE', now
        if self.phase == 'REVERSE':
            if self.started is None:
                self.started = now
            if now - self.started < self.reverse_seconds:
                return self.reverse_pwm
            self.phase, self.started = 'COMPLETE', None
        return (1500, 1500) if self.phase == 'COMPLETE' else None


def target_angle_deg(east, north, heading):
    error = (math.atan2(north, east) - heading + math.pi) % (2 * math.pi) - math.pi
    return math.degrees(error)


def vectornav_heading(yaw_deg, offset_deg):
    # Same north-zero clockwise -> east-zero CCW conversion as V3.
    return math.pi / 2 - math.radians(yaw_deg + offset_deg)


def mix_pwm(base_pwm, turn):
    left_delta = base_pwm - 1500 - turn
    right_delta = base_pwm - 1500 + turn

    scale = max(
        1.0,
        abs(left_delta) / 400,
        abs(right_delta) / 400
    )

    return (
        round(1500 + left_delta / scale),
        round(1500 + right_delta / scale)
    )


def follow_command(ready, base_pwm, turn):
    return mix_pwm(base_pwm, turn) if ready else (1500, 1500)


def pd_thrust_command(controller, target_deg, thrust_percentage, now, pwm_for_percent):
    """The single steering-output path shared by all mission statuses."""
    base_pwm = pwm_for_percent(thrust_percentage)[0]
    turn = controller.calculate(target_deg, now)
    return follow_command(True, base_pwm, turn)


def camera_output_class(base):
    class CameraOutput(base):
        def _tick(self, now, pwm, telemetry, telemetry_age):
            if self.pending and all(f.done() for f in self.pending):
                self.sent_at = now
            previous_error = self.error
            result = super()._tick(now, pwm, telemetry, telemetry_age)
            # A transient stop-ACK failure must not outlive a later successful stop.
            if self.stop_ack and self.error == 'STOP_UNCONFIRMED':
                self.error = getattr(self, 'first_output_fault', '') or 'OUTPUT_STOPPED'
            elif (self.error and self.error != 'STOP_UNCONFIRMED'
                  and self.error != previous_error):
                self.first_output_fault = self.error
            return result
    return CameraOutput


def load_ros_environment():
    os.environ.setdefault('ROS_DOMAIN_ID', str(ROS_CONFIG['domain_id']))
    os.environ.setdefault(
        'RMW_IMPLEMENTATION', ROS_CONFIG['rmw_implementation']
    )
    os.environ.setdefault('ROS_LOCALHOST_ONLY', str(ROS_CONFIG['localhost_only']))

    import importlib.util

    try:
        available = all(
            importlib.util.find_spec(name) is not None
            for name in (
                'rclpy',
                'vectornav_msgs',
                'yolo_msgs',
                'isv_shape_navigation'
            )
        )
    except (ImportError, ModuleNotFoundError):
        available = False

    if available:
        return

    if os.environ.get('GPS_STAGE1_ENV_LOADED') == '1':
        raise RuntimeError(
            'ROS packages unavailable after sourcing overlays'
        )

    import subprocess

    command = '''set -e
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
source "$HOME/ros2_ws/src/isv_kaboat/kaboat2026/.runtime/install/setup.bash"
exec env -0
'''

    result = subprocess.check_output(
        ['bash', '-c', command]
    )

    for entry in result.split(b'\0'):
        if b'=' in entry:
            key, value = entry.split(b'=', 1)
            os.environ[os.fsdecode(key)] = os.fsdecode(value)

    os.environ['GPS_STAGE1_ENV_LOADED'] = '1'
    os.execv(sys.executable, [sys.executable, *sys.argv])


def main(args=None):
    load_ros_environment()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import NavSatFix, LaserScan, CompressedImage
    from yolo_msgs.msg import DetectionArray
    from vectornav_msgs.msg import CommonGroup
    from isv_shape_navigation.gps_mission import Fix, gps_vector
    from isv_shape_navigation.gps_input import inspect_fix
    from kaboat_gps_hopping.gps_hopping import GPSHoppingMission, GPSHoppingConfig
    from isv_water_trials.tank_core import guidance
    from types import SimpleNamespace
    from dataclasses import fields
    from pathlib import Path
    import numpy as np
    import cv2
    import yaml
    from isv_shape_navigation.core import Scan
    from isv_shape_navigation.follow import FollowConfig
    from isv_shape_navigation.route_mission import RouteConfig
    from isv_shape_navigation.avoidance_v3 import RouteMissionV3
    from std_msgs.msg import String, Empty
    from std_srvs.srv import SetBool, Trigger
    from mechaship_interfaces.srv import ThrottlePulseWidth
    from isv_water_trials.actuator_config import command_pwm
    from isv_shape_navigation.output import (
        PairedOutput,
        OutputReadiness
    )
    from isv_shape_navigation.output_runtime import (
        CommandLease,
        acquire_output_lock,
        conflicting_output_nodes
    )

    class Follower(Node):
        def __init__(self):
            super().__init__('gps_stage1_follower')
            self.stage_config = validate_status_config(STATUS_CONFIG)

            defaults = dict(
                **COMMON_CONFIG,
                **gps_variable_defaults(),
                docking_target=self.stage_config[4]['target_shape'],
                docking_arrival_radius_m=self.stage_config[4]['arrival_radius_m'],
                docking_goals_json=json.dumps(self.stage_config[4]['gps_by_shape']),
                search_target=self.stage_config[5]['target_buoy'],
                search_goals_json=json.dumps(self.stage_config[5]['gps_by_buoy']),
                detector_model=self.stage_config[4]['model_path'],
                horizontal_fov_deg=self.stage_config[4]['horizontal_fov_deg'],
                confidence_threshold=self.stage_config[4]['confidence_threshold'],
                detection_timeout_sec=self.stage_config[4]['detection_timeout_sec'],
                image_timeout_sec=self.stage_config[4]['image_timeout_sec'],
                stage_thrust_percentages=[float(self.stage_config[s]['thrust_percentage']) for s in range(8)],
            )

            self.p = {}
            for name, value in defaults.items():
                self.p[name] = self.declare_parameter(
                    name, value
                ).value

            for name, value in self.p.items():
                if isinstance(value, float) and not math.isfinite(value):
                    raise ValueError('Invalid parameter: ' + name)

            if not (
                -85 <= self.p['goal_latitude'] <= 85
                and -180 <= self.p['goal_longitude'] <= 180
                and 0 <= self.p['turn_limit_us'] <= 400
                and self.p['kp'] >= 0 and self.p['kd'] >= 0
                and self.p['gps_timeout'] > 0
                and self.p['heading_timeout'] > 0
                and self.p['max_gps_sigma'] > 0
                and self.p['arrival_radius_m'] > 0
                and self.p['docking_arrival_radius_m'] > 0
                and 0 < self.p['horizontal_fov_deg'] < 180
                and 0 <= self.p['confidence_threshold'] <= 1
                and self.p['detection_timeout_sec'] > 0
                and self.p['image_timeout_sec'] > 0
            ):
                raise ValueError('Invalid GPS/controller parameters')

            if not 5.0 <= self.p['output_rate_hz'] <= 10.0:
                raise ValueError('Output rate must be 5..10 Hz')
            self.stage_thrust_percentages = self.p['stage_thrust_percentages']
            if (len(self.stage_thrust_percentages) != 8
                    or not all(isinstance(v, (int, float)) and not isinstance(v, bool)
                        and math.isfinite(v) and 0 <= v <= 100 for v in self.stage_thrust_percentages)):
                raise ValueError('stage_thrust_percentages requires eight values, each 0..100')

            self.pd = PixelPD(
                self.p['kp'],
                self.p['kd'],
                self.p['turn_limit_us']
            )

            self.fix = None
            self.heading = None
            self.heading_stamp = -math.inf
            self.gps_received_at = -math.inf
            self.heading_received_at = -math.inf
            self.telemetry_at = -math.inf

            self.telemetry = {}
            self.target = None
            self.reason = 'WAIT_DATA'
            route_goals = json.loads(self.p['route_goals_json'])
            hop_goal = json.loads(self.p['hop_goal_json'])
            dock_start_goal = json.loads(self.p['dock_start_goal_json'])
            avoidance_start_goal = json.loads(self.p['avoidance_start_goal_json'])
            avoidance_goal = json.loads(self.p['avoidance_goal_json'])
            avoidance_intermediate = json.loads(self.p['avoidance_intermediate_json'])
            if not all(isinstance(v, list) for v in (route_goals, hop_goal, dock_start_goal,
                                                   avoidance_start_goal, avoidance_goal, avoidance_intermediate)):
                raise ValueError('GPS goals must be JSON arrays')
            self.progress = GPSRouteProgress(
                (self.p['goal_latitude'], self.p['goal_longitude']),
                route_goals, hop_goal or None, dock_start_goal or None,
                avoidance_start_goal or None, avoidance_goal or None,
                arrival_radius_m=self.p['arrival_radius_m'],
                avoidance_intermediate=avoidance_intermediate
            )
            self.hopper = None
            if self.progress.hop_goal:
                self.hopper = GPSHoppingMission(
                    SimpleNamespace(max_percent=100.0),
                    guidance,
                    GPSHoppingConfig(
                        goal_configured=True,
                        goal_latitude=self.progress.hop_goal[0],
                        goal_longitude=self.progress.hop_goal[1],
                        gps_arrival_radius=self.p['arrival_radius_m'],
                        gps_timeout=self.p['gps_timeout'],
                        heading_timeout=self.p['heading_timeout'],
                        max_gps_sigma=self.p['max_gps_sigma']
                    )
                )
            self.hop_arrival = HoppingArrivalSequence(
                reverse_seconds=self.stage_config[2]['reverse_seconds'],
                wait_seconds=self.stage_config[2]['wait_seconds'],
                reverse_pwm=self.stage_config[2]['reverse_pwm'])
            self.dock_arrival = DockStartArrivalSequence(
                reverse_seconds=self.stage_config[3]['reverse_seconds'],
                reverse_pwm=self.stage_config[3]['reverse_pwm'],
                arrival_radius_m=self.p['arrival_radius_m'])
            settings_file = Path(self.p['v3_settings_file']).expanduser()
            self.v3_settings = yaml.safe_load(settings_file.read_text())['course_mission']['ros__parameters']
            cfg = FollowConfig(**{f.name: self.v3_settings[f.name] for f in fields(FollowConfig)
                                  if f.name in self.v3_settings})
            route_cfg = RouteConfig(**{f.name: self.v3_settings[f.name] for f in fields(RouteConfig)
                                      if f.name in self.v3_settings})
            self.v3_selector = SimpleNamespace(config=cfg, route_config=route_cfg, raw_scan=None)
            self.scan = None
            self.fallback_scan = None
            self.final_target_deg = None
            self.avoidance_reason = ''
            self.docking_goals = docking_gps_values(json.loads(self.p['docking_goals_json']))
            if self.p['docking_target'] not in self.docking_goals:
                raise ValueError("Docking target must have a STATUS_CONFIG[4]['gps_by_shape'] entry")
            self.search_goals = docking_gps_values(json.loads(self.p['search_goals_json']))
            if self.p['search_target'] not in self.search_goals:
                raise ValueError("Search target must have a STATUS_CONFIG[5]['gps_by_buoy'] entry")
            search = self.stage_config[5]
            search_goal = self.search_goals[self.p['search_target']]
            self.search_orbit = (BuoyOrbit(
                search_goal, search['orbit_direction_by_buoy'][self.p['search_target']],
                search['orbit_radius_m'], search['orbit_point_count'], search['orbit_arrival_radius_m'])
                if search_goal is not None else None)
            self.avoidance_start_arrival = HoppingArrivalSequence(
                reverse_seconds=self.stage_config[6]['reverse_seconds'],
                wait_seconds=self.stage_config[6]['wait_seconds'],
                reverse_pwm=self.stage_config[6]['reverse_pwm'])
            self.avoidance_start_hopper = None
            if self.progress.avoidance_start_goal is not None:
                goal = self.progress.avoidance_start_goal
                self.avoidance_start_hopper = GPSHoppingMission(
                    SimpleNamespace(max_percent=100.0), guidance,
                    GPSHoppingConfig(goal_configured=True,
                        goal_latitude=goal[0], goal_longitude=goal[1],
                        gps_arrival_radius=self.p['arrival_radius_m'], gps_timeout=self.p['gps_timeout'],
                        heading_timeout=self.p['heading_timeout'], max_gps_sigma=self.p['max_gps_sigma'])
                )
            self.width = None
            self.image_at = -math.inf
            self.image_stamp = -math.inf
            self.camera_target = None
            self.camera_target_v = None  # Display only; horizontal steering still uses u.
            self.camera_target_at = -math.inf
            self.camera_target_stamp = -math.inf
            self.docking_mode = 'WAIT'

            self.lease = CommandLease()
            self.readiness = OutputReadiness(settle_seconds=self.p['output_settle_seconds'])

            self.output = None
            self.output_lock = None

            # Preserve the supplied automatic output-start behavior.
            self.auto_follow_allowed = bool(
                self.p['enable_output']
                and self.p['auto_follow']
            )

            # Explicit stop or output fault inhibits auto-restart.
            self.stop_latched = False

            self.conflicts = []
            self.graph_at = -math.inf
            self.last_print_at = -math.inf
            self.logged_output_error = ''

            self.status_pub = self.create_publisher(
                String,
                '~/status',
                10
            )

            self.create_subscription(
                NavSatFix, self.p['gps_topic'], self.gps,
                qos_profile_sensor_data
            )
            self.create_subscription(
                CommonGroup, self.p['heading_topic'], self.imu,
                qos_profile_sensor_data
            )
            self.create_subscription(
                LaserScan, self.v3_settings.get('scan_topic', '/scan_preprocessed'),
                lambda msg: self.lidar(msg, False), qos_profile_sensor_data
            )
            self.create_subscription(
                LaserScan, self.v3_settings.get('scan_fallback_topic', '/scan'),
                lambda msg: self.lidar(msg, True), qos_profile_sensor_data
            )
            self.create_subscription(
                CompressedImage, self.p['image_topic'], self.image,
                qos_profile_sensor_data
            )
            self.create_subscription(
                DetectionArray, self.p['detection_topic'], self.detections,
                qos_profile_sensor_data
            )

            self.create_subscription(
                String,
                '/actuators/status',
                self.feedback,
                qos_profile_sensor_data
            )

            self.create_subscription(
                Empty,
                '/water_trial/stop',
                self.external_stop,
                10
            )

            if self.p['enable_output']:
                if self.get_parameter('use_sim_time').value:
                    raise ValueError(
                        'Hardware output requires real time'
                    )

                self.output_lock = acquire_output_lock()

                self.arm = self.create_client(
                    SetBool,
                    '/actuators/arm'
                )

                self.stop_client = self.create_client(
                    Trigger,
                    '/actuators/stop'
                )

                self.pwm = [
                    self.create_client(
                        ThrottlePulseWidth,
                        '/actuators/throttle_' + side +
                        '/set_pulse_width'
                    )
                    for side in ('left', 'right')
                ]

                self.output = camera_output_class(PairedOutput)(
                    lambda v: self.arm.call_async(
                        SetBool.Request(data=v)
                    ),
                    lambda l, r: [
                        client.call_async(
                            ThrottlePulseWidth.Request(
                                pulse_width=value
                            )
                        )
                        for client, value in zip(
                            self.pwm, (l, r)
                        )
                    ],
                    lambda: self.stop_client.call_async(
                        Trigger.Request()
                    ),
                    self.services_ready,
                    timeout=0.6,
                    stop_ready=self.stop_client.service_is_ready
                )

            self.get_logger().warning(
                f"GPS stage 1: goal=({self.p['goal_latitude']}, "
                f"{self.p['goal_longitude']}), "
                f"output={self.p['enable_output']}, "
                f"auto_follow={self.p['auto_follow']}. "
                'Status 0 start -> status 1 route -> status 2 hopping -> status 3 dock start -> status 4 docking -> status 5 search -> status 6 avoidance start -> status 7 V3 avoidance. '
                'Unset GPS goals remain unconfigured.'
            )
            self.get_logger().info(
                f"External YOLO: use model={self.p['detector_model']}; "
                f"target={self.p['docking_target']}; detections={self.p['detection_topic']}. "
                'This follower does not load or select the detector weights.'
            )

            self.create_service(
                Trigger, '~/start', self.start
            )
            self.create_service(
                Trigger, '~/stop', self.stop_service
            )

            self.create_timer(0.1, self.control)
            self.create_timer(
                1.0 / self.p['output_rate_hz'],
                self.output_tick
            )

        def ros_now(self):
            return self.get_clock().now().nanoseconds / 1e9

        def gps(self, msg):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            observation = inspect_fix(
                msg, stamp, self.ros_now(), self.p['gps_timeout'],
                self.p['max_gps_sigma'],
                self.fix.stamp if self.fix else None
            )
            self.gps_received_at = time.monotonic()
            if observation['reason'] == 'GPS_OUT_OF_ORDER':
                return
            self.fix = (Fix(msg.latitude, msg.longitude, observation['sigma'], stamp)
                        if observation['accepted'] else None)

        def imu(self, msg):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            if (stamp <= self.heading_stamp
                    or msg.header.frame_id != self.p['heading_frame']
                    or not -0.1 <= self.ros_now() - stamp <= self.p['heading_timeout']
                    or not math.isfinite(msg.yawpitchroll.x)):
                return
            self.heading = vectornav_heading(
                msg.yawpitchroll.x, self.p['heading_offset_deg']
            )
            self.heading_stamp = stamp
            self.heading_received_at = time.monotonic()

        def lidar(self, msg, fallback):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            previous = self.fallback_scan if fallback else self.scan
            cfg = self.v3_settings
            if (msg.header.frame_id != cfg.get('scan_frame', 'velodyne')
                    or not msg.ranges or not -0.1 <= self.ros_now() - stamp <= self.v3_selector.config.scan_timeout
                    or (previous is not None and stamp <= previous.stamp)
                    or not all(math.isfinite(v) for v in (msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max))
                    or msg.angle_increment == 0 or not 0 <= msg.range_min < msg.range_max):
                return
            scan = Scan(np.asarray(msg.ranges), msg.angle_min, msg.angle_increment,
                        max(cfg.get('avoidance_lidar_min_range', 0.1), msg.range_min),
                        msg.range_max, stamp, math.radians(cfg.get('scan_yaw_deg', 0.0)),
                        cfg.get('scan_x', 0.0), cfg.get('scan_y', 0.0))
            if fallback:
                self.fallback_scan = scan
            else:
                self.scan = scan

        def navigation_scan(self):
            for scan in (self.scan, self.fallback_scan):
                if scan is not None and -0.1 <= self.ros_now() - scan.stamp <= self.v3_selector.config.scan_timeout:
                    return scan
            return None

        def image(self, msg):
            if self.progress.status != 4:
                return
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            if not 0 <= self.ros_now() - stamp <= self.p['image_timeout_sec']:
                return
            try:
                image = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
            except (ValueError, cv2.error):
                image = None
            if image is not None and image.shape[1] > 0:
                if self.width != float(image.shape[1]):
                    self.camera_target = None
                self.width = float(image.shape[1])
                self.image_at = time.monotonic()
                self.image_stamp = stamp

        def detections(self, msg):
            if self.progress.status != 4:
                return
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            rows = []
            vertical = {}
            if self.width is not None and 0 <= self.ros_now() - stamp <= self.p['detection_timeout_sec']:
                for detection in msg.detections:
                    if detection.class_name.replace('_', ' ') != self.p['docking_target']:
                        continue
                    center = detection.bbox.center
                    x = float(getattr(center, 'position', center).x)
                    score = float(detection.score)
                    try:
                        y = float(getattr(center, 'position', center).y)
                    except (AttributeError, TypeError, ValueError):
                        y = None
                    if (math.isfinite(score) and math.isfinite(x)
                            and score >= self.p['confidence_threshold'] and 0 <= x < self.width):
                        rows.append((score, x))
                        vertical[(score, x)] = y if y is not None and math.isfinite(y) else None
            self.camera_target = max(rows, key=lambda row: row[0]) if rows else None
            self.camera_target_v = vertical.get(self.camera_target)
            if self.camera_target is not None:
                self.camera_target_at = time.monotonic()
                self.camera_target_stamp = stamp

        def camera_target_angle(self, now):
            stamp_now = self.ros_now()
            if (self.camera_target is None or self.width is None
                    or not 0 <= now - self.camera_target_at <= self.p['detection_timeout_sec']
                    or not 0 <= now - self.image_at <= self.p['image_timeout_sec']
                    or not 0 <= stamp_now - self.camera_target_stamp <= self.p['detection_timeout_sec']
                    or not 0 <= stamp_now - self.image_stamp <= self.p['image_timeout_sec']):
                return None
            return pixel_angle(self.camera_target[1], self.width, self.p['horizontal_fov_deg'])

        def feedback(self, msg):
            try:
                value = json.loads(msg.data)
                if not isinstance(value, dict):
                    return
            except (ValueError, TypeError):
                return

            self.telemetry = value
            self.telemetry_at = time.monotonic()

        def position_ready(self, now):
            stamp_now = self.ros_now()
            return (
                self.fix is not None
                and -0.1 <= stamp_now - self.fix.stamp <= self.p['gps_timeout']
                and 0 <= now - self.gps_received_at <= self.p['gps_timeout']
            )

        def gps_ready(self, now):
            stamp_now = self.ros_now()
            return (
                self.position_ready(now) and self.heading is not None
                and -0.1 <= stamp_now - self.heading_stamp <= self.p['heading_timeout']
                and 0 <= now - self.heading_received_at <= self.p['heading_timeout']
            )

        def navigation_ready(self, now):
            if self.progress.status == 4:
                return (self.navigation_scan() is not None
                        and (self.camera_target_angle(now) is not None or
                             (self.docking_goals[self.p['docking_target']] is not None and self.gps_ready(now))))
            needs_scan = (self.progress.status in (1, 3, 5, 7)
                          or (self.progress.status == 2 and self.hop_arrival.phase == 'APPROACH'))
            return (self.gps_ready(now)
                    and (self.progress.status != 5 or self.fix.sigma <= self.p['max_gps_sigma'])
                    and (not needs_scan or self.navigation_scan() is not None))

        def v3_goal_angle(self, desired_deg):
            scan = self.navigation_scan()
            if scan is None:
                self.avoidance_reason = 'WAIT_SCAN'
                return None
            raw = self.fallback_scan
            self.v3_selector.raw_scan = (raw if raw is not None
                and -0.1 <= self.ros_now() - raw.stamp <= self.v3_selector.config.scan_timeout else None)
            selected, report = RouteMissionV3.select_corridor(
                self.v3_selector, scan, math.radians(desired_deg),
                self.v3_selector.config.footprint_radius + self.v3_selector.config.clearance)
            stage = self.stage_config[self.progress.status]
            selected, report = apply_course_avoidance_weights(selected, report,
                stage['avoidance_distance_weight'], stage['avoidance_angle_weight'])
            self.avoidance_reason = report.get('reason', '')
            return math.degrees(selected) if selected is not None else None

        def services_ready(self):
            return all(
                c.service_is_ready()
                for c in (
                    self.arm,
                    self.stop_client,
                    *self.pwm
                )
            )

        def start(self, request, response):
            now = time.monotonic()
            blocker = 'OUTPUT_DISABLED'

            if self.output:
                blocker = self.readiness.check(
                    now,
                    self.telemetry,
                    now - self.telemetry_at,
                    self.services_ready()
                )

                conflicts = conflicting_output_nodes(
                    self.get_node_names(),
                    self.get_name()
                )

                if conflicts:
                    blocker = (
                        'OUTPUT_CONFLICT: ' + ','.join(conflicts)
                    )

            response.success = bool(
                not blocker and self.output.start()
            )

            if response.success:
                self.auto_follow_allowed = True
                self.stop_latched = False

            response.message = blocker or (
                'STARTED'
                if response.success
                else 'ALREADY_ACTIVE_OR_STOPPING'
            )
            return response

        def stop(self, reason):
            # Latch explicit stops and output faults.
            self.stop_latched = True

            self.lease.publish(
                time.monotonic(),
                (1500, 1500),
                False
            )

            if self.output:
                self.output.request_stop(reason)

            self.reason = reason

        def external_stop(self, message):
            if (
                self.output
                and (
                    self.output.stopping
                    or self.output.stop_ack
                )
                and self.output.error
            ):
                self.stop_latched = True
                self.lease.publish(
                    time.monotonic(),
                    (1500, 1500),
                    False
                )
                return

            self.stop('EXTERNAL_STOP')

        def stop_service(self, request, response):
            self.stop('OPERATOR_STOP')
            self.pd.reset()

            response.success = True
            response.message = (
                'Stop requested; check status.stop_ack '
                'for MCU acknowledgment'
            )
            return response

        def control(self):
            now = time.monotonic()
            ready = self.navigation_ready(now)
            pwm = (1500, 1500)
            self.target = None
            self.final_target_deg = None
            self.avoidance_reason = ''
            goal = (self.docking_goals[self.p['docking_target']]
                    if self.progress.status == 4 else
                    self.progress.goal if self.progress.status != 5 else None)
            orbit = self.search_orbit
            if self.progress.status == 5 and orbit is not None:
                if ready and not self.stop_latched and not orbit.waypoints:
                    orbit.start(lambda point: math.hypot(*gps_vector(self.fix, *point)))
                goal = orbit.goal
            hopper_state = None
            transitioned_to_search = False
            if ready and self.progress.status != 4 and goal is not None:
                east, north = gps_vector(
                    self.fix, goal[0], goal[1]
                )
                if self.progress.status < 2 and not self.stop_latched:
                    if self.progress.advance(math.hypot(east, north)):
                        self.pd.reset()
                        goal = self.progress.goal
                        if goal is not None:
                            east, north = gps_vector(self.fix, goal[0], goal[1])
                elif self.progress.status == 7 and not self.stop_latched:
                    if self.progress.advance_avoidance(math.hypot(east, north)):
                        self.pd.reset()
                        goal = self.progress.goal
                        if goal is not None:
                            east, north = gps_vector(self.fix, goal[0], goal[1])
            if ready and self.progress.status != 4 and goal is not None:
                angle = target_angle_deg(east, north, self.heading)
                self.target = (math.hypot(east, north), angle)
                self.final_target_deg = angle
                if self.progress.status in (2, 6):
                    hopper = self.hopper if self.progress.status == 2 else self.avoidance_start_hopper
                    hopper.fix = self.fix
                    hopper.heading = self.heading
                    hopper.heading_stamp = self.heading_stamp
                    hopper_state, _ = hopper.step(self.ros_now())

            if self.progress.status == 4:
                gps_angle, gps_distance = None, None
                if goal is not None and self.position_ready(now):
                    east, north = gps_vector(self.fix, goal[0], goal[1])
                    gps_distance = math.hypot(east, north)
                    if self.gps_ready(now):
                        gps_angle = target_angle_deg(east, north, self.heading)
                if gps_distance is not None and gps_distance <= self.p['docking_arrival_radius_m'] and not self.stop_latched:
                    self.progress.finish_docking()
                    self.pd.reset()
                    self.target = None
                    self.camera_target = None
                    self.docking_mode = 'WAIT'
                    goal = None
                    transitioned_to_search = True
                else:
                    mode, distance, angle = select_docking_target(
                        self.camera_target_angle(now), gps_angle, gps_distance
                    )
                    if mode != self.docking_mode:
                        self.pd.reset()
                    self.docking_mode = mode
                    if angle is not None:
                        self.target = (distance, angle)
                        self.final_target_deg = angle

            if ready and self.target is not None and not self.stop_latched:
                override_pwm = None
                self.reason = 'GPS_FOLLOW'
                if self.progress.status == 1:
                    self.final_target_deg = self.v3_goal_angle(self.target[1])
                    self.reason = 'ROUTE_V3' if self.final_target_deg is not None else 'ROUTE_NO_DIRECTION'
                elif self.progress.status == 2:
                    arrived = hopper_state in ('GPS_ARRIVAL_HOLD', 'GPS_HOLD')
                    executing = bool(self.output and self.output.active and self.output.armed)
                    override = self.hop_arrival.step(now, arrived, executing)
                    if override is not None:
                        override_pwm = override
                    elif hopper_state not in ('GPS_TRANSIT', 'ALIGN_GPS'):
                        override_pwm = (1500, 1500)
                    self.reason = ('HOP_' + self.hop_arrival.phase
                                   if self.hop_arrival.phase in ('REVERSE', 'WAIT')
                                   else hopper_state)
                    if override_pwm is None and hopper_state in ('GPS_TRANSIT', 'ALIGN_GPS'):
                        self.final_target_deg = self.v3_goal_angle(self.target[1])
                        self.reason = 'HOP_V3' if self.final_target_deg is not None else 'HOP_NO_DIRECTION'
                    if self.hop_arrival.phase == 'COMPLETE':
                        self.progress.finish_hopping()
                        self.pd.reset()
                        self.target = None
                        self.final_target_deg = None
                        goal = self.progress.goal
                        override_pwm = (1500, 1500)
                        self.reason = 'STATUS3_DOCK_START'
                elif self.progress.status in (3, 7):
                    executing = bool(self.output and self.output.active and self.output.armed)
                    override = (self.dock_arrival.step(now, self.target[0], executing)
                                if self.progress.status == 3 else None)
                    if override is not None:
                        override_pwm = override
                        self.final_target_deg = None
                        self.reason = 'DOCK_START_' + self.dock_arrival.phase
                        if self.dock_arrival.phase == 'COMPLETE':
                            self.progress.finish_dock_start()
                            self.pd.reset()
                            self.target = None
                            self.final_target_deg = None
                            goal = self.docking_goals[self.p['docking_target']]
                            self.reason = 'STATUS4_DOCKING'
                    else:
                        self.final_target_deg = self.v3_goal_angle(self.target[1])
                        prefix = 'DOCK_START' if self.progress.status == 3 else 'AVOIDANCE'
                        self.reason = prefix + ('_V3' if self.final_target_deg is not None else '_NO_DIRECTION')
                elif self.progress.status == 4:
                    self.final_target_deg = self.v3_goal_angle(self.target[1])
                    self.reason = 'DOCK_' + self.docking_mode + (
                        '_V3' if self.final_target_deg is not None else '_NO_DIRECTION')
                elif self.progress.status == 5:
                    executing = bool(self.output and self.output.active and self.output.armed)
                    if executing and orbit.advance(self.target[0]):
                        self.pd.reset()
                        goal = orbit.goal
                        if goal is not None:
                            east, north = gps_vector(self.fix, *goal)
                            self.target = (math.hypot(east, north), target_angle_deg(east, north, self.heading))
                    if goal is not None:
                        self.final_target_deg = self.v3_goal_angle(self.target[1])
                        self.reason = 'SEARCH_ORBIT_V3' if self.final_target_deg is not None else 'SEARCH_NO_DIRECTION'
                    else:
                        self.progress.finish_search()
                        self.pd.reset()
                        self.target = None
                        self.final_target_deg = None
                        goal = self.progress.goal
                        override_pwm = (1500, 1500)
                        self.reason = 'STATUS6_AVOIDANCE_START'
                elif self.progress.status == 6:
                    arrived = hopper_state in ('GPS_ARRIVAL_HOLD', 'GPS_HOLD')
                    executing = bool(self.output and self.output.active and self.output.armed)
                    override = self.avoidance_start_arrival.step(now, arrived, executing)
                    if override is not None:
                        override_pwm = override
                    elif hopper_state not in ('GPS_TRANSIT', 'ALIGN_GPS'):
                        override_pwm = (1500, 1500)
                    self.reason = ('AVOIDANCE_START_' + self.avoidance_start_arrival.phase
                                   if self.avoidance_start_arrival.phase != 'APPROACH'
                                   else 'AVOIDANCE_START_' + hopper_state)
                    if self.avoidance_start_arrival.phase == 'COMPLETE':
                        self.progress.finish_avoidance_start()
                        self.pd.reset()
                        self.target = None
                        self.final_target_deg = None
                        goal = self.progress.goal
                        override_pwm = (1500, 1500)
                        self.reason = 'STATUS7_AVOIDANCE'
                if override_pwm is not None:
                    self.pd.reset()
                    pwm = override_pwm
                    self.final_target_deg = None
                elif self.target is None or self.final_target_deg is None:
                    self.pd.reset()
                    pwm = (1500, 1500)
                else:
                    pwm = pd_thrust_command(self.pd, self.final_target_deg,
                        self.stage_thrust_percentages[self.progress.status], now, command_pwm)
                self.lease.publish(now, pwm, True)

            else:
                self.pd.reset()
                self.hop_arrival.step(now, False, False)
                self.dock_arrival.step(now, math.inf, False)
                self.avoidance_start_arrival.step(now, False, False)
                self.reason = ('STATUS5_SEARCH' if transitioned_to_search else
                               'WAIT_ROUTE_GPS' if goal is None and self.progress.status == 1
                               else 'WAIT_HOP_GPS' if goal is None and self.progress.status == 2
                               else 'WAIT_DOCK_START_GPS' if goal is None and self.progress.status == 3
                               else 'WAIT_DOCK_TARGET_OR_GPS' if self.progress.status == 4
                               else 'WAIT_SEARCH_GPS' if orbit is None and self.progress.status == 5
                               else 'WAIT_SEARCH_SENSORS' if self.progress.status == 5
                               else 'WAIT_AVOIDANCE_START_GPS' if goal is None and self.progress.status == 6
                               else 'WAIT_AVOIDANCE_GOAL_GPS' if goal is None and self.progress.status == 7
                               else 'WAIT_GPS_IMU_NEUTRAL')
                self.lease.publish(
                    now,
                    (1500, 1500),
                    True
                )

            stopped = bool(
                self.output
                and not self.output.active
                and self.output.error
            )

            status = dict(
                status=self.progress.status,
                thrust_percentage=self.stage_thrust_percentages[self.progress.status],
                route_point=self.progress.route_index + 1 if self.progress.status == 1 else None,
                avoidance_point=self.progress.avoidance_index + 1 if self.progress.status == 7 else None,
                avoidance_final_goal=(self.progress.avoidance_index == len(self.progress.avoidance_intermediate))
                    if self.progress.status == 7 else None,
                hopping_phase=self.hop_arrival.phase if self.progress.status == 2 else None,
                dock_start_phase=self.dock_arrival.phase if self.progress.status == 3 else None,
                docking_mode=self.docking_mode if self.progress.status == 4 else None,
                docking_target=self.p['docking_target'],
                search_target=self.p['search_target'],
                search_phase=('ORBIT' if orbit and orbit.index else 'APPROACH')
                    if self.progress.status == 5 else None,
                search_point=orbit.index + 1 if self.progress.status == 5 and orbit and orbit.goal else None,
                avoidance_start_phase=self.avoidance_start_arrival.phase if self.progress.status == 6 else None,
                detector_model=self.p['detector_model'],
                target_x=self.camera_target[1] if self.progress.status == 4 and self.camera_target is not None else None,
                final_target_deg=self.final_target_deg,
                avoidance_reason=self.avoidance_reason,
                state=(
                    'OUTPUT_STOPPED'
                    if stopped
                    else self.reason
                ),
                goal_latitude=goal[0] if goal else None,
                goal_longitude=goal[1] if goal else None,
                distance_m=self.target[0] if self.target else None,
                target_deg=self.target[1] if self.target else None,
                kp=self.p['kp'],
                kd=self.p['kd'],
                intended_pwm=pwm,
                output_rate_hz=self.p['output_rate_hz'],
                output_enabled=self.output is not None,
                active=bool(
                    self.output and self.output.active
                ),
                auto_start_pending=bool(
                    self.auto_follow_allowed
                    and not self.stop_latched
                    and not self.output.active
                    if self.output else False
                ),
                stop_latched=self.stop_latched,
                armed=bool(
                    self.output and self.output.armed
                ),
                stop_ack=bool(
                    self.output and self.output.stop_ack
                ),
                acknowledged_pwm=(
                    self.output.last_ack_pwm
                    if self.output else None
                ),
                output_error=(
                    self.output.error if self.output else ''
                )
            )

            self.status_pub.publish(
                String(data=json.dumps(status))
            )

            if now - self.last_print_at >= 1.0:
                self.last_print_at = now
                self.get_logger().info(
                    mission_log_line(status, self.telemetry, now-self.telemetry_at,
                                     self.camera_target_v)
                )

        def output_tick(self):
            if not self.output:
                return

            now = time.monotonic()

            self.readiness.observe(
                now,
                self.telemetry,
                now - self.telemetry_at
            )

            if now - self.graph_at >= 1:
                self.conflicts = conflicting_output_nodes(
                    self.get_node_names(),
                    self.get_name()
                )
                self.graph_at = now

            pwm, error = self.lease.read(now)

            # Retain the supplied output readiness and stop/fault behavior.
            # Never auto-rearm after an explicit stop or fault.
            if (
                self.auto_follow_allowed
                and not self.stop_latched
                and not self.output.active
                and not self.output.stopping
                and not self.output.error
            ):
                blocker = self.readiness.check(
                    now,
                    self.telemetry,
                    now - self.telemetry_at,
                    self.services_ready()
                )

                if (
                    not blocker
                    and not self.conflicts
                    and not error
                ):
                    self.output.start()

            # Neutral when GPS/IMU input is unavailable, or explicitly stopped.
            if (
                not self.navigation_ready(now)
                or self.stop_latched
            ):
                pwm = (1500, 1500)

            if (
                error or self.conflicts
            ) and self.output.active:
                self.stop(
                    error or 'DATA_OR_OUTPUT_CONFLICT'
                )

            self.output.tick(
                now,
                pwm,
                self.telemetry,
                now - self.telemetry_at
            )

            if self.output.error != self.logged_output_error:
                self.logged_output_error = self.output.error

                if self.output.error:
                    self.get_logger().error(
                        'OUTPUT_STOPPED: ' + self.output.error
                    )

    from rclpy.signals import SignalHandlerOptions

    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO
    )

    def interrupt(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)

    node = Follower()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if rclpy.ok() and node.output:
            node.stop('SHUTDOWN')

            deadline = time.monotonic() + 2.0

            while (
                not node.output.stop_ack
                and time.monotonic() < deadline
            ):
                node.output_tick()
                rclpy.spin_once(
                    node,
                    timeout_sec=0.02
                )

            if not node.output.stop_ack:
                node.get_logger().error(
                    'STOP_UNCONFIRMED'
                )

        node.destroy_node()

        if node.output_lock:
            node.output_lock.close()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
