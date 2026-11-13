"""V3 objective and candidate selection without ROS or actuator output."""
import json
import math
import os
import unittest
from unittest.mock import patch

import numpy as np

from kaboat_avoidance.avoidance_v3 import (
    ANGLE_WEIGHT, DISTANCE_WEIGHT, RouteMissionV3, choose_gap_v3, lidar_range_at_angle,
    score_candidate,
)
from isv_shape_navigation.core import Scan
from isv_shape_navigation.follow import FollowConfig
from isv_shape_navigation.gps_mission import GPSConfig, Fix
from isv_shape_navigation.supplied_pwm import amplify_turn_pwm
from isv_shape_navigation.supplied_pwm import heading_pwm
from kaboat_avoidance.route_mission import RouteConfig


class AvoidanceV3Tests(unittest.TestCase):
    def setUp(self):
        self.config = RouteConfig(avoidance_range=3., avoidance_horizon=3.,
            wp_latitudes=(35.0001, 35.0002, 35.0003),
            wp_longitudes=(129., 129., 129.))

    def scan(self, blocked=False, now=100.):
        ranges = np.full(721, np.inf)
        if blocked:
            ranges[354:367] = 1.
        return Scan(ranges, -math.pi, math.pi/360, .1, 3., now)

    def test_exact_objective_and_weight_ratio(self):
        score = score_candidate(math.radians(10), 0., 2.)
        self.assertEqual(ANGLE_WEIGHT, 50*DISTANCE_WEIGHT)
        self.assertAlmostEqual(score['total_score'],
            DISTANCE_WEIGHT/(2.+1e-3) + ANGLE_WEIGHT*10)
        self.assertAlmostEqual(score_candidate(math.radians(179),
            math.radians(-179), 2.)['goal_angle_error_deg'], 2.)

    def test_turn_gain_preserves_neutral_without_per_side_cap(self):
        self.assertEqual(RouteMissionV3.output_turn_gain, 1.5)
        self.assertIsNone(getattr(RouteMissionV3, 'output_limit_percent', None))
        self.assertEqual(amplify_turn_pwm((1500, 1500), 1.5), (1500, 1500))
        turned = amplify_turn_pwm((1620, 1512), 1.5)
        self.assertEqual(turned, (1647, 1485))
        self.assertLess(turned[1], 1500)

    def test_range_is_measured_at_candidate_bearing(self):
        scan = self.scan()
        scan.ranges[370] = 1.25
        self.assertAlmostEqual(lidar_range_at_angle(scan, math.radians(5)), 1.25)
        self.assertEqual(lidar_range_at_angle(scan, 0.), 3.)

    def test_selects_only_collision_checked_candidates(self):
        angle, report = choose_gap_v3(self.scan(blocked=True), 0., self.config, .25)
        self.assertIsNotNone(angle)
        self.assertGreater(abs(angle), math.radians(20))
        self.assertEqual(report['algorithm'], 'V3_INVERSE_RANGE_GOAL_ANGLE')
        self.assertEqual(sum(row['selected'] for row in report['candidate_scores']), 1)
        self.assertEqual(report['selected_score'],
                         min(row['total_score'] for row in report['candidate_scores']))
        json.dumps(report, allow_nan=False)

    def test_open_water_goes_direct_and_near_collision_stops(self):
        angle, report = choose_gap_v3(self.scan(), 0., self.config, .25)
        self.assertEqual(angle, 0.)
        self.assertTrue(report['selected_direct'])
        scan = self.scan()
        scan.ranges[360] = .2
        angle, report = choose_gap_v3(scan, 0., self.config, .25)
        self.assertIsNone(angle)
        self.assertEqual(report['reason'], 'OBSTACLE_STOP')

    def test_v3_keeps_a_best_effort_bearing_when_all_gaps_are_blocked(self):
        mission = RouteMissionV3(FollowConfig(), lambda *_args, **_kwargs: (0., 0.),
                                 GPSConfig(goal_configured=True), self.config)
        scan = self.scan()
        scan.ranges[:] = 0.2
        angle, report = mission.select_corridor(scan, math.radians(15), .25)
        self.assertAlmostEqual(math.degrees(angle), 15.)
        self.assertEqual(report['reason'], 'BEST_EFFORT')
        self.assertEqual(report['fallback_reason'], 'OBSTACLE_STOP')
        self.assertFalse(report['direct_clear'])
        self.assertIn('WAIT_SCAN', mission.motion_states)

    def test_wp1_avoidance_then_wp2_gps_hopping_and_hold(self):
        mission = RouteMissionV3(FollowConfig(), lambda *_args, **_kwargs: (8., 8.),
                                 GPSConfig(goal_configured=True), self.config)
        mission.executing = True
        mission.fix = Fix(self.config.wp_latitudes[0], self.config.wp_longitudes[0], .4, 100.)
        mission.heading, mission.heading_stamp = 0., 100.
        state, _ = mission.step(100., self.scan())
        self.assertEqual(state, 'WP1_REVERSE')
        self.assertEqual(mission.intended_pwm, (1300, 1300))
        state, _ = mission.step(100.9, self.scan())
        self.assertEqual(state, 'WP1_REVERSE')
        self.assertEqual(mission.intended_pwm, (1300, 1300))
        mission.heading_stamp = 101.1
        mission.fix = Fix(self.config.wp_latitudes[0], self.config.wp_longitudes[0], .4, 101.1)
        state, _ = mission.step(101.1, self.scan())
        self.assertEqual(state, 'ALIGN_GPS')
        self.assertNotEqual(mission.intended_pwm, (1500, 1500))
        self.assertEqual(mission.stage, 'GPS_HOP_WP2')
        self.assertIn(1, mission.announced_waypoints)

        mission.fix = Fix(35.00015, 129., .4, 102.)
        mission.heading, mission.heading_stamp = math.pi/2, 102.
        state, wheels = mission.step(102., None)
        self.assertEqual(state, 'GPS_TRANSIT')
        expected = heading_pwm(mission.distance_to_wp, -mission.heading_error_deg,
                               forward_boost=self.config.forward_pwm_boost)
        self.assertEqual(mission.intended_pwm, expected)
        self.assertEqual(wheels, mission.percentages())
        self.assertGreater(mission.intended_pwm[0], 1532)

        mission.fix = Fix(self.config.wp_latitudes[1], self.config.wp_longitudes[1], .4, 103.)
        state, wheels = mission.step(103., None)
        self.assertEqual(state, 'WP2_REVERSE')
        self.assertEqual(mission.intended_pwm, (1300, 1300))
        self.assertEqual(wheels, mission.percentages())
        state, _ = mission.step(103.9, None)
        self.assertEqual(state, 'WP2_REVERSE')
        self.assertEqual(mission.intended_pwm, (1300, 1300))
        mission.fix = Fix(self.config.wp_latitudes[1], self.config.wp_longitudes[1], .4, 105.1)
        state, _ = mission.step(105.1, None)
        self.assertEqual(state, 'GPS_HOLD')
        self.assertIn(2, mission.announced_waypoints)
        self.assertEqual(mission.stage, 'GPS_HOP_WP2')
        self.assertFalse(mission.completed)

        mission.fix = Fix(35.00015, 129., .4, 106.)
        mission.heading_stamp = 106.
        state, _ = mission.step(106., None)
        self.assertEqual(state, 'GPS_TRANSIT')
        state, _ = mission.step(110., None)
        self.assertEqual(state, 'WAIT_GPS')
        self.assertEqual(mission.intended_pwm, (1500, 1500))
        mission.reset()
        self.assertEqual(mission.stage, 'TO_WP1')

    def test_wp2_resume_survives_reset_only_for_this_process(self):
        with patch.dict(os.environ, {'KABOAT_V3_RESUME_WP2': '1'}):
            mission = RouteMissionV3(FollowConfig(), lambda *_args, **_kwargs: (0., 0.),
                                     GPSConfig(goal_configured=True), self.config)
            self.assertEqual(mission.stage, 'GPS_HOP_WP2')
            mission.reset()
            self.assertEqual(mission.stage, 'GPS_HOP_WP2')
        mission.reset()
        self.assertEqual(mission.stage, 'TO_WP1')


if __name__ == '__main__':
    unittest.main()
