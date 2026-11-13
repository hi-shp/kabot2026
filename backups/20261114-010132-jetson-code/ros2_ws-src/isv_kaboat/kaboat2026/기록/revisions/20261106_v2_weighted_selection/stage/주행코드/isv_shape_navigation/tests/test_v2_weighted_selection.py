"""Pure weighted V2 selection; no sensor nodes or actuator services."""
from dataclasses import replace
import json
import math
import unittest
import numpy as np

from isv_shape_navigation.avoidance_v2 import RouteMissionV2, choose_gap, score_candidate, score_weights
from isv_shape_navigation.core import Scan
from isv_shape_navigation.follow import FollowConfig
from isv_shape_navigation.gps_mission import GPSConfig, Fix
from isv_shape_navigation.navigation_display import decision_display
from isv_shape_navigation.route_mission import RouteConfig


def scan(now=100., blocked=False):
    ranges=np.full(721,np.inf)
    if blocked:ranges[354:367]=1.
    return Scan(ranges,-math.pi,math.pi/360,.1,3.,now)


class WeightedV2Tests(unittest.TestCase):
    def setUp(self):
        self.cfg=RouteConfig(avoidance_range=3.,avoidance_horizon=3.,
            wp_latitudes=(35.0001,35.0002,35.0003),wp_longitudes=(129.,129.,129.))

    def mission(self):
        return RouteMissionV2(FollowConfig(footprint_radius=.1,clearance=.15),
            lambda *_:(0.,0.),GPSConfig(goal_configured=True),self.cfg)

    def feed(self,mission,now=100.,yaw=math.pi/2,blocked=False):
        mission.fix=Fix(35.,129.,.4,now);mission.heading=yaw;mission.heading_stamp=now
        return mission.step(now,scan(now,blocked))

    def test_all_three_costs_are_normalized_and_sum_to_the_reported_score(self):
        row=score_candidate(math.pi/2,0.,.25,self.cfg,0.)
        self.assertAlmostEqual(row['wp_cost'],.5)
        self.assertAlmostEqual(row['clearance_cost'],.5)
        self.assertAlmostEqual(row['direction_cost'],.5)
        self.assertAlmostEqual(row['total_score'],.5)
        self.assertAlmostEqual(row['total_score'],sum(row[k] for k in ('weighted_wp','weighted_clearance','weighted_direction')))

    def test_extra_clearance_can_outweigh_a_small_wp_alignment_advantage(self):
        narrow=score_candidate(math.radians(10),0.,.05,self.cfg,0.)
        wide=score_candidate(math.radians(30),0.,.5,self.cfg,0.)
        self.assertLess(narrow['wp_cost'],wide['wp_cost'])
        self.assertLess(wide['total_score'],narrow['total_score'])

    def test_direction_term_selects_the_previous_side_for_symmetric_safe_gaps(self):
        source=scan(blocked=True)
        left,report=choose_gap(source,0.,self.cfg,.25,previous_angle=math.radians(60))
        right,_=choose_gap(source,0.,self.cfg,.25,previous_angle=math.radians(-60))
        self.assertGreater(left,0.)
        self.assertLess(right,0.)
        selected=[row for row in report['candidate_scores'] if row['selected']]
        self.assertEqual(len(selected),1)
        self.assertEqual(selected[0]['total_score'],min(row['total_score'] for row in report['candidate_scores']))

    def test_wp_term_can_overrule_previous_side_when_goal_moves(self):
        angle,_=choose_gap(scan(blocked=True),math.radians(35),self.cfg,.25,
            previous_angle=math.radians(-60))
        self.assertGreater(angle,0.)

    def test_colliding_goal_cannot_win_even_with_only_wp_weight(self):
        cfg=replace(self.cfg,v2_wp_weight=1.,v2_clearance_weight=0.,v2_direction_weight=0.)
        source=scan(blocked=True)
        angle,report=choose_gap(source,0.,cfg,.25,previous_angle=0.)
        self.assertGreater(abs(angle),math.radians(20))
        self.assertTrue(all(abs(row['angle_deg'])>20 for row in report['candidate_scores']))
        with self.assertRaises(ValueError):score_candidate(0.,0.,-.01,cfg,0.)

    def test_near_obstacle_stop_and_no_return_water_behavior_are_preserved(self):
        source=scan();source.ranges[360]=.2
        angle,report=choose_gap(source,0.,self.cfg,.25)
        self.assertIsNone(angle);self.assertEqual(report['reason'],'OBSTACLE_STOP')
        angle,report=choose_gap(scan(),0.,self.cfg,.25)
        self.assertEqual(angle,0.);self.assertTrue(report['selected_direct'])
        self.assertEqual(report['candidate_scores'][0]['candidate_id'],'WP_DIRECT')
        self.assertEqual(report['candidate_scores'][0]['clearance_m'],None)
        json.dumps(report,allow_nan=False)

    def test_initial_reference_has_zero_change_cost_and_angle_wrap_is_shortest(self):
        initial=score_candidate(.3,0.,math.inf,self.cfg)
        self.assertEqual(initial['direction_cost'],0.)
        self.assertIsNone(initial['direction_change_deg'])
        wrapped=score_candidate(math.radians(179),math.radians(-179),.5,self.cfg,math.radians(-179))
        self.assertAlmostEqual(wrapped['wp_error_deg'],2.)
        self.assertAlmostEqual(wrapped['direction_change_deg'],2.)

    def test_imu_rotation_is_not_mistaken_for_a_new_command_direction(self):
        mission=self.mission();self.feed(mission)
        reference=mission.previous_world_direction
        self.feed(mission,100.1,math.pi/2+math.radians(15))
        report=mission.avoidance
        self.assertAlmostEqual(report['previous_direction_deg'],-15.)
        self.assertAlmostEqual(report['candidate_scores'][0]['direction_change_deg'],0.)
        self.assertAlmostEqual(mission.previous_world_direction,reference)

    def test_reset_lost_input_and_new_leg_clear_direction_reference(self):
        mission=self.mission();self.feed(mission)
        self.assertIsNotNone(mission.previous_world_direction)
        mission.reset();self.assertIsNone(mission.previous_world_direction)
        self.feed(mission);mission.stage='WP1_TO_WP2';mission.wp_index=1
        self.feed(mission,100.1)
        self.assertIsNone(mission.avoidance['previous_direction_deg'])
        mission.fix=None;mission.step(100.2,scan(100.2))
        self.assertIsNone(mission.previous_world_direction)

    def test_old_history_does_not_penalize_new_sensor_input(self):
        mission=self.mission();self.feed(mission)
        self.feed(mission,101.)
        self.assertIsNone(mission.avoidance['previous_direction_deg'])

    def test_weights_validate_and_ratios_are_normalized(self):
        scaled=replace(self.cfg,v2_wp_weight=6.,v2_clearance_weight=3.,v2_direction_weight=1.)
        self.assertEqual(score_weights(scaled),dict(wp=.6,clearance=.3,direction=.1))
        for kwargs in ({'v2_wp_weight':-1.},{'v2_direction_weight':float('nan')},
                       {'v2_wp_weight':0.,'v2_clearance_weight':0.,'v2_direction_weight':0.},
                       {'v2_preferred_clearance':0.}):
            with self.assertRaises(ValueError):replace(self.cfg,**kwargs)

    def test_display_explains_weighted_winner_and_direct_route_has_no_fake_gap(self):
        _,avoid=choose_gap(scan(blocked=True),0.,self.cfg,.25,previous_angle=math.radians(60))
        view=decision_display(dict(heading_error_deg=0.,yaw=0.,command_kind='AVOIDANCE',avoidance=avoid))
        self.assertEqual(view['selected_gap'],avoid['selected_gap_index'])
        self.assertEqual(sum(row['selected'] for row in view['gaps']),1)
        self.assertIn('가중합 최소',view['reason'])
        self.assertIn('0.60/0.30/0.10',view['reason'])
        _,direct=choose_gap(scan(),0.,self.cfg,.25)
        view=decision_display(dict(heading_error_deg=0.,yaw=0.,command_kind='HEADING',avoidance=direct))
        self.assertIsNone(view['selected_gap'])


if __name__=='__main__':unittest.main()
