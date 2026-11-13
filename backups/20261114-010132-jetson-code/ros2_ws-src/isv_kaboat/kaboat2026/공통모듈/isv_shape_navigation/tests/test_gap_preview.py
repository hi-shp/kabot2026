import copy
import json
import math
import unittest
from unittest.mock import patch
import numpy as np

from isv_shape_navigation.gap_preview import GapPreview
from isv_shape_navigation.avoidance_v2 import RouteMissionV2, choose_gap
from isv_shape_navigation.core import Scan
from isv_shape_navigation.follow import FollowConfig
from isv_shape_navigation.gps_mission import GPSConfig
from isv_shape_navigation.route_mission import RouteConfig


def scan(now=100.,blocked=True):
    values=np.full(721,np.inf)
    if blocked:values[354:367]=1.
    return Scan(values,-math.pi,math.pi/360,.1,3.,now)


class GapPreviewTests(unittest.TestCase):
    def setUp(self):
        self.route=RouteConfig(avoidance_range=3.,avoidance_horizon=3.)
        self.follow=FollowConfig(footprint_radius=.1,clearance=.15)
        self.preview=GapPreview()

    def observe(self,source=None,state='WAIT_GPS',now=100.,raw=None):
        return self.preview.snapshot(now,source,raw,self.route,self.follow,state)

    def test_gps_and_heading_absent_do_not_hide_observed_gaps(self):
        source=scan()
        for state in GapPreview.WAIT_STATES:
            d=self.observe(source,state)
            self.assertEqual(d['status'],'LIVE');self.assertEqual(len(d['gaps']),2)
            self.assertTrue(d['observation_only']);self.assertIsNone(d['selected_gap']);self.assertIsNone(d['command_deg'])
            self.assertTrue(all(not g['selected'] for g in d['gaps']))
            self.assertNotIn('candidate_scores',d);self.assertNotIn('selected_angle_deg',d)

    def test_same_candidates_as_existing_planner(self):
        source=scan();_,actual=choose_gap(source,math.radians(22),self.route,.25)
        d=self.observe(source)
        self.assertEqual([(g['left_deg'],g['right_deg']) for g in d['gaps']],
                         [(g['left_deg'],g['right_deg']) for g in actual['gaps']])

    def test_no_controller_or_sensor_mutation_and_no_motion(self):
        mission=RouteMissionV2(self.follow,lambda *_:(0.,0.),GPSConfig(goal_configured=True),self.route)
        source=scan();state,_=mission.step(100.,source);self.assertEqual(state,'WAIT_GPS')
        before=repr(mission.__dict__);ranges=source.ranges.copy()
        d=self.observe(source)
        self.assertEqual(repr(mission.__dict__),before);np.testing.assert_array_equal(source.ranges,ranges)
        self.assertEqual(mission.intended_pwm,(1500,1500));self.assertFalse(mission.executing)
        json.dumps(d,allow_nan=False)

    def test_stale_or_missing_scan_is_not_a_candidate(self):
        for source in (None,scan(now=98.)):
            d=self.observe(source);self.assertEqual(d['status'],'WAIT_SCAN');self.assertEqual(d['gaps'],[])

    def test_raw_near_obstacle_still_excludes_gaps(self):
        raw=scan();raw.ranges[360]=.12
        d=self.observe(scan(),raw=raw)
        self.assertEqual(d['check_reason'],'OBSTACLE_STOP');self.assertEqual(d['gaps'],[])

    def test_invalid_samples_and_no_return_water_keep_existing_rules(self):
        bad=scan(blocked=False);bad.ranges[:]=np.nan
        self.assertEqual(self.observe(bad)['gaps'],[])
        self.assertGreater(len(self.observe(scan(blocked=False))['gaps']),0)

    def test_same_scan_is_cached_and_motion_uses_original_planner(self):
        source=scan()
        with patch('isv_shape_navigation.gap_preview.choose_gap',wraps=choose_gap) as planner:
            self.observe(source);self.observe(source,now=100.1);self.assertEqual(planner.call_count,1)
            self.assertIsNone(self.observe(source,state='AVOID_OBSTACLE'))
            self.assertEqual(planner.call_count,1)

    def test_stale_raw_scan_is_not_reused(self):
        raw=scan(now=98.);raw.ranges[360]=.12
        self.assertEqual(len(self.observe(scan(),raw=raw)['gaps']),2)


if __name__=='__main__':unittest.main()
