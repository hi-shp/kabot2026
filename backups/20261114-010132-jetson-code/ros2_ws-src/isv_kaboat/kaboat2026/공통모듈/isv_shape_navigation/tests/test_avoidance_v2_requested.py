"""Pure V2 regression tests: no ROS, sensor launch, or actuator clients."""
import math
import unittest
import numpy as np

from isv_shape_navigation.avoidance_v2 import RouteMissionV2, choose_gap
from isv_shape_navigation.core import Scan
from isv_shape_navigation.follow import FollowConfig
from isv_shape_navigation.gps_mission import GPSConfig, Fix
from isv_shape_navigation.route_mission import RouteConfig


def scan(value=np.inf, now=100., maximum=3.):
    return Scan(np.full(721, value), -math.pi, math.pi/360, .1, maximum, now)


class RequestedV2Tests(unittest.TestCase):
    def setUp(self):
        self.route=RouteConfig(avoidance_range=3.,avoidance_horizon=3.,
            wp_latitudes=(35.0001,35.0002,35.0003),wp_longitudes=(129.,129.,129.))

    def test_open_water_without_any_measured_far_return_is_clear(self):
        source=scan()
        angle,report=choose_gap(source,0.,self.route,.25,raw_scan=source)
        self.assertAlmostEqual(angle,0.)
        self.assertEqual(report['reason'],'CLEAR')
        self.assertTrue(report['selected_direct'])
        self.assertFalse(len(source.points()))  # No artificial 3.25m return.

    def test_valid_processed_scan_does_not_require_a_raw_far_return(self):
        angle,report=choose_gap(scan(),math.radians(20),self.route,.25)
        self.assertAlmostEqual(angle,math.radians(20))
        self.assertEqual(report['reason'],'CLEAR')

    def test_invalid_ranges_do_not_become_clear_water(self):
        for value in (np.nan,0.,-np.inf):
            with self.subTest(value=value):
                angle,report=choose_gap(scan(value),0.,self.route,.25)
                self.assertIsNone(angle)
                self.assertEqual(report['reason'],'NO_SAFE_GAP')

    def test_invalid_original_scan_is_not_hidden_by_preprocessed_infinity(self):
        angle,report=choose_gap(scan(),0.,self.route,.25,raw_scan=scan(np.nan))
        self.assertIsNone(angle)
        self.assertGreater(report['diagnostics']['rejected_invalid_scan'],0)

    def test_twenty_centimetre_return_still_triggers_near_obstacle_stop(self):
        raw=scan();raw.ranges[360]=.2
        angle,report=choose_gap(scan(),0.,self.route,.25,raw_scan=raw)
        self.assertIsNone(angle)
        self.assertEqual(report['reason'],'OBSTACLE_STOP')
        self.assertAlmostEqual(report['diagnostics']['closest'],.2)

    def test_raw_obstacle_removed_by_preprocessing_still_blocks_the_path(self):
        raw=scan();raw.ranges[360]=1.
        angle,report=choose_gap(scan(),0.,self.route,.25,raw_scan=raw)
        # The straight path is still blocked. A newly checked side heading may
        # now be returned instead of discarding the entire angular opening.
        self.assertIsNotNone(angle)
        self.assertFalse(report['direct_clear'])
        self.assertGreater(report['diagnostics']['rejected_collision'],0)
        points=raw.points();along=points[:,0]*math.cos(angle)+points[:,1]*math.sin(angle)
        cross=-points[:,0]*math.sin(angle)+points[:,1]*math.cos(angle)
        self.assertGreaterEqual(float(np.hypot(along-np.clip(along,0,3.),cross).min()),.25)

    def test_no_reverse_recovery_even_when_rear_has_measured_clearance(self):
        mission=RouteMissionV2(FollowConfig(footprint_radius=.1,clearance=.15),
            lambda *_:(0.,0.),GPSConfig(goal_configured=True),self.route)
        mission.executing=True
        for now in (100.,100.3,101.,104.):
            primary=scan(np.nan,now)
            rear=scan(np.nan,now,30.)
            angles=rear.angle_min+np.arange(len(rear.ranges))*rear.angle_increment
            rear.ranges[np.abs(np.arctan2(np.sin(angles-math.pi),np.cos(angles-math.pi)))<math.radians(25)]=2.
            mission.fix=Fix(35.,129.,.4,now)
            mission.heading,mission.heading_stamp=math.pi/2,now
            mission.raw_scan=rear
            state,_=mission.step(now,primary)
            self.assertEqual(state,'NO_SAFE_GAP')
            self.assertEqual(mission.intended_pwm,(1500,1500))
            self.assertEqual(mission.command_kind,'NEUTRAL')
        self.assertNotIn('RECOVERY_REVERSE',mission.motion_states)
        self.assertNotIn('RECOVERY_WAIT',mission.motion_states)

    def test_stale_original_scan_is_discarded(self):
        mission=RouteMissionV2(FollowConfig(footprint_radius=.1,clearance=.15),
            lambda *_:(0.,0.),GPSConfig(goal_configured=True),self.route)
        mission.raw_scan=scan(now=90.)
        mission.step(100.,scan())
        self.assertIsNone(mission.raw_scan)


if __name__=='__main__':unittest.main()
