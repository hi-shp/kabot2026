import math
import unittest
import numpy as np
from isv_shape_navigation.core import Scan
from isv_shape_navigation.follow import FollowConfig,FusedTarget
from isv_shape_navigation.gps_mission import GPSConfig,Fix
from isv_shape_navigation.route_mission import RouteConfig,RouteMission,choose_corridor
from isv_shape_navigation.supplied_pwm import heading_pwm,obstacle_pwm


def clear_scan(now):
    return Scan(np.full(361,np.inf),-math.pi,math.pi/180,.2,30.,now)


def guidance(position,heading,goal,percent):
    error=math.atan2(goal[1],goal[0])-heading
    turn=max(-percent,min(percent,percent*error/.8))
    base=percent*max(0.,math.cos(error))
    left,right=base-turn,base+turn
    scale=max(1.,abs(left)/percent,abs(right)/percent)
    return left/scale,right/scale


class RouteTests(unittest.TestCase):
    def setUp(self):
        # Legacy visual mode still available; new geometry/braking are tested separately.
        self.route=RouteConfig(wp_latitudes=(35.,35.0002,35.0004),wp_longitudes=(128.,128.,128.),
                               dock_geometry_enabled=False,wp2_brake_seconds=0.)
        self.mission=RouteMission(FollowConfig(),guidance,GPSConfig(goal_configured=True),self.route)
        self.mission.executing=True;self.mission.neutral_confirmed=True

    def inputs(self,now,index=0):
        self.mission.fix=Fix(self.route.wp_latitudes[index],self.route.wp_longitudes[index],.4,now)
        self.mission.heading,self.mission.heading_stamp=math.pi/2,now
        return clear_scan(now)

    def reach_wp2(self):
        for now,index in ((100.,0),(101.,0),(102.,1),(103.,1)):
            self.mission.step(now,self.inputs(now,index))
        self.assertEqual(self.mission.stage,'WP2_HOLD')
        return 103.

    def finish_hold(self,start=103.):
        for i in range(103):
            now=start+i*.1
            self.mission.step(now,self.inputs(now,1))
        self.assertIn(self.mission.stage,('WP2_TO_WP3_ALIGN','DOCK_TO_WP3'))
        return now

    def test_visits_wp1_before_wp2(self):
        self.mission.step(100.,self.inputs(100.,1))
        self.assertEqual(self.mission.wp_index,0)
        self.assertEqual(self.mission.stage,'TO_WP1')

    def test_preview_does_not_advance_mission(self):
        self.mission.executing=False
        for now in (100.,101.,102.):self.mission.step(now,self.inputs(now))
        self.assertEqual(self.mission.wp_index,0)

    def test_ten_second_hold_with_neutral_feedback(self):
        now=self.reach_wp2()
        for i in range(100):
            now=103.+i*.1
            self.mission.step(now,self.inputs(now,1))
            self.assertEqual(self.mission.intended_pwm,(1500,1500))
        self.assertEqual(self.mission.stage,'WP2_HOLD')
        self.mission.step(113.,self.inputs(113.,1))
        self.assertEqual(self.mission.stage,'WP2_TO_WP3_ALIGN')

    def test_no_neutral_feedback_no_hold_countdown(self):
        self.reach_wp2();self.mission.neutral_confirmed=False
        for i in range(130):
            now=103.+i*.1;self.mission.step(now,self.inputs(now,1))
        self.assertEqual(self.mission.hold_remaining,10.)

    def test_operator_pause_does_not_count_hold_time(self):
        self.reach_wp2()
        for i in range(30):
            now=103.+i*.1;self.mission.step(now,self.inputs(now,1))
        remaining=self.mission.hold_remaining
        self.mission.executing=False
        for i in range(120):
            now=106.+i*.1;self.mission.step(now,self.inputs(now,1))
        self.assertAlmostEqual(self.mission.hold_remaining,remaining)

    def test_camera_ignored_before_hold_finishes(self):
        self.reach_wp2()
        for now in (103.1,103.2,103.3,103.4):
            self.mission.observe([FusedTarget(320.,240.,8,1,5.,0.,0.,5.,now)],now)
        self.assertIsNone(self.mission.target)

    def test_wrong_gps_region_cannot_complete_dock(self):
        self.reach_wp2();now=self.finish_hold()
        self.mission.step(now+.1,self.inputs(now+.1,1))
        self.assertEqual(self.mission.stage,'DOCK_TO_WP3')
        for i in range(20):
            now+=.1;self.mission.observe([FusedTarget(320.,240.,8,1,1.4,0.,0.,1.4,now)],now)
            state,_=self.mission.step(now,self.inputs(now,1))
        self.assertEqual(state,'WP3_POSITION_MISMATCH')
        self.assertFalse(self.mission.completed)

    def test_complete_route_requires_visual_docking_and_wp3(self):
        self.reach_wp2();now=self.finish_hold()
        self.mission.step(now+.1,self.inputs(now+.1,1))
        for i in range(20):
            now+=.1;self.mission.observe([FusedTarget(320.,240.,8,1,1.4,0.,0.,1.4,now)],now)
            state,_=self.mission.step(now,self.inputs(now,2))
        self.assertEqual(state,'MISSION_COMPLETE')
        self.assertTrue(self.mission.completed)
        self.assertEqual(self.mission.intended_pwm,(1500,1500))

    def test_gps_wp3_alone_cannot_finish(self):
        self.reach_wp2();now=self.finish_hold()
        for i in range(5):
            now+=.1;self.mission.step(now,self.inputs(now,2))
        self.assertFalse(self.mission.completed)

    def test_pwm_directions_match_right_positive_supplied_table(self):
        self.mission.fix=Fix(34.9998,128.,.4,100.)
        self.mission.heading,self.mission.heading_stamp=0.,100.
        self.mission.step(100.,clear_scan(100.))
        self.assertLess(*self.mission.intended_pwm)  # north is left of east

    def test_unknown_scan_blocks_route(self):
        value=clear_scan(100.);object.__setattr__(value,'ranges',np.full(361,np.nan))
        self.inputs(100.)
        state,_=self.mission.step(100.,value)
        self.assertEqual(state,'NO_SAFE_GAP')
        self.assertEqual(self.mission.intended_pwm,(1500,1500))


class AvoidanceTests(unittest.TestCase):
    def test_clear_scan_selects_goal_direction(self):
        angle,report=choose_corridor(clear_scan(100.),0.,RouteConfig(),.77)
        self.assertAlmostEqual(angle,0.)
        self.assertTrue(report['gaps'])

    def test_front_obstacle_selects_side_corridor(self):
        scan=clear_scan(100.)
        scan.ranges[174:187]=1.8
        angle,report=choose_corridor(scan,0.,RouteConfig(),.77)
        self.assertIsNotNone(angle)
        self.assertGreater(abs(math.degrees(angle)),20.)

    def test_no_fallback_through_narrow_or_blocked_gap(self):
        scan=clear_scan(100.);scan.ranges[:]=1.
        angle,report=choose_corridor(scan,0.,RouteConfig(),.77)
        self.assertIsNone(angle)

    def test_single_close_return_cannot_be_dropped_as_noise(self):
        scan=clear_scan(100.);scan.ranges[180]=.6
        self.assertIsNone(choose_corridor(scan,0.,RouteConfig(),.77)[0])


class PWMTests(unittest.TestCase):
    def test_forward_boost_preserves_stop_and_pivot(self):
        self.assertEqual(heading_pwm(10.,0.,forward_boost=50),(1650,1650))
        self.assertEqual(heading_pwm(1.,0.,forward_boost=50),(1620,1620))
        self.assertEqual(obstacle_pwm(3.,0.,forward_boost=50),(1730,1730))
        self.assertEqual(heading_pwm(0.,0.,holding=True,forward_boost=50),(1500,1500))
        for angle in (-100.,-20.,20.,100.):
            self.assertEqual(heading_pwm(0.,angle,holding=True,forward_boost=50),heading_pwm(0.,angle,holding=True))
        self.assertEqual(obstacle_pwm(3.,75.,forward_boost=50),(1900,1520))

    def test_mission_uses_forward_boost(self):
        route=RouteConfig()
        mission=RouteMission(FollowConfig(),guidance,GPSConfig(goal_configured=True),route)
        now=100.
        mission.fix=Fix(route.wp_latitudes[0]-.0001,route.wp_longitudes[0],.4,now)
        mission.heading=math.pi/2;mission.heading_stamp=now
        mission.step(now,clear_scan(now))
        self.assertEqual(mission.intended_pwm,(1650,1650))

    def test_reference_values(self):
        self.assertEqual(heading_pwm(10.,0.),(1600,1600))
        self.assertEqual(heading_pwm(1.,0.),(1570,1570))
        self.assertEqual(heading_pwm(10.,30.),(1740,1460))
        self.assertEqual(heading_pwm(10.,-100.),(1200,1850))
        self.assertEqual(heading_pwm(0.,-2.,True),(1475,1525))
        self.assertEqual(obstacle_pwm(3.,0.),(1680,1680))
        self.assertEqual(obstacle_pwm(3.,20.),(1830,1530))

    def test_invalid_commands_rejected(self):
        for function in (heading_pwm,obstacle_pwm):
            with self.assertRaises(ValueError):function(math.nan,0.)


if __name__=='__main__':unittest.main()
