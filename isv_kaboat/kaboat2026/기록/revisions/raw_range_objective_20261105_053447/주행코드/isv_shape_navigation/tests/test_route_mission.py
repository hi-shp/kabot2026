import math
import unittest
import numpy as np
from isv_shape_navigation.core import Scan
from isv_shape_navigation.follow import FollowConfig,FusedTarget
from isv_shape_navigation.gps_mission import GPSConfig,Fix
from isv_shape_navigation.route_mission import RouteConfig,RouteMission,choose_corridor,limit_course_scan
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


class WeightedAvoidanceTests(unittest.TestCase):
    def scene(self):
        scan=clear_scan(100.)
        scan.ranges[225]=2.9
        return scan

    def test_angle_only_retains_goal_direction(self):
        cfg=RouteConfig(avoidance_distance_weight=0.,avoidance_angle_weight=1.)
        angle,report=choose_corridor(self.scene(),0.,cfg,.77)
        self.assertAlmostEqual(angle,0.)
        self.assertAlmostEqual(report['objective']['selected_score'],0.)

    def test_inverse_distance_weight_prefers_larger_safe_spacing(self):
        scan=self.scene()
        _,baseline=choose_corridor(scan,0.,RouteConfig(avoidance_distance_weight=0.),.77)
        cfg=RouteConfig(avoidance_distance_weight=1.,avoidance_angle_weight=.00001)
        angle,report=choose_corridor(scan,0.,cfg,.77)
        objective=report['objective']
        self.assertGreater(objective['selected_width'],baseline['objective']['selected_width'])
        self.assertGreater(abs(angle),0.)
        self.assertAlmostEqual(objective['selected_score'],min(row['score'] for row in objective['candidates']))
        for row in objective['candidates']:
            self.assertAlmostEqual(row['score'],1./(row['width']+cfg.avoidance_distance_epsilon)+.00001*row['error_deg'])

    def test_no_weights_can_override_close_obstacle_or_unknown_scan(self):
        cfg=RouteConfig(avoidance_distance_weight=1000000.,avoidance_angle_weight=0.)
        scan=self.scene();scan.ranges[180]=.6
        self.assertEqual(choose_corridor(scan,0.,cfg,.77)[1]['reason'],'OBSTACLE_STOP')
        scan.ranges[:]=np.nan
        self.assertEqual(choose_corridor(scan,0.,cfg,.77)[1]['reason'],'NO_SAFE_GAP')

    def test_weights_are_finite_nonnegative_and_not_both_zero(self):
        for dw,aw in [(-1.,1.),(1.,-1.),(0.,0.),(math.inf,1.),(1.,math.nan),(True,1.)]:
            with self.subTest(dw=dw,aw=aw),self.assertRaises(ValueError):
                RouteConfig(avoidance_distance_weight=dw,avoidance_angle_weight=aw)

    def test_missing_sides_use_documented_range_cap(self):
        _,report=choose_corridor(clear_scan(100.),0.,RouteConfig(),.77)
        self.assertEqual(report['objective']['selected_width'],10.)

    def test_lidar_cutoff_includes_exact_five_metres(self):
        raw=Scan(np.array([.3,4.999,5.,5.01,np.inf,0.,np.nan]),0.,.1,.2,30.,100.)
        bounded=limit_course_scan(raw,5.)
        np.testing.assert_allclose(bounded.ranges[:6],[.3,4.999,0.,0.,0.,0.])
        self.assertTrue(np.isnan(bounded.ranges[6]))
        self.assertEqual(bounded.range_max,5.)
        self.assertEqual(raw.ranges[2],5.)
        self.assertTrue(np.isposinf(raw.ranges[4]))

    def test_far_only_scan_is_unknown_not_free(self):
        scan=limit_course_scan(clear_scan(100.),5.)
        angle,report=choose_corridor(scan,0.,RouteConfig(),.77)
        self.assertIsNone(angle)
        self.assertEqual(report['reason'],'NO_SAFE_GAP')

    def test_epsilon_and_lidar_limit_must_be_valid(self):
        for kwargs in [dict(avoidance_distance_epsilon=0.),dict(avoidance_distance_epsilon=-1.),
                       dict(avoidance_distance_epsilon=math.nan),dict(avoidance_lidar_range=.1)]:
            with self.subTest(kwargs=kwargs),self.assertRaises(ValueError):RouteConfig(**kwargs)

    def test_small_tank_stop_radius_boundary(self):
        scan=clear_scan(100.)
        scan.ranges[0]=.399
        self.assertEqual(choose_corridor(scan,0.,RouteConfig(),.4)[1]['reason'],'OBSTACLE_STOP')
        scan.ranges[0]=.401
        self.assertIsNotNone(choose_corridor(scan,0.,RouteConfig(),.4)[0])

    def test_min_range_excludes_only_returns_below_point_one(self):
        cfg=RouteConfig()
        raw=Scan(np.array([.099,.1,.11,.5]),0.,.1,cfg.avoidance_lidar_min_range,30.,100.)
        bounded=limit_course_scan(raw,5.)
        points=bounded.points()
        np.testing.assert_allclose(np.linalg.norm(points,axis=1),[.1,.11,.5])
        for kwargs in [dict(avoidance_lidar_min_range=0.),dict(avoidance_lidar_min_range=-1.),
                       dict(avoidance_lidar_min_range=5.)]:
            with self.subTest(kwargs=kwargs),self.assertRaises(ValueError):RouteConfig(**kwargs)

    def test_small_selected_angle_is_used_for_pwm(self):
        cfg=RouteConfig()
        mission=RouteMission(FollowConfig(),guidance,GPSConfig(goal_configured=True),cfg)
        mission.fix=Fix(cfg.wp_latitudes[0]-.0001,cfg.wp_longitudes[0],.4,100.)
        mission.heading=math.pi/2-math.radians(2.)
        mission.heading_stamp=100.
        mission.step(100.,clear_scan(100.))
        selected=mission.avoidance['selected_angle_deg']
        self.assertAlmostEqual(selected,3.)
        self.assertEqual(mission.intended_pwm,heading_pwm(mission.distance_to_wp,-selected,forward_boost=50))


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
