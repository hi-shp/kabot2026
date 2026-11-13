"""Offline GPS/vision stage-transition and output-direction tests."""
import math
import unittest
import numpy as np

from isv_shape_navigation.core import Pose, Scan
from isv_shape_navigation.follow import FollowConfig, FusedTarget
from isv_shape_navigation.gps_mission import Fix, GPSConfig, GPSDockMission, gps_vector
from isv_shape_navigation.planner import Grid


def scan(stamp):
    return Scan(np.full(361, 20.), -math.pi, math.pi/180, .2, 30., stamp)


def target(stamp, x=5., y=0.):
    return FusedTarget(320., 240., 8, 1, x, y, 0., math.hypot(x, y), stamp)


def guidance(position, heading, goal, percent):
    error = math.atan2(goal[1], goal[0])-heading
    base = percent*max(0., math.cos(error))
    turn = max(-percent, min(percent, percent*error/.8))
    left, right = base-turn, base+turn
    scale = max(1., abs(left)/percent, abs(right)/percent)
    return left/scale, right/scale


class GPSMissionTests(unittest.TestCase):
    def setUp(self):
        self.cfg = GPSConfig(goal_configured=True, goal_latitude=35., goal_longitude=128.)
        self.mission = GPSDockMission(FollowConfig(), guidance, self.cfg)

    def fix(self, now, lat=35., lon=128., sigma=.5):
        self.mission.fix = Fix(lat, lon, sigma, now)
        self.mission.heading, self.mission.heading_stamp = math.pi/2, now

    def arrive(self):
        for now in (100., 101., 102.):
            self.fix(now)
            self.mission.step(now, scan(now))
        self.assertEqual(self.mission.phase, 'SEARCH_SHAPE')

    def observe(self, start=102.1, x=5., y=0.):
        for index in range(4):
            now = start+index*.1
            self.mission.observe([target(now, x, y)], now)
        return now

    def test_goal_not_invented(self):
        self.cfg.goal_configured = False
        self.assertEqual(self.mission.step(100., scan(100.)), ('WAIT_GPS_GOAL', (0., 0.)))

    def test_early_camera_does_not_bypass_gps(self):
        self.observe(100.)
        self.assertIsNone(self.mission.target)
        self.assertEqual(self.mission.phase, 'GPS_TRANSIT')

    def test_east_north_projection(self):
        fix = Fix(35., 128., .5, 100.)
        east, north = gps_vector(fix, 35., 128.0001)
        self.assertGreater(east, 9.)
        self.assertLess(abs(north), .01)
        east, north = gps_vector(fix, 35.0001, 128.)
        self.assertAlmostEqual(east, 0.)
        self.assertGreater(north, 11.)

    def test_north_goal_forward_and_east_goal_right_turn(self):
        self.fix(100., lat=34.9999)
        state, wheels = self.mission.step(100., scan(100.))
        self.assertEqual(state, 'GPS_TRANSIT')
        self.assertGreater(wheels[0], 0.)
        self.assertAlmostEqual(*wheels)
        self.fix(100.1, lon=127.9999)
        state, wheels = self.mission.step(100.1, scan(100.1))
        self.assertEqual(state, 'ALIGN_GPS')
        self.assertGreater(wheels[0], 0.)
        self.assertLess(wheels[1], 0.)

    def test_duplicate_fix_does_not_complete_arrival(self):
        self.fix(100.)
        for now in (100., 100.5, 101., 101.5):
            self.mission.heading_stamp = now
            self.mission.step(now, scan(now))
        self.assertEqual(self.mission.phase, 'GPS_TRANSIT')
        self.arrive()

    def test_gps_uncertainty_and_missing_heading_stop(self):
        self.fix(100., sigma=4.)
        self.assertEqual(self.mission.step(100., scan(100.))[0], 'GPS_UNCERTAIN')
        self.fix(101., lat=34.9999)
        self.mission.heading_stamp = 100.
        self.assertEqual(self.mission.step(101., scan(101.))[0], 'WAIT_HEADING')

    def test_far_goal_and_stale_gps_stop(self):
        self.fix(100., lat=34.)
        self.assertEqual(self.mission.step(100., scan(100.))[0], 'GPS_GOAL_TOO_FAR')
        self.assertEqual(self.mission.step(103., scan(103.))[0], 'WAIT_GPS')

    def test_search_turns_without_forward_motion(self):
        self.arrive()
        state, wheels = self.mission.step(102.1, scan(102.1))
        self.assertEqual(state, 'SEARCH_SHAPE')
        self.assertAlmostEqual(sum(wheels), 0.)
        self.mission.heading += math.radians(65.)
        self.mission.heading_stamp = 102.2
        _, changed = self.mission.step(102.2, scan(102.2))
        self.assertGreater(changed[0], 0.)

    def test_search_timeout_stops(self):
        self.arrive()
        self.assertEqual(self.mission.step(163., scan(163.)), ('SEARCH_TIMEOUT', (0., 0.)))

    def test_old_camera_frame_after_arrival_is_ignored(self):
        self.arrive()
        self.mission.observe([target(101.9)], 102.1)
        self.assertIsNone(self.mission.target)

    def test_gps_vision_entry_and_completion(self):
        self.arrive()
        now = self.observe()
        self.assertEqual(self.mission.step(now, scan(now))[0], 'APPROACH_SHAPE')
        # Continuous observations emulate approach, with no teleport to a new target.
        for index in range(1, 9):
            now += .1
            self.mission.observe([target(now, 5.-index*.45)], now)
            state, wheels = self.mission.step(now, scan(now))
        self.assertEqual(state, 'REGION_ARRIVAL_HOLD')
        self.assertEqual(wheels, (0., 0.))
        for _ in range(12):
            now += .1
            self.mission.observe([target(now, 1.4)], now)
            state, _ = self.mission.step(now, scan(now))
        self.assertEqual(state, 'REGION_REACHED')
        self.assertEqual(self.mission.step(now+10, None), ('REGION_REACHED', (0., 0.)))

    def test_align_before_visual_forward_motion(self):
        self.arrive()
        now = self.observe(x=5., y=2.)
        state, wheels = self.mission.step(now, scan(now))
        self.assertEqual(state, 'ALIGN_SHAPE')
        self.assertAlmostEqual(sum(wheels), 0.)

    def test_visual_loss_does_not_resume_gps_navigation(self):
        self.arrive()
        now = self.observe()
        self.mission.step(now, scan(now))
        self.assertEqual(self.mission.step(now+1., scan(now+1.))[0], 'TARGET_LOST')
        self.assertNotEqual(self.mission.phase, 'GPS_TRANSIT')

    def test_optional_slam_blocks_unknown_corridor(self):
        self.arrive()
        now = self.observe()
        self.cfg.use_slam = True
        self.assertEqual(self.mission.step(now, scan(now))[0], 'WAIT_SLAM_POSE')
        self.mission.pose = Pose(2., 2., 0., now)
        cells = np.full((100,100), -1, dtype=np.int8)
        self.mission.map = Grid(cells, .1, (0., 0., 0.), .77)
        self.mission.map_received = now
        self.assertEqual(self.mission.step(now, scan(now))[0], 'SLAM_CORRIDOR_BLOCKED')

    def test_configuration_checks_hull_clearance(self):
        with self.assertRaises(ValueError):
            GPSDockMission(FollowConfig(), guidance, GPSConfig(region_stop_distance=.5))


if __name__ == '__main__':
    unittest.main()
