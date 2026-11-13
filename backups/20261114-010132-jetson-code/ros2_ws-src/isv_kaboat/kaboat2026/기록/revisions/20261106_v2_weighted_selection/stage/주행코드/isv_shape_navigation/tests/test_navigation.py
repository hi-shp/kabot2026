import math
import tempfile
from pathlib import Path
from types import SimpleNamespace
import unittest
import cv2
import numpy as np

from isv_shape_navigation.core import (Detection, Mission, MissionConfig, Pose, Scan,
                                       fuse_detection, thrust_to_pwm)
from isv_shape_navigation.planner import Grid
from isv_shape_navigation.perception import Camera, detect_shapes
from isv_shape_navigation.output import PairedOutput, OutputReadiness, telemetry_ready
from isv_shape_navigation.node import quaternion_yaw
from isv_shape_navigation.demo import run, scene


class FusionTests(unittest.TestCase):
    def setUp(self):
        self.pose = Pose(2, 5, 0, 1)
        self.image, self.scan = scene(self.pose, 1)
        self.detection = [d for d in detect_shapes(self.image, 1) if d.label == 'triangle'][0]

    def test_camera_scan_world_position(self):
        point = fuse_detection(self.detection, self.scan, self.pose, 500, 320)
        np.testing.assert_allclose(point, [8, 5], atol=.06)

    def test_unsynchronized_scan_rejected(self):
        pose = Pose(2, 5, 0, 3)
        self.assertIsNone(fuse_detection(self.detection, self.scan, pose, 500, 320))

    def test_missing_calibration_rejected(self):
        self.assertIsNone(fuse_detection(self.detection, self.scan, self.pose, 0, 320))

    def test_infinity_is_not_target_depth(self):
        scan = Scan(np.full(100, math.inf), -1, .02, .2, 25, 1)
        self.assertIsNone(fuse_detection(self.detection, scan, self.pose, 500, 320))

    def test_clockwise_scan(self):
        scan = self.scan
        reverse = Scan(scan.ranges[::-1], scan.angle_min+(len(scan.ranges)-1)*scan.angle_increment,
                       -scan.angle_increment, scan.range_min, scan.range_max, scan.stamp)
        np.testing.assert_allclose(fuse_detection(self.detection, reverse, self.pose, 500, 320), [8, 5], atol=.06)

    def test_mixed_ranges_rejected(self):
        ranges = self.scan.ranges.copy()
        ranges[::2] /= 2
        scan = Scan(ranges, self.scan.angle_min, self.scan.angle_increment, .2, 25, 1)
        self.assertIsNone(fuse_detection(self.detection, scan, self.pose, 500, 320))

    def test_sensor_translation_and_rotation(self):
        # A flat wall at reference x=5; scanner translated 0.3m forward and yawed.
        angles = np.linspace(-.7, .7, 301)
        scanner_yaw = .15
        ranges = 4.7/np.cos(angles+scanner_yaw)
        scan = Scan(ranges, angles[0], angles[1]-angles[0], .2, 25, 1)
        camera_x, camera_y, camera_yaw = .5, .2, .1
        bearing = math.atan2(-camera_y, 5-camera_x)-camera_yaw
        u = 320-500*math.tan(bearing)
        d = Detection('triangle', .9, u, 240, 80, 80, 1)
        point = fuse_detection(d, scan, Pose(0, 0, 0, 1), 500, 320,
                               camera_x=camera_x, camera_y=camera_y, camera_yaw=camera_yaw,
                               scan_x=.3, scan_yaw=scanner_yaw)
        np.testing.assert_allclose(point, [5, 0], atol=.06)


class PlannerTests(unittest.TestCase):
    def test_unknown_and_exterior_blocked(self):
        cells = np.zeros((40, 40), dtype=np.int8); cells[20, 20] = -1
        grid = Grid(cells, .1, radius=.2)
        self.assertFalse(grid.valid((20, 20)))
        self.assertFalse(grid.valid((0, 0)))
        self.assertEqual(grid.plan([-1, 1], [2, 2]), [])

    def test_route_around_obstacle(self):
        cells = np.zeros((60, 60), dtype=np.int8); cells[20:45, 28:32] = 100
        grid = Grid(cells, .1, radius=.3)
        path = grid.plan([1, 3], [5, 3])
        self.assertGreater(len(path), 2)
        self.assertTrue(all(grid.segment_free(a, b) for a, b in zip(path, path[1:])))

    def test_narrow_gap_rejected_for_hull(self):
        cells = np.zeros((80, 80), dtype=np.int8)
        cells[:, 39:41] = 100; cells[34:46, 39:41] = 0  # 1.2m opening < 1.54m diameter
        self.assertEqual(Grid(cells, .1, radius=.77).plan([2, 4], [6, 4]), [])

    def test_rotated_map_origin(self):
        grid = Grid(np.zeros((40, 40)), .1, origin=(10, 20, math.pi/2), radius=.1)
        point = grid.world((12, 17))
        self.assertEqual(grid.cell(point), (12, 17))

    def test_no_diagonal_corner_cut(self):
        cells = np.full((9, 9), 100)
        cells[3, 3] = cells[4, 4] = 0
        grid = Grid(cells, 1, radius=.05)
        self.assertFalse(grid.segment_free([3.5, 3.5], [4.5, 4.5]))
        self.assertEqual(grid.plan([3.5, 3.5], [4.5, 4.5]), [])

    def test_goal_not_snapped_through_wall(self):
        cells = np.zeros((40, 40)); cells[10:20, 10:20] = 100
        self.assertEqual(Grid(cells, .1, radius=.1).plan([3, 3], [1.5, 1.5]), [])


class MissionTests(unittest.TestCase):
    def setUp(self):
        self.mission = Mission()
        self.grid = Grid(np.zeros((120, 120)), .1)
        self.pose = Pose(2, 5, 0, 1)
        _, self.scan = scene(self.pose, 1)
        for t in (.7, .8, .9, 1.):
            self.mission.observe([8, 5], self.pose, t)

    def test_follow_known_marker(self):
        state, command, path = self.mission.step(1, self.pose, self.scan, self.grid)
        self.assertEqual(state, 'APPROACH_SHAPE')
        self.assertGreater(command[0], 0)
        self.assertTrue(path)

    def test_stale_pose_stops(self):
        state, command, _ = self.mission.step(2, self.pose, self.scan, self.grid)
        self.assertEqual(state, 'WAIT_POSE'); self.assertEqual(command, (0, 0))

    def test_stale_scan_stops(self):
        state, command, _ = self.mission.step(2, Pose(2, 5, 0, 2), self.scan, self.grid)
        self.assertEqual(state, 'WAIT_SCAN'); self.assertEqual(command, (0, 0))

    def test_localization_not_ready(self):
        state, command, _ = self.mission.step(1, self.pose, self.scan, self.grid, localization_ready=False)
        self.assertEqual(state, 'WAIT_LOCALIZATION'); self.assertEqual(command, (0, 0))

    def test_missing_map_stops(self):
        self.assertEqual(self.mission.step(1, self.pose, self.scan, None)[0], 'WAIT_MAP')

    def test_close_obstacle_stops(self):
        scan = Scan(np.array([.5, .5, .5]), -.02, .02, .2, 20, 1)
        state, command, _ = self.mission.step(1, self.pose, scan, self.grid)
        self.assertEqual(state, 'OBSTACLE_STOP'); self.assertEqual(command, (0, 0))

    def test_empty_scan_stops(self):
        scan = Scan(np.full(10, math.inf), -.5, .1, .2, 20, 1)
        self.assertEqual(self.mission.step(1, self.pose, scan, self.grid)[0], 'SCAN_UNOBSERVED')

    def test_duplicate_frame_does_not_confirm(self):
        mission = Mission()
        for _ in range(10):
            mission.observe([8, 5], self.pose, 1)
        self.assertEqual(mission.count, 1)

    def test_lock_does_not_jump_to_next_bay(self):
        self.assertFalse(self.mission.observe([8, 8], self.pose, 1.1))
        np.testing.assert_allclose(self.mission.target, [8, 5])

    def test_close_acquisition_does_not_back_away_to_approach(self):
        mission = Mission()
        pose = Pose(6, 5, 0, 1)
        for stamp in (.7, .8, .9, 1.):
            mission.observe([8, 5], pose, stamp)
        self.assertEqual(mission.phase, 'ENTER_REGION')
        self.assertGreater(mission.goal()[0], pose.x)

    def test_already_inside_standoff_does_not_acquire_behind_boat(self):
        mission = Mission()
        self.assertFalse(mission.observe([3, 5], self.pose, 1))
        self.assertIsNone(mission.target)

    def test_temporarily_occluded_target_retains_slam_goal(self):
        pose = Pose(2, 5, 0, 5)
        _, scan = scene(pose, 5)
        self.assertEqual(self.mission.step(5, pose, scan, self.grid)[0], 'APPROACH_SHAPE')

    def test_target_memory_expires(self):
        pose = Pose(2, 5, 0, 40)
        _, scan = scene(pose, 40)
        self.assertEqual(self.mission.step(40, pose, scan, self.grid)[0], 'TARGET_LOST')

    def test_two_stage_arrival_requires_dwell(self):
        for stamp, x, expected in [(1, 5.5, 'APPROACH_SHAPE'), (2.1, 5.5, 'ENTER_REGION'),
                                    (2.2, 6.8, 'ENTER_REGION'), (3.3, 6.8, 'REGION_REACHED')]:
            pose = Pose(x, 5, 0, stamp); _, scan = scene(pose, stamp)
            state, command, _ = self.mission.step(stamp, pose, scan, self.grid)
            self.assertEqual(state, expected); self.assertEqual(command, (0, 0))
        self.assertTrue(self.mission.completed)

    def test_invalid_hull_clearance_config(self):
        with self.assertRaises(ValueError):
            MissionConfig(entry_standoff=.8)

    def test_ccw_motor_mixing_and_limit(self):
        left, right = thrust_to_pwm(.16, .12, .25)
        self.assertGreater(right, left)
        self.assertLessEqual(max(left, right), 1600)
        self.assertEqual(thrust_to_pwm(math.nan, 0), (1500, 1500))


class PerceptionTests(unittest.TestCase):
    def test_synthetic_shapes_without_weights(self):
        triangle = np.array([[100, 50], [50, 150], [150, 150]])
        cross = np.array([[90, 40], [120, 40], [120, 80], [160, 80], [160, 110], [120, 110],
                          [120, 150], [90, 150], [90, 110], [50, 110], [50, 80], [90, 80]])
        for label in ('triangle', 'circle', 'square', 'rectangle', 'cross'):
            with self.subTest(label=label):
                frame = np.zeros((240, 320, 3), dtype=np.uint8)
                if label == 'triangle':
                    cv2.fillPoly(frame, [triangle], (0, 0, 255))
                elif label == 'circle':
                    cv2.circle(frame, (120, 120), 50, (0, 0, 255), -1)
                elif label == 'cross':
                    cv2.fillPoly(frame, [cross], (0, 0, 255))
                else:
                    cv2.rectangle(frame, (50, 50), (150 if label == 'square' else 210, 150), (0, 0, 255), -1)
                self.assertEqual([d.label for d in detect_shapes(frame, 1)], [label])

    def test_rotated_square_is_square(self):
        frame = np.zeros((240, 320, 3), dtype=np.uint8)
        cv2.fillPoly(frame, [np.array([[120, 50], [190, 120], [120, 190], [50, 120]])], (0, 0, 255))
        self.assertEqual(detect_shapes(frame, 1)[0].label, 'square')

    def test_different_color_ignored(self):
        frame = np.zeros((240, 320, 3), dtype=np.uint8)
        cv2.circle(frame, (120, 120), 50, (255, 0, 0), -1)
        self.assertEqual(detect_shapes(frame, 1, 'red'), [])

    def test_calibration_required_and_size_checked(self):
        camera = Camera()
        with self.assertRaises(ValueError):
            camera.rectify(np.zeros((480, 640, 3), dtype=np.uint8))
        camera.configure(640, 480, [500, 0, 320, 0, 500, 240, 0, 0, 1], [0]*5)
        self.assertTrue(camera.valid(640, 480))
        self.assertFalse(camera.valid(320, 240))

    def test_bad_quaternion_rejected(self):
        with self.assertRaises(ValueError):
            quaternion_yaw(SimpleNamespace(x=0, y=0, z=0, w=0))


class Future:
    def __init__(self, result=None, done=True):
        self.value, self.finished, self.canceled = result, done, False
    def done(self): return self.finished
    def cancelled(self): return self.canceled
    def exception(self): return None
    def result(self): return self.value
    def cancel(self): self.canceled = self.finished = True


class OutputTests(unittest.TestCase):
    def setUp(self):
        self.calls = []
        def arm(value):
            self.calls.append(('arm', value)); return Future(SimpleNamespace(success=True))
        def pair(l, r):
            self.calls.append(('pair', l, r))
            return [Future(SimpleNamespace(status=True)), Future(SimpleNamespace(status=True))]
        def stop():
            self.calls.append(('stop',)); return Future(SimpleNamespace(success=True))
        self.output = PairedOutput(arm, pair, stop, lambda: True)
        self.telemetry = dict(receiving=True, mode=2, estop_active=False, armed=True)

    def test_mcu_startup_settles_before_arm_can_be_requested(self):
        gate = OutputReadiness()
        self.assertIn('초기화', gate.check(0, self.telemetry, 0, True))
        self.assertIn('초기화', gate.check(7, self.telemetry, 0, True))
        self.assertEqual(gate.check(7.5, self.telemetry, 0, True), '')
        self.assertIn('서비스', gate.check(8, self.telemetry, 0, False))
        self.assertIn('AUTO', gate.check(8, dict(self.telemetry, mode=1), 0, True))
        self.assertIn('비상정지', gate.check(8, dict(self.telemetry, estop_active=True), 0, True))

    def test_usb_loss_restarts_initialization_wait(self):
        gate = OutputReadiness()
        gate.check(0, self.telemetry, 0, True)
        self.assertEqual(gate.check(8, self.telemetry, 0, True), '')
        self.assertIn('통신', gate.check(9, self.telemetry, 1, True))
        self.assertIn('초기화', gate.check(10, self.telemetry, 0, True))
        self.assertIn('초기화', gate.check(17, self.telemetry, 0, True))
        self.assertEqual(gate.check(17.5, self.telemetry, 0, True), '')

    def test_receiver_reconnect_restarts_wait_without_losing_usb(self):
        gate = OutputReadiness()
        gate.observe(0,dict(self.telemetry,rc_valid=False),0)
        self.assertEqual(gate.check(8,dict(self.telemetry,rc_valid=False),0,True),'')
        gate.observe(9,dict(self.telemetry,rc_valid=True),0)
        self.assertIn('초기화',gate.check(9,dict(self.telemetry,rc_valid=True),0,True))
        self.assertIn('초기화',gate.check(16,dict(self.telemetry,rc_valid=True),0,True))
        self.assertEqual(gate.check(16.5,dict(self.telemetry,rc_valid=True),0,True),'')

    def test_late_completed_ack_is_not_treated_as_current_output(self):
        self.output.start()
        self.output.tick(0,(1560,1560),self.telemetry,0)
        self.output.tick(.1,(1560,1560),self.telemetry,0)
        self.output.tick(.41,(1560,1560),self.telemetry,0)
        self.assertFalse(self.output.active)
        self.assertEqual(self.output.error,'service timeout')
        self.assertIsNone(self.output.last_ack_pwm)

    def test_acknowledged_pwm_is_recorded_only_after_both_mcu_responses(self):
        self.output.start()
        self.output.tick(0, (1560, 1570), self.telemetry, 0)
        self.output.tick(.1, (1560, 1570), self.telemetry, 0)
        self.assertIsNone(self.output.last_ack_pwm)
        self.output.tick(.2, (1580, 1590), self.telemetry, 0)
        self.assertEqual(self.output.last_ack_pwm, (1560, 1570))
        self.assertEqual(self.output.last_ack_at, .2)

    def test_preview_does_not_arm_or_stop(self):
        for t in range(10):
            self.output.tick(t, (1560, 1560), self.telemetry, 0)
        self.assertEqual(self.calls, [])

    def test_arm_then_submit_both_motors(self):
        self.output.start()
        self.output.tick(0, (1560, 1560), self.telemetry, 0)
        self.assertEqual(self.calls, [('arm', True)])
        self.output.tick(.1, (1560, 1560), self.telemetry, 0)
        self.assertEqual(self.calls[-1], ('pair', 1560, 1560))

    def test_rc_selection_stops_without_rearm(self):
        self.output.start(); self.output.tick(0, (1560, 1560), self.telemetry, 0)
        rc = dict(self.telemetry, mode=1)
        self.output.tick(.1, (1560, 1560), rc, 0)
        self.output.tick(.2, (1560, 1560), self.telemetry, 0)
        self.output.tick(.3, (1560, 1560), self.telemetry, 0)
        self.assertFalse(self.output.active)
        self.assertTrue(self.output.stop_ack)
        self.assertEqual(len([c for c in self.calls if c[0] == 'arm']), 1)

    def test_partial_pair_rejection_stops(self):
        self.output.pair = lambda *_: [Future(SimpleNamespace(status=True)), Future(SimpleNamespace(status=False))]
        self.output.start(); self.output.tick(0, (1560, 1560), self.telemetry, 0)
        self.output.tick(.1, (1560, 1560), self.telemetry, 0)
        self.output.tick(.2, (1560, 1560), self.telemetry, 0)
        self.output.tick(.3, (1560, 1560), self.telemetry, 0)
        self.assertFalse(self.output.active); self.assertTrue(self.output.stop_ack)

    def test_timeout_does_not_queue_more_pwm(self):
        self.output.pair = lambda *_: [Future(done=False), Future(done=False)]
        self.output.start(); self.output.tick(0, (1560, 1560), self.telemetry, 0)
        self.output.tick(.1, (1560, 1560), self.telemetry, 0)
        for t in (.2, .3, .8, .9):
            self.output.tick(t, (1580, 1580), self.telemetry, 0)
        self.assertFalse(self.output.active)
        self.assertTrue(self.output.stop_ack)

    def test_estop_or_stale_telemetry_blocks_arm(self):
        self.assertFalse(telemetry_ready(dict(self.telemetry, estop_active=True)))
        self.output.start(); self.output.tick(0, (1560, 1560), self.telemetry, 1)
        self.assertEqual(self.calls, []); self.assertFalse(self.output.active)

    def test_stop_works_when_pwm_service_disappears(self):
        self.output.start(); self.output.tick(0, (1560, 1560), self.telemetry, 0)
        self.output.tick(.1, (1560, 1560), self.telemetry, 0)
        self.output.stop_ready = lambda: True
        self.output.ready = lambda: False
        self.output.request_stop()
        self.output.tick(.2, (1500, 1500), self.telemetry, 0)
        self.output.tick(.3, (1500, 1500), self.telemetry, 0)
        self.assertTrue(self.output.stop_ack)


class IntegrationTests(unittest.TestCase):
    def test_controller_completes_detour_inside_inflated_map(self):
        cells = np.zeros((100, 100), dtype=np.int8)
        cells[40:60, 35:45] = 100
        grid = Grid(cells, .1)
        mission = Mission()
        x, y, angle = 2., 5., 0.
        states = set()
        for tick in range(600):
            now = 1+tick*.1
            pose = Pose(x, y, angle, now)
            # Isolate controller behaviour with repeated fixed-marker observations.
            mission.observe([8, 5], pose, now)
            _, scan = scene(pose, now)
            state, (forward, turn), _ = mission.step(now, pose, scan, grid)
            states.add(state)
            self.assertTrue(grid.valid(grid.cell([x, y])))
            if mission.completed:
                break
            angle = math.atan2(math.sin(angle+turn*.25), math.cos(angle+turn*.25))
            x += forward*.2*math.cos(angle)
            y += forward*.2*math.sin(angle)
        self.assertTrue(mission.completed)
        self.assertIn('ALIGN_PATH', states)

    def test_complete_camera_scan_map_pipeline(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parent) as directory:
            self.assertTrue(Path(directory).resolve().is_relative_to(Path(__file__).resolve().parent))
            summary = run(directory)
            self.assertTrue(summary['reached'])
            self.assertEqual(summary['states'], ['SEARCH_SHAPE', 'APPROACH_SHAPE', 'ENTER_REGION', 'REGION_REACHED'])
            self.assertTrue((Path(directory)/'trajectory.csv').exists())


if __name__ == '__main__':
    unittest.main()
