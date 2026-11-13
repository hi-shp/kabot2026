"""Behavioral tests for class selection, stale inputs, geometry and native output."""
import math
import unittest
import numpy as np

from isv_shape_navigation.core import Scan
from isv_shape_navigation.follow import (FollowConfig, Follower, FusedTarget,
                                         MODEL_CLASSES, fusion_rows, native_pwm)


def scan(stamp, ranges=None, reverse=False):
    return Scan(np.full(361, 10.) if ranges is None else np.asarray(ranges),
                math.pi if reverse else -math.pi,
                -math.pi/180 if reverse else math.pi/180, .2, 30., stamp)


def observation(stamp, x=5., y=0., cid=8, track=1):
    return FusedTarget(320., 240., cid, track, x, y, 0., math.hypot(x, y), stamp)


class FollowTests(unittest.TestCase):
    def setUp(self):
        self.calls = []
        def native_guidance(position, heading, target, percent):
            self.calls.append((position, heading, target, percent))
            return (percent/2, percent) if target[1] > 0 else (percent, percent/2) if target[1] < 0 else (percent, percent)
        self.cfg = FollowConfig()
        self.follower = Follower(self.cfg, native_guidance)

    def stabilize(self, start=100., x=5., y=0., cid=8, track=1):
        for index in range(4):
            stamp = start+index*.1
            self.follower.observe([observation(stamp, x, y, cid, track)], stamp)
        return start+.3

    def test_all_nine_classes_are_selected_by_name(self):
        for index, label in enumerate(MODEL_CLASSES):
            self.assertEqual(FollowConfig(target_label=label).target_class_id, index)
        self.assertEqual(FollowConfig(target_label='RED_TRIANGLE').target_class_id, 8)

    def test_new_model_order_is_configurable(self):
        cfg = FollowConfig(class_names=('buoy', 'red triangle'), target_label='red triangle')
        self.assertEqual(cfg.target_class_id, 1)

    def test_unknown_shape_is_rejected(self):
        with self.assertRaises(ValueError):
            FollowConfig(target_label='red square')

    def test_invalid_output_or_standoff_is_rejected(self):
        for kwargs in ({'max_percent': 100.}, {'target_standoff': .5}, {'stable_observations': 1}):
            with self.assertRaises(ValueError): FollowConfig(**kwargs)

    def test_requires_distinct_stable_frames(self):
        self.follower.observe([observation(100.)], 100.)
        for _ in range(20): self.follower.observe([observation(100.)], 100.)
        self.assertEqual(self.follower.count, 1)
        self.assertEqual(self.follower.step(100., scan(100.))[0], 'VERIFY_TARGET')

    def test_uses_installed_guidance_contract_and_ccw_geometry(self):
        now = self.stabilize(y=1.)
        state, wheels = self.follower.step(now, scan(now))
        self.assertEqual(state, 'FOLLOW_TARGET')
        self.assertLess(wheels[0], wheels[1])
        self.assertEqual(self.calls[-1], ((0., 0.), 0., (5., 1.), 16.))

    def test_right_target_requires_stronger_left_thruster(self):
        now = self.stabilize(y=-1.)
        self.assertGreater(*self.follower.step(now, scan(now))[1])

    def test_other_classes_cannot_become_target(self):
        now = self.stabilize(cid=6)
        self.assertEqual(self.follower.step(now, scan(now)), ('WAIT_TARGET', (0., 0.)))

    def test_multiple_same_class_targets_stop(self):
        now = self.stabilize()
        self.follower.observe([observation(now+.1), observation(now+.1, y=1., track=2)], now+.1)
        self.assertEqual(self.follower.step(now+.1, scan(now+.1)), ('AMBIGUOUS_TARGET', (0., 0.)))

    def test_track_switch_requires_reverification(self):
        now = self.stabilize()
        self.follower.observe([observation(now+.1, track=2)], now+.1)
        self.assertEqual(self.follower.count, 1)
        self.assertEqual(self.follower.step(now+.1, scan(now+.1))[0], 'VERIFY_TARGET')

    def test_hold_has_distance_hysteresis(self):
        now = self.stabilize(x=1.4)
        self.assertEqual(self.follower.step(now, scan(now)), ('HOLD_DISTANCE', (0., 0.)))
        self.follower.observe([observation(now+.1, x=1.6)], now+.1)
        self.assertEqual(self.follower.step(now+.1, scan(now+.1))[0], 'HOLD_DISTANCE')
        self.follower.observe([observation(now+.2, x=1.8)], now+.2)
        self.assertEqual(self.follower.step(now+.2, scan(now+.2))[0], 'FOLLOW_TARGET')

    def test_target_timeout_stops_without_search_motion(self):
        now = self.stabilize()
        self.assertEqual(self.follower.step(now+1, scan(now+1)), ('TARGET_LOST', (0., 0.)))

    def test_scan_timeout_stops(self):
        now = self.stabilize()
        self.assertEqual(self.follower.step(now, scan(now-1)), ('WAIT_SCAN', (0., 0.)))

    def test_unsynchronized_fusion_stops(self):
        now = self.stabilize()
        self.assertEqual(self.follower.step(now, scan(now-.3)), ('WAIT_SYNC', (0., 0.)))

    def test_unobserved_scan_stops(self):
        now = self.stabilize()
        self.assertEqual(self.follower.step(now, scan(now, np.full(361, np.nan))), ('SCAN_UNOBSERVED', (0., 0.)))

    def test_obstacle_corridor_stops(self):
        now = self.stabilize()
        values = np.full(361, 10.); values[175:186] = 1.
        self.assertEqual(self.follower.step(now, scan(now, values)), ('OBSTACLE_STOP', (0., 0.)))

    def test_clockwise_scan_is_supported(self):
        now = self.stabilize()
        self.assertEqual(self.follower.step(now, scan(now, reverse=True))[0], 'FOLLOW_TARGET')

    def test_transform_is_applied_before_native_guidance(self):
        self.cfg.camera_yaw_deg = 90
        now = self.stabilize()
        self.follower.step(now, scan(now))
        self.assertAlmostEqual(self.calls[-1][2][0], 0., places=8)
        self.assertAlmostEqual(self.calls[-1][2][1], 5., places=8)

    def test_invalid_native_guidance_cannot_reach_actuators(self):
        now = self.stabilize()
        self.follower.guidance = lambda *_args, **_kwargs: (math.nan, 100.)
        self.assertEqual(self.follower.step(now, scan(now)), ('GUIDANCE_INVALID', (0., 0.)))


class FusionTests(unittest.TestCase):
    def test_real_eight_column_layout_and_offset(self):
        rows = fusion_rows([123., 320., 240., 8., -1., 3., 4., 0., 5.], 100., rows=1, offset=1)
        self.assertEqual(rows[0].class_id, 8)
        self.assertEqual(rows[0].track_id, -1)

    def test_malformed_layout_is_rejected(self):
        for kwargs in ({'rows': 2}, {'cols': 7}, {'offset': -1}):
            with self.assertRaises(ValueError):
                fusion_rows([320., 240., 8., 1., 5., 0., 0., 5.], 100., **kwargs)

    def test_nonfinite_or_inconsistent_geometry_is_rejected(self):
        for row in ([320., 240., 8., 1., math.nan, 0., 0., 5.],
                    [320., 240., 8.5, 1., 5., 0., 0., 5.],
                    [320., 240., 8., 1., 5., 0., 0., 1.]):
            self.assertEqual(fusion_rows(row, 100.), [])

    def test_pwm_is_delegated_to_native_calibration_per_side(self):
        calls = []
        def converter(percent, side):
            calls.append((percent, side))
            return (1560, 1500) if side == 'left' else (1500, 1460)
        self.assertEqual(native_pwm((15., -10.), converter), (1560, 1460))
        self.assertEqual(calls, [(15., 'left'), (-10., 'right')])


if __name__ == '__main__':
    unittest.main()
