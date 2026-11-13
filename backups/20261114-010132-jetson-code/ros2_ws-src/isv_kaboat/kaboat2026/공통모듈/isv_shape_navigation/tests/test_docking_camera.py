"""Offline geometry/identity tests; real model and ROS tests are separate."""
import unittest
from kaboat_docking.core import Calibration, Detection, MODEL_CLASSES, TargetSelector, bearing_deg, validate_model_names


class DockingCameraTests(unittest.TestCase):
    def marker(self, tid=3, cid=8, x=320, score=.9, size=40):
        return Detection(cid, score, tid, (x-size/2, 220., x+size/2, 260.))

    def test_class_mapping_rejects_reordering(self):
        validate_model_names(dict(enumerate(MODEL_CLASSES)))
        wrong = list(MODEL_CLASSES)
        wrong[0], wrong[8] = wrong[8], wrong[0]
        with self.assertRaises(ValueError):
            validate_model_names(wrong)

    def test_initial_confidence_only_selected_class(self):
        selector = TargetSelector(8)
        target = selector.update([self.marker(cid=0, score=.99), self.marker(tid=2, score=.8),
                                  self.marker(tid=3, score=.9)], 0., 640, 480)
        self.assertEqual(target.track_id, 3)

    def test_identity_not_replaced_by_higher_confidence(self):
        selector = TargetSelector()
        selector.update([self.marker()], 0., 640, 480)
        target = selector.update([self.marker(score=.7), self.marker(tid=4, x=330, score=.99)], .1, 640, 480)
        self.assertEqual(target.track_id, 3)

    def test_temporary_loss_and_gated_changed_id(self):
        selector = TargetSelector()
        selector.update([self.marker()], 0., 640, 480)
        self.assertIsNone(selector.update([], .1, 640, 480))
        self.assertEqual(selector.status(.1), 'TRACKING')
        self.assertEqual(selector.status(1.), 'LOST')
        self.assertIsNone(selector.update([self.marker(tid=4, x=600)], 1., 640, 480))
        self.assertEqual(selector.update([self.marker(tid=5, x=330)], 1.1, 640, 480).track_id, 5)

    def test_ambiguous_reacquisition_is_rejected(self):
        selector = TargetSelector()
        selector.update([self.marker()], 0., 640, 480)
        self.assertIsNone(selector.update([self.marker(tid=4), self.marker(tid=5)], .2, 640, 480))

    def test_expired_reacquisition_never_automatically_switches(self):
        selector = TargetSelector()
        selector.update([self.marker()], 0., 640, 480)
        self.assertIsNone(selector.update([self.marker()], 4., 640, 480))
        self.assertEqual(selector.status(4.), 'LOST')

    def test_untracked_and_reused_id_jump_rejected(self):
        selector = TargetSelector()
        self.assertIsNone(selector.update([self.marker(tid=-1)], 0., 640, 480))
        selector.update([self.marker()], 0., 640, 480)
        self.assertIsNone(selector.update([self.marker(x=600)], .1, 640, 480))

    def test_bearing_center_sign_and_offset(self):
        calibration = Calibration(640, 480, (500., 0., 320., 0., 500., 240., 0., 0., 1.), (0.,)*5)
        self.assertAlmostEqual(bearing_deg(320, 240, 640, 480, calibration), 0.)
        self.assertGreater(bearing_deg(220, 240, 640, 480, calibration), 0.)
        self.assertLess(bearing_deg(420, 240, 640, 480, calibration), 0.)
        self.assertAlmostEqual(bearing_deg(320, 240, 640, 480, calibration, 10.), 10.)

    def test_distortion_correction(self):
        import cv2
        import numpy as np
        k = (500., 0., 320., 0., 500., 240., 0., 0., 1.)
        d = (.2, -.1, .01, .01, 0.)
        point, _ = cv2.projectPoints(np.array([[.2, .1, 1.]]), np.zeros(3), np.zeros(3),
                                     np.asarray(k).reshape(3, 3), np.asarray(d))
        import math
        self.assertAlmostEqual(bearing_deg(*point[0, 0], 640, 480, Calibration(640, 480, k, d)),
                               -math.degrees(math.atan(.2)), places=5)

    def test_no_invented_calibration_or_fov(self):
        with self.assertRaises(ValueError):
            bearing_deg(320, 240, 640, 480)
        with self.assertRaises(ValueError):
            bearing_deg(320, 240, 640, 480, allow_fov_fallback=True)
        self.assertAlmostEqual(bearing_deg(320, 240, 640, 480,
                               allow_fov_fallback=True, horizontal_fov_deg=90), 0.)

    def test_invalid_camera_info(self):
        for calibration in [Calibration(640, 480, (0.,)*9, ()),
                            Calibration(1280, 720, (500., 0., 320., 0., 500., 240., 0., 0., 1.), ())]:
            with self.assertRaises(ValueError):
                bearing_deg(320, 240, 640, 480, calibration)


if __name__ == '__main__':
    unittest.main()
