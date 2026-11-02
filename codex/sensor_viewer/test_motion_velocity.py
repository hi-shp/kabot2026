import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'isv_water_trials'))
from isv_water_trials.tank_safety import MotionVelocity


class MotionVelocityTests(unittest.TestCase):
    def test_callbacks_accumulate_while_control_loop_is_busy(self):
        estimator=MotionVelocity()
        for i in range(8):
            estimator.add(100+i*.1,i*.02,0.,0.,10+i*.1)
        self.assertAlmostEqual(estimator.estimate(10.75,100.75)[0],.2)

    def test_first_pair_and_short_gap_keep_measured_velocity(self):
        estimator=MotionVelocity()
        estimator.add(100.,0.,0.,0.,10.)
        self.assertIsNone(estimator.estimate(10.,100.))
        estimator.add(100.1,.01,0.,0.,10.1)
        self.assertAlmostEqual(estimator.estimate(10.1,100.1)[0],.1)
        self.assertAlmostEqual(estimator.estimate(10.55,100.55)[0],.1)
        self.assertIsNone(estimator.estimate(10.8,100.8))

    def test_repeated_or_old_header_cannot_keep_output_alive(self):
        estimator=MotionVelocity()
        estimator.add(100.,0.,0.,0.,10.)
        estimator.add(100.1,.01,0.,0.,10.1)
        estimator.add(100.1,.01,0.,0.,11.)
        self.assertIsNone(estimator.estimate(11.,101.))
        self.assertIsNone(estimator.estimate(10.2,101.))

    def test_jump_and_clock_reset_need_new_samples(self):
        for jump in ((100.2,5.),(99.,.02),(101.2,.02)):
            estimator=MotionVelocity()
            estimator.add(100.,0.,0.,0.,10.)
            estimator.add(100.1,.01,0.,0.,10.1)
            estimator.add(jump[0],jump[1],0.,0.,10.2)
            self.assertIsNone(estimator.estimate(10.2,jump[0]))

    def test_heading_wrap_and_body_axes(self):
        estimator=MotionVelocity()
        estimator.add(100.,0.,0.,math.radians(179),10.)
        estimator.add(100.1,-.01,0.,math.radians(-179),10.1)
        u,v,r=estimator.estimate(10.1,100.1)
        self.assertAlmostEqual(u,.1*math.cos(math.radians(1)))
        self.assertAlmostEqual(r,math.radians(20))


if __name__=='__main__':unittest.main()
