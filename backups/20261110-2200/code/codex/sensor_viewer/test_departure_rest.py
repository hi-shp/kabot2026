import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'isv_water_trials'))
from isv_water_trials.tank_safety import DepartureRest


class DepartureRestTests(unittest.TestCase):
    def sample(self,gate,t,velocity=(0,0,0),gyro=(0,0,0),neutral=True,q=(0,0,0,1)):
        return gate.update(t,(100+t,200+t),velocity,gyro,q,neutral)

    def test_start_and_each_arrival_require_minimum_neutral(self):
        gate=DepartureRest()
        for i in range(30):self.assertIsNone(self.sample(gate,i/10))
        reference=self.sample(gate,3.)
        self.assertEqual(reference['quaternion_xyzw'],[0,0,0,1])
        self.assertGreaterEqual(reference['stable_s'],1)
        self.assertFalse(gate.active)
        gate.begin();self.assertTrue(gate.active)
        self.assertIsNone(self.sample(gate,6.))

    def test_rotation_or_drift_restarts_stable_window(self):
        for velocity,gyro in [((.12,0,0),(0,0,0)),((0,0,.15),(0,0,0)),((0,0,0),(.15,0,0))]:
            gate=DepartureRest()
            for i in range(40):self.sample(gate,i/10)
            self.assertIsNone(self.sample(gate,4.,velocity,gyro))
            for i in range(41,51):self.assertIsNone(self.sample(gate,i/10))
            self.assertIsNotNone(self.sample(gate,5.2))

    def test_gentle_drift_is_allowed_and_recorded_not_assumed_zero(self):
        gate=DepartureRest();reference=None
        for i in range(31):reference=self.sample(gate,i/10,velocity=(.06,.01,.04),gyro=(.01,0,.04))
        self.assertIsNotNone(reference)
        self.assertAlmostEqual(reference['velocity_body_mean'][0],.06)
        self.assertEqual(reference['limits']['speed_m_s'],.08)

    def test_stale_duplicate_invalid_and_non_neutral_do_not_release(self):
        for problem in ('duplicate','gap','neutral','nan'):
            gate=DepartureRest(minimum=5.)
            for i in range(45):self.sample(gate,i/10)
            if problem=='duplicate':
                for i in range(45,70):self.assertIsNone(gate.update(i/10,(104.4,204.4),(0,0,0),(0,0,0),(0,0,0,1),True))
            elif problem=='gap':self.assertIsNone(self.sample(gate,6.))
            elif problem=='neutral':self.assertIsNone(self.sample(gate,4.5,neutral=False))
            else:self.assertIsNone(self.sample(gate,4.5,gyro=(math.nan,0,0)))
            self.assertTrue(gate.active)

    def test_quaternion_sign_does_not_cancel_reference(self):
        gate=DepartureRest();reference=None
        for i in range(51):reference=self.sample(gate,i/10,q=(0,0,0,1 if i%2 else -1))
        self.assertAlmostEqual(abs(reference['quaternion_xyzw'][3]),1)


if __name__=='__main__':unittest.main()
