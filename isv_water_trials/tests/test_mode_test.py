from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from isv_water_trials.mode_test import test_pwm


class PwmTests(unittest.TestCase):
    def test_requested_percent_and_independent_sides(self):
        self.assertEqual(test_pwm(50), (1700, 1700))
        self.assertEqual(test_pwm(10), (1540, 1540))
        self.assertEqual(test_pwm(30), (1620, 1620))
        self.assertEqual(test_pwm(100), (1900, 1900))
        self.assertEqual(test_pwm(-100), (1100, 1100))
        self.assertEqual(test_pwm(50, 'left'), (1700, 1500))
        self.assertEqual(test_pwm(50, 'right'), (1500, 1700))
        with self.assertRaises(ValueError): test_pwm(30, 'unknown')
        for percent in (101, -101, float('nan'), float('inf')):
            with self.assertRaises(ValueError): test_pwm(percent)


if __name__ == '__main__':
    unittest.main()
