import unittest
from isv_shape_navigation.supplied_pwm import soften_turn_pwm
from isv_shape_navigation.route_mission import RouteConfig
class GentleTurnTests(unittest.TestCase):
 def test_large_turn_cap_preserves_mean(self):
  self.assertEqual(soften_turn_pwm((1800,1300)),(1650,1450))
  self.assertEqual(soften_turn_pwm((1300,1800)),(1450,1650))
 def test_small_turn_is_halved(self):
  self.assertEqual(soften_turn_pwm((1700,1600)),(1675,1625))
 def test_neutral_and_equal_thrust_unchanged(self):
  for pwm in [(1500,1500),(1650,1650),(1400,1400)]:self.assertEqual(soften_turn_pwm(pwm),pwm)
 def test_pivot_is_gentler(self):
  self.assertEqual(soften_turn_pwm((1560,1440)),(1530,1470))
 def test_original_can_be_requested(self):
  self.assertEqual(soften_turn_pwm((1850,1200),1.,800),(1850,1200))
 def test_config_rejects_invalid_tuning(self):
  for kwargs in [dict(turn_pwm_scale=0.),dict(turn_pwm_scale=2.),dict(turn_pwm_max_difference=0)]:
   with self.assertRaises(ValueError):RouteConfig(**kwargs)
