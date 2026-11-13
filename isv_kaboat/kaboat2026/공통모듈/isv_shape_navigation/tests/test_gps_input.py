import math
from types import SimpleNamespace as NS
import unittest

from isv_shape_navigation.gps_input import inspect_fix, observation_report


def message(status=0,covariance_type=2,sigma=.5,latitude=35.2324707,longitude=129.0827164):
    return NS(latitude=latitude,longitude=longitude,status=NS(status=status),
              position_covariance_type=covariance_type,
              position_covariance=[sigma*sigma,0.,0.,0.,sigma*sigma,0.,0.,0.,1.])


class GPSDisplayTests(unittest.TestCase):
    def inspect(self,value,stamp=100.,previous=None):
        return inspect_fix(value,stamp,100.,2.,2.,previous)

    def test_valid_fix_is_available_to_navigation_and_display(self):
        view=self.inspect(message())
        self.assertTrue(view['accepted']);self.assertTrue(view['nav_valid'])
        self.assertAlmostEqual(view['latitude'],35.2324707)
        self.assertEqual(view['sigma'],.5)

    def test_received_coordinates_survive_unknown_accuracy(self):
        view=self.inspect(message(covariance_type=0))
        self.assertFalse(view['accepted']);self.assertFalse(view['nav_valid'])
        self.assertEqual(view['reason'],'GPS_COVARIANCE_UNKNOWN')
        self.assertAlmostEqual(view['longitude'],129.0827164)

    def test_no_fix_and_stale_timestamp_remain_visible_but_cannot_navigate(self):
        for value,stamp,reason in [(message(status=-1),100.,'GPS_NO_FIX'),(message(),0.,'GPS_STALE')]:
            view=self.inspect(value,stamp)
            self.assertFalse(view['accepted']);self.assertFalse(view['nav_valid'])
            self.assertIsNotNone(view['latitude']);self.assertEqual(view['reason'],reason)

    def test_uncertain_fix_keeps_existing_navigation_filter_behavior(self):
        view=self.inspect(message(sigma=3.))
        self.assertTrue(view['accepted']);self.assertFalse(view['nav_valid'])
        self.assertEqual(view['reason'],'GPS_UNCERTAIN')

    def test_bad_covariance_and_coordinate_are_explained(self):
        value=message();value.position_covariance[0]=-1
        self.assertEqual(self.inspect(value)['reason'],'GPS_COVARIANCE_INVALID')
        view=self.inspect(message(latitude=math.nan))
        self.assertIsNone(view['latitude']);self.assertFalse(view['accepted'])

    def test_duplicate_timestamp_is_reported_without_replacing_navigation_fix(self):
        view=self.inspect(message(),previous=100.)
        self.assertEqual(view['reason'],'GPS_OUT_OF_ORDER');self.assertFalse(view['accepted'])

    def test_no_new_messages_are_labelled_as_last_received_position(self):
        view=self.inspect(message())
        result=observation_report(view,103.,3.,2.)
        self.assertFalse(result['nav_valid']);self.assertEqual(result['reason'],'GPS_STALE')
        self.assertAlmostEqual(result['latitude'],35.2324707)


if __name__=='__main__':unittest.main()
