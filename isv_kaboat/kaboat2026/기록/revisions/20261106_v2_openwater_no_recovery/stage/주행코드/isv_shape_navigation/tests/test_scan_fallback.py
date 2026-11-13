import unittest
from types import SimpleNamespace as NS
from isv_shape_navigation.core import select_navigation_scan
class ScanFallbackTests(unittest.TestCase):
 def test_fresh_processed_preferred(self):
  a,b=NS(stamp=9.8),NS(stamp=9.9)
  self.assertEqual(select_navigation_scan(a,b,10.,.5),(a,'primary'))
 def test_late_processed_uses_only_fresh_raw(self):
  a,b=NS(stamp=9.4),NS(stamp=9.9)
  self.assertEqual(select_navigation_scan(a,b,10.,.5),(b,'fallback'))
 def test_both_stale_are_unavailable(self):
  a,b=NS(stamp=9.),NS(stamp=9.2)
  self.assertEqual(select_navigation_scan(a,b,10.,.5),(a,'unavailable'))
 def test_future_primary_is_not_accepted(self):
  a,b=NS(stamp=11.),NS(stamp=9.9)
  self.assertEqual(select_navigation_scan(a,b,10.,.5),(b,'fallback'))
 def test_no_data_is_unavailable(self):
  self.assertEqual(select_navigation_scan(None,None,10.,.5),(None,'unavailable'))
