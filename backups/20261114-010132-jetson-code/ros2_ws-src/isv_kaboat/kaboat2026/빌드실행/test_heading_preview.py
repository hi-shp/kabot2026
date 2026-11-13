import math,unittest
from types import SimpleNamespace as NS
from viewer_heading import HeadingPreview

def msg(angle,stamp=10.,frame='vectornav'):
    return NS(header=NS(frame_id=frame,stamp=NS(sec=int(stamp),nanosec=round((stamp-int(stamp))*1e9))),yawpitchroll=NS(x=angle))
class HeadingPreviewTests(unittest.TestCase):
    def test_cardinal_directions_match_existing_navigation(self):
        for raw,expected in [(0,math.pi/2),(90,0),(180,-math.pi/2),(-90,math.pi)]:
            h=HeadingPreview();h.update(msg(raw),10.);self.assertAlmostEqual(h.snapshot(10.1)['yaw'],expected)
    def test_stale_wrong_frame_invalid_and_out_of_order_are_not_fresh(self):
        h=HeadingPreview();self.assertIsNone(h.snapshot(10.)['yaw'])
        for m in [msg(20,frame='wrong'),msg(float('nan')),msg(20,stamp=8)]:h.update(m,10.)
        self.assertIsNone(h.snapshot(10.)['yaw'])
        h.update(msg(0),10.);h.update(msg(90,stamp=9.9),10.)
        self.assertAlmostEqual(h.snapshot(10.1)['yaw'],math.pi/2)
        self.assertIsNone(h.snapshot(11.)['yaw'])
    def test_existing_offset_is_read_without_modification(self):
        h=HeadingPreview(offset_deg=5.);h.update(msg(10.),10.)
        self.assertAlmostEqual(h.snapshot(10.)['yaw'],math.pi/2-math.radians(15.))

if __name__=='__main__':unittest.main()
