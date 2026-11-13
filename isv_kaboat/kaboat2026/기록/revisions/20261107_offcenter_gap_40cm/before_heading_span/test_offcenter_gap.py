import json
import math
from pathlib import Path
import time
import unittest
import numpy as np
from isv_shape_navigation.avoidance_v2 import Gap, checked_gap_runs, choose_gap, corridor_samples_valid
from isv_shape_navigation.core import Scan
from isv_shape_navigation.route_mission import RouteConfig


def scan(values=None):
    return Scan(np.full(721,np.inf) if values is None else values,-math.pi,math.pi/360,.1,3.,100.)


class OffCenterGapTests(unittest.TestCase):
    def setUp(self):
        self.cfg=RouteConfig(avoidance_range=3.,avoidance_horizon=3.)

    def gap(self):
        return Gap(math.radians(-30),math.radians(30),np.array([2.,-1.]),np.array([2.,1.]))

    def test_clear_original_center_is_retained_exactly(self):
        g=self.gap();rows=checked_gap_runs(g,lambda a:(True,1.))
        self.assertEqual(len(rows),1);self.assertFalse(rows[0].rescued)
        self.assertEqual(rows[0].angle,g.angle);np.testing.assert_array_equal(rows[0].center,g.center)

    def test_blocked_center_does_not_discard_safe_side(self):
        rows=checked_gap_runs(self.gap(),lambda a:(math.radians(-25)<=a<=math.radians(-5),.1))
        self.assertEqual(len(rows),1);self.assertTrue(rows[0].rescued)
        self.assertLess(rows[0].angle,math.radians(-5));self.assertGreater(rows[0].angle,math.radians(-25))
        self.assertLess(rows[0].left,0);self.assertLess(rows[0].right,0)
        self.assertAlmostEqual(float(np.linalg.norm(rows[0].center)),float(np.linalg.norm(self.gap().center)))

    def test_fully_blocked_or_tiny_free_sliver_is_not_a_gap(self):
        self.assertEqual(checked_gap_runs(self.gap(),lambda a:(False,-.1)),[])
        self.assertEqual(checked_gap_runs(self.gap(),lambda a:(math.radians(1)<a<math.radians(5),.1)),[])

    def test_separate_free_runs_are_not_joined_across_obstacle(self):
        rows=checked_gap_runs(self.gap(),lambda a:(abs(a)>math.radians(10),.1))
        self.assertEqual(len(rows),2);self.assertLess(rows[0].left,0);self.assertGreater(rows[1].right,0)
        for row in rows:self.assertGreater(abs(row.angle),math.radians(10))

    def test_invalid_or_near_obstacle_rules_still_apply(self):
        source=scan(np.full(721,np.nan));self.assertIsNone(choose_gap(source,0.,self.cfg,.25)[0])
        source=scan();source.ranges[360]=.12
        angle,r=choose_gap(source,0.,self.cfg,.25)
        self.assertIsNone(angle);self.assertEqual(r['reason'],'OBSTACLE_STOP')

    def test_42cm_rule_does_not_keep_the_old_60cm_width_floor(self):
        source=scan()
        for sign in (-1,1):
            for d in (13.5,14.,14.5):source.ranges[int(round(360+sign*d*2))]=1.
        _,report=choose_gap(source,0.,self.cfg,.21)
        central=[g for g in report['gaps'] if g['right_deg']<0<g['left_deg']]
        self.assertTrue(central)
        self.assertGreater(central[0]['width_m'],.42)
        self.assertLess(central[0]['width_m'],.60)
        _,wide=choose_gap(source,0.,self.cfg,.26)
        self.assertFalse([g for g in wide['gaps'] if g['right_deg']<0<g['left_deg']])

    def test_recorded_free_side_survives_center_blocker_without_changing_safety(self):
        fixture=json.loads((Path(__file__).parent/'fixtures/offcenter_gap.json').read_text())
        rescued=0;timings=[]
        for row in fixture['frames']:
            def restore(d):
                values=[np.nan if v is None else np.inf if v=='+inf' else -np.inf if v=='-inf' else v for v in d['ranges']]
                return Scan(np.array(values),d['angle_min'],d['angle_increment'],d['range_min'],d['range_max'],d['stamp'])
            source=restore(row['scan']);raw=restore(row['raw']) if row['raw'] is not None else None
            t=time.perf_counter();angle,report=choose_gap(source,0.,self.cfg,.25,raw_scan=raw);timings.append((time.perf_counter()-t)*1000)
            self.assertIsNotNone(angle);self.assertGreater(len(report['gaps']),0)
            rescued+=report['diagnostics']['rescued_gaps']
            points=source.points()
            if raw is not None:
                other=raw.points();other=other[np.linalg.norm(other,axis=1)<3.25];points=np.concatenate((points,other))
            for candidate in report['candidate_scores']:
                a=math.radians(candidate['angle_deg'])
                self.assertTrue(corridor_samples_valid(source,a,3.,.25))
                if raw is not None:self.assertTrue(corridor_samples_valid(raw,a,3.,.25))
                along=points[:,0]*math.cos(a)+points[:,1]*math.sin(a)
                cross=-points[:,0]*math.sin(a)+points[:,1]*math.cos(a)
                self.assertGreaterEqual(float(np.hypot(along-np.clip(along,0,3.),cross).min()),.25)
            self.assertEqual(self.cfg.avoidance_horizon,3.)
        self.assertGreater(rescued,0)
        print('RECORDED_SCAN_TIMINGS_MS',*[round(v,2) for v in timings])


if __name__=='__main__':unittest.main()
