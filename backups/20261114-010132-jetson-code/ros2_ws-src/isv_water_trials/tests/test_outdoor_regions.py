import unittest
import numpy as np
from isv_water_trials.outdoor_regions import OperatingRegion, gps_regions
from isv_water_trials.outdoor_planner import OutdoorPlanner
from test_outdoor import CONFIG, known_model


def box(x,y,w,h,kind='include'):
    return {'kind':kind,'points':[[x,y],[x+w,y],[x+w,y+h],[x,y+h]]}


class RegionTests(unittest.TestCase):
    def test_union_has_no_internal_seam_and_exclusions_win(self):
        r=OperatingRegion([box(-12,-12,24,24),box(8,-12,12,24),box(2,-2,3,4,'exclude')])
        values=r.clearance([[10,0],[3,0],[21,0],[0,0]])
        self.assertGreater(values[0],4)
        self.assertLess(values[1],0);self.assertLess(values[2],0);self.assertGreater(values[3],0)
        # Exclusion crossing the outer boundary and overlapping exclusions are legal.
        r=OperatingRegion([box(0,0,10,10),box(8,3,5,4,'exclude'),box(7,4,2,2,'exclude')])
        self.assertLess(r.clearance([[9,5]])[0],0)
        self.assertGreater(r.clearance([[5,5]])[0],0)

    def test_concave_polygons_and_disconnected_islands(self):
        r=OperatingRegion([{'kind':'include','points':[[0,0],[10,0],[10,4],[4,4],[4,10],[0,10]]},box(20,0,10,10)])
        self.assertGreater(r.clearance([[2,8]])[0],0)
        self.assertLess(r.clearance([[8,8]])[0],0)
        self.assertEqual(r.reachable([[2,8],[25,5]],[2,2],.7).tolist(),[True,False])

    def test_invalid_and_empty_areas(self):
        for regions in ([],[box(0,0,3,3,'exclude')],[box(0,0,3,3),box(-1,-1,5,5,'exclude')],
                        [{'kind':'include','points':[[0,0],[10,10],[0,10],[10,0]]}]):
            with self.assertRaises(ValueError):OperatingRegion(regions)
        with self.assertRaises(ValueError):gps_regions([box(200,0,1,1)])

    def test_planner_respects_hole_and_both_route_layers(self):
        regions=[box(-12,-12,24,24),box(3,-1,2,2,'exclude')]
        p=OutdoorPlanner(known_model(),CONFIG,20)
        scan={'angles':np.linspace(-np.pi,np.pi,720,endpoint=False),'ranges':np.full(720,np.inf),'range_min':.1,'range_max':30.,'stamp':102.}
        state={'position':[0,0],'yaw':0,'velocity':[0,0,0],'goal':[8,0],'position_sigma_m':.1,'test_regions':regions}
        result=p.plan(state,scan,[[100+i*.1,0,0] for i in range(21)],102)
        r=OperatingRegion(regions)
        for key in ('path','route','raw_route'):
            self.assertGreater(len(result[key]),0)
            self.assertTrue(np.all(r.clearance(result[key])>0),key)
        self.assertLessEqual(max(abs(v) for v in result['command']),.2)
        # Recovery still cannot pass through excluded geometry.
        from dataclasses import replace
        p.navigator.cfg=replace(p.navigator.cfg,margin=.1)
        p.navigator.origin=np.zeros(2)
        self.assertLess(p.navigator.safety_clearance([[4,0,0]],np.empty((0,3)),40,40)[0],0)


if __name__=='__main__':unittest.main()
