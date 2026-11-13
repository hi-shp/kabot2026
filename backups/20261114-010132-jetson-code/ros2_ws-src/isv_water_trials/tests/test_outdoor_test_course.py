import unittest
import numpy as np
from isv_water_trials.outdoor_test_course import polygon, hull_clearance, create_course, PlanComparison
from isv_water_trials.outdoor_planner import OutdoorPlanner
from test_outdoor import CONFIG,known_model


AREA=[[-12,-12],[12,-12],[12,12],[-12,12]]


class CourseTests(unittest.TestCase):
    def test_geometry_rejects_concave_or_crossing_and_checks_full_hull(self):
        np.testing.assert_allclose(polygon(AREA[::-1]),AREA)
        for points in ([[-10,-10],[10,10],[10,-10],[-10,10]],[[0,0],[10,0],[3,3],[0,10]]):
            with self.assertRaises(ValueError):polygon(points)
        self.assertLess(hull_clearance([[11.8,0,0]],AREA,1.1,.5)[0],0)
        self.assertGreater(hull_clearance([[0,0,1.2]],AREA,1.1,.5)[0],10)

    def test_generation_requires_observed_room_and_avoids_ownship(self):
        state={'position':[0,0],'yaw':0,'position_sigma_m':.4}
        scan={'angles':np.linspace(-np.pi,np.pi,720,endpoint=False),'ranges':np.full(720,np.inf),'range_max':30.,'range_min':.1}
        course=create_course(AREA,state,scan,CONFIG)
        self.assertEqual(len(course['goals']),2);self.assertEqual(len(course['obstacles']),1)
        self.assertGreater(np.linalg.norm(course['obstacles'][0][:2]),2)
        self.assertGreater(np.linalg.norm(np.diff(course['goals'],axis=0)),7)
        for ranges in (np.full(720,.5),np.full(720,np.nan)):
            with self.assertRaises(ValueError):create_course(AREA,state,dict(scan,ranges=ranges),CONFIG)
        with self.assertRaises(ValueError):create_course(AREA,dict(state,position=[12,0]),scan,CONFIG)

    def test_boundary_cannot_be_relaxed_by_recovery(self):
        planner=OutdoorPlanner(known_model(),CONFIG,20);n=planner.navigator
        n.test_boundary=np.array(AREA);n.normal_margin=1.4
        from dataclasses import replace
        n.cfg=replace(n.cfg,margin=.1)
        # Center is inside but hull plus required uncertainty buffer is outside.
        self.assertLess(n.safety_clearance([[11,0,0]],np.empty((0,3)),40,40)[0],n.cfg.margin)

    def test_real_wall_still_blocks_virtual_course(self):
        planner=OutdoorPlanner(known_model(),CONFIG,20)
        scan={'angles':np.linspace(-np.pi,np.pi,720,endpoint=False),'ranges':np.full(720,.3),'range_min':.1,'range_max':30.,'stamp':102.}
        state={'position':[0,0],'yaw':0,'velocity':[0,0,0],'goal':[8,0],'position_sigma_m':.1,'test_area':AREA,'virtual_obstacles':[[4,2,.4]]}
        with self.assertRaises(ValueError):planner.plan(state,scan,[[100+i*.1,0,0] for i in range(21)],102)

    def test_feasible_course_plans_inside_area_with_output_cap(self):
        planner=OutdoorPlanner(known_model(),CONFIG,20)
        scan={'angles':np.linspace(-np.pi,np.pi,720,endpoint=False),'ranges':np.full(720,np.inf),'range_min':.1,'range_max':30.,'stamp':102.}
        state={'position':[0,0],'yaw':0,'velocity':[0,0,0],'goal':[8,0],'position_sigma_m':.1,'test_area':AREA,'virtual_obstacles':[[4,0,.45]]}
        result=planner.plan(state,scan,[[100+i*.1,0,0] for i in range(21)],102)
        self.assertGreater(sum(result['command']),0)
        self.assertLessEqual(max(abs(v) for v in result['command']),.2)
        self.assertTrue(np.all(np.abs(result['path'])<12))
        self.assertGreater(result['obstacle_discs'],10)

    def test_frozen_plan_comparison_uses_time_not_nearest_path_point(self):
        c=PlanComparison();c.plan(100,[[.2*i,0] for i in range(1,16)],.2)
        events=c.observe(101,[.5,0]);self.assertAlmostEqual(events[0]['error_m'],.5)
        self.assertEqual(c.observe(101,[.5,0]),[])
        c.observe(102,[2.,0]);self.assertAlmostEqual(c.summary()['rmse_m'],np.sqrt(.25/2))


if __name__=='__main__':unittest.main()
