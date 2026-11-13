import math
from dataclasses import replace
import time
import unittest
import numpy as np
from test_outdoor import CONFIG, known_model
from isv_water_trials.outdoor_obstacles import ObstacleTracker, compact_obstacles
from isv_water_trials.outdoor_planner import LearnedNavigator, OutdoorPlanner
from isv_water_trials.outdoor_model import ResponseModel
from isv_water_trials.codex_nav.navigation_map import NavigationMap


def scan_circle(center, sensor=(0.,0.), yaw=0., radius=.35):
    angles=np.linspace(-math.pi,math.pi,720,endpoint=False)
    unit=np.c_[np.cos(angles+yaw),np.sin(angles+yaw)]
    delta=np.asarray(center)-sensor; along=unit@delta
    disc=along**2-(delta@delta-radius**2)
    ranges=np.full(720,np.inf); hit=(disc>=0)&(along>0)
    ranges[hit]=along[hit]-np.sqrt(disc[hit])
    return angles,ranges


class NavigationTests(unittest.TestCase):
    def test_tracker_estimates_crossing_velocity_and_occlusion(self):
        tracker=ObstacleTracker()
        for i in range(15):
            a,r=scan_circle([6.,-1.+i*.1])
            tracker.update(a,r,[0.,0.],0.,10+i*.2,30.)
        rows,info=tracker.prediction(12.8,np.zeros(2))
        self.assertEqual(len(rows),1);self.assertTrue(info[0]['confirmed'])
        self.assertAlmostEqual(rows[0,4],.5,delta=.12)
        self.assertAlmostEqual(rows[0,3],0.,delta=.12)
        a,r=scan_circle([50,50]);tracker.update(a,r,[0.,0.],0.,13.,30.)
        self.assertEqual(len(tracker.prediction(13.,np.zeros(2))[0]),1)
        self.assertEqual(len(tracker.prediction(14.,np.zeros(2))[0]),0)

    def test_ego_translation_rotation_do_not_make_static_object_move(self):
        tracker=ObstacleTracker()
        for i in range(15):
            sensor=np.array([i*.04,i*.02]);yaw=i*.025
            a,r=scan_circle([6.,1.],sensor,yaw)
            tracker.update(a,r,sensor,yaw,10+i*.2,30.)
        rows,_=tracker.prediction(12.8,np.zeros(2))
        self.assertEqual(len(rows),1)
        self.assertLess(np.linalg.norm(rows[0,3:5]),.08)

    def test_repeated_scan_does_not_fabricate_velocity(self):
        tracker=ObstacleTracker();a,r=scan_circle([5,0])
        tracker.update(a,r,[0.,0.],0.,10.,30.)
        for _ in range(20): tracker.update(a,r,[0.,0.],0.,10.,30.)
        self.assertEqual(tracker.tracks[0].hits,1)
        a,r=scan_circle([8,0]);tracker.update(a,r,[0.,0.],0.,1.,30.)
        self.assertEqual(len(tracker.tracks),1)

    def test_obstacle_compaction_encloses_every_original_disc(self):
        rng=np.random.default_rng(7);discs=np.c_[rng.uniform(0,2,(500,2)),rng.uniform(.01,.2,500)]
        merged=compact_obstacles(discs)
        for d in discs:
            self.assertTrue(np.any(np.linalg.norm(merged[:,:2]-d[:2],axis=1)+d[2]<=merged[:,2]+1e-9))
        self.assertLess(len(merged),len(discs)/2)

    def test_dynamic_crossing_checked_at_future_times(self):
        nav=LearnedNavigator(ResponseModel(known_model()),CONFIG,.2)
        z=np.zeros((1,8));z[0,:2]=[20,20]
        nav.dynamic=np.array([[20,23,.2,0,-1.,0.]])
        self.assertGreater(nav.moving_clearance(z,0)[0],1.)
        self.assertLess(nav.moving_clearance(z,3)[0],0.)
        nav.recovery=True;nav.cfg=replace(nav.cfg,margin=.1)
        self.assertLess(nav.moving_clearance(z,3)[0],-.2)

    def test_rollout_rejects_crossing_while_receding_object_is_clear(self):
        nav=LearnedNavigator(ResponseModel(known_model()),CONFIG,.2)
        initial=np.zeros(8);initial[:2]=[20,20]
        sequences=np.zeros((1,15,2))
        nav.dynamic=np.array([[20,23,.2,0,-1.,.05]])
        _,crossing=nav.rollout(initial,sequences,np.empty((0,3)),40,40)
        nav.dynamic=np.array([[20,23,.2,0,1.,.05]])
        _,receding=nav.rollout(initial,sequences,np.empty((0,3)),40,40)
        self.assertLess(crossing[0],0.)
        self.assertGreater(receding[0],nav.cfg.margin)

    def test_recovery_keeps_output_cap_and_does_not_weaken_dynamic_margin(self):
        nav=LearnedNavigator(ResponseModel(known_model()),CONFIG,.5)
        nav.recovery=True;nav.cfg=replace(nav.cfg,margin=.1)
        q=nav.inverse([0,0,0],[2,2])
        self.assertLessEqual(abs(q).max(),.2)
        z=np.zeros((1,8));z[0,:2]=[20,20]
        nav.dynamic=np.array([[21.,20.,.1,0,0,0]])
        recovery=nav.moving_clearance(z,0)[0]-nav.cfg.margin
        nav.cfg=replace(nav.cfg,margin=nav.normal_margin)
        normal=nav.moving_clearance(z,0)[0]-nav.cfg.margin
        self.assertAlmostEqual(recovery,normal)

    def test_smooth_route_does_not_cut_obstacle_and_cache_moves_with_origin(self):
        nav=LearnedNavigator(ResponseModel(known_model()),CONFIG,.2)
        obs=NavigationMap(40,40,.4);obs.obstacles=np.array([[24.,20.,1.]])
        obs.free_seen[:]=0;obs.time=0
        z=np.zeros(8);z[:2]=[20,20];goal=np.array([28,20])
        nav.now=1.;path=nav.guidance(z,obs,goal,0)
        self.assertGreater(len(path),2)
        samples=np.concatenate([np.linspace(a,b,10) for a,b in zip(path[:-1],path[1:])])
        self.assertGreaterEqual(obs.clearance(samples).min(),nav.cfg.margin+.015)
        world=nav.route_world.copy();nav.origin=np.array([.01,0]);nav.now=1.1
        obs.obstacles[:,:2]-=nav.origin
        shifted=nav.guidance(z,obs,goal-nav.origin,1)
        np.testing.assert_allclose(shifted[1:]+nav.origin,world[1:])

    def test_scan_range_follows_driver_and_can_be_capped(self):
        state={'position':[0,0],'yaw':0.,'velocity':[0,0,0],'goal':[8,0]}
        scan={'angles':np.linspace(-math.pi,math.pi,720,endpoint=False),'ranges':np.full(720,np.inf),'range_min':.1,'range_max':80.}
        history=[[100+i*.1,0,0] for i in range(21)]
        for cap,expected in [(0,80.),(30,30.)]:
            planner=OutdoorPlanner(known_model(),dict(CONFIG,scan_range_m=cap),20)
            result=planner.plan(state,scan,history,102.)
            self.assertEqual(result['scan_range_m'],expected)
            self.assertLessEqual(result['obstacle_discs'],720)

    def test_unknown_beams_do_not_become_obstacles_at_ownship(self):
        planner=OutdoorPlanner(known_model(),CONFIG,20)
        scan={'angles':np.linspace(-math.pi,math.pi,720,endpoint=False),'ranges':np.full(720,np.inf),'range_min':.1}
        scan['ranges'][0:20]=np.nan  # Unknown behind; intended forward remains observed.
        result=planner.plan({'position':[0,0],'yaw':0.,'velocity':[0,0,0],'goal':[8,0]},scan,[[100+i*.1,0,0] for i in range(21)],102.)
        self.assertEqual(result['obstacle_discs'],0)

    def test_dense_nearby_wall_is_retained_and_low_speed_recovery_is_used(self):
        planner=OutdoorPlanner(known_model(),dict(CONFIG,planner_deadline_s=5.),20)
        a=np.linspace(-math.pi,math.pi,720,endpoint=False);r=np.full(720,np.inf)
        hit=np.sin(a)<-.01;r[hit]=-.65/np.sin(a[hit]);r[r>30]=np.inf
        out=planner.plan({'position':[0,0],'yaw':0.,'velocity':[0,0,0],'goal':[8,0]},
            {'angles':a,'ranges':r,'range_min':.1,'range_max':30},
            [[100+i*.1,0,0] for i in range(21)],102.)
        self.assertTrue(out['recovery'])
        self.assertLess(out['clearance_m'],.4)
        self.assertGreaterEqual(out['clearance_m'],.1)
        self.assertLessEqual(max(abs(x) for x in out['command']),.2)


if __name__=='__main__': unittest.main()
