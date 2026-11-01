import math
import unittest
import numpy as np
from test_tank_global_match import fixture
from isv_water_trials.tank_global_match import GlobalMatcher,DisplayTrack,MotionHistory


class PathCandidatesTests(unittest.TestCase):
    def test_thin_wall_crossing_rejected_even_with_free_endpoints(self):
        grid,info=fixture(False);grid[10:50,50]=100
        matcher=GlobalMatcher(info,grid.tobytes())
        history=[(0,0,0),(4,0,0)]
        candidates=[dict(x=7,y=3,yaw_deg=0),dict(x=7,y=6,yaw_deg=0)]
        result=matcher.path_metrics(candidates,history,(4,0,0))
        self.assertFalse(result[0]['valid']);self.assertTrue(result[1]['valid'])
        self.assertGreater(result[0]['samples'],80)

    def test_rotated_map_and_candidate_apply_same_geometry(self):
        grid,info=fixture(False);grid[10:50,50]=100
        info.update(position=[10.,20.,0.],orientation=[0.,0.,math.sin(math.pi/4),math.cos(math.pi/4)])
        matcher=GlobalMatcher(info,grid.tobytes())
        result=matcher.path_metrics([dict(x=7,y=27,yaw_deg=90),dict(x=4,y=27,yaw_deg=90)],[(0,0,0),(4,0,0)],(4,0,0))
        self.assertFalse(result[0]['valid']);self.assertTrue(result[1]['valid'])

    def test_unknown_is_not_wall_but_outside_is_invalid(self):
        grid,info=fixture(False);grid[20:60,30:70]=-1
        matcher=GlobalMatcher(info,grid.tobytes())
        result=matcher.path_metrics([dict(x=6,y=4,yaw_deg=0),dict(x=1,y=4,yaw_deg=0)],[(0,0,0),(2,0,0)],(2,0,0))
        self.assertTrue(result[0]['valid']);self.assertLess(result[0]['known'],1)
        self.assertFalse(result[1]['valid'])

    def test_hull_width_rejects_near_wall_path(self):
        grid,info=fixture(False);grid[10:50,50]=100
        m=GlobalMatcher(info,grid.tobytes())
        result=m.path_metrics([dict(x=4.9,y=3,yaw_deg=0)],[(0,0,0)],(0,0,0))
        self.assertFalse(result[0]['valid'])

    def test_filter_removes_impossible_best_and_scores_incumbent_on_same_scan(self):
        grid,info=fixture(False);grid[10:50,50]=100
        matcher=GlobalMatcher(info,grid.tobytes())
        bad=dict(x=7,y=3,yaw_deg=0,score=.95,inliers=.9,known=1.,free=1.)
        good=dict(bad,y=6,score=.85)
        points=np.array([[1.,0.],[0.,1.],[-1.,0.]])
        result=matcher.constrain_path(dict(candidates=[bad,good]),[(0,0,0),(4,0,0)],(4,0,0),points,bad)
        self.assertEqual(len(result['candidates']),1)
        self.assertEqual(result['candidates'][0]['y'],6)
        self.assertEqual(result['path_rejected'],1)
        self.assertFalse(result['incumbent_path_valid'])
        self.assertTrue(math.isfinite(result['incumbent_score']))

    def test_trajectory_end_follows_current_pose_after_correction(self):
        track=DisplayTrack();best=dict(x=10,y=20,yaw_deg=90,score=.9,inliers=.9,known=1.,free=1.)
        track.adopt(dict(candidates=[best],path_checked=True),(1,0,0),(2,0,0),1.)
        np.testing.assert_allclose(track.trail([(0,0,0),(1,0,0),(2,0,0)]),[[10,19],[10,20],[10,21]])
        self.assertEqual(track.trail([(0,0,0),(2,0,0)])[-1],[track.pose((2,0,0))['x'],track.pose((2,0,0))['y']])

    def test_higher_candidate_reprojects_whole_trail_without_bridge(self):
        track=DisplayTrack();best=dict(x=1.,y=2.,yaw_deg=0.,score=.85,inliers=.8,known=.95,free=.95)
        track.adopt({'candidates':[best]},(0,0,0),(0,0,0),1.)
        history=[(0,0,0),(1,0,0),(2,0,0)]
        np.testing.assert_allclose(track.trail(history),[[1,2],[2,2],[3,2]])
        changed=dict(best,x=8.,y=8.,yaw_deg=90.,score=.9)
        result=dict(candidates=[changed],path_checked=True,incumbent_score=.85,incumbent_path_valid=True)
        self.assertTrue(track.adopt(result,(2,0,0),(2,0,0),2.))
        np.testing.assert_allclose(track.trail(history),[[8,6],[8,7],[8,8]])
        self.assertEqual(track.revision,2)
        result.update(incumbent_score=.899)
        result['candidates']=[dict(changed,x=20)]
        self.assertFalse(track.adopt(result,(2,0,0),(2,0,0),3.))

    def test_impossible_incumbent_replaced_even_without_score_gain(self):
        track=DisplayTrack();best=dict(x=1.,y=2.,yaw_deg=0.,score=.95,inliers=.8,known=.95,free=.95)
        track.adopt({'candidates':[best]},(0,0,0),(0,0,0),1.)
        self.assertTrue(track.adopt(dict(candidates=[dict(best,x=7.,score=.81)],path_checked=True,
                                        incumbent_score=.95,incumbent_path_valid=False),(0,0,0),(0,0,0),2.))
        self.assertEqual(track.pose((0,0,0))['x'],7.)

    def test_motion_history_is_bounded_and_resettable(self):
        history=MotionHistory()
        for i in range(5000):history.add((i*.05,0,0))
        self.assertEqual(len(history.snapshot()),4096)
        history.clear();self.assertEqual(history.snapshot(),[])


if __name__=='__main__':unittest.main()
