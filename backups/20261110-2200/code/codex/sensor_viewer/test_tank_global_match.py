import math
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'isv_water_trials'))
from isv_water_trials.tank_global_match import GlobalMatcher,ScanEvidence,DisplayTrack,handoff_ready,scan_points,scan_stable,wrap


def fixture(asymmetric=True):
    grid=np.zeros((80,100),np.int8);grid[:2,:]=100;grid[-2:,:]=100;grid[:,:2]=100;grid[:,-2:]=100
    if asymmetric:
        grid[15:38,65:68]=100;grid[35:38,65:90]=100;grid[60:70,20:35]=100
    info=dict(width=100,height=80,resolution=.1,position=[0.,0.,0.],orientation=[0.,0.,0.,1.])
    return grid,info


def rays(grid,pose):
    angles=np.linspace(-np.pi,np.pi,360,endpoint=False);r=np.full(360,np.inf)
    for i,a in enumerate(angles+pose[2]):
        for d in np.arange(.1,15,.04):
            x=int((pose[0]+d*np.cos(a))/.1);y=int((pose[1]+d*np.sin(a))/.1)
            if x<0 or y<0 or x>=grid.shape[1] or y>=grid.shape[0]:break
            if grid[y,x]>=65:r[i]=d;break
    return r


class GlobalMatchTests(unittest.TestCase):
    def test_vectorized_diversity_preserves_original_greedy_choices(self):
        rng=np.random.default_rng(7);poses=rng.normal(size=(2500,3));metrics=rng.normal(size=(2500,4))
        chosen=[]
        for i in np.argsort(-metrics[:,0]):
            if metrics[i,0]<=-1:continue
            if all(np.linalg.norm(poses[i,:2]-poses[j,:2])>.8 or abs(wrap(poses[i,2]-poses[j,2]))>math.radians(15) for j in chosen):
                chosen.append(i)
                if len(chosen)==64:break
        actual,_=GlobalMatcher.diverse(poses,metrics)
        np.testing.assert_array_equal(actual,poses[chosen])

    def test_fast_review_preserves_distant_equal_candidates(self):
        grid,info=fixture(False);pose=(3.,2.,.7);points=scan_points(rays(grid,pose),-np.pi,2*np.pi/360)
        candidates=[dict(x=3.,y=2.,yaw_deg=math.degrees(.7)),dict(x=7.,y=6.,yaw_deg=math.degrees(.7+math.pi))]
        result=GlobalMatcher(info,grid.tobytes()).refine(candidates,points)
        self.assertEqual(len(result['candidates']),2)
        self.assertEqual(result['state'],'AMBIGUOUS')
        self.assertEqual(result['method'],'local_candidate_review')

    def test_display_uses_score_without_claiming_unique_position(self):
        best=dict(x=10.,y=20.,yaw_deg=90.,score=.85,inliers=.8,known=.95,free=.95)
        result={'candidates':[best,dict(best,score=.84)]};track=DisplayTrack()
        self.assertFalse(handoff_ready(result))
        self.assertTrue(track.adopt(result,(0.,0.,0.),(1.,0.,0.),5.))
        position=track.pose((2.,0.,math.pi/2))
        self.assertAlmostEqual(position['x'],10.)
        self.assertAlmostEqual(position['y'],22.)
        self.assertAlmostEqual(position['yaw_deg'],180.)
        self.assertTrue(position['provisional'])
        self.assertFalse(track.adopt({'candidates':[dict(best,x=50.)]},(0.,0.,0.),(2.,0.,0.),6.))
        self.assertTrue(track.conflict)

    def test_display_correction_is_low_weight_and_low_score_is_rejected(self):
        best=dict(x=1.,y=2.,yaw_deg=0.,score=.85,inliers=.8,known=.95,free=.95)
        track=DisplayTrack()
        self.assertFalse(track.adopt({'candidates':[dict(best,score=.79)]},(0,0,0),(0,0,0),1.))
        track.adopt({'candidates':[best]},(0,0,0),(0,0,0),1.)
        track.adopt({'candidates':[dict(best,x=1.5,yaw_deg=10.)]},(0,0,0),(0,0,0),2.)
        pose=track.pose((0,0,0));self.assertAlmostEqual(pose['x'],1.1);self.assertAlmostEqual(pose['yaw_deg'],2.)

    def test_handoff_score_and_separation(self):
        best=dict(x=1.,y=2.,yaw_deg=20.,score=.86,inliers=.8,known=.95,free=.95)
        self.assertTrue(handoff_ready({'candidates':[best,dict(score=.78)]}))
        self.assertFalse(handoff_ready({'candidates':[best,dict(score=.85)]}))
        self.assertFalse(handoff_ready({'candidates':[dict(best,score=.79),dict(score=.6)]}))
        self.assertFalse(handoff_ready({'candidates':[dict(best,free=.7),dict(score=.6)]}))
        self.assertFalse(handoff_ready({'candidates':[dict(best,score=float('nan')),dict(score=.6)]}))
        self.assertFalse(handoff_ready({'candidates':[best]}))
        self.assertFalse(handoff_ready({'candidates':[best,dict(score=.78)]},min_score=.9))

    def test_asymmetric_room_recovers_unknown_heading(self):
        grid,info=fixture();pose=(4.,4.,.7);r=rays(grid,pose)
        result=GlobalMatcher(info,grid.tobytes()).search(scan_points(r,-np.pi,2*np.pi/360))
        best=result['candidates'][0]
        self.assertLess(math.hypot(best['x']-pose[0],best['y']-pose[1]),.2,result)
        self.assertLess(abs(wrap(math.radians(best['yaw_deg'])-pose[2])),.08,result)
        self.assertEqual(result['state'],'UNIQUE',result)

    def test_symmetric_pool_keeps_multiple_candidates(self):
        grid,info=fixture(False);r=rays(grid,(3.,2.,.7))
        result=GlobalMatcher(info,grid.tobytes()).search(scan_points(r,-np.pi,2*np.pi/360))
        self.assertEqual(result['state'],'AMBIGUOUS',result)
        self.assertLess(result['margin'],.1)

    def test_sparse_scans_rejected_and_moving_scans_not_reused(self):
        with self.assertRaises(ValueError):scan_points([float('inf')]*360,-np.pi,2*np.pi/360)
        self.assertFalse(scan_stable(np.ones(100)*2,np.ones(100)*3))
        self.assertTrue(scan_stable(np.ones(100)*2,np.ones(100)*2.02))

    def test_empty_map_is_rejected(self):
        grid,info=fixture();grid[:]=0
        with self.assertRaises(ValueError):GlobalMatcher(info,grid.tobytes())

    def test_evidence_transforms_endpoints_and_ray_origins(self):
        evidence=ScanEvidence();cloud=np.tile([2.,0.],(48,1))
        evidence.add(1.,(1.,1.,0.),cloud)
        points,info=evidence.collect(1.5,(2.,1.,math.pi/2),cloud)
        np.testing.assert_allclose(points[0],[0.,-1.,0.,1.],atol=1e-12)
        np.testing.assert_allclose(points[-1],[2.,0.,0.,0.],atol=1e-12)
        self.assertEqual(info['observation_count'],2)

    def test_stationary_repeats_and_odom_jump_do_not_accumulate(self):
        evidence=ScanEvidence();cloud=np.tile([2.,0.],(48,1))
        for i in range(20):evidence.add(i*.1,(1.,1.,0.),cloud)
        self.assertEqual(evidence.collect(2.,(1.,1.,0.),cloud)[1]['observation_count'],1)
        evidence.add(2.,(10.,10.,0.),cloud)
        self.assertEqual(len(evidence.frames),1)
        self.assertEqual(evidence.collect(30.,(10.,10.,0.),cloud)[1]['observation_count'],1)

    def test_ray_origin_extension_preserves_single_scan_score(self):
        grid,info=fixture();matcher=GlobalMatcher(info,grid.tobytes())
        cloud=scan_points(rays(grid,(4.,4.,.7)),-np.pi,2*np.pi/360)
        np.testing.assert_allclose(matcher.evaluate([[4.,4.,.7]],cloud),
                                   matcher.evaluate([[4.,4.,.7]],np.column_stack((cloud,np.zeros_like(cloud)))))

    def test_multiple_viewpoints_recover_current_pose(self):
        grid,info=fixture();evidence=ScanEvidence()
        for i in range(8):
            pose=(3.3+.1*i,4.,.7)
            cloud=scan_points(rays(grid,pose),-np.pi,2*np.pi/360)
            evidence.add(1.+i*.2,pose,cloud)
        points,meta=evidence.collect(2.4,pose,cloud)
        self.assertGreater(meta['observation_count'],1)
        result=GlobalMatcher(info,grid.tobytes()).search(points)
        best=result['candidates'][0]
        self.assertLess(math.hypot(best['x']-4,best['y']-4),.2,result)
        self.assertLess(abs(wrap(math.radians(best['yaw_deg'])-.7)),.08,result)
        self.assertEqual(result['state'],'UNIQUE',result)


if __name__=='__main__':unittest.main()
