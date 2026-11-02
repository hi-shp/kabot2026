import ast
from concurrent.futures import Future
import math
from pathlib import Path
import time
from types import SimpleNamespace as N
import unittest
import numpy as np
from test_tank_global_match import fixture,rays
from isv_water_trials.tank_global_match import GlobalMatcher,DisplayTrack,scan_points,wrap


class LocalTrackingTests(unittest.TestCase):
    def setUp(self):
        grid,info=fixture();self.matcher=GlobalMatcher(info,grid.tobytes())
        self.truth=(4.,4.,.7);self.points=scan_points(rays(grid,self.truth),-np.pi,2*np.pi/360)

    def test_drift_converges_with_bounded_steps_and_without_global_search(self):
        self.matcher.search=lambda *_:self.fail('Global search must stay off')
        seed=dict(x=4.25,y=3.8,yaw_deg=math.degrees(.75))
        for _ in range(8):
            result=self.matcher.track(seed,self.points)
            self.assertTrue(result['accepted'],result)
            self.assertLessEqual(result['translation_m'],.0800001)
            self.assertLessEqual(abs(result['rotation_deg']),.500001)
            seed=result['candidate']
        self.assertLess(math.hypot(seed['x']-4,seed['y']-4),.08)
        self.assertLess(abs(wrap(math.radians(seed['yaw_deg'])-.7)),math.radians(1.5))
        self.assertTrue(result['supported'])

    def test_large_offset_needs_reselection_not_a_jump(self):
        result=self.matcher.track(dict(x=5.2,y=4.,yaw_deg=40.),self.points)
        self.assertFalse(result['accepted'])

    def test_correct_pose_is_supported_without_shift(self):
        result=self.matcher.track(dict(x=4.,y=4.,yaw_deg=math.degrees(.7)),self.points)
        self.assertTrue(result['supported']);self.assertLess(result['translation_m'],.01)

    def test_weak_boundary_peak_keeps_well_supported_prior(self):
        def evaluate(poses,points):
            poses=np.asarray(poses).reshape(-1,3)
            score=.94+.008*(poses[:,1]-4.)
            return np.column_stack((score,np.ones((len(poses),3))))
        self.matcher.evaluate=evaluate
        result=self.matcher.track(dict(x=4.,y=4.,yaw_deg=40.),self.points)
        self.assertTrue(result['accepted'],result)
        self.assertTrue(result['supported'])
        self.assertLess(result['translation_m'],1e-8)
        self.assertAlmostEqual(result['rotation_deg'],0.)

    def test_distinct_boundary_peak_is_still_rejected(self):
        def evaluate(poses,points):
            poses=np.asarray(poses).reshape(-1,3)
            score=np.exp(-.5*((poses[:,1]-4.55)/.08)**2)
            return np.column_stack((score,np.ones((len(poses),3))))
        self.matcher.evaluate=evaluate
        result=self.matcher.track(dict(x=4.,y=4.,yaw_deg=40.),self.points)
        self.assertFalse(result['accepted'])

    def test_unobservable_axis_keeps_selected_prior_while_correcting_observable_axis(self):
        def evaluate(poses,points):
            poses=np.asarray(poses).reshape(-1,3)
            score=np.exp(-.5*((poses[:,0]-4.)/.2)**2)
            return np.column_stack((score,np.ones((len(poses),3))))
        self.matcher.evaluate=evaluate
        seed=dict(x=4.25,y=3.8,yaw_deg=40.)
        for _ in range(6):
            result=self.matcher.track(seed,self.points)
            self.assertTrue(result['accepted'],result);seed=result['candidate']
            self.assertAlmostEqual(seed['y'],3.8)
            self.assertAlmostEqual(seed['yaw_deg'],40.)
        self.assertLess(abs(seed['x']-4.),.06)
        self.assertTrue(result['supported'])

    def test_async_result_uses_capture_odom_and_rejects_stale_generation(self):
        path=Path(__file__).resolve().parents[2]/'isv_water_trials/isv_water_trials/tank_localize.py'
        cls=next(n for n in ast.walk(ast.parse(path.read_text(encoding='utf-8'))) if isinstance(n,ast.ClassDef) and n.name=='Localizer')
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='manual_tick')
        ns=dict(math=math,time=time,scan_points=scan_points)
        exec(compile(ast.Module(body=[method],type_ignores=[]),str(path),'exec'),ns)
        for generation,stamp,valid in ((1,99.9,True),(0,99.9,False),(1,98.,False)):
            candidate=dict(x=4.,y=4.,yaw_deg=0.,score=.95,inliers=1.,known=1.,free=1.)
            track=DisplayTrack();track.adopt(dict(candidates=[candidate]),(0,0,0),(0,0,0),99.)
            future=Future();future.set_result(dict(accepted=True,candidate=dict(candidate,x=4.08),supported=True,translation_m=.08,rotation_deg=0.,reason='tracking'))
            now=time.monotonic();node=N(get_clock=lambda:N(now=lambda:N(nanoseconds=100_000_000_000)),
              odom=(99.9,(.2,0,0)),session=dict(session_id='map'),map_id='map',session_at=now,
              imu=N(header=N(stamp=N(sec=99,nanosec=900000000)),angular_velocity=N(x=0.,y=0.,z=0.)),imu_at=now,
              tracking_future=future,tracking_generation=generation,generation=1,tracking_stamp=stamp,
              tracking_reference=(0,0,0),display=track,local_reason='',local_confirmations=2,local_good_at=0.,report={},
              scan=None,next_tracking=0.,publish=lambda *a,**k:None)
            ns['manual_tick'](node)
            self.assertEqual(node.local_good_at>0,valid)
            self.assertAlmostEqual(track.pose((.2,0,0))['x'],4.28 if valid else 4.2)
            if valid:
                confirmed_at=node.local_good_at
                for _ in range(3):
                    rejected=Future();rejected.set_result(dict(accepted=False,reason='boundary'))
                    node.tracking_future=rejected
                    ns['manual_tick'](node)
                    self.assertEqual(node.local_good_at,confirmed_at)
                    self.assertEqual(node.local_confirmations,0)
                    self.assertAlmostEqual(track.pose((.2,0,0))['x'],4.28)
                # Rejections never refresh the grace period; unsupported applied
                # corrections invalidate it immediately.
                unsupported=Future();unsupported.set_result(dict(accepted=True,candidate=candidate,
                    supported=False,translation_m=0.,rotation_deg=0.,reason='unconfirmed'))
                node.tracking_future=unsupported
                ns['manual_tick'](node)
                self.assertEqual(node.local_good_at,0.)
                # A normal turn continues matching; extreme rotation still
                # invalidates the navigation confirmation.
                node.imu.angular_velocity.z=.8
                node.local_confirmations=1
                turning=Future();turning.set_result(dict(accepted=True,candidate=candidate,
                    supported=True,translation_m=0.,rotation_deg=0.,reason='tracking'))
                node.tracking_future=turning;ns['manual_tick'](node)
                self.assertGreater(node.local_good_at,0.)
                node.imu.angular_velocity.z=1.3;ns['manual_tick'](node)
                self.assertEqual(node.local_good_at,0.)


if __name__=='__main__':unittest.main()
