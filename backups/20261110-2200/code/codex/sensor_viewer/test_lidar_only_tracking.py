import ast
import math
from pathlib import Path
import sys
import time
from concurrent.futures import Future
from types import SimpleNamespace as N
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'isv_water_trials'))
from isv_water_trials.tank_global_match import DisplayTrack


class LidarOnlyTests(unittest.TestCase):
    def setUp(self):
        source=Path(__file__).resolve().parents[2]/'isv_water_trials/isv_water_trials/tank_localize.py'
        cls=next(n for n in ast.walk(ast.parse(source.read_text(encoding='utf-8'))) if isinstance(n,ast.ClassDef) and n.name=='Localizer')
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='manual_tick')
        namespace=dict(math=math,time=time)
        exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),namespace)
        self.tick=namespace['manual_tick']
        track=DisplayTrack()
        candidate=dict(x=4.,y=5.,yaw_deg=90.,score=.95,inliers=1.,known=1.,free=1.)
        track.adopt(dict(candidates=[candidate]),(0.,0.,0.),(0.,0.,0.),99.)
        self.node=N(get_clock=lambda:N(now=lambda:N(nanoseconds=100_000_000_000)),
            odom=(99.9,(.2,0.,0.)),session=dict(session_id='map'),map_id='map',session_at=time.monotonic(),
            map_correction=False,scan=N(header=N(frame_id='velodyne',stamp=N(sec=99,nanosec=900000000)),
            ranges=[3.]*360,range_min=.1,range_max=30.),tracking_future=None,
            display=track,report={},publish=lambda *a,**kw:None)

    def test_only_odometry_changes_current_pose_and_old_trail_stays_fixed(self):
        n=self.node;history=[(0.,0.,0.),(.1,0.,0.)]
        before=n.display.trail(history);revision=n.display.revision
        self.tick(n)
        self.assertTrue(n.lidar_tracking_ready)
        self.assertEqual(n.report['tracking_source'],'lidar_odometry')
        self.assertIsNone(n.report['local_correction'])
        n.odom=(99.95,(.4,0.,.1));self.tick(n)
        self.assertEqual(n.display.revision,revision)
        self.assertEqual(n.display.trail(history),before)
        current=n.display.pose(n.odom[1])
        self.assertAlmostEqual(current['x'],4.)
        self.assertAlmostEqual(current['y'],5.4)
        self.assertAlmostEqual(current['yaw_deg'],90.+math.degrees(.1))

    def test_pending_map_correction_cannot_change_anchor(self):
        future=Future();future.set_result(dict(candidate=dict(x=80.,y=80.,yaw_deg=180.)))
        self.node.tracking_future=future;anchor=self.node.display.anchor
        self.tick(self.node)
        self.assertEqual(self.node.display.anchor,anchor)
        self.assertIsNone(self.node.tracking_future)

    def test_stale_missing_invalid_scan_or_odometry_does_not_authorize_motion(self):
        for problem in ('old_scan','old_odom','few_returns','frame','missing_scan','reset'):
            self.setUp();n=self.node
            if problem=='old_scan':n.scan.header.stamp.sec=98
            if problem=='old_odom':n.odom=(98.,n.odom[1])
            if problem=='few_returns':n.scan.ranges=[float('inf')]*360
            if problem=='frame':n.scan.header.frame_id='wrong'
            if problem=='missing_scan':n.scan=None
            if problem=='reset':n.manual_odom_reset=True
            self.tick(n);self.assertFalse(n.lidar_tracking_ready,problem)


if __name__=='__main__':unittest.main()
