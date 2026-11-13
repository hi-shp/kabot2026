import ast
import json
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace as N
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'isv_water_trials'))
from isv_water_trials.tank_global_match import DisplayTrack,MotionHistory,wrap
from tank_feed import position_edit_blocked


def methods(*names):
    path=Path(__file__).resolve().parents[2]/'isv_water_trials/isv_water_trials/tank_localize.py'
    cls=next(n for n in ast.walk(ast.parse(path.read_text(encoding='utf-8'))) if isinstance(n,ast.ClassDef) and n.name=='Localizer')
    ns=dict(math=math,time=time,json=json,DisplayTrack=DisplayTrack,wrap=wrap,String=lambda **kw:N(**kw))
    exec(compile(ast.Module(body=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in names],type_ignores=[]),str(path),'exec'),ns)
    return ns


class RcLocalizationTests(unittest.TestCase):
    def test_reposition_requires_fresh_rc_while_trial_runs(self):
        now=time.monotonic()
        values=dict(tank_status=dict(execute=True,received_monotonic=now))
        self.assertTrue(position_edit_blocked(values))
        values['actuators']=dict(mode=1,receiving=True,received_monotonic=now)
        self.assertFalse(position_edit_blocked(values))
        values['actuators']['received_monotonic']=now-1
        self.assertTrue(position_edit_blocked(values))
        values['actuators'].update(mode=2,received_monotonic=now)
        self.assertTrue(position_edit_blocked(values))

    def test_single_odom_spike_does_not_destroy_manual_reference(self):
        ns=methods('receive_odom')
        node=N(odom=(10.,(1.,2.,0.)),manual_locked=True,odom_outliers=0,motion=MotionHistory())
        def msg(t,x):return N(header=N(frame_id='tank_odom',stamp=N(sec=10,nanosec=round(t*1e9))),child_frame_id='velodyne',pose=N(pose=N(position=N(x=x,y=2.),orientation=N(x=0.,y=0.,z=0.,w=1.))))
        ns['receive_odom'](node,msg(.1,20.))
        self.assertEqual(node.odom[1],(1.,2.,0.))
        ns['receive_odom'](node,msg(.2,1.02))
        self.assertEqual(node.odom_outliers,0)
        self.assertFalse(getattr(node,'manual_odom_reset',False))
        self.assertEqual(node.odom[1],(1.02,2.,0.))

    def test_missing_tracking_keeps_last_arrow_and_trail_without_fresh_stamp(self):
        ns=methods('broadcast');sent=[]
        pose=dict(x=1.,y=2.,yaw_deg=90.,stamp_s=8.)
        node=N(odom=(8.,(0,0,0)),session=dict(session_id='map'),map_id='map',session_at=time.monotonic(),
               get_clock=lambda:N(now=lambda:N(nanoseconds=10_000_000_000)),display=DisplayTrack(),motion=MotionHistory(),
               last_display=pose,review_pose=None,manual_locked=True,trail_key=None,
               report=dict(display_trail_m=[[1.,1.],[1.,2.]]),background_period=60.,review_period=2.,status=N(publish=sent.append))
        ns['broadcast'](node)
        report=json.loads(sent[0].data)
        self.assertTrue(report['display_pose']['held'])
        self.assertEqual(report['display_pose']['stamp_s'],8.)
        self.assertFalse(report['tracking_valid'])
        self.assertEqual(report['display_trail_m'],[[1.,1.],[1.,2.]])

    def test_odom_reset_rebases_last_pose_but_requires_map_confirmation(self):
        ns=methods('receive_odom');track=DisplayTrack()
        c=dict(x=4.,y=5.,yaw_deg=0.,score=.9,inliers=1.,known=1.,free=1.)
        track.adopt(dict(candidates=[c]),(1.,2.,0.),(1.,2.,0.),10.)
        node=N(odom=(10.,(1.,2.,0.)),manual_locked=True,odom_outliers=0,motion=MotionHistory(),display=track,
               generation=0,evidence=N(clear=lambda:None),local_good_at=time.monotonic(),local_confirmations=5)
        for i in range(1,4):
            msg=N(header=N(frame_id='tank_odom',stamp=N(sec=10,nanosec=i*100000000)),child_frame_id='velodyne',
                  pose=N(pose=N(position=N(x=50.,y=20.),orientation=N(x=0.,y=0.,z=0.,w=1.))))
            ns['receive_odom'](node,msg)
        pose=track.pose(node.odom[1])
        self.assertAlmostEqual(pose['x'],4.);self.assertAlmostEqual(pose['y'],5.)
        self.assertFalse(node.manual_odom_reset)
        self.assertEqual(node.local_good_at,0.);self.assertEqual(node.local_confirmations,0)


if __name__=='__main__':unittest.main()
