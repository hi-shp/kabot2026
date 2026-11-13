import ast
import json
import math
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace as N
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'isv_water_trials'))
from isv_water_trials.tank_global_match import DisplayTrack,identify_candidates


class LocalizationControlsTests(unittest.TestCase):
    def test_manual_navigation_uses_anchor_tracking_not_slam(self):
        from isv_water_trials.tank_pose import known_position
        path=Path(__file__).resolve().parents[2]/'isv_water_trials/isv_water_trials/tank_pose.py'
        cls=next(n for n in ast.walk(ast.parse(path.read_text(encoding='utf-8'))) if isinstance(n,ast.ClassDef) and n.name=='TankPose')
        def message():return N(header=N(stamp=N()),pose=N(pose=N(position=N(),orientation=N())))
        ns=dict(math=math,time=time,json=json,known_position=known_position,Odometry=message,PoseWithCovarianceStamped=message)
        methods=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in ('manual_tracking','matched','update')]
        exec(compile(ast.Module(body=methods,type_ignores=[]),str(path),'exec'),ns)
        sent=[];node=N(manual_locked=False,seed=N(publish=lambda _:None),pose_pub=N(publish=sent.append),
                      get_parameter=lambda _:N(value='map1'),get_clock=lambda:N(now=lambda:N(nanoseconds=10_000_000_000,to_msg=lambda:None)),
                      map_info=dict(width=10,height=10,resolution=1.,position=[0.,0.,0.],orientation=[0.,0.,0.,1.]),map_cells=bytes([0]*100))
        report=dict(map_id='map1',manual_locked=True,map_tracking_ready=True,display_pose=dict(x=3.,y=4.,yaw_deg=90.,stamp_s=9.9))
        ns['manual_tracking'](node,N(data=json.dumps(report)))
        ns['matched'](node,None) # Any SLAM correction is ignored while locked.
        ns['update'](node) # No TF object exists; the manual branch must be independent.
        self.assertTrue(node.ready);self.assertEqual(sent[0].pose.pose.position.x,3.)
        self.assertAlmostEqual(sent[0].pose.pose.orientation.z,math.sqrt(.5))
        node.manual_report['map_tracking_ready']=False
        ns['update'](node)
        self.assertFalse(node.ready);self.assertEqual(len(sent),1)
        node.manual_report['map_tracking_ready']=True
        node.manual_report['display_pose']['stamp_s']=8.
        ns['update'](node)
        self.assertTrue(node.manual_locked);self.assertFalse(node.ready);self.assertEqual(len(sent),1)

    def test_manual_selection_hides_candidates_and_blocks_search_until_request(self):
        path=Path(__file__).resolve().parents[2]/'isv_water_trials/isv_water_trials/tank_localize.py'
        tree=ast.parse(path.read_text(encoding='utf-8'))
        cls=next(n for n in ast.walk(tree) if isinstance(n,ast.ClassDef) and n.name=='Localizer')
        methods=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in ('initial_seen','tick','request')]
        namespace=dict(time=time,math=math,threading=threading,DisplayTrack=DisplayTrack,identify_candidates=identify_candidates,wrap=lambda x:(x+math.pi)%(2*math.pi)-math.pi)
        exec(compile(ast.Module(body=methods,type_ignores=[]),str(path),'exec'),namespace)
        reports=[];node=N(sent_stamp=None,odom=(1.,(0,0,0)),generation=0,pending=object(),future=object(),
                         cancel=threading.Event(),background_period=60.,display=DisplayTrack(),publish=lambda *a,**k:reports.append((a,k)))
        msg=N(header=N(frame_id='tank_map',stamp=N(sec=1,nanosec=0)),pose=N(pose=N(position=N(x=5.,y=6.),orientation=N(x=0.,y=0.,z=math.sin(math.pi/4),w=math.cos(math.pi/4)))))
        namespace['initial_seen'](node,msg)
        self.assertTrue(node.manual_locked);self.assertFalse(node.requested);self.assertTrue(node.cancel.is_set())
        self.assertEqual(node.review_candidates,[]);self.assertEqual(reports[-1][1]['candidates'],[])
        pose=node.display.pose((2,0,0));self.assertAlmostEqual(pose['x'],5.);self.assertAlmostEqual(pose['y'],8.)
        node.future=None;node.manual_tick=lambda:node.publish('MANUAL_TRACKING','local correction only')
        namespace['tick'](node)
        self.assertEqual(reports[-1][0][0],'MANUAL_TRACKING')
        namespace['request'](node,None)
        self.assertFalse(node.manual_locked);self.assertTrue(node.requested)
        self.assertEqual(len(node.review_candidates),1)

    def test_candidate_identity_survives_rank_swap(self):
        before=identify_candidates([dict(x=1,y=2,yaw_deg=359),dict(x=6,y=7,yaw_deg=90)],[])
        after=identify_candidates([dict(x=6.1,y=7,yaw_deg=91),dict(x=1.1,y=2,yaw_deg=1)],before)
        self.assertEqual(after[0]['candidate_id'],before[1]['candidate_id'])
        self.assertEqual(after[1]['candidate_id'],before[0]['candidate_id'])
        self.assertNotEqual(identify_candidates([dict(x=40,y=20,yaw_deg=0)],before)[0]['candidate_id'],before[0]['candidate_id'])

    def test_request_preserves_pose_history_and_starts_local_review(self):
        path=Path(__file__).resolve().parents[2]/'isv_water_trials/isv_water_trials/tank_localize.py'
        tree=ast.parse(path.read_text(encoding='utf-8'))
        cls=next(n for n in ast.walk(tree) if isinstance(n,ast.ClassDef) and n.name=='Localizer')
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='request')
        namespace=dict(time=time,math=math,DisplayTrack=DisplayTrack,identify_candidates=identify_candidates,wrap=lambda x:(x+math.pi)%(2*math.pi)-math.pi)
        exec(compile(ast.Module(body=[method],type_ignores=[]),str(path),'exec'),namespace)
        track=DisplayTrack();candidate=dict(x=3,y=2,yaw_deg=90,score=.9,inliers=.9,known=1.,free=1.)
        track.adopt(dict(candidates=[candidate]),(0,0,0),(0,0,0),1.)
        evidence=object();motion=object();published=[]
        node=N(generation=0,pending=None,confirmations=0,requested=True,future=object(),cancel=threading.Event(),
               odom=(2.,(1,0,0)),display=track,review_candidates=identify_candidates([candidate],[]),review_pose=(0,0,0),
               background_period=60.,evidence=evidence,motion=motion,publish=lambda *a:published.append(a))
        before=track.pose(node.odom[1]);namespace['request'](node,None)
        self.assertEqual(track.pose(node.odom[1]),before)
        self.assertIs(node.evidence,evidence);self.assertIs(node.motion,motion)
        self.assertEqual(node.next_review,0);self.assertGreater(node.next_search,time.monotonic()+50)
        self.assertTrue(node.cancel.is_set());self.assertEqual(len(node.review_candidates),1)
        self.assertAlmostEqual(node.review_candidates[0]['y'],3.)


if __name__=='__main__':unittest.main()
