import math
from pathlib import Path
import sys
from types import SimpleNamespace as N
import unittest
from unittest.mock import patch
from concurrent.futures import Future
from queue import SimpleQueue
import tempfile
import threading
import time
import json
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'isv_water_trials'))
from isv_water_trials.tank_core import Grid, guidance, pwm, wrap
from isv_water_trials.indoor_analyze import fit_channel
from telemetry_core import quaternion_angles, vectornav_value, cloud_value, GpsTrack
from tank_feed import TankFeed
from jetson_server import Store


def message(cells, angle=0.):
    return N(header=N(frame_id='tank_map'),data=np.array(cells,dtype=np.int8).ravel(),
             info=N(width=len(cells[0]),height=len(cells),resolution=.1,
                    origin=N(position=N(x=2.,y=-1.),orientation=N(x=0.,y=0.,z=math.sin(angle/2),w=math.cos(angle/2)))))


class TankTests(unittest.TestCase):
    def test_rotation_and_cell_roundtrip(self):
        grid = Grid(message(np.zeros((30,30)),math.pi/3),.1)
        self.assertEqual(grid.index(grid.point((8,17))),(8,17))
        self.assertFalse(grid.safe((-1,1)))

    def test_unknown_blocks_route(self):
        cells = np.zeros((30,30)); cells[:,15] = -1
        grid = Grid(message(cells),.1)
        with self.assertRaises(ValueError): grid.path(grid.point((5,15)),grid.point((25,15)))

    def test_astar_goes_around_wall(self):
        cells = np.zeros((40,40)); cells[8:30,20] = 100
        grid = Grid(message(cells),.15)
        path = grid.path(grid.point((10,20)),grid.point((30,20)))
        self.assertGreater(len(path),2)
        self.assertTrue(all(grid.line_clear(a,b) for a,b in zip(path[:-1],path[1:])))

    def test_guidance_is_bounded_and_turns_ccw(self):
        for angle in np.linspace(-math.pi,math.pi,100):
            left,right = guidance((0,0),0,(math.cos(angle),math.sin(angle)),20)
            self.assertLessEqual(max(abs(left),abs(right)),20.000001)
        left,right = guidance((0,0),0,(0,1),20)
        self.assertLess(left,right)
        self.assertEqual(pwm(0),1500)
        self.assertEqual(pwm(20),1580)

    def test_degrees_wrap(self):
        q = N(x=0.,y=0.,z=math.sin(-math.pi/4),w=math.cos(-math.pi/4))
        self.assertAlmostEqual(quaternion_angles(q)['deg_360']['yaw'],270)

    def test_point_decode_preserves_rings_and_finite(self):
        fields = [N(name=k,offset=o,datatype=t,count=1) for k,o,t in [('x',0,7),('y',4,7),('z',8,7),('ring',12,4)]]
        dtype = np.dtype({'names':['x','y','z','ring'],'formats':['<f4','<f4','<f4','<u2'],'offsets':[0,4,8,12],'itemsize':16})
        data = np.zeros(16*100,dtype=dtype); data['x']=2; data['ring']=np.tile(np.arange(16),100); data['x'][0]=np.nan
        msg = N(fields=fields,width=1600,height=1,point_step=16,row_step=25600,is_bigendian=False,data=data.tobytes(),header=N(frame_id='velodyne'))
        payload,meta = cloud_value(msg,500,100)
        decoded = np.frombuffer(payload,dtype='<f4').reshape(-1,4)
        self.assertEqual(len(set(decoded[:,3])),16)
        self.assertLessEqual(len(decoded),500)
        self.assertTrue(np.isfinite(decoded).all())

    def test_response_model_needs_excitation(self):
        t = np.arange(0,20,.1)
        result,_ = fit_channel(t,np.zeros(len(t)),np.zeros(len(t)))
        self.assertFalse(result['qualified'])

    def test_response_model_coefficients_and_holdout(self):
        t = np.arange(0,60,.1); q = .2*np.sin(t*.35)+.1*np.sin(t*.9)
        v = np.zeros(len(t))
        for i in range(len(t)-1): v[i+1]=v[i]+.1*(-.7*v[i]+.8*q[i])
        result,trace = fit_channel(t,v,q)
        self.assertTrue(result['qualified'],result)
        self.assertAlmostEqual(result['decay_per_s'],.7,delta=.08)
        self.assertAlmostEqual(result['pwm_gain'],.8,delta=.08)
        self.assertGreater(len(trace),20)


class MapApiTests(unittest.TestCase):
    def setUp(self):
        self.feed = TankFeed.__new__(TankFeed)
        self.feed.store = Store(N(max_points=5000,cloud_hz=5.,max_range=100.))
        self.feed.store.value('tank_session',{'session_id':'test'})
        self.feed.grid = message(np.zeros((40,40)))
        self.feed.pending = SimpleQueue(); self.feed.lock = threading.RLock()
        self.sent = []
        self.feed.node = N(get_subscriptions_info_by_topic=lambda _: [N(node_name='kabot_sensor_monitor')],
                           get_clock=lambda:N(now=lambda:N(to_msg=lambda:None)))
        self.feed.publisher = N(publish=self.sent.append)
        self.feed.pose_type = lambda:N(header=N(),pose=N(pose=N(position=N(),orientation=N()),covariance=[0.]*36))

    def request(self,path,data):
        future = Future(); self.feed.pending.put((path,data,future)); self.feed.work()
        return future.result()

    def test_route_saves_without_propulsion(self):
        with tempfile.TemporaryDirectory() as directory, patch('tank_feed.Path.home',return_value=Path(directory)):
            status,_ = self.request('/api/route',{'a':{'x':3.05,'y':.05},'b':{'x':4.75,'y':1.75}})
            self.assertEqual(status,200)
            route = json.loads((Path(directory)/'water_trials/tank/route.json').read_text())
            self.assertEqual(route['session_id'],'test')
            self.assertEqual(self.sent,[])

    def test_monitor_does_not_count_as_localization(self):
        status,_ = self.request('/api/initial_pose',{'x':3.,'y':0.,'yaw_deg':90})
        self.assertEqual(status,400); self.assertEqual(self.sent,[])

    def test_localization_orientation_degrees(self):
        self.feed.node.get_subscriptions_info_by_topic = lambda _:[N(node_name='slam_toolbox')]
        status,_ = self.request('/api/initial_pose',{'x':3.,'y':0.,'yaw_deg':90})
        self.assertEqual(status,200)
        self.assertAlmostEqual(self.sent[0].pose.pose.orientation.z,math.sqrt(.5))

    def test_reposition_is_blocked_during_execution(self):
        self.feed.store.value('tank_status',{'execute':True})
        status,_ = self.request('/api/initial_pose',{'x':3.,'y':0.,'yaw_deg':90})
        self.assertEqual(status,400); self.assertEqual(self.sent,[])

    def test_candidate_selection_uses_latest_server_pose(self):
        self.feed.node.get_subscriptions_info_by_topic=lambda _:[N(node_name='slam_toolbox')]
        self.feed.node.get_clock=lambda:N(now=lambda:N(nanoseconds=100_000_000_000,to_msg=lambda:None))
        self.feed.store.value('tank_localization_candidates',dict(map_id='test',live_candidates=[
            dict(candidate_id='stable-id',x=3.5,y=.5,yaw_deg=120.,stamp_s=99.9)]))
        status,_=self.request('/api/initial_pose',dict(map_id='test',candidate_id='stable-id',x=999,y=999,yaw_deg=0))
        self.assertEqual(status,200)
        self.assertEqual(self.sent[0].pose.pose.position.x,3.5)
        self.assertAlmostEqual(self.sent[0].pose.pose.orientation.z,math.sin(math.pi/3))
        for data in [dict(map_id='old-map',candidate_id='stable-id'),dict(map_id='test',candidate_id='removed')]:
            self.assertEqual(self.request('/api/initial_pose',data)[0],400)
        self.feed.store.values['tank_localization_candidates']['live_candidates'][0]['stamp_s']=98.
        self.assertEqual(self.request('/api/initial_pose',dict(map_id='test',candidate_id='stable-id'))[0],400)
        self.assertEqual(len(self.sent),1)


if __name__=='__main__': unittest.main()
