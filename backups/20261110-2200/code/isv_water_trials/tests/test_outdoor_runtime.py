"""Run actual ROS-node methods against in-memory IO; no hardware calls."""
import ast
from concurrent.futures import Future
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace as N
import unittest
from unittest.mock import Mock
import numpy as np
from isv_water_trials import outdoor
from isv_water_trials.outdoor_model import ResponseModel, local_xy
from test_outdoor import CONFIG, known_model


def node_type(root):
    tree=ast.parse(Path(outdoor.__file__).read_text(encoding='utf-8'))
    main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
    cls=next(n for n in main.body if isinstance(n,ast.ClassDef))
    cls.bases=[ast.Name(id='object',ctx=ast.Load())]
    cls.body=[n for n in cls.body if not isinstance(n,ast.FunctionDef) or n.name!='__init__']
    scope=dict(vars(outdoor),root=Path(root),config=CONFIG,String=lambda **kw:N(**kw),SetBool=N(Request=lambda **kw:N(**kw)),ThrottlePulseWidth=N(Request=lambda **kw:N(**kw)))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[cls],type_ignores=[])),'outdoor-node-methods','exec'),scope)
    return scope['Outdoor']


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.n=node_type(self.tmp.name)();n=self.n;now=time.monotonic()
        n.store=Mock();n.state=Mock();n.commands=[];n.model=ResponseModel(known_model())
        n.lineage=None;n.baseline_model=None;n.validation_model=None
        n.store.fork_learning.return_value={'id':'branch','parent_model_id':n.model.data['id'],'started_t':100.,'data_cutoff_t':99.}
        n.execute=False;n.frozen=False;n.learning=True;n.fit_generation=0;n.plan_generation=0;n.fit_status='';n.ack=None;n.error=''
        n.get_node_names=lambda:[];n.pending=[];n.pending_at=0.;n.plan_result=None;n.command=[.2,.2];n.command_at=now
        n.route={'points':[[35,129],[35.0001,129],[35.0002,129]]};n.goal=0;n.arrivals=0;n.trail=[]
        n.fit_job=None;n.fit_at=now;n.plan_job=None;n.plan_at=now;n.last_disk=now;n.fit_pool=Mock();n.plan_pool=Mock()
        n.rows=[];n.disk_ok=True;n.ready_since=now-3;n.scan={'stamp':100};n.scan_at=now;n.actuator_at=now
        n.actuator={'receiving':True,'age_s':.01,'observed_left_us':1480,'observed_right_us':1480}
        n.state.origin=[35,129];n.state.pose_at.return_value={'position':[0,0],'yaw':0}
        n.state.snapshot.return_value={'position':[0,0],'yaw':0,'velocity':[0,0,0],'position_sigma_m':.5}
        n.sent=[];n.send=lambda q,now,disarm=False:n.sent.append((list(q),disarm));n.ros_now=lambda:100.

    def request(self,action,**kw):
        self.n.request(N(data=json.dumps(dict(action=action,request_id='test',**kw))))
        return self.n.ack

    def test_passive_bringup_never_sends_actuator_commands(self):
        self.n.tick();self.assertFalse(self.n.sent)

    def test_pause_keeps_history_and_does_not_reboot_sensors(self):
        self.n.rows=[{'keep':True}];self.n.trail=[[1,2,3,4]]
        self.assertTrue(self.request('pause')['ok']);self.assertFalse(self.n.learning)
        self.assertEqual(len(self.n.rows),1);self.assertEqual(len(self.n.trail),1)
        self.n.state.reset.assert_not_called()

    def test_resume_keeps_validation_and_running_trial(self):
        self.n.execute=True;self.n.learning=False;self.n.validation_model=self.n.model
        self.assertTrue(self.request('resume')['ok']);self.assertTrue(self.n.learning)
        self.assertTrue(self.n.execute);self.n.state.reset.assert_not_called()
        self.assertIs(self.n.validation_model,self.n.model)

    def test_apply_keeps_learning_and_late_fit_cannot_replace_validation(self):
        future=Future();new=known_model();new['id']='fedcba9876543210';future.set_result(new)
        self.n.fit_job=future;self.n.fit_job_generation=0
        self.assertTrue(self.request('apply_model')['ok']);self.n.tick()
        self.assertTrue(self.n.learning)
        self.assertEqual(self.n.validation_model.data['id'],'0123456789abcdef')
        self.assertEqual(self.n.model.data['id'],'fedcba9876543210')

    def test_apply_requires_qualified_model_and_stopped_trial(self):
        self.n.model.data['qualified']=False
        self.assertFalse(self.request('apply_model')['ok'])
        self.n.model.data['qualified']=True;self.n.execute=True
        self.assertFalse(self.request('apply_model')['ok']);self.assertIsNone(self.n.validation_model)

    def test_start_uses_validation_not_current_model(self):
        self.assertTrue(self.request('apply_model')['ok'])
        new=known_model();new['id']='fedcba9876543210';self.n.model=ResponseModel(new)
        self.assertTrue(self.request('start',percent=20)['ok'])
        self.assertEqual(self.n.planner.model.data['id'],'0123456789abcdef')

    def test_model_test_requires_area_keeps_learning_and_restores_route(self):
        from isv_water_trials.outdoor_model import latlon
        n=self.n;self.request('apply_model');before=json.loads(json.dumps(n.route))
        self.assertFalse(self.request('start_model_test',percent=20)['ok'])
        n.scan=dict(stamp=100,angles=np.linspace(-np.pi,np.pi,720,endpoint=False),ranges=np.full(720,np.inf),range_min=.1,range_max=30.)
        area=[latlon(x,y,n.state.origin) for x,y in [(-12,-12),(12,-12),(12,12),(-12,12)]]
        self.assertFalse(self.request('start_model_test',percent=101,area=area)['ok'])
        self.assertTrue(self.request('start_model_test',percent=20,area=area)['ok'],n.ack)
        self.assertTrue(n.execute);self.assertTrue(n.learning)
        self.assertEqual(n.test_course['model_id'],n.validation_model.data['id'])
        self.assertNotEqual(n.route,before)
        n.plan_at=0;n.actuator_at=n.scan_at=time.monotonic()
        n.tick()
        self.assertEqual(n.plan_pool.submit.call_args.args[1]['test_area'],n.test_course['area'])
        n.state.snapshot.return_value=None;n.plan_job=None;n.tick()
        self.assertTrue(n.execute);self.assertEqual(n.sent[-1][0],[0,0])
        self.assertTrue(self.request('stop')['ok'])
        self.assertFalse(n.test_course['running']);self.assertEqual(n.route,before)

    def test_route_is_cyclic_one_two_three_one(self):
        n=self.n;n.execute=True;seen=[]
        for i in (0,1,2,0):
            n.state.snapshot.return_value={'position':local_xy(*n.route['points'][i],n.state.origin),'yaw':0,'velocity':[0,0,0],'position_sigma_m':.5}
            n.tick();seen.append(n.goal)
        self.assertEqual(seen,[1,2,0,1]);self.assertEqual(n.arrivals,4)

    def test_sensor_loss_keeps_process_and_sends_neutral(self):
        n=self.n;n.execute=True;n.state.snapshot.return_value=None;n.tick()
        self.assertTrue(n.execute);self.assertEqual(n.sent[-1][0],[0,0])

    def test_stop_discards_planner_result_then_disarms(self):
        n=self.n;n.execute=True;n.plan_job_generation=0;n.plan_started=time.monotonic()
        n.plan_job=Future();n.plan_job.set_result({'command':[.2,.2],'path':[]})
        self.request('stop');n.tick()
        self.assertFalse(n.execute);self.assertIsNone(n.plan_result)
        self.assertEqual(n.sent[-1],([0.,0.],True))

    def test_conflicting_motor_program_blocks_start(self):
        self.n.validation_model=self.n.model;self.n.get_node_names=lambda:['rc_auto_mode_test']
        self.assertFalse(self.request('start',percent=20)['ok']);self.assertFalse(self.n.execute)

    def test_route_edit_stops_before_new_route(self):
        self.n.execute=True
        ack=self.request('route',points=[[35,129],[35.0001,129],[35.0002,129]])
        self.assertTrue(ack['ok']);self.assertFalse(self.n.execute);self.assertEqual(len(self.n.route['points']),3)

    def test_clear_path_does_not_delete_training(self):
        self.n.trail=[[1,2,3,4]];self.n.rows=[{'retained':True}]
        self.assertTrue(self.request('clear_trail')['ok'])
        self.assertEqual(self.n.trail,[]);self.assertEqual(len(self.n.rows),1)

    def test_late_sensor_header_is_rejected(self):
        self.n.ros_now=lambda:100.
        with self.assertRaises(ValueError):self.n.stamp(N(header=N(stamp=N(sec=95,nanosec=0))))


if __name__=='__main__':unittest.main()
