"""Relative-frame invariants and safety state transitions without hardware I/O."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from concurrent.futures import Future
import numpy as np
import yaml
from docking_mapping.geometry import transform,apply
from docking_mapping.tracking import DockTracker,relative_goal
from docking_mapping.registration import Result
from docking_mapping.control import DockingControl,auto_ready,validate_config
from docking_mapping.actuation import DockingOutput
from docking_mapping.safety import corridor_clear
from docking_mapping.hopping import HoppingPoint
from docking_mapping.gui_bridge import MappingBridge
import test_hopping


def settings():
    return yaml.safe_load((Path(__file__).resolve().parents[1]/'config/mapping.yaml').read_text())['autonomy']


class Turn:
    def reset(self):pass
    def adjust(self,pwm,*args,**kwargs):return pwm


def controller(config=None):
    return DockingControl(config or settings(),lambda pos,heading,target,percent:(percent,percent),
                          lambda p:tuple(round(1500+x*4) for x in p),Turn())


AUTO=dict(mode=2,receiving=True,estop_active=False,stop_latched=False)


def sample(now,distance=0.,heading=0.,base_from_dock=None):
    return SimpleNamespace(valid=True,stamp=now,goal=dict(point_base=[distance,0.,0.],distance_m=distance,
        bearing_deg=0.,heading_error_deg=heading),base_from_dock=np.eye(4) if base_from_dock is None else base_from_dock)


def hop(hold=.2):return SimpleNamespace(radius_m=1.,hold_seconds=hold)


class TrackingTests(unittest.TestCase):
    def fixture(self):
        mapper,ref,hop,_=test_hopping.HoppingTests().fixture()
        tracker=DockTracker(ref,hop,np.eye(4),settings()['tracking'])
        return mapper,ref,hop,tracker

    def test_gps_jump_moves_only_display_anchor_and_preserves_reference(self):
        _,ref,hop,t=self.fixture();points=ref.points.copy();coordinates=hop.point_dock
        matrix=ref.enu_from_dock.copy();initial=[]
        def fit(source,target,guess,config):
            initial.append(guess.copy());return Result(matrix.copy(),True,'OK')
        with patch('docking_mapping.tracking.align',side_effect=fit):
            self.assertTrue(t.process(ref.points_enu(),10.,np.eye(4),np.eye(3)))
            goal=deepcopy(t.goal);world=t.world_from_dock.copy()
            jumped=transform(translation=[2.,3.,0.])
            self.assertTrue(t.process(ref.points_enu(),10.5,jumped,np.eye(3)))
        self.assertEqual(t.goal,goal)
        np.testing.assert_allclose(initial[1],initial[0],atol=1e-10)
        np.testing.assert_allclose(t.world_from_dock[:3,3]-world[:3,3],[2.,3.,0.])
        before=apply(ref.points,world);after=apply(ref.points,t.world_from_dock)
        np.testing.assert_allclose(after-before,np.tile([2.,3.,0.],(len(before),1)),atol=1e-10)
        hp0=apply([coordinates],world);hp1=apply([coordinates],t.world_from_dock)
        np.testing.assert_allclose(hp1-hp0,[[2.,3.,0.]],atol=1e-10)
        np.testing.assert_array_equal(ref.points,points);self.assertEqual(hop.point_dock,coordinates)

    def test_real_gicp_tracks_fixed_source_and_moving_structure(self):
        _,ref,hop,t=self.fixture()
        self.assertTrue(t.process(ref.points_enu(),10.,np.eye(4),np.eye(3)),t.metrics)
        before=np.array(t.goal['point_base'])
        moved=apply(ref.points,transform(translation=[.12,-.08,.03])@ref.enu_from_dock)
        self.assertTrue(t.process(moved,10.5,np.eye(4),np.eye(3)),t.metrics)
        np.testing.assert_allclose(np.array(t.goal['point_base'])-before,[.12,-.08,.03],atol=.04)

    def test_invalid_partial_cloud_and_reacquisition_gap(self):
        _,ref,hop,t=self.fixture()
        self.assertTrue(t.process(ref.points_enu(),10.,np.eye(4),np.eye(3)))
        self.assertFalse(t.process(np.empty((0,3)),10.5,np.eye(4),np.eye(3)))
        self.assertFalse(t.valid);self.assertIsNone(t.goal)
        with self.assertRaisesRegex(ValueError,'EXPLICIT_RESTART'):t.process(ref.points_enu(),15.,np.eye(4),np.eye(3))

    def test_initial_requires_gps_but_relative_tracking_can_continue_without_it(self):
        _,ref,hop,t=self.fixture()
        with self.assertRaisesRegex(ValueError,'GPS'):t.process(ref.points_enu(),9.,None,np.eye(3))
        self.assertTrue(t.process(ref.points_enu(),10.,np.eye(4),np.eye(3)))
        self.assertTrue(t.process(ref.points_enu(),10.5,None,np.eye(3)))
        self.assertIsNone(t.world_from_dock);self.assertTrue(t.valid)

    def test_base_extrinsic_and_ccw_heading(self):
        h=HoppingPoint('id',1,(2.,1.,0.),heading_dock_deg=90.)
        goal=relative_goal(transform(translation=[1.,0.,0.]),h)
        self.assertAlmostEqual(goal['distance_m'],np.sqrt(10))
        self.assertAlmostEqual(goal['bearing_deg'],np.degrees(np.arctan2(1,3)))
        self.assertAlmostEqual(goal['heading_error_deg'],90.)


class ControlTests(unittest.TestCase):
    def tick(self,c,now,distance=0.,heading=0.,**kwargs):
        return c.tick(now,AUTO,0.,sample(now,distance,heading),hop(),**kwargs)

    def test_neutral_boot_and_wait_timeout(self):
        c=controller();self.assertEqual(self.tick(c,0.),(1500,1500));self.assertEqual(c.state,'IDLE')
        c.start(0.);c.tick(.1,AUTO,0.,None,hop());self.assertEqual(c.state,'WAIT_FOR_REGISTRATION')
        c.tick(11.,AUTO,0.,None,hop());self.assertEqual(c.state,'TRACKING_LOST')

    def test_hold_completes_without_reverse_and_no_auto_restart(self):
        c=controller();c.start(0.);self.tick(c,.1);self.assertEqual(c.state,'HOPING')
        self.tick(c,.4);self.assertEqual(c.state,'DONE');self.assertEqual(c.pwm,(1500,1500))
        self.tick(c,.5,distance=5.,forward_clear=True);self.assertEqual(c.state,'DONE')

    def test_position_or_heading_violation_resets_hold_timer(self):
        for distance,heading in [(2.,0.),(0.,20.)]:
            c=controller();c.start(0.);self.tick(c,.1)
            self.tick(c,.15,distance=distance,heading=heading,forward_clear=True)
            self.assertIsNone(c.hold_since);self.assertEqual(c.state,'APPROACH_HOPPING_POINT')
            self.tick(c,.2);self.assertEqual(c.hold_since,.2)
            self.tick(c,.3);self.assertEqual(c.state,'HOPING')

    def test_stale_or_rejected_registration_neutral_latches_loss(self):
        for reading in (None,sample(0.)):
            c=controller();c.start(0.);self.tick(c,.1)
            c.tick(2.,AUTO,0.,reading,hop(),forward_clear=True)
            self.assertEqual(c.state,'TRACKING_LOST');self.assertEqual(c.pwm,(1500,1500))
            self.tick(c,2.1);self.assertEqual(c.state,'TRACKING_LOST')

    def test_manual_estop_mcu_loss_and_fault_immediately_neutral(self):
        for telemetry,age,healthy,state in [(dict(AUTO,mode=1),0.,True,'ABORT'),
             (dict(AUTO,estop_active=True),0.,True,'EMERGENCY'),(AUTO,1.,True,'ABORT'),(AUTO,0.,False,'ABORT')]:
            c=controller();c.start(0.);self.tick(c,.1,distance=5.,forward_clear=True)
            c.tick(.2,telemetry,age,sample(.2),hop(),hardware_healthy=healthy)
            self.assertEqual(c.state,state);self.assertEqual(c.pwm,(1500,1500))

    def test_unknown_forward_stops_and_pwm_has_low_limit_and_slew(self):
        c=controller();c.start(0.);self.tick(c,.1,distance=5.)
        self.assertEqual(c.state,'ABORT')
        c.start(1.);previous=(1500,1500)
        for now in np.arange(1.1,2.,.1):
            pwm=self.tick(c,float(now),distance=5.,forward_clear=True)
            self.assertLessEqual(max(abs(p-1500) for p in pwm),40)
            self.assertLessEqual(max(abs(p-q) for p,q in zip(pwm,previous)),4)
            previous=pwm

    def reverse_config(self):
        c=settings();c['reverse'].update(enabled=True,path_verified=True,heading_dock_deg=0.)
        return c

    def test_reverse_requires_observed_rear_and_heading(self):
        c=controller(self.reverse_config());c.start(0.);self.tick(c,.1)
        self.tick(c,.4,rear_clear=False);self.assertEqual(c.state,'ABORT')
        c=controller(self.reverse_config());c.start(0.);self.tick(c,.1)
        self.tick(c,.4,rear_clear=True);self.assertEqual(c.state,'DOCKING_REVERSE')
        self.assertLess(c.pwm[0],1500)
        self.tick(c,.5,rear_clear=False);self.assertEqual(c.state,'ABORT');self.assertEqual(c.pwm,(1500,1500))

    def test_reverse_relative_distance_done_and_time_limit_abort(self):
        c=controller(self.reverse_config());c.start(0.);self.tick(c,.1);self.tick(c,.4,rear_clear=True)
        # Boat x=-.6 in dock frame => T_base_from_dock translation +.6.
        reading=sample(.5,base_from_dock=transform(translation=[.6,0.,0.]))
        c.tick(.5,AUTO,0.,reading,hop(),rear_clear=True)
        self.assertEqual(c.state,'DONE');self.assertAlmostEqual(c.reverse_progress_m,.6)
        c=controller(self.reverse_config());c.start(0.);self.tick(c,.1);self.tick(c,.4,rear_clear=True)
        self.tick(c,6.,rear_clear=True);self.assertEqual(c.state,'ABORT')

    def test_reverse_wrong_direction_or_lateral_drift_aborts(self):
        for translation in ([-.4,0.,0.],[0.,.4,0.]):
            c=controller(self.reverse_config());c.start(0.);self.tick(c,.1);self.tick(c,.4,rear_clear=True)
            c.tick(.5,AUTO,0.,sample(.5,base_from_dock=transform(translation=translation)),hop(),rear_clear=True)
            self.assertEqual(c.state,'ABORT')


class GateTests(unittest.TestCase):
    def test_emergency_bypasses_pending_normal_gui_command(self):
        import json
        calls=[]
        bridge=MappingBridge(controls_enabled=True,transport=lambda p:(calls.append(p['action']) or dict(success=True)))
        bridge.update(json.dumps(dict(preview_schema=1,status={},map_points=[],current_scan=[],rc_path=[])))
        bridge.gate.acquire()
        try:
            with self.assertRaises(RuntimeError):bridge.send(dict(action='load'))
            self.assertTrue(bridge.send(dict(action='auto_emergency'))['success'])
            self.assertEqual(calls,['auto_emergency'])
        finally:bridge.gate.release()

    def test_existing_output_driver_pairs_pwm_and_releases_only_after_stop_ack(self):
        # Fake service transport, real shared PairedOutput/CommandLease logic.
        config=settings();config.update(dry_run=False,enable_output=True,hardware_output_authorized=True)
        calls=[]
        class Client:
            def __init__(self,name):self.name=name
            def service_is_ready(self):return True
            def call_async(self,request):
                calls.append((self.name,request));future=Future()
                future.set_result(SimpleNamespace(success=True,status=True));return future
        owner=SimpleNamespace(closed=False)
        def release():owner.closed=True
        owner.close=release
        node=SimpleNamespace(create_client=lambda kind,name:Client(name),get_node_names=lambda:[],get_name=lambda:'docking_mapping')
        with patch('isv_shape_navigation.output_runtime.acquire_output_lock',return_value=owner):
            out=DockingOutput(node,config)
            import time
            out.readiness.since=time.monotonic()-8.
            out.start(AUTO,0.)
            out.publish((1504,1504),True)
            out.tick(AUTO,0.)
            out.tick(dict(AUTO,armed=True),0.)
            self.assertEqual([name for name,_ in calls],['/actuators/arm',
                '/actuators/throttle_left/set_pulse_width','/actuators/throttle_right/set_pulse_width'])
            out.request_stop('RC override')
            self.assertFalse(owner.closed)
            out.tick(dict(AUTO,mode=1),0.)
            self.assertEqual(calls[-1][0],'/actuators/stop');self.assertFalse(owner.closed)
            out.tick(dict(AUTO,mode=1),0.)
            self.assertTrue(owner.closed);self.assertIsNone(out.owner)

    def test_dry_run_constructs_no_hardware_clients_or_output_authority(self):
        node=SimpleNamespace(create_client=lambda *args:(_ for _ in ()).throw(AssertionError('hardware client created')))
        out=DockingOutput(node,settings());out.start(AUTO,0.);out.publish((1540,1540),True);out.tick(AUTO,0.)
        self.assertFalse(out.enabled);self.assertEqual(out.clients,[]);self.assertIsNone(out.owner)

    def test_invalid_configuration_rejected(self):
        for name,value in [('approach_percent',50.),('dry_run','false'),('tracking_timeout_s',float('nan'))]:
            c=settings();c[name]=value
            with self.assertRaises(ValueError):validate_config(c)

    def test_corridor_requires_observation_not_empty_cloud(self):
        c=settings()['corridor'];c['envelope_verified']=True
        self.assertFalse(corridor_clear([],c,True))
        angles=np.linspace(-.4,.4,120)
        points=np.column_stack([-4*np.cos(angles),4*np.sin(angles),np.ones(len(angles))*.5])
        self.assertTrue(corridor_clear(points,c,True))
        self.assertFalse(corridor_clear(np.vstack([points,[-1.2,0.,.5]]),c,True))
        c['envelope_verified']=False;self.assertFalse(corridor_clear(points,c,True))

    def test_auto_telemetry_strict(self):
        self.assertTrue(auto_ready(AUTO,0.))
        for data,age in [(dict(AUTO,mode=1),0.),(dict(AUTO,receiving=False),0.),(AUTO,-1.),(AUTO,.6)]:
            self.assertFalse(auto_ready(data,age))


if __name__=='__main__':unittest.main()
