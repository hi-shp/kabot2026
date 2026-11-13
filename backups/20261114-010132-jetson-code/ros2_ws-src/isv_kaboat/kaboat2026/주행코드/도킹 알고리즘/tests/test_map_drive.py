"""Real shared output driver with fake ROS services; no hardware I/O."""
from concurrent.futures import Future
from types import SimpleNamespace as NS
from unittest.mock import patch
import time,unittest
from docking_mapping.map_drive import MapDrive
from docking_mapping.actuation import DockingOutput
from test_mapping import config

AUTO=dict(mode=2,receiving=True,rc_valid=True,estop_active=False,stop_latched=False,armed=False)

def plan(stamp=10.,state='APPROACH_TARGET'):
    return dict(state=state,pwm=[1635,1635] if state=='APPROACH_TARGET' else [1500,1500],
                pose_stamp_s=stamp,imu_stamp_s=stamp,scan_stamp_s=stamp)

class MapDriveTests(unittest.TestCase):
    def setUp(self):
        self.calls=[]
        class Client:
            def __init__(client,name):client.name=name
            def service_is_ready(client):return True
            def call_async(client,request):
                self.calls.append((client.name,request));future=Future()
                future.set_result(NS(success=True,status=True));return future
        self.node=NS(create_client=lambda kind,name:Client(name),get_node_names=lambda:[],get_name=lambda:'docking_mapping')
        c=dict(config()['autonomy'],dry_run=False,enable_output=True,hardware_output_authorized=True)
        self.port=DockingOutput(self.node,c);self.port.readiness.since=time.monotonic()-8.
        self.owner=NS(closed=False)
        self.owner.close=lambda:setattr(self.owner,'closed',True)
        self.owner_patch=patch('isv_shape_navigation.output_runtime.acquire_output_lock',return_value=self.owner);self.owner_patch.start()
        self.gate=MapDrive(1.,.5)
    def tearDown(self):self.owner_patch.stop()
    def start(self):
        self.gate.prepare(self.port);self.gate.start(plan(),AUTO,0.,10.,time.monotonic())
    def tick(self,reading=None,status=None,ros=10.):
        self.gate.tick(plan() if reading is None else reading,AUTO if status is None else status,0.,ros,time.monotonic())
    def test_default_and_prepare_send_no_arm_or_pwm_and_do_not_claim_output(self):
        self.tick();self.assertEqual(self.calls,[])
        self.gate.prepare(self.port)
        for _ in range(5):self.tick()
        self.assertEqual(self.calls,[]);self.assertIsNone(self.port.owner);self.assertFalse(self.gate.requested)
    def test_explicit_start_uses_original_paired_services_and_stop_ack_ownership(self):
        self.start();self.tick();self.tick(status=dict(AUTO,armed=True))
        self.assertEqual([c[0] for c in self.calls],['/actuators/arm','/actuators/throttle_left/set_pulse_width','/actuators/throttle_right/set_pulse_width'])
        self.assertEqual([c[1].pulse_width for c in self.calls[1:]],[1635,1635])
        self.gate.stop('operator stop');self.assertEqual(self.gate.commanded_pwm,(1500,1500))
        self.assertFalse(self.owner.closed)
        self.tick(status=dict(AUTO,armed=True));self.assertEqual(self.calls[-1][0],'/actuators/stop')
        self.assertFalse(self.owner.closed);self.tick();self.assertTrue(self.owner.closed)
        count=len(self.calls)
        for _ in range(4):self.tick()
        self.assertEqual(len(self.calls),count);self.assertFalse(self.gate.requested)
    def test_missing_stale_plan_invalid_rc_manual_and_estop_prevent_arm(self):
        self.gate.prepare(self.port)
        for p,t,age in [(None,AUTO,0.),(plan(5),AUTO,0.),(plan(),dict(AUTO,rc_valid=False),0.),
            (plan(),dict(AUTO,mode=1),0.),(plan(),dict(AUTO,estop_active=True),0.),(plan(),AUTO,1.),
            (plan(state='BLOCKED'),AUTO,0.)]:
            with self.assertRaises(ValueError):self.gate.start(p,t,age,10.,time.monotonic())
        self.assertEqual(self.calls,[]);self.assertIsNone(self.port.owner)
    def test_stale_scan_or_registration_loss_stop_without_automatic_restart(self):
        for failure in ['stale','lost','blocked','arrival','manual','rc_invalid','estop']:
            with self.subTest(failure=failure):
                self.calls.clear();self.owner.closed=False
                self.port=DockingOutput(self.node,dict(config()['autonomy'],dry_run=False,enable_output=True,hardware_output_authorized=True))
                self.port.readiness.since=time.monotonic()-8.;self.gate=MapDrive(1.,.5)
                if True:
                    self.start();self.tick();self.tick(status=dict(AUTO,armed=True))
                    p=plan();status=dict(AUTO,armed=True);ros=10.
                    if failure=='stale':ros=10.6
                    if failure=='lost':p=None
                    if failure in ('blocked','arrival'):p=plan(state='BLOCKED' if failure=='blocked' else 'ARRIVED')
                    if failure=='manual':status['mode']=1
                    if failure=='rc_invalid':status['rc_valid']=False
                    if failure=='estop':status['estop_active']=True
                    self.gate.tick(p,status,0.,ros,time.monotonic())
                    self.assertFalse(self.gate.requested);self.assertEqual(self.gate.commanded_pwm,(1500,1500))
                    self.assertEqual(self.calls[-1][0],'/actuators/stop');self.tick()
                    count=len(self.calls);self.tick();self.assertEqual(len(self.calls),count)
    def test_driver_failure_latches_and_does_not_rearm(self):
        self.start();self.port.ready=lambda:False
        self.tick();self.assertFalse(self.gate.requested)
        for _ in range(4):self.tick()
        self.assertFalse(any(name=='/actuators/arm' for name,_ in self.calls))
    def test_pending_stop_prevents_reprepare_and_start(self):
        self.start();self.tick();self.gate.stop()
        with self.assertRaises(ValueError):self.gate.prepare(self.port)
        with self.assertRaises(ValueError):self.gate.start(plan(),AUTO,0.,10.,time.monotonic())

if __name__=='__main__':unittest.main()
