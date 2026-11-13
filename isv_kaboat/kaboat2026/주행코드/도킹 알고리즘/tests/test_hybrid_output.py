from copy import deepcopy
from concurrent.futures import Future
from types import SimpleNamespace as NS
from unittest.mock import patch
import time,unittest
from docking_mapping.actuation import DockingOutput
from docking_mapping.map_drive import MapDrive
from test_mapping import config

AUTO=dict(mode=2,receiving=True,rc_valid=True,estop_active=False,stop_latched=False,armed=False)
def plan(stamp,mode='GPS_APPROACH',state='APPROACH_TARGET'):
    return dict(state=state,navigation_mode=mode,pwm=[1500,1500] if state=='WAIT_DOCK_ALIGNMENT' else [1635,1635],
        pose_stamp_s=stamp,imu_stamp_s=stamp,scan_stamp_s=stamp,gps_stamp_s=stamp,last_registration_stamp_s=stamp)

class HandoffOutputTests(unittest.TestCase):
    def test_actual_output_adapter_transitions_from_gps_through_neutral_hold_to_lidar_without_rearming(self):
        calls=[]
        class Client:
            def __init__(self,name):self.name=name
            def service_is_ready(self):return True
            def call_async(self,request):
                calls.append((self.name,request));f=Future();f.set_result(NS(success=True,status=True));return f
        node=NS(create_client=lambda kind,name:Client(name),get_node_names=lambda:[],get_name=lambda:'docking_mapping')
        out=DockingOutput(node,dict(config()['autonomy'],dry_run=False,enable_output=True,hardware_output_authorized=True))
        out.readiness.since=time.monotonic()-8;gate=MapDrive(1.,.5);gate.prepare(out)
        owner=NS(close=lambda:None)
        with patch('isv_shape_navigation.output_runtime.acquire_output_lock',return_value=owner):
            gate.start(plan(10),AUTO,0.,10.,time.monotonic())
            gate.tick(plan(10),AUTO,0.,10.,time.monotonic())
            gate.tick(plan(10),dict(AUTO,armed=True),0.,10.,time.monotonic())
            gate.tick(plan(10.1,state='WAIT_DOCK_ALIGNMENT'),dict(AUTO,armed=True),0.,10.1,time.monotonic())
            self.assertTrue(gate.requested);self.assertEqual(gate.commanded_pwm,(1500,1500))
            gate.tick(plan(10.2,mode='LIDAR_DOCKING'),dict(AUTO,armed=True),0.,10.2,time.monotonic())
            self.assertTrue(gate.requested)
            self.assertEqual(sum(name=='/actuators/arm' for name,_ in calls),1)
            self.assertFalse(any(name=='/actuators/stop' for name,_ in calls))
            self.assertEqual(gate.commanded_pwm,(1635,1635))
            gate.stop();gate.tick(None,dict(AUTO,armed=True),0.,10.3,time.monotonic())
            self.assertEqual(calls[-1][0],'/actuators/stop')
    def test_gps_and_last_registration_expire_independently_of_fresh_scan_timestamps(self):
        gate=MapDrive(1.,.5)
        p=plan(10);p['gps_stamp_s']=7.;self.assertEqual(gate.plan_reason(p,10.),'STALE_GPS')
        p=plan(10,mode='LIDAR_DOCKING');p['last_registration_stamp_s']=8.
        self.assertEqual(gate.plan_reason(p,10.),'STALE_REGISTRATION')
        p=plan(10,state='WAIT_DOCK_ALIGNMENT');p['pwm']=[1635,1635]
        self.assertEqual(gate.plan_reason(p,10.),'INVALID_HOLD_PLAN')

if __name__=='__main__':unittest.main()
