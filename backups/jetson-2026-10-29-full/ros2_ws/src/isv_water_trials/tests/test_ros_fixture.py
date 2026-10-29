"""Actual ROS transport, fake sensors/motor services, localhost isolated domain only."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import unittest

HAS_ROS=importlib.util.find_spec('rclpy') is not None


@unittest.skipUnless(HAS_ROS,'ROS runtime required (run on Jetson)')
class RosFixtureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ['ROS_DOMAIN_ID']='217'
        os.environ['ROS_LOCALHOST_ONLY']='1'
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import NavSatFix, LaserScan
        from vectornav_msgs.msg import CommonGroup
        from std_msgs.msg import Bool
        from mechaship_interfaces.srv import ThrottlePulseWidth
        from isv_water_trials.common import load_config
        rclpy.init()
        cls.rclpy=rclpy
        cls.node=Node('water_trial_fixture')
        cls.executor=MultiThreadedExecutor(num_threads=4)
        cls.executor.add_node(cls.node)
        cls.pause=False;cls.drop_after_drive=False;cls.reject_drive=False;cls.requests=[]
        cls.gps=cls.node.create_publisher(NavSatFix,'/fixture/fix',qos_profile_sensor_data)
        cls.vn=cls.node.create_publisher(CommonGroup,'/fixture/common',qos_profile_sensor_data)
        cls.estop=cls.node.create_publisher(Bool,'/fixture/estop',qos_profile_sensor_data)
        cls.scan=cls.node.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)

        def sensors():
            if cls.pause:return
            gps=NavSatFix();gps.header.stamp=cls.node.get_clock().now().to_msg()
            gps.latitude=35.;gps.longitude=129.;gps.status.status=0
            gps.position_covariance_type=2;gps.position_covariance=[.01,0.,0.,0.,.01,0.,0.,0.,.1]
            vn=CommonGroup();vn.header.stamp=gps.header.stamp;vn.group_fields=0x28;vn.yawpitchroll.x=90.
            scan=LaserScan();scan.header.stamp=gps.header.stamp;scan.header.frame_id='velodyne'
            scan.angle_min=-3.14159;scan.angle_increment=6.28318/360;scan.angle_max=3.14159
            scan.range_min=.1;scan.range_max=30.;scan.ranges=[10.]*360
            cls.gps.publish(gps);cls.vn.publish(vn);cls.estop.publish(Bool(data=False));cls.scan.publish(scan)
        cls.timer=cls.node.create_timer(.05,sensors)
        def service(side):
            def receive(request,response):
                cls.requests.append((side,request.pulse_width))
                if cls.drop_after_drive and request.pulse_width>1500:cls.pause=True
                response.status=not (cls.reject_drive and request.pulse_width>1500)
                return response
            return receive
        cls.services=[cls.node.create_service(ThrottlePulseWidth,'/fixture/'+side,service(side)) for side in ('left','right')]
        cls.thread=threading.Thread(target=cls.executor.spin,daemon=True);cls.thread.start()
        cls.temp=tempfile.TemporaryDirectory();cls.root=Path(cls.temp.name)
        cls.config=load_config();cls.config.update(frame_confirmed=True,gps_topic='/fixture/fix',vectornav_topic='/fixture/common',
                                                 estop_topic='/fixture/estop',left_service='/fixture/left',right_service='/fixture/right')
        cls.cfg=cls.root/'boat.json';cls.cfg.write_text(json.dumps(cls.config))

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown();cls.thread.join(timeout=3);cls.node.destroy_node();cls.rclpy.shutdown();cls.temp.cleanup()

    def run_case(self,drop,reject=False):
        type(self).pause=False;type(self).drop_after_drive=drop;type(self).reject_drive=reject;type(self).requests=[]
        out=self.root/('reject' if reject else 'drop' if drop else 'normal')
        recorder=subprocess.Popen([sys.executable,'-u','-c','from isv_water_trials.recorder import main; main()',
                                    '--config',str(self.cfg),'--output',str(out),'--duration','9'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
        try:
            until=time.monotonic()+6
            while time.monotonic()<until and self.node.count_subscribers('/fixture/fix')==0:time.sleep(.05)
            code="from isv_water_trials import trial; trial.plan=lambda *a:[trial.Step('straight',2 if "+str(drop)+" else .5,1530,1530),trial.Step('end',.3,1500,1500)]; trial.main()"
            result=subprocess.run([sys.executable,'-u','-c',code,'surge','--config',str(self.cfg),'--execute',
                                   '--operator-confirmed','--exclusive-control-confirmed'],capture_output=True,text=True,timeout=18)
            if drop or reject:self.assertNotEqual(result.returncode,0,result.stdout+result.stderr)
            else:self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            logs,_=recorder.communicate(timeout=15)
            self.assertEqual(recorder.returncode,0,logs)
            self.assertTrue(any(p>1500 for _,p in self.requests),'No fake drive request seen')
            self.assertCountEqual(self.requests[-2:],[('left',1500),('right',1500)])
            sessions=list(out.iterdir());self.assertEqual(len(sessions),1)
            rows=[json.loads(line) for line in (sessions[0]/'raw.jsonl').read_text().splitlines()]
            self.assertTrue(any(r['kind']=='gps' for r in rows))
            self.assertTrue(any(r['kind']=='scan' for r in rows))
            self.assertTrue(any(r['kind']=='command' and json.loads(r['msg']['data']).get('status')=='acknowledged' for r in rows))
            analyzed=subprocess.run([sys.executable,'-c','from isv_water_trials.analyze import main;main()',str(sessions[0])],capture_output=True,text=True,timeout=10)
            self.assertEqual(analyzed.returncode,0,analyzed.stdout+analyzed.stderr)
            self.assertFalse(json.loads((sessions[0]/'mpc_model_candidate.json').read_text())['deployment_ready'])
        finally:
            if recorder.poll() is None:
                recorder.terminate();recorder.wait(timeout=5)
            type(self).pause=False

    def test_01_record_acknowledged_commands_and_analyze(self):self.run_case(False)

    def test_02_stale_sensors_abort_and_send_neutral(self):self.run_case(True)

    def test_03_rejected_actuator_ack_aborts_and_stops(self):self.run_case(False,True)

    def test_04_existing_controller_logging_proxy(self):
        from mechaship_interfaces.srv import ThrottlePulseWidth
        from std_msgs.msg import String
        type(self).pause=False;type(self).drop_after_drive=False;type(self).reject_drive=False
        messages=[]
        subscription=self.node.create_subscription(String,'/water_trial/command',lambda msg:messages.append(json.loads(msg.data)),100)
        child=subprocess.Popen([sys.executable,'-u','-c','from isv_water_trials.proxy import main; main()',
                                '--config',str(self.cfg)],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
        clients=[]
        try:
            for side,pwm in [('left',1535),('right',1540)]:
                client=self.node.create_client(ThrottlePulseWidth,'/water_trial/throttle_'+side+'/set_pulse_width')
                clients.append(client)
                self.assertTrue(client.wait_for_service(timeout_sec=5),'Proxy service did not appear')
                future=client.call_async(ThrottlePulseWidth.Request(pulse_width=pwm))
                until=time.monotonic()+3
                while not future.done() and time.monotonic()<until:time.sleep(.02)
                self.assertTrue(future.done())
                self.assertTrue(future.result().status)
            until=time.monotonic()+2
            while time.monotonic()<until and not any(m.get('status')=='acknowledged' for m in messages):time.sleep(.02)
            valid=[m for m in messages if m.get('status')=='acknowledged']
            self.assertTrue(valid,messages)
            self.assertEqual(valid[-1]['left_pwm_us'],1535)
            self.assertEqual(valid[-1]['right_pwm_us'],1540)
        finally:
            child.terminate()
            child.communicate(timeout=5)
            for client in clients:self.node.destroy_client(client)
            self.node.destroy_subscription(subscription)


if __name__=='__main__':unittest.main()
