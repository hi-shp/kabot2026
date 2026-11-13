"""ROS adapter wiring without ROS installed; no DDS/hardware execution."""
import json
import sys
import types
import unittest
from concurrent.futures import Future
import numpy as np
from types import SimpleNamespace as NS
from unittest.mock import patch
from pathlib import Path
from test_ros_adapter import Harness,Message,header
from test_route_mission import guidance
from isv_shape_navigation.follow_node import make_node


class CourseAdapterTests(unittest.TestCase):
    def test_real_course_adapter_uses_preview_and_publishes_expected_namespace(self, output_enabled=False, action=None):
        harness=Harness(lambda *_:None)
        class FakeNode(harness.FakeNode):
            def declare_parameter(self,name,value,*args):
                self.params[name]=(False if name=='gui_enabled' else output_enabled if name=='enable_output' else value)
            def create_subscription(self,kind,topic,callback,qos,**kwargs):
                return super().create_subscription(kind,topic,callback,qos)
            def create_timer(self,period,callback,**kwargs):
                return NS(period=period,callback=callback,group=kwargs.get('callback_group'),cancel=lambda:None)
            def get_name(self):return 'course_mission'
            def get_node_names(self):return ['course_mission']
            def create_client(self,kind,name,**kwargs):
                if not hasattr(self,'clients'):self.clients={}
                calls=[]
                def send(request):
                    calls.append(request)
                    future=Future();future.set_result(NS(success=True,status=True))
                    return future
                client=NS(service_is_ready=lambda:True,call_async=send,calls=calls,group=kwargs.get('callback_group'))
                self.clients[name]=client
                return client
        modules={}
        def module(name,**attrs):
            result=types.ModuleType(name);vars(result).update(attrs);modules[name]=result
        class Service:
            Request=Response=Message
        module('rclpy');module('rclpy.node',Node=FakeNode)
        module('rclpy.callback_groups',MutuallyExclusiveCallbackGroup=object)
        module('rclpy.qos',qos_profile_sensor_data='sensor',QoSProfile=lambda **k:NS(**k),
               DurabilityPolicy=NS(TRANSIENT_LOCAL=1),ReliabilityPolicy=NS(RELIABLE=1))
        module('rcl_interfaces');module('rcl_interfaces.msg',ParameterDescriptor=lambda **k:NS(**k))
        module('rcl_interfaces.srv',SetParametersAtomically=Service)
        for package,names in {
            'sensor_msgs.msg':['LaserScan','NavSatFix','Image'],
            'std_msgs.msg':['Float32MultiArray','String','Empty'],
            'yolo_msgs.msg':['DetectionArray'],
            'vectornav_msgs.msg':['CommonGroup'],
            'geometry_msgs.msg':['PoseWithCovarianceStamped'],
            'nav_msgs.msg':['OccupancyGrid'],
        }.items():
            module(package.split('.')[0]);module(package,**{name:Message for name in names})
        module('std_srvs');module('std_srvs.srv',Trigger=Service,SetBool=Service)
        module('mechaship_interfaces');module('mechaship_interfaces.srv',ThrottlePulseWidth=Service)
        module('isv_water_trials');module('isv_water_trials.tank_core',guidance=guidance)
        module('isv_water_trials.actuator_config',command_pwm=lambda percent,side:(1500,1500))
        module('cv_bridge',CvBridge=lambda:None)
        module('ament_index_python');module('ament_index_python.packages',
            get_package_share_directory=lambda _:str(Path(__file__).resolve().parents[1]))
        with patch.dict(sys.modules,modules), patch('isv_shape_navigation.follow_node.acquire_output_lock',return_value=NS(close=lambda:None)):
            node=make_node('course')
            if action is not None:
                action(node)
                return
            self.assertIsNone(node.output)
            self.assertIn('/course/start',node.services)
            self.assertIn('/course/gui_command',node.services)
            self.assertIn('/course/status',node.publishers)
            self.assertIn('/gps/fix',node.subscriptions)
            self.assertIn('/vectornav/raw/common',node.subscriptions)
            self.assertNotIn('/course/capture_goal',node.services)
            self.assertNotIn('/map',node.subscriptions)
            self.assertNotIn('/tank/pose',node.subscriptions)
            self.assertNotIn('/tank/session',node.subscriptions)
            self.assertFalse(node.controller.gps_config.use_slam)
            node.update()
            report=json.loads(node.publishers['/course/status'].messages[-1].data)
            self.assertEqual(report['state'],'WAIT_GPS')
            self.assertEqual(report['intended_pwm'],[1500,1500])
            self.assertEqual(report['target_class_id'],8)
            self.assertIn('decision',report)
            self.assertIn('slam_display',report)
            node.on_fix(Message(header=header(100.,'gps'),status=NS(status=0),
                position_covariance_type=2,latitude=35.186192906,longitude=128.56547113,
                position_covariance=[.25,0.,0.,0.,.25,0.,0.,0.,1.]))
            node.on_heading(Message(header=header(100.,'vectornav'),yawpitchroll=NS(x=0.)))
            node.update()
            self.assertEqual(node.last_reason,'WAIT_SCAN')
            report=json.loads(node.publishers['/course/status'].messages[-1].data)
            self.assertAlmostEqual(report['position']['latitude'],35.186192906)
            self.assertAlmostEqual(report['position']['longitude'],128.56547113)
            self.assertFalse(report['output']['start_ready'])
            self.assertIn('PREVIEW',report['output']['blocker'])
            # Receiving coordinates with unknown covariance must not blank the GUI.
            node.on_fix(Message(header=header(100.1,'gps'),status=NS(status=0),
                position_covariance_type=0,latitude=35.2324707,longitude=129.0827164,
                position_covariance=[0.]*9))
            node.update()
            report=json.loads(node.publishers['/course/status'].messages[-1].data)
            self.assertIsNone(report['position'])
            self.assertEqual(report['state'],'WAIT_GPS')
            self.assertAlmostEqual(report['gps_observation']['latitude'],35.2324707)
            self.assertEqual(report['gps_observation']['reason'],'GPS_COVARIANCE_UNKNOWN')
            self.assertFalse(report['gps_observation']['nav_valid'])
            self.assertEqual(report['intended_pwm'],[1500,1500])
            node.on_telemetry(Message(data='[]'))
            self.assertEqual(node.telemetry,{})
            # A GUI start during the firmware's neutral initialization must not arm.
            starts=[]
            node.output=NS(ready=lambda:True,start=lambda:(starts.append(True) or True))
            node.last_reason='TO_WP1'
            telemetry=dict(receiving=True,mode=2,armed=False,estop_active=False)
            with patch('isv_shape_navigation.follow_node.time.monotonic',return_value=10.):
                node.on_telemetry(Message(data=json.dumps(telemetry)))
                response=node.start(Message(),Message())
                self.assertFalse(response.success)
                self.assertIn('초기화',response.message)
                self.assertEqual(starts,[])
            with patch('isv_shape_navigation.follow_node.time.monotonic',return_value=17.5):
                # Continuous telemetry keeps the original connection timestamp.
                node.telemetry_at=17.5
                node.command_lease.publish(17.5,(1650,1650),True)
                response=node.start(Message(),Message())
                self.assertTrue(response.success)
                self.assertEqual(starts,[True])

    def test_output_timer_uses_fresh_navigation_and_stops_when_it_expires(self):
        def action(node):
            telemetry=dict(receiving=True,mode=2,armed=False,estop_active=False,rc_valid=True)
            with patch('isv_shape_navigation.follow_node.time.monotonic',return_value=0.):
                node.on_telemetry(Message(data=json.dumps(telemetry)))
            with patch('isv_shape_navigation.follow_node.time.monotonic',return_value=7.5):
                node.telemetry_at=7.4
                node.on_telemetry(Message(data=json.dumps(telemetry)))
                node.on_fix(Message(header=header(100.,'gps'),status=NS(status=0),
                    position_covariance_type=2,latitude=35.18614,longitude=128.56547113,
                    position_covariance=[.25,0.,0.,0.,.25,0.,0.,0.,1.]))
                node.on_heading(Message(header=header(100.,'vectornav'),yawpitchroll=NS(x=0.)))
                node.on_scan(Message(header=header(100.,'velodyne'),ranges=[float('inf')]*361,
                    angle_min=-np.pi,angle_increment=np.pi/180,range_min=.2,range_max=30.))
                node.update()
                self.assertTrue(node.start(Message(),Message()).success)
                node.update_output()
            self.assertEqual(node.output_timer.period,.02)
            self.assertIsNot(node.output_timer.group,node.timer.group)
            for client in node.clients.values():self.assertIs(client.group,node.output_timer.group)
            telemetry['armed']=True
            for stamp in (7.52,7.54):
                with patch('isv_shape_navigation.follow_node.time.monotonic',return_value=stamp):
                    node.on_telemetry(Message(data=json.dumps(telemetry)))
                    node.update_output()
            self.assertTrue(node.output.active)
            self.assertGreater(node.output.last_ack_pwm[0],1500)
            # No fresh planner result: the independent timer must stop, not replay it.
            with patch('isv_shape_navigation.follow_node.time.monotonic',return_value=7.8):
                node.on_telemetry(Message(data=json.dumps(telemetry)))
                node.update_output()
            self.assertFalse(node.output.active)
            self.assertEqual(node.output.error,'navigation command expired')
            self.assertEqual(len(node.arm_client.calls),1)
            self.assertEqual(len(node.stop_client.calls),1)
        self.test_real_course_adapter_uses_preview_and_publishes_expected_namespace(True,action)

    def test_raw_radar_updates_during_gps_wait_without_driving_navigation(self):
        def action(node):
            node.on_display_scan(Message(header=header(100.,'velodyne'),ranges=[2.]*361,
                angle_min=-np.pi,angle_increment=np.pi/180,range_min=.1,range_max=30.))
            self.assertIsNone(node.scan)
            node.update()
            report=json.loads(node.publishers['/course/status'].messages[-1].data)
            self.assertEqual(report['state'],'WAIT_GPS')
            self.assertEqual(report['lidar_display']['status'],'LIVE')
            self.assertGreater(len(report['lidar_display']['points']),0)
            self.assertEqual(report['intended_pwm'],[1500,1500])
        self.test_real_course_adapter_uses_preview_and_publishes_expected_namespace(False,action)

    def test_all_detected_classes_are_drawn_but_only_selected_target_is_followed(self):
        def action(node):
            def detection(label,cid):
                return NS(class_name=label,class_id=cid,score=.9,
                    bbox=NS(center=NS(position=NS(x=200.,y=200.)),size=NS(x=60.,y=60.)))
            node.on_yolo(Message(header=header(100.,'camera_frame'),
                detections=[detection('blue circle',0),detection('red triangle',8)]))
            self.assertEqual(len(node.yolo),1)
            self.assertEqual(len(node.display_yolo),2)
            node.update()
            report=json.loads(node.publishers['/course/status'].messages[-1].data)
            self.assertEqual([item['target'] for item in report['camera_detections']],[False,True])
            node.dashboard=NS(image_extension='.jpg',camera=lambda _:None)
            node.image_bridge=NS(imgmsg_to_cv2=lambda *a,**k:np.zeros((480,640,3),dtype=np.uint8))
            with patch('cv2.putText') as labels:
                node.on_course_image(Message())
            rendered=[call.args[1] for call in labels.call_args_list]
            self.assertTrue(any('blue circle' in label for label in rendered))
            self.assertTrue(any('red triangle' in label and 'TARGET' in label for label in rendered))
        self.test_real_course_adapter_uses_preview_and_publishes_expected_namespace(False,action)


if __name__=='__main__':unittest.main()
