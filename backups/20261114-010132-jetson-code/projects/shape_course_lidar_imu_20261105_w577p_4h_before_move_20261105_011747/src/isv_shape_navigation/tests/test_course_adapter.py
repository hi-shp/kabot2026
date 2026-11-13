"""ROS adapter wiring without ROS installed; no DDS/hardware execution."""
import json
import sys
import types
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch
from pathlib import Path
from test_ros_adapter import Harness,Message,header
from test_route_mission import guidance
from isv_shape_navigation.follow_node import make_node


class CourseAdapterTests(unittest.TestCase):
    def test_real_course_adapter_uses_preview_and_publishes_expected_namespace(self):
        harness=Harness(lambda *_:None)
        class FakeNode(harness.FakeNode):
            def declare_parameter(self,name,value,*args):
                self.params[name]=False if name=='gui_enabled' else value
        modules={}
        def module(name,**attrs):
            result=types.ModuleType(name);vars(result).update(attrs);modules[name]=result
        class Service:
            Request=Response=Message
        module('rclpy');module('rclpy.node',Node=FakeNode)
        module('rclpy.qos',qos_profile_sensor_data='sensor',QoSProfile=lambda **k:NS(**k),
               DurabilityPolicy=NS(TRANSIENT_LOCAL=1),ReliabilityPolicy=NS(RELIABLE=1))
        module('rcl_interfaces');module('rcl_interfaces.msg',ParameterDescriptor=lambda **k:NS(**k))
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
        with patch.dict(sys.modules,modules):
            node=make_node('course')
            self.assertIsNone(node.output)
            self.assertIn('/course/start',node.services)
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


if __name__=='__main__':unittest.main()
