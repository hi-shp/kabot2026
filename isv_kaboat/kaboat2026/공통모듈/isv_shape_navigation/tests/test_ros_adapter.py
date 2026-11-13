"""Exercise ROS adapter callbacks with message/client doubles (no DDS)."""
import json
import math
import sys
import types
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch
import numpy as np

from isv_shape_navigation import node as adapter
from isv_shape_navigation.core import Pose as CorePose, Detection
from isv_shape_navigation.demo import scene


def header(stamp=0., frame=''):
    return NS(stamp=NS(sec=int(stamp), nanosec=round((stamp-int(stamp))*1e9)), frame_id=frame)


def pose_message():
    return NS(position=NS(x=0., y=0., z=0.), orientation=NS(x=0., y=0., z=0., w=1.))


class Message:
    def __init__(self, **kwargs):
        self.header = header()
        self.pose = pose_message()
        self.poses = []
        vars(self).update(kwargs)


class Publisher:
    def __init__(self): self.messages = []
    def publish(self, msg): self.messages.append(msg)
    def get_subscription_count(self): return 0


class Harness:
    def __init__(self, action):
        self.action, self.running, self.time, self.node = action, True, 100., None
        harness = self
        class FakeNode:
            def __init__(self, name):
                harness.node = self
                self.params = {'use_sim_time': False}
                self.subscriptions, self.publishers, self.services = {}, {}, {}
            def declare_parameter(self, name, value): self.params[name] = value
            def get_parameter(self, name): return NS(value=self.params[name])
            def create_subscription(self, kind, topic, callback, qos):
                self.subscriptions[topic] = callback
                return callback
            def create_publisher(self, kind, topic, qos):
                publisher = Publisher(); self.publishers[topic] = publisher
                return publisher
            def create_service(self, kind, name, callback):
                self.services[name] = callback
                return callback
            def create_timer(self, period, callback): return NS(cancel=lambda: None)
            def get_clock(self):
                return NS(now=lambda: NS(nanoseconds=round(harness.time*1e9),
                                        to_msg=lambda: header(harness.time).stamp))
            def get_logger(self): return NS(info=lambda *_: None, warning=lambda *_: None, error=lambda *_: None)
            def destroy_node(self): pass
        self.FakeNode = FakeNode

    def spin(self, node, timeout_sec):
        self.action(self, node)
        self.running = False

    def run(self):
        class Service:
            Request = Message
        modules = {}
        def module(name, **attributes):
            result = types.ModuleType(name)
            vars(result).update(attributes)
            modules[name] = result
            return result
        module('rclpy', init=lambda **_: None, ok=lambda: self.running,
               spin_once=self.spin, shutdown=lambda: None)
        module('rclpy.node', Node=self.FakeNode)
        module('rclpy.qos', qos_profile_sensor_data='sensor', QoSProfile=lambda **k: NS(**k),
               DurabilityPolicy=NS(TRANSIENT_LOCAL=1), ReliabilityPolicy=NS(RELIABLE=1))
        module('rclpy.signals', SignalHandlerOptions=NS(NO=0))
        module('rclpy.executors', ExternalShutdownException=type('ExternalShutdownException', (Exception,), {}))
        for package, names in {
            'sensor_msgs.msg': ['LaserScan', 'Image', 'CameraInfo'],
            'geometry_msgs.msg': ['PoseWithCovarianceStamped', 'PoseStamped'],
            'nav_msgs.msg': ['OccupancyGrid', 'Path'],
            'std_msgs.msg': ['String', 'Empty'],
        }.items():
            module(package.split('.')[0])
            module(package, **{name: Message for name in names})
        module('std_srvs'); module('std_srvs.srv', Trigger=Service, SetBool=Service)
        module('mechaship_interfaces'); module('mechaship_interfaces.srv', ThrottlePulseWidth=Service)
        module('yolo_msgs'); module('yolo_msgs.msg', DetectionArray=Message)
        module('cv_bridge', CvBridge=lambda: NS(imgmsg_to_cv2=lambda msg, **_: msg.frame))
        with patch.dict(sys.modules, modules):
            adapter.main([])
        return self.node

    def initialize_inputs(self, node):
        node.on_session(Message(data=json.dumps(dict(session_id='test-map', localization_ready=True))))
        cells = np.zeros((100, 80), dtype=np.int8)
        cells[[0, -1], :] = 100; cells[:, [0, -1]] = 100
        node.on_map(Message(header=header(self.time, 'tank_map'), data=cells.ravel().tolist(),
                            info=NS(width=80, height=100, resolution=.1, origin=pose_message())))
        node.on_camera(Message(header=header(self.time, 'camera_frame'), width=640, height=480,
                               k=[500, 0, 320, 0, 500, 240, 0, 0, 1], d=[0.]*5, distortion_model='plumb_bob'))

    def send_frame(self, node, stamp, frame_override=None):
        self.time = stamp
        pose = CorePose(2, 5, 0, stamp)
        frame, scan = scene(pose, stamp)
        p = pose_message(); p.position.x, p.position.y = pose.x, pose.y
        node.on_pose(Message(header=header(stamp, 'tank_map'), pose=NS(pose=p)))
        node.on_scan(Message(header=header(stamp, 'velodyne'), ranges=scan.ranges.tolist(),
                             angle_min=scan.angle_min, angle_increment=scan.angle_increment,
                             range_min=.2, range_max=25.))
        node.on_image(Message(header=header(stamp, 'camera_frame'), frame=frame if frame_override is None else frame_override))


class AdapterTests(unittest.TestCase):
    def test_slam_output_observes_receiver_reinitialization(self):
        def action(harness,node):
            telemetry=dict(receiving=True,mode=2,armed=False,estop_active=False,rc_valid=False)
            with patch('isv_shape_navigation.node.time.monotonic',return_value=0.):
                node.on_actuator(Message(data=json.dumps(telemetry)))
            self.assertEqual(node.output_readiness.check(8,telemetry,0,True),'')
            telemetry['rc_valid']=True
            with patch('isv_shape_navigation.node.time.monotonic',return_value=9.):
                node.telemetry_at=8.9
                node.on_actuator(Message(data=json.dumps(telemetry)))
            self.assertIn('초기화',node.output_readiness.check(9,telemetry,0,True))
            self.assertEqual(node.output_readiness.check(16.5,telemetry,0,True),'')
        Harness(action).run()

    def test_control_rate_must_fit_the_mcu_watchdog(self):
        def action(harness,node):
            node.p['control_rate']=1.
            with self.assertRaisesRegex(ValueError,'MCU watchdog'):node._validate()
        Harness(action).run()

    def test_default_topics_camera_fusion_path_and_preview(self):
        def action(harness, node):
            harness.initialize_inputs(node)
            for stamp in (100., 100.1, 100.2, 100.3):
                harness.send_frame(node, stamp); node.update()
            self.assertEqual(node.last_reason, 'APPROACH_SHAPE')
            np.testing.assert_allclose(node.mission.target, [8, 5], atol=.06)
            self.assertIsNone(node.output)
            for topic in ('/scan', '/tank/pose', '/map', '/tank/session', '/camera/image_raw', '/camera/camera_info'):
                self.assertIn(topic, node.subscriptions)
            self.assertTrue(node.publishers['/shape_navigation/path'].messages[-1].poses)
        Harness(action).run()

    def test_changed_session_clears_goal_and_old_sensor_data(self):
        def action(harness, node):
            harness.initialize_inputs(node)
            for stamp in (100., 100.1, 100.2, 100.3):
                harness.send_frame(node, stamp)
            node.on_session(Message(data=json.dumps(dict(session_id='another-map', localization_ready=True))))
            self.assertIsNone(node.mission.target); self.assertIsNone(node.grid)
            self.assertEqual(len(node.poses), 0); self.assertEqual(len(node.scans), 0)
        Harness(action).run()

    def test_initial_session_does_not_discard_transient_map(self):
        def action(harness, node):
            node.on_map(Message(header=header(100, 'tank_map'), data=[0]*400,
                                info=NS(width=20, height=20, resolution=.2, origin=pose_message())))
            node.on_session(Message(data=json.dumps(dict(session_id='test-map', localization_ready=True))))
            self.assertIsNotNone(node.grid)
        Harness(action).run()

    def test_wrong_scan_frame_is_not_used(self):
        def action(harness, node):
            node.on_scan(Message(header=header(100, 'unrelated_lidar'), ranges=[2.]*100,
                                 angle_min=-1., angle_increment=.02, range_min=.2, range_max=20.))
            self.assertEqual(len(node.scans), 0)
        Harness(action).run()

    def test_nonfinite_detection_cannot_crash_json_status(self):
        def action(harness, node):
            harness.initialize_inputs(node); harness.send_frame(node, 100.)
            node.accept_detections([Detection('triangle', .9, math.nan, 240, 50, 50, 100.)], 100.)
            node.update()
            self.assertEqual(node.last_detections, [])
        Harness(action).run()

    def test_two_same_class_candidates_are_ambiguous(self):
        def action(harness, node):
            harness.initialize_inputs(node); harness.send_frame(node, 100.)
            node.mission.reset()
            detection = Detection('triangle', .9, 320, 240, 80, 80, 100.)
            node.accept_detections([detection, detection], 100.)
            self.assertEqual(node.mission.count, 0)
        Harness(action).run()

    def test_preview_start_cannot_call_actuators(self):
        def action(harness, node):
            response = node.start(Message(), Message())
            self.assertFalse(response.success)
            self.assertIn('Preview', response.message)
        Harness(action).run()

    def test_stop_topic_echo_is_ignored_after_stop(self):
        def action(harness, node):
            calls = []
            node.output = NS(active=False, armed=False, request_stop=lambda *_: calls.append('stop'), owned=False)
            node.external_stop(Message())
            self.assertEqual(calls, [])
            node.output = None
        Harness(action).run()

    def test_backward_bag_clock_allows_new_images(self):
        def action(harness, node):
            harness.initialize_inputs(node); harness.send_frame(node, 100.)
            node.update()
            harness.time = 1.; node.update()
            self.assertEqual(node.last_image, -math.inf)
            harness.send_frame(node, 1.1)
            self.assertEqual(node.last_image, 1.1)
        Harness(action).run()


if __name__ == '__main__':
    unittest.main()
