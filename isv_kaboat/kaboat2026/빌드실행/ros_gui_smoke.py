"""Isolated DDS domain + preview output: exercise the actual GUI ROS service.

Run with ROS_DOMAIN_ID=151. Never launch sensors, create actuator clients or arm.
"""
import json
import os
import time
import rclpy
from rcl_interfaces.msg import Parameter,ParameterValue
from rcl_interfaces.srv import SetParametersAtomically
from isv_shape_navigation.follow_node import make_node

assert os.environ.get('ROS_DOMAIN_ID')=='151','An isolated test DDS domain is required'
rclpy.init(args=['--ros-args','-p','gui_enabled:=false','-p','enable_output:=false'])
node=make_node('course')
assert node.output is None and node.dashboard is None
client=node.create_client(SetParametersAtomically,'/course/gui_command')
try:
    assert client.wait_for_service(timeout_sec=3)
    def command(value):
        request=SetParametersAtomically.Request(parameters=[Parameter(name='ui_request',
            value=ParameterValue(type=4,string_value=json.dumps(value)))])
        future=client.call_async(request)
        until=time.monotonic()+3
        while not future.done() and time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.05)
        assert future.done(),'GUI RPC timed out'
        return future.result().result
    assert command({'action':'set_target','target_label':'blue circle'}).successful
    assert node.cfg.target_label=='blue circle' and node.cfg.target_class_id==0
    assert command({'action':'set_target','target_label':'red triangle'}).successful
    assert node.cfg.target_class_id==8
    assert not command({'action':'set_target','target_label':'purple star'}).successful
    assert command({'action':'set_route','waypoints':[[35.,129.],[35.0001,129.],[35.0001,129.0001]]}).successful
    assert node.controller.route_config.wp_latitudes==(35.,35.0001,35.0001)
    assert not command({'action':'capture_wp','index':0}).successful  # No GPS.
    assert not command({'action':'start'}).successful  # Preview can never enable propulsion.
    assert command({'action':'stop'}).successful
    assert not command({'action':'pwm','left':1600}).successful
    assert node.output is None
    print('ROS_GUI_SMOKE_OK: actual ROS service; target/route applied; invalid target/no-fix/start/PWM rejected; no actuator output')
finally:
    node.destroy_node();rclpy.shutdown()
