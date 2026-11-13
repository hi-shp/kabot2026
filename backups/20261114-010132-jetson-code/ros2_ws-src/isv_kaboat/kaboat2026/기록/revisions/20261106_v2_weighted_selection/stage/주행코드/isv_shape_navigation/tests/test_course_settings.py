import json
import math
from types import SimpleNamespace as NS
import unittest

from isv_shape_navigation.course_settings import apply_course_settings, handle_settings_rpc
from isv_shape_navigation.follow import FollowConfig, MODEL_CLASSES
from isv_shape_navigation.route_mission import RouteConfig


class SettingsTests(unittest.TestCase):
    def setUp(self):
        self.resets=0
        self.cfg=FollowConfig()
        def reset():self.resets+=1
        self.node=NS(output=None,cfg=self.cfg,dashboard=None,yolo=['old'],yolo_stamp=1.,
            now=lambda:10.,p={'gps_timeout':2.,'max_gps_sigma':3.},
            controller=NS(config=self.cfg,route_config=RouteConfig(),fix=None,reset=reset))

    def test_each_learned_target_is_selected_and_observations_reset(self):
        for label in MODEL_CLASSES:
            apply_course_settings(self.node,{'action':'set_target','target_label':label})
            self.assertEqual(self.node.cfg.target_label,label)
            self.assertEqual(self.node.cfg.target_class_id,MODEL_CLASSES.index(label))
        self.assertEqual(self.resets,9)
        self.assertEqual(self.node.yolo,[])
        self.assertEqual(self.node.yolo_stamp,-math.inf)

    def test_running_armed_pending_and_stop_drain_all_preserve_settings(self):
        for state in ('active','armed','pending','stopping'):
            output=NS(active=False,armed=False,pending=[],stopping=False)
            setattr(output,state,True)
            self.node.output=output
            with self.assertRaises(ValueError):
                apply_course_settings(self.node,{'action':'set_target','target_label':'blue circle'})
            self.assertIs(self.node.cfg,self.cfg)
            self.assertEqual(self.resets,0)

    def test_bad_route_is_atomic_and_three_valid_coordinates_apply(self):
        old=self.node.controller.route_config
        for values in ([],[[35,129]]*2,[[True,129]]*3,[[math.nan,129]]*3,[[90,129]]*3):
            with self.assertRaises(ValueError):
                apply_course_settings(self.node,{'action':'set_route','waypoints':values})
            self.assertIs(self.node.controller.route_config,old)
        points=[[35.,129.],[35.0001,129.],[35.0001,129.0001]]
        apply_course_settings(self.node,{'action':'set_route','waypoints':points})
        self.assertEqual(self.node.controller.route_config.wp_latitudes,(35.,35.0001,35.0001))
        self.assertEqual(self.resets,1)

    def test_capture_requires_fresh_accurate_fix(self):
        for stamp,sigma in ((1.,1.),(10.,100.),(10.,math.nan)):
            self.node.controller.fix=NS(stamp=stamp,sigma=sigma,latitude=35.1,longitude=129.1)
            with self.assertRaises(ValueError):apply_course_settings(self.node,{'action':'capture_wp','index':1})
        fix=NS(stamp=10.,sigma=.5,latitude=35.1,longitude=129.1)
        self.node.controller.fix=fix
        apply_course_settings(self.node,{'action':'capture_wp','index':1})
        self.assertEqual(self.node.controller.route_config.wp_latitudes[1],35.1)
        self.assertIs(self.node.controller.fix,fix)

    def rpc(self,command):
        request=NS(parameters=[NS(name='ui_request',value=NS(type=4,string_value=json.dumps(command)))])
        return handle_settings_rpc(self.node,request,NS(result=NS(successful=False,reason=''))).result

    def test_rpc_routes_start_stop_reset_through_existing_controller_gates(self):
        calls=[]
        def delegate(action):
            def run(request,response):
                calls.append(action);response.success=action!='start';response.message='existing '+action
                return response
            return run
        for action in ('start','stop','reset'):setattr(self.node,action,delegate(action))
        self.assertFalse(self.rpc({'action':'start'}).successful)
        self.assertTrue(self.rpc({'action':'stop'}).successful)
        self.assertTrue(self.rpc({'action':'reset'}).successful)
        self.assertEqual(calls,['start','stop','reset'])
        for action in ('arm','pwm','exec'):
            self.assertFalse(self.rpc({'action':action}).successful)
        self.assertEqual(calls,['start','stop','reset'])

    def test_unknown_target_or_malformed_rpc_preserves_current_target(self):
        for command in ({'action':'set_target','target_label':'purple star'},[],None):
            self.assertFalse(self.rpc(command).successful)
        self.assertIs(self.node.cfg,self.cfg)
        self.assertEqual(self.resets,0)


if __name__=='__main__':unittest.main()
