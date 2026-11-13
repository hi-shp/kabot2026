import math
import unittest
import numpy as np
from isv_shape_navigation.core import Scan
from isv_shape_navigation.local_dock import LocalDock,LocalDockConfig,detect_dock,motion_clear
from isv_shape_navigation.follow import FollowConfig,FusedTarget
from isv_shape_navigation.route_mission import RouteMission,RouteConfig
from isv_shape_navigation.gps_mission import GPSConfig,Fix


def scene(now,position=(0.,0.),yaw=0.,back=True):
    """Raycast three adjoining U docks, width 2.4m, depth 3m."""
    walls=[((4.,y),(7.,y)) for y in (-3.6,-1.2,1.2,3.6)]
    if back:walls.append(((7.,-3.6),(7.,3.6)))
    angles=np.linspace(-math.pi,math.pi,721);directions=np.column_stack((np.cos(angles+yaw),np.sin(angles+yaw)))
    distances=np.full(len(angles),np.inf);origin=np.array(position)
    for p,q in walls:
        p,q=np.array(p),np.array(q);v=q-p;w=p-origin
        den=directions[:,0]*v[1]-directions[:,1]*v[0]
        with np.errstate(divide='ignore',invalid='ignore'):
            r=(w[0]*v[1]-w[1]*v[0])/den
            t=(w[0]*directions[:,1]-w[1]*directions[:,0])/den
        valid=(np.abs(den)>1e-9)&(r>=.2)&(t>=0)&(t<=1)
        distances=np.minimum(distances,np.where(valid,r,np.inf))
    return Scan(distances,-math.pi,math.pi/360,.2,30.,now)


class GeometryTests(unittest.TestCase):
    def test_three_bays_camera_selects_middle_and_computes_q(self):
        result=detect_dock(scene(10.),np.array([7.,0.]),LocalDockConfig(),.77)
        self.assertIsNotNone(result)
        np.testing.assert_allclose(result['m'],[4.,0.],atol=.15)
        np.testing.assert_allclose(result['q'],[2.5,0.],atol=.15)
        np.testing.assert_allclose(result['c'],[5.5,0.],atol=.15)
        np.testing.assert_allclose(result['n'],[1.,0.],atol=.05)

    def test_heading_rotated_and_sensor_translated(self):
        pos=np.array([1.,.2]);yaw=.2;c,s=math.cos(yaw),math.sin(yaw)
        rotate=np.array([[c,s],[-s,c]])
        result=detect_dock(scene(10.,pos,yaw),rotate@(np.array([7.,0.])-pos),LocalDockConfig(),.77)
        self.assertIsNotNone(result)
        np.testing.assert_allclose(result['m'],rotate@(np.array([4.,0.])-pos),atol=.2)
        self.assertAlmostEqual(math.atan2(result['n'][1],result['n'][0])+yaw,0.,delta=.05)

    def test_missing_back_wall_refuses_dock(self):
        self.assertIsNone(detect_dock(scene(10.,back=False),[7.,0.],LocalDockConfig(),.77))

    def test_camera_on_divider_is_ambiguous(self):
        self.assertIsNone(detect_dock(scene(10.),[7.,1.2],LocalDockConfig(),.77))

    def test_geometry_survives_inside_bay(self):
        result=detect_dock(scene(10.,(5.5,0.)),[1.5,0.],LocalDockConfig(),.77)
        self.assertIsNotNone(result)
        np.testing.assert_allclose(result['m'],[-1.5,0.],atol=.15)

    def test_unknown_rear_blocks_reverse(self):
        scan=scene(10.);r=scan.ranges.copy();r[:160]=np.nan
        scan=Scan(r,scan.angle_min,scan.angle_increment,.2,30.,10.)
        self.assertFalse(motion_clear(scan,[-1.,0.],.77))

    def test_tiny_rear_fov_cannot_establish_clear_capsule(self):
        scan=Scan(np.full(11,np.inf),math.pi-.05,.01,.2,30.,10.)
        self.assertFalse(motion_clear(scan,[-1.,0.],.77))

    def test_obstacle_in_reverse_sweep_blocks_output(self):
        scan=scene(10.,(5.5,0.));r=scan.ranges.copy();r[0:4]=.9;r[-4:]=.9
        scan=Scan(r,scan.angle_min,scan.angle_increment,.2,30.,10.)
        self.assertFalse(motion_clear(scan,[-1.,0.],.77))

    def test_repeated_frames_do_not_confirm_geometry(self):
        ctrl=LocalDock()
        for i in range(5):
            ctrl.step(10.+i*.01,scene(10.),[7.,0.],0.,LocalDockConfig(),FollowConfig(),True,True,True)
        self.assertEqual(ctrl.stable,1);self.assertEqual(ctrl.phase,'DOCK_ACQUIRE')

    def test_stale_geometry_time_does_not_count_as_hold(self):
        ctrl=LocalDock();cfg=LocalDockConfig(dock_hold_seconds=.3);now=10.
        for _ in range(5):
            now+=.1;ctrl.step(now,scene(now,(5.5,0.)),[1.5,0.],0.,cfg,FollowConfig(),True,True,True)
        ctrl.phase='DOCK_HOLD';ctrl.since=now
        state,pwm,done=ctrl.step(now+5.,scene(now+5.,(5.5,0.)),[1.5,0.],0.,cfg,FollowConfig(),True,True,True)
        self.assertFalse(done);self.assertEqual(ctrl.phase,'DOCK_HOLD');self.assertIsNone(ctrl.since)

    def test_gps_check_is_at_docked_position_not_at_q(self):
        ctrl=LocalDock();cfg=LocalDockConfig();now=10.
        for _ in range(5):
            now+=.1;ctrl.step(now,scene(now,(2.5,0.)),[4.5,0.],0.,cfg,FollowConfig(),True,True,False)
        ctrl.phase='DOCK_ENTER'
        state,pwm,_=ctrl.step(now+.1,scene(now+.1,(2.5,0.)),[4.5,0.],0.,cfg,FollowConfig(),True,True,False)
        self.assertEqual(state,'DOCK_ENTER');self.assertGreater(min(pwm),1500)
        for _ in range(5):
            now+=.1;state,pwm,_=ctrl.step(now+.1,scene(now+.1,(5.5,0.)),[1.5,0.],0.,cfg,FollowConfig(),True,True,False)
        self.assertEqual(state,'WP3_POSITION_MISMATCH');self.assertEqual(pwm,(1500,1500))

    def test_lidar_imu_dock_and_reverse_exit(self):
        ctrl=LocalDock();cfg=LocalDockConfig(dock_hold_seconds=.2);now=10.
        states=[]
        for pos,iterations in [((0.,0.),6),((2.5,0.),6),((5.5,0.),10),((2.3,0.),6)]:
            for _ in range(iterations):
                now+=.1
                state,pwm,done=ctrl.step(now,scene(now,pos),[7-pos[0],-pos[1]],0.,cfg,FollowConfig(),True,True,True)
                states.append(state)
                if done:break
            if done:break
        self.assertIn('DOCK_ENTER',states);self.assertIn('DOCK_REVERSE',states)
        self.assertTrue(done,states);self.assertEqual(pwm,(1500,1500))

    def test_preview_does_not_advance(self):
        ctrl=LocalDock()
        for i in range(5):ctrl.step(10+i*.1,scene(10+i*.1),[7.,0.],0.,LocalDockConfig(),FollowConfig(),False,True,True)
        self.assertEqual(ctrl.phase,'DOCK_ACQUIRE');self.assertIsNone(ctrl.locked_yaw)

    def test_route_adapter_runs_geometry_dock_to_completion(self):
        cfg=RouteConfig(dock_hold_seconds=.2,dock_calibration_verified=True)
        ctrl=RouteMission(FollowConfig(),None,GPSConfig(goal_configured=True),cfg)
        ctrl.stage='DOCK_TO_WP3';ctrl.phase='SEARCH_SHAPE';ctrl.search_started=0.;ctrl.wp_index=2
        ctrl.executing=True;ctrl.neutral_confirmed=True;now=10.;states=[]
        for px in (0.,2.5,5.5,2.3):
            for _ in range(14):
                now+=.1
                ctrl.fix=Fix(cfg.wp_latitudes[2],cfg.wp_longitudes[2],.3,now)
                ctrl.heading,ctrl.heading_stamp=0.,now
                ctrl.observe([FusedTarget(320.,240.,8,1,7-px,0.,0.,7-px,now)],now)
                state,_=ctrl.step(now,scene(now,(px,0.)));states.append(state)
                if ctrl.completed:break
            if ctrl.completed:break
        self.assertTrue(ctrl.completed,states)
        self.assertIn('DOCK_TO_Q',states);self.assertIn('DOCK_REVERSE',states)
        self.assertEqual(ctrl.intended_pwm,(1500,1500))


class BrakeTests(unittest.TestCase):
    def setUp(self):
        self.cfg=RouteConfig()
        self.ctrl=RouteMission(FollowConfig(),None,GPSConfig(goal_configured=True),self.cfg)
        self.ctrl.stage='WP2_BRAKE';self.ctrl.wp_index=1;self.ctrl.executing=True
    def step(self,t,scan=None):
        self.ctrl.fix=Fix(self.cfg.wp_latitudes[1],self.cfg.wp_longitudes[1],.4,t)
        self.ctrl.heading,self.ctrl.heading_stamp=0.,t
        clear=Scan(np.full(721,np.inf),-math.pi,math.pi/360,.2,30.,t)
        return self.ctrl.step(t,scan or clear)
    def test_exact_command_pulse_then_neutral_and_ten_second_hold(self):
        for t in (100.,100.1,100.4):
            state,_=self.step(t);self.assertEqual(state,'WP2_BRAKE');self.assertEqual(self.ctrl.intended_pwm,(1400,1400))
        self.assertEqual(self.step(100.5)[0],'WP2_HOLD');self.assertEqual(self.ctrl.intended_pwm,(1500,1500))
        self.assertEqual(self.ctrl.hold_remaining,10.)
    def test_pause_aborts_pulse_instead_of_replaying(self):
        self.step(100.);self.ctrl.executing=False
        self.assertEqual(self.step(100.2)[0],'WP2_BRAKE_ABORTED')
        self.ctrl.executing=True;self.assertEqual(self.step(101.)[0],'WP2_BRAKE_ABORTED')
    def test_rear_obstacle_blocks_pulse(self):
        scan=Scan(np.ones(721)*.5,-math.pi,math.pi/360,.2,30.,100.)
        self.assertEqual(self.step(100.,scan)[0],'WP2_BRAKE_BLOCKED');self.assertEqual(self.ctrl.intended_pwm,(1500,1500))
    def test_default_radius_is_one_metre(self):self.assertEqual(self.cfg.wp_radius,1.)

    def test_provisional_camera_calibration_does_not_drive_dock(self):
        self.assertEqual(self.ctrl.step_local_dock(100.,None)[0],'DOCK_CALIBRATION_REQUIRED')
        self.assertEqual(self.ctrl.intended_pwm,(1500,1500))


if __name__=='__main__':unittest.main()
