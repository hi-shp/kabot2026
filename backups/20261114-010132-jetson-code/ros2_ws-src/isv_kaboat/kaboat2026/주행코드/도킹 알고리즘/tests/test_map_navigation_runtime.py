"""Hybrid handoff with fake registration and sensors, no device output."""
from concurrent.futures import Future
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
import tempfile,threading,time,unittest
import numpy as np
from docking_mapping.map_navigation_runtime import MapNavigationDryRun
from docking_mapping.geometry import ENU
from isv_shape_navigation.core import Scan
from test_mapping import config,structure

class FakeMatcher:
    def __init__(self,points,extrinsic,c):self.points_map=structure();self.calls=0;self.block=None;self.enter=None
    def match(self,points,rotation,seed,stamp,tracking,cancelled):
        self.calls+=1
        if self.enter is not None:self.enter.set()
        if self.block is not None:self.block.wait(3)
        return dict(accepted=not cancelled(),reason='CONVERGED',stamp=stamp,pose=np.eye(4),rotation=np.eye(3),rmse=.05,
                    inlier_ratio=1.,information_ratio=.1)

class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();root=Path(self.tmp.name)/'주행코드/도킹 알고리즘';root.mkdir(parents=True)
        settings=root.parents[1]/'설정';settings.mkdir()
        field=Path(__file__).resolve().parents[3]/'설정/course_mission.yaml'
        (settings/'course_mission.yaml').write_text(field.read_text())
        c=config();c['scan_motion']['enabled']=False
        self.clock=100.;self.target=dict(schema=2,map_name='map_test',point_map_xy_m=[5.,0.])
        self.enu=ENU([35.,129.,0.]);self.calls=[];self.clients=[]
        self.node=NS(root=root,config=c,data_lock=threading.Lock(),map_lock=threading.Lock(),gps=[],imu=[],latest_cloud=None,
            rc=dict(mode=2,rc_valid=True,receiving=True,estop_active=False,stop_latched=False),arrivals={'rc':time.monotonic()},
            session=NS(active=False),autonomy=NS(active=False),gps_map_loaded=True,gps_map_name='map_test',gps_target=self.target,
            geographic_maps=NS(read_target=lambda name:deepcopy(self.target)),mapper=NS(enu=self.enu,map=NS(points=structure)),
            stamp=lambda m:m.stamp,now_s=lambda:self.clock,fresh=lambda stamp:-.1<=self.clock-stamp<=1.5,
            create_subscription=lambda *args:None,create_timer=lambda *args:None)
        self.fake=patch('docking_mapping.map_navigation_runtime.BestMapMatcher',FakeMatcher);self.fake.start()
        self.runtime=MapNavigationDryRun(self.node);self.sample(-20.)
    def tearDown(self):
        if self.runtime.matcher is not None and self.runtime.matcher.block is not None:self.runtime.matcher.block.set()
        self.runtime.close();self.fake.stop();self.tmp.cleanup()
    def sample(self,x=-20.,sigma=.1):
        lla=self.enu.to_lla([x,0,0])
        self.node.gps=[NS(stamp=self.clock,status=NS(status=0),position_covariance_type=2,
            position_covariance=[sigma*sigma,0,0,0,sigma*sigma,0,0,0,sigma*sigma],latitude=lla[0],longitude=lla[1],altitude=lla[2])]
        self.node.imu=[NS(stamp=self.clock,header=NS(frame_id='vectornav'),orientation=NS(x=0.,y=0.,z=0.,w=1.),angular_velocity=NS(x=0.,y=0.,z=0.))]
        self.runtime.primary=Scan(np.full(720,np.inf),-np.pi,2*np.pi/720,.1,25.,self.clock)
        self.node.arrivals['rc']=time.monotonic()
    def accepted_job(self,accepted=True):
        f=Future();f.set_result(dict(accepted=accepted,reason='CONVERGED' if accepted else 'HIGH_RMSE',stamp=self.clock,
            pose=np.eye(4),rotation=np.eye(3),rmse=.05 if accepted else 9.,inlier_ratio=1.,information_ratio=.1))
        self.runtime.job=f;self.runtime.job_generation=self.runtime.generation
    def lock_map(self):
        self.runtime.start()
        for _ in range(3):
            self.sample(-20.);self.accepted_job();self.runtime.process_latest();self.clock+=.1
    def fake_output(self):
        return NS(enabled=True,owner=None,driver=None,fault='',readiness=NS(check=lambda *args:''),ready=lambda:True,
            start=lambda *args:self.calls.append('start'),request_stop=lambda *args:self.calls.append('stop'),
            publish=lambda *args:self.calls.append('publish'),tick=lambda *args:None,
            snapshot=lambda:dict(armed=False,stop_confirmed=None,fault=''))
    def test_far_gps_approach_produces_v3_plan_before_any_registration(self):
        self.runtime.start();self.runtime.process_latest();s=self.runtime.snapshot()
        self.assertEqual(s['phase'],'GPS_APPROACH');self.assertEqual(s['plan']['navigation_mode'],'GPS_APPROACH')
        self.assertGreater(s['plan']['goal']['distance_m'],24.)
        self.assertEqual(s['plan']['map_obstacle_points'],0);self.assertFalse(self.runtime.localization.locked)
        self.assertFalse(s['drive']['requested']);self.assertEqual(self.calls,[])
    def test_three_matches_switch_and_gps_is_not_read_after_handoff(self):
        self.lock_map();self.assertTrue(self.runtime.localization.locked)
        self.assertEqual(self.runtime.phase,'LIDAR_DOCKING')
        self.sample(100.);self.node.gps=None;self.runtime.process_latest()
        self.assertEqual(self.runtime.plan['navigation_mode'],'LIDAR_DOCKING')
        self.assertAlmostEqual(self.runtime.plan['goal']['distance_m'],5.)
    def test_near_target_without_alignment_holds_neutral_instead_of_gps_arrival(self):
        self.sample(4.);self.runtime.start();self.runtime.process_latest()
        self.assertEqual(self.runtime.state,'WAIT_DOCK_ALIGNMENT');self.assertEqual(self.runtime.plan['pwm'],[1500,1500])
        self.assertTrue(self.runtime.active);self.assertFalse(self.runtime.localization.locked)
    def test_single_good_then_bad_match_does_not_switch(self):
        self.runtime.start();self.accepted_job();self.runtime.process_latest()
        self.clock+=.1;self.sample();self.accepted_job(False);self.runtime.process_latest()
        self.assertEqual(self.runtime.phase,'GPS_APPROACH');self.assertEqual(self.runtime.localization.confirmed,0)
    def test_short_registration_gap_predicts_from_last_valid_pose_then_stops_at_limit(self):
        self.lock_map();last=self.runtime.localization.poses[-1][0]
        self.sample();self.accepted_job(False);self.runtime.process_latest()
        self.assertIsNotNone(self.runtime.plan);self.assertEqual(self.runtime.localization.poses[-1][0],last)
        self.clock=last+.4;self.sample();self.runtime.process_latest()
        self.assertEqual(self.runtime.plan['pose_source'],'LIDAR_PREDICTING')
        self.clock=last+1.1;self.sample();self.runtime.process_latest()
        self.assertFalse(self.runtime.active);self.assertEqual(self.runtime.state,'TRACKING_LOST')
        self.assertIsNone(self.runtime.plan)
    def test_background_gicp_does_not_block_gps_guidance_or_apply_results_after_stop(self):
        self.sample(0.);self.runtime.start();matcher=self.runtime.matcher
        matcher.block=threading.Event();matcher.enter=threading.Event()
        self.node.latest_cloud=NS(stamp=self.clock,header=NS(frame_id='velodyne'),fields=[])
        with patch('docking_mapping.map_navigation_runtime.read_scalar_fields',return_value=structure()):
            t=time.monotonic();self.runtime.process_latest();self.assertLess(time.monotonic()-t,.3)
        self.assertTrue(matcher.enter.wait(.5));self.assertIsNotNone(self.runtime.plan)
        self.runtime.stop();matcher.block.set();self.runtime.job.result(timeout=2)
        self.runtime.process_latest();self.assertFalse(self.runtime.active);self.assertIsNone(self.runtime.plan)
    def test_real_output_requires_separate_start_but_can_start_in_gps_phase(self):
        self.runtime.start();self.runtime.process_latest();output=self.fake_output()
        with patch('docking_mapping.actuation.DockingOutput',return_value=output):self.runtime.prepare_output()
        self.assertEqual(self.calls,[]);self.runtime.start_drive();self.assertEqual(self.calls,['start'])
        self.runtime.output_tick();self.assertIn('publish',self.calls)
        self.runtime.on_telemetry(dict(self.node.rc,rc_valid=False))
        self.assertFalse(self.runtime.active);self.assertFalse(self.runtime.drive.requested)
        self.assertEqual(self.runtime.drive.commanded_pwm,(1500,1500))
    def test_stale_gps_or_scan_and_position_jump_prevent_approach(self):
        self.runtime.start();self.runtime.process_latest();self.clock+=3.
        self.node.imu[0].stamp=self.clock
        self.runtime.process_latest();self.assertIsNone(self.runtime.plan)
        self.clock+=.1;self.sample(-20.);self.runtime.process_latest()
        self.clock+=.1;self.sample(100.);self.runtime.process_latest()
        self.assertEqual(self.runtime.reason,'GPS_APPROACH_POSITION_JUMP');self.assertIsNone(self.runtime.plan)
    def test_stop_cancels_queued_start_and_old_result(self):
        self.runtime.start();generation=self.runtime.generation;self.accepted_job();self.runtime.stop()
        with self.assertRaisesRegex(ValueError,'cancelled'):self.runtime.start_drive(generation)
        with self.assertRaisesRegex(ValueError,'cancelled'):self.runtime.start(generation)
        self.assertIsNone(self.runtime.plan)
    def test_delayed_but_fresh_scan_uses_synchronized_historical_imu(self):
        from copy import deepcopy
        self.runtime.start();old=deepcopy(self.node.imu[0]);old.stamp=self.clock-.3
        self.node.imu.insert(0,old)
        self.runtime.primary=Scan(np.full(720,np.inf),-np.pi,2*np.pi/720,.1,25.,old.stamp)
        self.runtime.process_latest();self.assertIsNotNone(self.runtime.plan)
        self.assertEqual(self.runtime.plan['imu_stamp_s'],old.stamp)
        self.assertLessEqual(self.clock-old.stamp,self.runtime.parameters['heading_timeout'])
    def test_no_imu_near_scan_is_rejected_even_when_latest_imu_is_fresh(self):
        self.runtime.start()
        self.runtime.primary=Scan(np.full(720,np.inf),-np.pi,2*np.pi/720,.1,25.,self.clock-.3)
        self.runtime.process_latest();self.assertIsNone(self.runtime.plan)
        self.assertEqual(self.runtime.reason,'WAIT_IMU_SYNC')
    def test_start_refreshes_old_gui_plan_from_live_scan_without_relaxing_expiry(self):
        self.runtime.start();self.runtime.process_latest();self.runtime.drive.prepare(self.fake_output())
        self.clock+=.6;self.sample(-20.)
        self.runtime.start_drive(self.runtime.generation)
        self.assertTrue(self.runtime.drive.requested);self.assertEqual(self.calls,['start'])
        self.assertEqual(self.runtime.plan['scan_stamp_s'],self.clock)

if __name__=='__main__':unittest.main()
