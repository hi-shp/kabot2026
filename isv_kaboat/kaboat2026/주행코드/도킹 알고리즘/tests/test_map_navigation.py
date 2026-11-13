from copy import deepcopy
from pathlib import Path
from dataclasses import fields
import hashlib,json,math,tempfile,unittest,shutil
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
from docking_mapping.geometry import apply,transform
from docking_mapping.map_navigation import MapLidarTracker,relative_target,MapObstacleScan,plan_v3
from docking_mapping.geographic_map import GeographicMapStore
from test_mapping import config,structure
import test_geographic_map as fixtures
from isv_shape_navigation.core import Scan
from kaboat_avoidance.route_mission import RouteConfig
from kaboat_avoidance.avoidance_v3 import choose_gap_v3

class MapGoalTests(unittest.TestCase):
    def test_actual_saved_map_goal_upgrade_and_registration_geometry(self):
        fixture=Path(__file__).resolve().parents[1]/'actual_map'
        if not fixture.is_dir():self.skipTest('Isolated saved-map fixture not installed')
        with tempfile.TemporaryDirectory() as tmp:
            folders=list(fixture.glob('map_*'));self.assertEqual(len(folders),1)
            folder=Path(tmp)/folders[0].name;shutil.copytree(folders[0],folder)
            store=GeographicMapStore(tmp);old=json.loads((folder/'gps_target.json').read_text())
            points=np.load(folder/'docking_map.npz')['points']
            goal=store.upgrade_target(folder.name)
            self.assertEqual(goal['latitude'],old['latitude']);self.assertEqual(goal['longitude'],old['longitude'])
            self.assertEqual(len(points),692);self.assertEqual(len(goal['point_map_xy_m']),2)
            t=MapLidarTracker(points,np.eye(4),config())
            # Saved survey geometry itself must support a well-constrained pose.
            self.assertTrue(t.process(points,1.,np.eye(3),np.eye(4)),t.metrics)
            np.testing.assert_allclose(t.pose,np.eye(4),atol=.025)
            tampered=dict(goal,map_origin_lla=[0,0,0]);(folder/'gps_target.json').write_text(json.dumps(tampered))
            with self.assertRaises(ValueError):store.read_target(folder.name)
    def test_legacy_target_upgrades_without_changing_saved_coordinates_or_geometry(self):
        with tempfile.TemporaryDirectory() as tmp:
            mapper=fixtures.GeographicMapTests().sample();store=GeographicMapStore(tmp);folder=store.save(mapper)
            legacy=dict(schema=1,map_name=folder.name,latitude=35.12002,longitude=129.10003,updated_utc='original')
            target_file=folder/'gps_target.json';target_file.write_text(json.dumps(legacy))
            before=hashlib.sha256((folder/'docking_map.npz').read_bytes()).hexdigest()
            upgraded=store.upgrade_target(folder.name)
            self.assertEqual(upgraded['schema'],2);self.assertEqual(upgraded['updated_utc'],'original')
            self.assertEqual(upgraded['latitude'],legacy['latitude']);self.assertEqual(upgraded['longitude'],legacy['longitude'])
            self.assertEqual(upgraded['map_geometry_sha256'],before)
            self.assertEqual(store.read_target(folder.name)['point_map_xy_m'],upgraded['point_map_xy_m'])
            self.assertEqual(hashlib.sha256((folder/'docking_map.npz').read_bytes()).hexdigest(),before)
    def test_relative_target_uses_only_map_pose_and_map_target(self):
        pose=transform(Rotation.from_euler('z',90,degrees=True).as_matrix(),[2,3,10])
        goal=relative_target(pose,[2,8]);self.assertAlmostEqual(goal['distance_m'],5.)
        self.assertAlmostEqual(goal['bearing_deg'],0.,places=8)
        # There is deliberately no live GPS argument in the target calculation.
        for irrelevant_gps in [[0,0],[300,-500],[-1000,5000]]:
            self.assertEqual(relative_target(pose,[2,8]),goal)

class RegistrationTests(unittest.TestCase):
    def test_gicp_pose_tracks_and_ignores_gps_jump_after_initial_alignment(self):
        c=config();points=structure();tracker=MapLidarTracker(points,np.eye(4),c)
        before=tracker.points_map.copy();truth=transform(translation=[.2,-.1,.1]);seed=truth.copy();seed[0,3]+=1.
        self.assertTrue(tracker.process(apply(points,np.linalg.inv(truth)),1.,np.eye(3),seed),tracker.metrics)
        np.testing.assert_allclose(tracker.pose,truth,atol=.04)
        next_pose=transform(Rotation.from_euler('z',2,degrees=True).as_matrix(),[.3,-.05,.1])
        scan=apply(points,np.linalg.inv(next_pose));jump=next_pose.copy();jump[:3,3]+=[1000,-1000,500]
        self.assertTrue(tracker.process(scan,1.2,next_pose[:3,:3],jump),tracker.metrics)
        np.testing.assert_allclose(tracker.pose,next_pose,atol=.04)
        np.testing.assert_array_equal(tracker.points_map,before)
        self.assertFalse(tracker.points_map.flags.writeable)
    def test_no_initial_pose_no_overlap_and_stale_tracking_do_not_yield_a_valid_pose(self):
        t=MapLidarTracker(structure(),np.eye(4),config())
        with self.assertRaisesRegex(ValueError,'WAIT_INITIAL_ALIGNMENT'):t.process(structure(),1.,np.eye(3))
        self.assertFalse(t.process(np.zeros((3,3)),1.,np.eye(3),np.eye(4)))
        self.assertFalse(t.valid)

class V3AdapterTests(unittest.TestCase):
    def route(self):
        # The field YAML, including current gain/clearance/range, is read unchanged.
        path=Path(__file__).resolve().parents[1]/'course_mission_test.yaml'
        if not path.is_file():path=Path(__file__).resolve().parents[3]/'설정/course_mission.yaml'
        p=yaml.safe_load(path.read_text())['course_mission']['ros__parameters']
        return RouteConfig(**{f.name:p[f.name] for f in fields(RouteConfig) if f.name in p})
    def scan(self):return Scan(np.full(720,np.inf),-math.pi,2*math.pi/720,.1,25.,1.)
    def test_open_water_goal_and_arrival_without_any_output_interface(self):
        r=self.route();plan=plan_v3(np.eye(4),[5,0],np.empty((0,3)),self.scan(),r,.21)
        self.assertEqual(plan['state'],'APPROACH_TARGET');self.assertFalse(plan['hardware_output_enabled'])
        arrived=plan_v3(np.eye(4),[.1,0],np.empty((0,3)),self.scan(),r,.21)
        self.assertEqual(arrived['state'],'ARRIVED');self.assertEqual(arrived['pwm'],[1500,1500])
    def test_saved_map_obstacles_are_used_by_the_actual_v3_function(self):
        r=self.route();scan=self.scan();obstacles=np.array([[1.,y,z] for y in [-.12,-.06,0,.06,.12] for z in [-.1,.1]])
        plan=plan_v3(np.eye(4),[5,0],obstacles,scan,r,.21)
        fused=MapObstacleScan(scan,obstacles[:,:2]);angle,report=choose_gap_v3(fused,0.,r,.21)
        self.assertIsNotNone(angle);self.assertNotAlmostEqual(angle,0.)
        self.assertAlmostEqual(plan['selected_angle_deg'],math.degrees(angle))
        self.assertEqual(plan['avoidance']['selected_score'],report['selected_score'])
    def test_missing_scan_sectors_are_not_filled_as_free_space_by_saved_map(self):
        scan=self.scan();scan.ranges[:]=np.nan
        fused=MapObstacleScan(scan,[[1,0]])
        self.assertTrue(np.isnan(fused.ranges).all())
        plan=plan_v3(np.eye(4),[5,0],np.array([[1,0,0]]),scan,self.route(),.21)
        self.assertEqual(plan['pwm'],[1500,1500]);self.assertEqual(plan['state'],'BLOCKED')
    def test_live_obstacle_and_left_right_steering_keep_original_sign(self):
        route=self.route();scan=self.scan();scan.ranges[355:366]=1.
        plan=plan_v3(np.eye(4),[5,0],np.empty((0,3)),scan,route,.21)
        self.assertNotEqual(plan['state'],'APPROACH_TARGET')
        left=plan_v3(np.eye(4),[5,1],np.empty((0,3)),self.scan(),route,.21)
        right=plan_v3(np.eye(4),[5,-1],np.empty((0,3)),self.scan(),route,.21)
        self.assertLess(left['pwm'][0],left['pwm'][1]);self.assertGreater(right['pwm'][0],right['pwm'][1])

if __name__=='__main__':unittest.main()
