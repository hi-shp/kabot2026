"""Offline tests for YAML goals, ENU guidance, arrival and invalid inputs."""
import math
from pathlib import Path
import tempfile
import unittest
import pymap3d as pm
import yaml
from isv_shape_navigation.path_follower import PathFollower, _norm180

class PathFollowerTests(unittest.TestCase):
    origin=(35.23,129.08,0.)
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.config=Path(self.temp.name)/'route.yaml'
    def tearDown(self): self.temp.cleanup()
    def goals(self,*points):
        result=[]
        for e,n in points:
            lat,lon,_=pm.enu2geodetic(e,n,0.,*self.origin)
            result.append([float(lat),float(lon)])
        return result
    def follower(self,goals,radius=1.,override=None):
        self.config.write_text(yaml.safe_dump({'path_follower':{'goal_gps_coords':goals,'arrive_radius_m':radius}}))
        return PathFollower(self.origin,arrive_radius_m=override,config_path=self.config)
    def test_cardinal_bearings_are_north_zero_clockwise(self):
        for xy,angle in [((0,10),0),((10,0),90),((0,-10),-180),((-10,0),-90)]:
            f=self.follower(self.goals(xy));self.assertAlmostEqual(_norm180(f.update(0,0,0)-angle),0.,places=5)
    def test_heading_wrap_chooses_shortest_signed_turn(self):
        f=self.follower(self.goals((0,10)))
        self.assertAlmostEqual(f.update(0,0,359),1.,places=5)
        self.assertAlmostEqual(f.update(0,0,1),-1.,places=5)
        self.assertEqual(_norm180(180),-180)
    def test_yaml_is_authoritative_for_coordinates(self):
        targets=self.goals((12,4),(20,-3));f=self.follower(targets)
        self.assertEqual(f.goal_gps_coords,[tuple(p) for p in targets])
        self.assertAlmostEqual(f.goal_xy[0][0],12.,places=5)
        self.assertAlmostEqual(f.goal_xy[0][1],4.,places=5)
    def test_arrival_advances_and_final_target_marks_done(self):
        f=self.follower(self.goals((0,10),(10,10)))
        x,y=f.goal_xy[0];self.assertAlmostEqual(f.update(x,y,0),90.,places=5)
        self.assertEqual(f.idx,1);self.assertFalse(f.done())
        self.assertEqual(f.update(*f.goal_xy[1],90),0.)
        self.assertTrue(f.done());self.assertIsNone(f.current_target_xy());self.assertIsNone(f.current_target_gps())
    def test_outside_arrival_radius_does_not_advance(self):
        f=self.follower(self.goals((10,0)))
        x,y=f.goal_xy[0];f.update(x-1.01,y,90)
        self.assertFalse(f.done());f.update(x-.99,y,90);self.assertTrue(f.done())
    def test_overlapping_targets_are_consumed_together(self):
        f=self.follower([list(self.origin[:2])]*3)
        self.assertEqual(f.update(0,0,0),0.);self.assertTrue(f.done())
    def test_empty_route_is_already_complete(self):
        f=self.follower([]);self.assertTrue(f.done());self.assertEqual(f.update(0,0,30),0.)
    def test_explicit_radius_overrides_yaml(self):
        f=self.follower(self.goals((10,0)),radius=1.,override=3.)
        self.assertEqual(f.arrive_radius,3.)
    def test_invalid_coordinate_entries_rejected(self):
        for goals in [[[91,129]],[[35,181]],[[True,129]],[[float('nan'),129]],[[35]],['35,129']]:
            with self.subTest(goals=goals),self.assertRaises(ValueError): self.follower(goals)
    def test_invalid_radius_rejected(self):
        for radius in [0,-1,float('inf'),True,'one']:
            with self.subTest(radius=radius),self.assertRaises(ValueError): self.follower([],radius)
    def test_invalid_runtime_values_do_not_advance_target(self):
        f=self.follower(self.goals((10,0)))
        with self.assertRaises(ValueError): f.update(10,0,float('nan'))
        self.assertEqual(f.idx,0)
    def test_invalid_origin_rejected(self):
        self.follower([])
        for origin in [(35,129),(91,129,0),(35,129,float('nan'))]:
            with self.assertRaises(ValueError): PathFollower(origin,config_path=self.config)
    def test_yaml_missing_section_or_goals_rejected(self):
        for data in [None,[],{}, {'path_follower':{}},{'path_follower':{'goal_gps_coords':'bad'}}]:
            self.config.write_text(yaml.safe_dump(data))
            with self.assertRaises(ValueError): PathFollower(self.origin,config_path=self.config)
    def test_default_config_resolves_without_ros_node(self):
        f=PathFollower(self.origin)
        self.assertTrue(f.config_path.is_file());self.assertEqual(f.config_path.name,'path_follower.yaml')
    def test_missing_file_is_reported(self):
        with self.assertRaises(FileNotFoundError): PathFollower(self.origin,config_path=self.config)

if __name__=='__main__': unittest.main()
