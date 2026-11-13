import math
import unittest
import numpy as np
from isv_shape_navigation.core import Pose,Scan
from isv_shape_navigation.navigation_display import SlamDisplay,decision_display,direction,LidarDisplay


class LidarDisplayTests(unittest.TestCase):
    def test_preprocessing_timeout_does_not_clear_raw_display(self):
        display=LidarDisplay();scan=Scan(np.array([2.,1.]),0.,.1,.1,30.,100.)
        display.observe(scan,10.)
        live=display.snapshot(100.2,10.2)
        self.assertEqual(live['status'],'LIVE');self.assertEqual(len(live['points']),2)
        delayed=display.snapshot(100.7,10.7)
        self.assertEqual(delayed['status'],'DELAYED');self.assertEqual(delayed['points'],live['points'])
        self.assertEqual(SlamDisplay().snapshot(100.7,scan,.5)['body_points'],[])
        lost=display.snapshot(103.,13.)
        self.assertEqual(lost['status'],'STALE');self.assertEqual(lost['points'],[])

    def test_each_raw_frame_replaces_previous_observation(self):
        display=LidarDisplay()
        display.observe(Scan(np.array([2.]),0.,.1,.1,30.,100.),10.)
        display.observe(Scan(np.array([1.]),0.,.1,.1,30.,100.1),10.1)
        self.assertEqual(display.snapshot(100.1,10.1)['points'],[[1.,0.]])
        display.observe(Scan(np.array([np.inf]),0.,.1,.1,30.,100.2),10.2)
        self.assertEqual(display.snapshot(100.2,10.2)['points'],[])

    def test_unreceived_scan_is_waiting(self):
        self.assertEqual(LidarDisplay().snapshot(100.,10.)['status'],'WAIT')


class DecisionDisplayTests(unittest.TestCase):
    def test_gap_width_and_signed_waypoint_bend(self):
        report=dict(command_kind='AVOIDANCE',heading_error_deg=16.,yaw=0.,
            avoidance=dict(selected_angle_deg=28.,gaps=[dict(right_deg=20.,left_deg=50.)]))
        result=decision_display(report)
        self.assertEqual(result['turn_command_deg'],28.)
        self.assertEqual(result['wp_offset_deg'],12.)
        self.assertEqual(result['gaps'][0]['width_deg'],30.)
        self.assertTrue(result['gaps'][0]['selected'])
        self.assertEqual(direction(result['wp_offset_deg']),'좌 12.0°')

    def test_direct_command_is_not_misreported_as_avoidance_candidate(self):
        report=dict(command_kind='HEADING',heading_error_deg=100.,yaw=0.,
                    avoidance=dict(selected_angle_deg=75.,gaps=[]))
        self.assertEqual(decision_display(report)['turn_command_deg'],100.)

    def test_stop_has_no_turn_command(self):
        self.assertIsNone(decision_display(dict(command_kind='NEUTRAL',heading_error_deg=20.))['turn_command_deg'])

    def test_actual_heading_change_is_separate_from_command(self):
        display=SlamDisplay()
        report=dict(wp_index=1,stage='WP1_TO_WP2',yaw=0.,heading_error_deg=10.,command_kind='HEADING')
        display.angles(report)
        report['yaw']=math.radians(7.)
        result=display.angles(report)
        self.assertAlmostEqual(result['actual_heading_change_deg'],7.)
        self.assertEqual(result['turn_command_deg'],10.)


class SlamDisplayTests(unittest.TestCase):
    def setUp(self):
        self.display=SlamDisplay()
        self.scan=Scan(np.array([2.,np.inf,np.nan]),0.,.1,.2,30.,100.,0.,0.,0.)

    def test_world_projection_uses_slam_pose_and_scan_extrinsics(self):
        self.display.session('map-1',True,100.)
        self.display.pose(Pose(10.,20.,math.pi/2,100.))
        result=self.display.snapshot(100.,self.scan)
        np.testing.assert_allclose(result['world_points'][0],[10.,22.])
        self.assertEqual(result['closest']['distance'],2.)
        np.testing.assert_allclose(result['closest']['map_xy'],[10.,22.])

    def test_no_pose_or_bad_time_does_not_invent_map_obstacles(self):
        result=self.display.snapshot(100.,self.scan)
        self.assertEqual(result['world_points'],[])
        self.assertEqual(result['body_points'],[[2.,0.]])
        self.display.session('map-1',True,100.)
        self.display.pose(Pose(10.,20.,0.,99.6))
        self.assertEqual(self.display.snapshot(100.,self.scan)['world_points'],[])

    def test_map_reset_clears_old_pose_obstacles_and_trail(self):
        self.display.session('one',True,100.)
        self.display.pose(Pose(1.,2.,0.,100.))
        self.display.occupancy([0]*100,10,10,.1,(0.,0.,0.))
        self.display.session('two',True,100.1)
        result=self.display.snapshot(100.1,self.scan)
        self.assertIsNone(result['map']);self.assertIsNone(result['pose'])
        self.assertEqual(result['world_points'],[]);self.assertEqual(result['trail'],[])

    def test_downsampling_preserves_occupied_cells(self):
        cells=np.zeros((200,200),dtype=np.int8);cells[11,11]=100;cells[:2,:]=-1
        self.display.occupancy(cells.ravel().tolist(),200,200,.1,(3.,4.,.5))
        grid=self.display.grid
        self.assertEqual(grid['origin'],[3.,4.,.5])
        self.assertTrue(any(run[3]==100 for run in grid['runs']))
        self.assertLessEqual(grid['rows'],140)

    def test_stale_scan_is_not_kept_as_live_obstacles(self):
        self.assertEqual(self.display.snapshot(101.,self.scan)['body_points'],[])

    def test_map_bad_geometry_is_rejected(self):
        with self.assertRaises(ValueError):self.display.occupancy([0],10,10,.1,(0.,0.,0.))


if __name__=='__main__':unittest.main()
