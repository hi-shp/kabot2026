"""Map geometry and automatic handoff tests; no ROS or actuator IO."""
import ast
from concurrent.futures import Future
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace as N
import unittest
from unittest.mock import Mock,patch

import numpy as np

from isv_water_trials.tank_map_file import crop_files, bundle, read_bundle
from isv_water_trials.tank_global_match import GlobalMatcher, partial_handoff_ready, handoff_ready, scan_points
from isv_water_trials.tank_mission import DockingMission, docking_points, arrive_once
from isv_water_trials.tank_safety import select_command
from isv_water_trials.tank_waypoints import mission_points
from isv_water_trials.tank_record import recordings,TOPICS
import time


def files(angle=0.):
    grid=np.zeros((80,100),dtype=np.int8)
    grid[10,10:90]=100
    return {'tank.posegraph':b'graph','tank.data':b'data','grid.bin':grid.tobytes(),
            'grid.json':json.dumps(dict(width=100,height=80,resolution=.05,position=[-2.,-1.,0.],
                orientation=[0.,0.,math.sin(angle/2),math.cos(angle/2)])).encode()}


class CropTests(unittest.TestCase):
    def test_crop_preserves_resolution_world_origin_and_graph(self):
        original=files();cropped=crop_files(original,[-1.,-.5,1.,1.])
        grid=json.loads(cropped['grid.json'])
        self.assertEqual((grid['width'],grid['height']), (40,30))
        self.assertEqual(grid['resolution'],.05)
        np.testing.assert_allclose(grid['position'],[-1.,-.5,0.])
        self.assertEqual(cropped['tank.posegraph'],original['tank.posegraph'])
        self.assertEqual(len(original['grid.bin']),8000)

    def test_rotated_crop_masks_world_rectangle_without_resampling(self):
        cropped=crop_files(files(math.pi/6),[-1.,0.,0.,1.])
        info=json.loads(cropped['grid.json']);cells=np.frombuffer(cropped['grid.bin'],dtype=np.int8).reshape(info['height'],info['width'])
        rows,cols=np.nonzero(cells>=0);co,si=math.cos(math.pi/6),math.sin(math.pi/6)
        x=info['position'][0]+co*(cols+.5)*.05-si*(rows+.5)*.05
        y=info['position'][1]+si*(cols+.5)*.05+co*(rows+.5)*.05
        self.assertTrue(np.all((x>=-1)&(x<=0)&(y>=0)&(y<=1)))

    def test_cropped_bundle_round_trip_and_existing_file_protection(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'dock.tankmap';cropped=crop_files(files(),[-1.,-.5,1.,1.])
            bundle(path,cropped,seed=[0.,0.,0.]);identity,manifest,loaded=read_bundle(path)
            self.assertTrue(identity);self.assertEqual(loaded['grid.bin'],cropped['grid.bin'])
            self.assertEqual(manifest['seed'],[0.,0.,0.])
            with self.assertRaises(FileExistsError):bundle(path,cropped)

    def test_empty_or_off_map_crop_does_not_publish(self):
        for bounds in ([0,0,0,1],[100,100,101,101],[0,0,float('nan'),1]):
            with self.assertRaises(ValueError):crop_files(files(),bounds)


class PartialMatchTests(unittest.TestCase):
    def candidate(self,**extra):
        return dict(x=0.,y=0.,yaw_deg=30.,score=.45,inliers=.08,known=.1,free=.9,**extra)

    def test_small_partial_match_does_not_need_second_candidate_or_large_margin(self):
        result=dict(candidates=[self.candidate()],point_count=160)
        self.assertTrue(partial_handoff_ready(result))
        self.assertFalse(handoff_ready(result))

    def test_empty_stale_and_nonfinite_candidates_cannot_handoff(self):
        for result in ({},dict(candidates=[self.candidate()],point_count=160,display_fallback=True),
                       dict(candidates=[dict(self.candidate(),x=float('nan'))],point_count=160),
                       dict(candidates=[dict(self.candidate(),inliers=.01)],point_count=160)):
            self.assertFalse(partial_handoff_ready(result))

    def test_partial_field_of_view_is_accepted_without_360_degrees(self):
        points=scan_points([3.]*24,-.3,.025,partial=True)
        self.assertEqual(len(points),24)
        with self.assertRaises(ValueError):scan_points([3.]*24,-.3,.025)

    def test_crop_external_returns_do_not_erase_good_local_overlap(self):
        info=dict(width=80,height=80,resolution=.05,position=[-2.,-2.,0.],orientation=[0.,0.,0.,1.])
        grid=np.zeros((80,80),dtype=np.int8);grid[60,20:60]=100
        # The visible local wall is a small portion of a much larger live scan.
        local=np.c_[np.linspace(-.7,.7,16),np.full(16,1.025)]
        outside=np.c_[np.full(144,15.),np.linspace(-5.,5.,144)]
        points=np.vstack((local,outside))
        matcher=GlobalMatcher(info,grid.tobytes(),partial=True)
        result=matcher.validate([dict(x=0.,y=0.,yaw_deg=0.)],points);result['point_count']=len(points)
        self.assertTrue(partial_handoff_ready(result))

    def test_vessel_can_match_crop_while_approaching_from_outside_it(self):
        original=files();cropped=crop_files(original,[-1.6,-.55,1.6,-.35])
        info=json.loads(cropped['grid.json'])
        matcher=GlobalMatcher(info,cropped['grid.bin'],partial=True,
            navigation_info=json.loads(cropped['navigation_grid.json']),navigation_cells=cropped['navigation_grid.bin'])
        points=np.c_[np.linspace(-1.,1.,24),np.full(24,-.475)]
        result=matcher.validate([dict(x=0.,y=0.,yaw_deg=0.)],points);result['point_count']=len(points)
        self.assertTrue(partial_handoff_ready(result))


class MissionTests(unittest.TestCase):
    def test_resume_skips_exactly_the_interrupted_gps_target(self):
        mission=DockingMission.begin('map',2,6,'run')
        self.assertEqual((mission.skipped_index,mission.resume_index),(2,3))

    def test_last_non_loop_target_finishes_without_restarting_route(self):
        self.assertIsNone(DockingMission.begin('map',2,3,'run').resume_index)
        self.assertEqual(DockingMission.begin('map',2,3,'run',loop=True).resume_index,0)

    def test_a_b_pass_completes_at_b_without_wrapping_back_to_a(self):
        goal,legs,complete=arrive_once(0,0,2,True)
        self.assertEqual((goal,legs,complete),(1,1,False))
        self.assertEqual(arrive_once(goal,legs,2,True),(1,2,True))

    def test_single_point_and_close_points_are_supported(self):
        route=dict(frame_id='tank_map',session_id='map',a=dict(x=1.,y=2.))
        self.assertEqual(docking_points(route,'map'),((1.,2.),))
        route['b']=dict(x=1.01,y=2.)
        self.assertEqual(len(docking_points(route,'map')),2)
        self.assertTrue(arrive_once(0,0,1,True)[2])
        self.assertEqual(mission_points(dict(a=dict(x=1.,y=2.))),[('A',(1.,2.))])

    def test_other_map_route_cannot_be_started(self):
        with self.assertRaises(ValueError):docking_points(dict(frame_id='tank_map',session_id='other'),'map')


class HandoffTests(unittest.TestCase):
    def setUp(self):
        tree=ast.parse((Path(__file__).parents[1]/'isv_water_trials/outdoor.py').read_text(encoding='utf-8'))
        method=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='tick_priority_route')
        namespace=dict(json=json,Path=Path,math=math,DockingMission=DockingMission,docking_points=docking_points,
                       uuid=N(uuid4=lambda:N(hex='run')))
        exec(compile(ast.Module(body=[method],type_ignores=[]),'<handoff>','exec'),namespace)
        self.tick=namespace['tick_priority_route']
        self.folder=tempfile.TemporaryDirectory();self.addCleanup(self.folder.cleanup)
        self.marker=Path(self.folder.name)/'complete.json'

    def owner(self,**result):
        mission=DockingMission.begin('map',1,4,'run')
        self.marker.write_text(json.dumps(dict(mission_id='run',complete=True,neutral_confirmed=True,**result)))
        process=Mock();process.poll.return_value=0;process.returncode=0
        return N(execute=True,test_course=None,priority_stage='docking',priority_process=process,
                 priority_completion=self.marker,priority_mission=mission,arrivals=0,plan_generation=0,
                 goal=1,send=Mock(),priority_used_maps={'map'},tank_session_data={},tank_session_at=0.,
                 actuator_at=100.,actuator=dict(mode_name='AUTO'))

    def test_gps_resume_waits_until_docking_child_has_exited(self):
        owner=self.owner();owner.priority_process.poll.return_value=None
        self.assertTrue(self.tick(owner,100.))
        self.assertEqual(owner.goal,1);self.assertEqual(owner.arrivals,0);owner.send.assert_not_called()

    def test_completion_advances_target_and_counts_one_gps_arrival(self):
        owner=self.owner();self.tick(owner,100.)
        self.assertEqual((owner.goal,owner.arrivals,owner.priority_stage),(2,1,'gps'))
        self.tick(owner,100.1)
        self.assertEqual(owner.arrivals,1)

    def test_missing_neutral_confirmation_prevents_gps_resume(self):
        owner=self.owner();self.marker.write_text(json.dumps(dict(mission_id='run',complete=True,neutral_confirmed=False)))
        self.tick(owner,100.)
        self.assertEqual((owner.goal,owner.arrivals,owner.priority_stage),(1,0,'error'))
        owner.send.assert_called_once_with([0.,0.],100.,disarm=True)

    def test_stale_completion_from_another_run_is_ignored(self):
        owner=self.owner();self.marker.write_text(json.dumps(dict(mission_id='old',complete=True,neutral_confirmed=True)))
        self.tick(owner,100.)
        self.assertEqual(owner.goal,1);self.assertEqual(owner.priority_stage,'error')

    def test_neutralizing_does_not_launch_until_old_command_finishes(self):
        owner=self.owner();owner.priority_stage='neutralizing';owner.priority_started=99.9
        owner.pending=[Future()];owner.actuator.update(armed=False,receiving=True)
        self.tick(owner,100.)
        self.assertEqual(owner.priority_stage,'neutralizing')
        owner.send.assert_called_once_with([0.,0.],100.,disarm=True)

    def visible_owner(self):
        owner=self.owner();owner.priority_stage='gps';owner.priority_used_maps=set()
        owner.tank_session_data=dict(session_id='map',fixed_map=True,localization_ready=True)
        owner.tank_session_at=100.;owner.tank_match_at=100.;owner.ros_now=lambda:100.
        owner.tank_match_data=dict(map_id='map',docking_match_ready=True,docking_match_stamp_s=100.)
        owner.direct_follow=True;owner.follow_position_source='gps';owner.get_node_names=lambda:[]
        owner.route=dict(points=[[35.,129.],[35.1,129.],[35.2,129.],[35.3,129.]])
        return owner

    def test_old_localization_without_current_visible_match_does_not_trigger(self):
        owner=self.visible_owner();owner.tank_match_data['docking_match_ready']=False
        self.assertFalse(self.tick(owner,100.));owner.send.assert_not_called()
        owner.tank_match_data.update(docking_match_ready=True,docking_match_stamp_s=90.)
        self.assertFalse(self.tick(owner,100.));owner.send.assert_not_called()

    def test_current_match_neutralizes_before_start_and_does_not_skip_early(self):
        owner=self.visible_owner();folder=Path(self.folder.name)
        route=folder/'water_trials/tank/route.json';route.parent.mkdir(parents=True)
        route.write_text(json.dumps(dict(session_id='map',frame_id='tank_map',a=dict(x=1.,y=2.),b=dict(x=2.,y=2.))))
        with patch.object(Path,'home',return_value=folder):self.assertTrue(self.tick(owner,100.))
        self.assertEqual((owner.goal,owner.arrivals,owner.priority_stage),(1,0,'neutralizing'))
        self.assertEqual(owner.priority_mission.resume_index,2)
        owner.send.assert_called_once_with([0.,0.],100.,disarm=True)


class DockingAvoidanceTests(unittest.TestCase):
    def test_b_leg_commands_never_use_forward_thrust(self):
        scan=N(ranges=[10.]*360,range_min=.1,range_max=30.,angle_min=-math.pi,angle_increment=math.pi/180)
        grid=N(line_clear=lambda a,b:True)
        command,info=select_command(grid,scan,[0.,0.],0.,[0.,0.,0.],[-2.,0.],[-2.,0.],0.,0.,
                                    excite=False,reverse_only=True)
        self.assertLess(sum(command),0.);self.assertLessEqual(max(command),0.);self.assertFalse(info['reason'])

    def test_open_water_no_returns_work_but_missing_measurements_still_hold(self):
        grid=N(line_clear=lambda a,b:True)
        scan=N(ranges=[math.inf]*360,range_min=.1,range_max=30.,angle_min=-math.pi,angle_increment=math.pi/180)
        command,info=select_command(grid,scan,[0.,0.],0.,[0.,0.,0.],[2.,0.],[2.,0.],0.,0.,excite=False,open_water=True)
        self.assertGreater(sum(command),0.);self.assertFalse(info['reason'])
        scan.ranges=[math.nan]*360
        command,info=select_command(grid,scan,[0.,0.],0.,[0.,0.,0.],[2.,0.],[2.,0.],0.,0.,excite=False,open_water=True)
        self.assertEqual(command,(0.,0.));self.assertTrue(info['reason'])


class RecordingTests(unittest.TestCase):
    def test_reconnection_lists_previous_bags_without_replacing_them(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);first=root/'tank_20261111T000001Z_a';first.mkdir();(first/'data.db3').write_bytes(b'old')
            second=root/'tank_20261111T000002Z_b';second.mkdir();(second/'data.db3').write_bytes(b'new')
            (root/'recording_state.json').write_text(json.dumps(dict(state='recording',updated_unix_s=time.time(),current_bag=str(second))))
            value=recordings(root)
            self.assertEqual(value['state']['state'],'recording');self.assertEqual(len(value['sessions']),2)
            self.assertEqual((first/'data.db3').read_bytes(),b'old')
            self.assertEqual(recordings(root)['sessions'],value['sessions'])

    def test_compact_recording_covers_rc_sensors_and_commands(self):
        for topic in ('/scan','/gps/fix','/gps/fix_velocity','/vectornav/imu','/actuators/status','/outdoor/command'):
            self.assertIn(topic,TOPICS)
        self.assertNotIn('/velodyne_points',TOPICS)


if __name__=='__main__':unittest.main()
