"""No actuator access: collision envelopes, recording recovery and model limits."""
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'isv_water_trials'))
from test_tank import message
from types import SimpleNamespace as N
from isv_water_trials.tank_core import Grid
from isv_water_trials.tank_safety import excitation, select_command, trajectory_clear
from isv_water_trials.tank_storage import Journal, read_rows, atomic_json
from isv_water_trials.tank_model import predict
from isv_water_trials.indoor_analyze import main as analyze, fit_channel
from isv_water_trials.tank_session import command_for, identity


def room_scan(position, heading=0., obstacle=None):
    # Rectangle: x 2..14, y -1..11 (same origin as test grid).
    values = []
    for angle in np.linspace(-math.pi,math.pi,180,endpoint=False):
        dx,dy = math.cos(angle+heading),math.sin(angle+heading)
        distances = [(b-position[0])/dx for b in (2.,14.) if abs(dx)>1e-9]
        distances += [(b-position[1])/dy for b in (-1.,11.) if abs(dy)>1e-9]
        value = min(d for d in distances if d>0)
        if obstacle and abs(math.atan2(obstacle[1]-position[1],obstacle[0]-position[0])-angle-heading)<.03:
            value = min(value,math.dist(obstacle,position))
        values.append(value)
    return N(ranges=values,angle_min=-math.pi,angle_increment=2*math.pi/180,range_min=.1,range_max=30.)


class ExperimentTests(unittest.TestCase):
    def setUp(self): self.grid = Grid(message(np.zeros((120,120))),.75)

    def command(self, position=(6.,5.), velocity=(.1,0.,0.), scan=None, t=0.):
        return select_command(self.grid,scan or room_scan(position),position,0.,velocity,(11.,5.),(11.,5.),0.,t)

    def test_clear_path_cruises_at_twenty(self):
        output,info = self.command()
        self.assertEqual(output,(20.,20.)); self.assertEqual(info['phase'],'cruise')

    def test_schedule_has_coast_and_independent_steering(self):
        samples = [excitation(t) for t in np.arange(0,36,.25)]
        self.assertTrue(any(base==0 for base,_,_ in samples))
        self.assertTrue(any(turn>3 for _,turn,_ in samples))
        self.assertTrue(any(turn< -3 for _,turn,_ in samples))
        for t in np.arange(0,36,1.5):
            output,_ = self.command(t=t)
            self.assertLessEqual(max(abs(v) for v in output),20.)

    def test_all_missing_or_partial_scan_stops(self):
        for count in (180,20):
            scan = room_scan((6.,5.)); scan.ranges[:count] = [float('inf')]*count
            output,info = self.command(scan=scan)
            self.assertEqual(output,(0.,0.)); self.assertEqual(info['phase'],'HOLD')

    def test_inertia_near_wall_stops_early(self):
        output,info = self.command(position=(12.,5.),velocity=(.5,0.,0.))
        self.assertEqual(output,(0.,0.)); self.assertIn('타력',info['reason'])

    def test_lateral_drift_is_not_ignored(self):
        output,info = self.command(position=(7.,9.4),velocity=(0.,.4,0.))
        self.assertEqual(output,(0.,0.))

    def test_dynamic_scan_obstacle_blocks_static_free_map(self):
        output,info = self.command(velocity=(.3,0.,0.),scan=room_scan((6.,5.),obstacle=(7.,5.)))
        self.assertEqual(output,(0.,0.))

    def test_invalid_velocity_stops(self):
        for v in (None,(float('nan'),0.,0.),(.7,0.,0.)):
            self.assertEqual(self.command(velocity=v)[0],(0.,0.))

    def test_near_wall_disables_excitation(self):
        _,info = self.command(position=(6.,.5),velocity=(0.,0.,0.),t=19.)
        self.assertNotEqual(info['phase'],'coast')

    def test_subsample_obstacle_cannot_be_tunnelled(self):
        self.assertFalse(trajectory_clear(self.grid,[(6.,5.),(8.,5.)],np.array([[7.,5.]]),.75))

    def test_power_loss_truncated_tail_and_atomic_checkpoint(self):
        with tempfile.TemporaryDirectory() as path:
            journal = Journal(path,sync_seconds=0.,segment_seconds=0.)
            journal.write({'t':0,'goal':'A'}); journal.write({'t':1,'goal':'B'}); journal.close()
            with (Path(path)/'samples-000002.jsonl').open('ab') as f: f.write(b'{"t":')
            self.assertEqual(len(list(read_rows(path))),2)
            self.assertEqual(json.loads((Path(path)/'checkpoint.json').read_text())['goal'],'B')
            self.assertEqual(list(Path(path).glob('*.incoming')),[])

    def test_middle_corruption_is_not_silently_discarded(self):
        with tempfile.TemporaryDirectory() as path:
            (Path(path)/'samples.jsonl').write_text('{bad}\n',encoding='utf-8')
            with self.assertRaises(ValueError): list(read_rows(path))

    def test_prediction_rejects_unqualified_or_extrapolated_model(self):
        channel = dict(qualified=True,observed_command_range=[-.2,.2],observed_velocity_range=[-.5,.5],
                       decay_per_s=.7,pwm_gain=.8,bias=0.)
        model = dict(surge=channel,yaw=channel)
        self.assertIsNotNone(predict(model,(0,0),0,(0,0,0),20,20))
        self.assertIsNone(predict(model,(0,0),0,(0,0,0),50,50))
        self.assertIsNone(predict({},(0,0),0,(0,0,0),20,20))

    def test_holdout_horizons_and_coefficient_interval(self):
        t = np.arange(0,80,.1); q = .15*np.sin(t*.4)+.04*np.sin(t*1.1); v = np.zeros(len(t))
        for i in range(len(t)-1): v[i+1]=v[i]+.1*(-.6*v[i]+.9*q[i])
        result,_ = fit_channel(t,v,q)
        self.assertTrue(result['qualified'],result)
        self.assertEqual(set(result['horizon_validation']),{'1','2','3'})
        self.assertIn('decay_per_s',result['coefficient_ci95'])

    def test_preview_and_service_commands_do_not_implicitly_execute(self):
        self.assertNotIn('--execute',command_for('trial',[]))
        self.assertIn('all_sensors.launch.py',command_for('sensors',[]))
        self.assertIsNone(identity(999999999))

    def test_analysis_handles_empty_and_torn_recording(self):
        with tempfile.TemporaryDirectory() as path:
            root=Path(path)
            atomic_json(root/'metadata.json',dict(args={},mass_kg=25,length_m=1.1,width_m=.5,thruster_spacing_m=.32))
            (root/'samples-000001.jsonl').write_text('{"t":',encoding='utf-8')
            analyze([path])
            self.assertTrue((root/'analysis.html').exists())
            self.assertEqual(json.loads((root/'response_model.json').read_text(encoding='utf-8'))['samples'],0)

    def test_end_to_end_synthetic_logs_reports_and_multistep_validation(self):
        with tempfile.TemporaryDirectory() as path:
            root=Path(path)
            atomic_json(root/'metadata.json',dict(args={},mass_kg=25,length_m=1.1,width_m=.5,thruster_spacing_m=.32,synthetic=True))
            x=y=a=u=r=0.
            with (root/'samples.jsonl').open('w',encoding='utf-8') as stream:
                for i in range(1000):
                    t=i*.1; common=.09+.05*math.sin(t*.4); diff=.05*math.sin(t*.8)
                    left,right=round(1500+400*(common-diff)),round(1500+400*(common+diff))
                    stream.write(json.dumps(dict(t=t,execute=True,ack=True,state='RUNNING',
                        mcu=dict(receiving=True,mode=2,armed=True,age_s=.01,left=left,right=right),
                        odom=dict(x=x,y=y,yaw_rad=a,stamp_s=1000+t,age_s=.01),
                        imu=dict(gyro_sensor_z_rad_s=r,age_s=.01,accel_m_s2=[0,0,9.8])))+'\n')
                    u+=.1*(-.7*u+.8*common);r+=.1*(-.9*r+1.2*diff);a+=.1*r
                    x+=.1*u*math.cos(a); y+=.1*u*math.sin(a)
            analyze([path]); analyze([path])
            report=json.loads((root/'response_model.json').read_text(encoding='utf-8'))
            self.assertGreater(report['auto_samples'],900)
            self.assertIn('3',report['trajectory_validation'])
            self.assertTrue((root/'motion.csv').exists())
            self.assertIn('시정수 추정 이력',(root/'analysis.html').read_text(encoding='utf-8'))


if __name__=='__main__': unittest.main()
