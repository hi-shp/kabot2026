import json
import math
from pathlib import Path
import tempfile
import time
import unittest
import numpy as np
from isv_water_trials.outdoor_model import (ResponseModel, applied_command, logical_pwm, local_xy,
    latlon, fit_model, _grid, fingerprint, LiveComparison)
from isv_water_trials.outdoor_state import OutdoorState
from isv_water_trials.outdoor_store import OutdoorStore
from isv_water_trials.outdoor_planner import OutdoorPlanner

from isv_water_trials.outdoor_config import load_config
CONFIG = load_config(Path(__file__).parents[1]/'config/outdoor.json')


def known_model():
    c = np.zeros((8, 3)); c[:3] = np.diag([.85, .75, .8]); c[3] = [.1, .015, -.12]; c[4] = [.1, -.015, .12]
    c[5] = [.03, 0, -.02]; c[6] = [.03, 0, .02]
    return {'schema': 'outdoor-response-v1', 'dt_s': .2, 'delay_steps': 2, 'coefficients': c.tolist(),
            'signature': fingerprint(CONFIG), 'id': '0123456789abcdef', 'qualified': True,
            'command_min': [-.8, -.8], 'command_max': [.8, .8], 'velocity_limit': [.8, .2, .6]}


def synthetic_rows(count=1100, noisy=True):
    rng = np.random.default_rng(1907); model = ResponseModel(known_model())
    q = np.repeat(rng.uniform(-.65, .65, size=(math.ceil(count/7), 2)), 7, axis=0)[:count]
    z = np.zeros(3); rows = []; history = []
    for i in range(count):
        stamp = 1000+i*.2; history.append([stamp, *q[i]])
        rows.append({'t': stamp, 'velocity': (z+rng.normal(0,.002,3) if noisy else z).tolist(),
                     'valid': True, 'mode': 'RC', 'segment': 'one', 'commands': history[-30:]})
        z = model.step(z, q[i-2] if i >= 2 else [0,0])
    return rows


class OutdoorTests(unittest.TestCase):
    def test_output_mapping_uses_observed_not_rc_logical_polarity(self):
        s = {'receiving': True, 'age_s': .01, 'mode': 1, 'left': 1800, 'right': 1200,
             'observed_left_us': 1290, 'observed_right_us': 1290}
        np.testing.assert_allclose(applied_command(s, CONFIG), [.5, .5])
        self.assertEqual(logical_pwm([.5,.5]), [1700,1700])
        s['observed_right_us'] = 1690
        np.testing.assert_allclose(applied_command(s, CONFIG), [.5,-.5])
        s['receiving'] = False; self.assertIsNone(applied_command(s, CONFIG))

    def test_esc_neutral_is_zero_and_invalid_pulse_not_training(self):
        s = dict(receiving=True, age_s=0, observed_left_us=1480, observed_right_us=1480)
        self.assertEqual(applied_command(s,CONFIG), [0,0]);s['observed_right_us']=0
        self.assertIsNone(applied_command(s,CONFIG))

    def test_geographic_round_trip(self):
        origin=[35.23,129.08]; p=[35.2301,129.0802]
        np.testing.assert_allclose(latlon(*local_xy(*p,origin),origin),p,atol=1e-9)

    def test_state_ned_to_enu_and_lever_arm(self):
        c=dict(CONFIG,gps_lever_arm_m=[.5,0]); s=OutdoorState(c)
        s.imu(100,10,90,-.2); s.fix(100,10,35,129,0,[.25,0,0,0,.25,0,0,0,1],2)
        cov=[0.]*36;cov[0]=cov[7]=.01
        f=s.velocity(100,10,1,0,cov)
        self.assertAlmostEqual(f['yaw'],0);np.testing.assert_allclose(f['velocity'],[1,-.1,.2])
        self.assertAlmostEqual(f['position'][0],-.5);self.assertIsNone(s.snapshot(12))

    def test_bad_fix_does_not_get_trusted(self):
        s=OutdoorState(CONFIG);s.imu(100,10,0,0)
        s.fix(100,10,35,129,-1,[1]*9,2)
        self.assertIsNone(s.velocity(100,10,0,0,[.01]*36))

    def test_gps_display_before_velocity_and_scan_time_pose(self):
        s=OutdoorState(CONFIG);s.imu(100,10,90,0)
        s.fix(100,10,35,129,0,[.25,0,0,0,.25,0,0,0,1],2)
        self.assertIsNotNone(s.display_pose(10.1));self.assertIsNone(s.snapshot(10.1))
        cov=[0.]*36;cov[0]=cov[7]=.01;s.velocity(100,10,1,0,cov)
        s.imu(100.1,10.1,90,0)
        self.assertAlmostEqual(s.pose_at(100.1)['position'][0],.1)
        self.assertIsNone(s.pose_at(105))

    def test_unknown_covariance_and_time_mismatch(self):
        s=OutdoorState(CONFIG);s.imu(90,10,0,0);s.fix(100,10,35,129,0,[.01]*9,2)
        self.assertIsNone(s.velocity(100,10,0,0,[.01]*36))
        s.imu(101,10,0,0);self.assertIsNone(s.velocity(101,10,0,0,[0]*36))

    def test_restart_segments_never_interpolate_across_gap(self):
        rows=synthetic_rows(100)
        for row in rows[50:]: row['segment']='two'; row['t']+=20
        runs=_grid(rows,.2);self.assertEqual(len(runs),2)
        self.assertLess(runs[0][0][-1],runs[1][0][0]-19)

    def test_paused_data_is_recorded_but_not_fitted(self):
        rows=synthetic_rows(100)
        for r in rows[30:70]:r['valid']=False
        self.assertEqual(len(_grid(rows,.2)),2)

    def test_store_survives_restart_and_preserves_models(self):
        with tempfile.TemporaryDirectory() as tmp:
            store=OutdoorStore(tmp,CONFIG); row=synthetic_rows(1)[0];store.add(row)
            model=known_model();store.save_model(model);store.new_segment('pause');store.close()
            store=OutdoorStore(tmp,CONFIG);self.assertEqual(len(store.rows()),1)
            store.save_model(model);store.close()
            self.assertEqual(len(list((Path(tmp)/'models').rglob('0123456789abcdef.json'))),1)

    def test_predict_open_loop_delay(self):
        m=ResponseModel(known_model()); pred=m.predict([0,0,0],np.ones((5,2))*.2)
        np.testing.assert_allclose(pred[:2],0);self.assertGreater(pred[2,0],0)

    def test_dated_models_and_finalization_do_not_overwrite(self):
        with tempfile.TemporaryDirectory() as tmp:
            s=OutdoorStore(tmp,CONFIG);model=known_model();path=s.save_model(model)
            self.assertRegex(path.parent.name,r'^\d{4}-\d{2}-\d{2}$')
            before=path.read_bytes();s.finalize_model(model)
            self.assertEqual(path.read_bytes(),before);self.assertTrue(s.model_catalog()[0]['finalized'])
            s.position(100,35,129);s.position(101,35.1,129)
            self.assertEqual(len(s.trail(after=100.5)),1);s.close()

    def test_upstream_and_operating_axes_are_preserved(self):
        import hashlib
        package=Path(__file__).parents[1]/'isv_water_trials/codex_nav'
        manifest=json.loads((package/'UPSTREAM.json').read_text(encoding='utf-8'))
        for f in manifest['files']:
            text=(package/f['path']).read_text(encoding='utf-8').replace('from isv_water_trials.codex_nav.','from ')
            value=text.encode();blob=hashlib.sha1(b'blob '+str(len(value)).encode()+b'\0'+value).hexdigest()
            self.assertEqual(blob,f['git_blob_sha'],f['path'])
        live=json.loads((Path(__file__).parents[1]/'config/boat_live.json').read_text())
        self.assertEqual(CONFIG['lidar_yaw_deg'],live['lidar_yaw_deg'])
        self.assertEqual(CONFIG['heading_offset_deg'],live['vectornav_yaw_offset_deg'])

    def test_live_comparison_without_resetting_to_measurement(self):
        m=ResponseModel(known_model());comp=LiveComparison();rows=synthetic_rows(30,False);result=None
        for r in rows:result=comp.update(m,r['t'],r['velocity'],r['commands']) or result
        self.assertIsNotNone(result);self.assertLess(abs(result['predicted'][0]-result['measured'][0]),.06)

    def test_multi_candidate_fit_recovers_delay_and_heldout_prediction(self):
        data=fit_model(synthetic_rows(),CONFIG)
        self.assertEqual(data['delay_steps'],2)
        self.assertTrue(data['qualified'],data['reasons']);self.assertGreater(data['candidate_count'],10)
        self.assertLess(data['metrics']['horizons']['2']['velocity_rmse'][0],.03)
        self.assertGreater(data['metrics']['windows'],20)

    def test_insufficient_data_rejected(self):
        with self.assertRaises(ValueError):fit_model(synthetic_rows(20),CONFIG)

    def test_planner_open_water_uses_learned_model_and_cap(self):
        p=OutdoorPlanner(known_model(),CONFIG,20)
        state={'position':[0,0],'yaw':0,'velocity':[0,0,0],'goal':[8,0]}
        scan={'angles':np.linspace(-math.pi,math.pi,720,endpoint=False),'ranges':np.full(720,np.inf),'range_min':.1}
        history=[[100+i*.1,0,0] for i in range(21)]
        result=p.plan(state,scan,history,102)
        self.assertTrue(all(abs(v)<=.2+1e-9 for v in result['command']))
        self.assertGreater(sum(result['command']),0)
        self.assertGreater(result['path'][-1][0],0)

    def test_planner_blocked_hull_does_not_send_motion(self):
        p=OutdoorPlanner(known_model(),CONFIG,20)
        scan={'angles':np.linspace(-math.pi,math.pi,720,endpoint=False),'ranges':np.full(720,.3),'range_min':.1}
        with self.assertRaises(ValueError):p.plan({'position':[0,0],'yaw':0,'velocity':[0,0,0],'goal':[8,0]},scan,[[100,0,0],[100.2,0,0]],100.4)


if __name__=='__main__':unittest.main()
