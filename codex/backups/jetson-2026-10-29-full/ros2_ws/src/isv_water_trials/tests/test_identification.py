import contextlib
import io
import json
import math
from pathlib import Path
import tempfile
import unittest
import numpy as np

from isv_water_trials.common import load_config, clean_json
from isv_water_trials.identification import (prepare, analyze, linear_fit, local_derivatives,
                                            ThrustCalibration, interpolate, EARTH_RADIUS_M)
from isv_water_trials.trial import plan, main as trial_main
from isv_water_trials.mpc import choose, rollout


def synthetic(cfg, turn=False, duration=30):
    rows=[]
    for i,t in enumerate(np.arange(0,duration,.1)):
        angle=.2*t if turn else 0
        east=5*math.sin(angle) if turn else t
        north=5*(1-math.cos(angle)) if turn else 0
        lat=35+math.degrees(north/EARTH_RADIUS_M)
        lon=129+math.degrees(east/(EARTH_RADIUS_M*math.cos(math.radians(35))))
        stamp={'sec':int(t)+1,'nanosec':round((t-int(t))*1e9)}
        rows.append({'t':float(t),'kind':'gps','msg':{'header':{'stamp':stamp},'latitude':lat,'longitude':lon,
                     'position_covariance_type':2,'position_covariance':[.01,0,0,0,.01,0,0,0,.1],'status':{'status':0}}})
        rows.append({'t':float(t),'kind':'vn','msg':{'header':{'stamp':stamp},'group_fields':0x28,
                     'yawpitchroll':{'x':(90-math.degrees(angle)+180)%360-180,'y':0,'z':0},
                     'angularrate':{'z':-.2 if turn else 0}}})
    return rows


def model_fixture():
    cfg=load_config();cfg['frame_confirmed']=True
    return {'config':cfg,'boat':{k:cfg[k] for k in ('mass_kg','length_m','beam_m','thruster_spacing_m')},
            'identified_models':{
                'surge_command_response':{'status':'estimated','coefficients':{
                    'u_decay_per_s':-.3,'quadratic_decay_per_m':-.1,'pwm_sum_gain_m_s2_per_us':.003,'bias_m_s2':0}},
                'empirical_yaw_response':{'status':'estimated','coefficients':{
                    'r_decay_per_s':-.5,'pwm_difference_gain_rad_s2_per_us':-.004,'bias_rad_s2':0}}}}


class IdentificationTests(unittest.TestCase):
    def setUp(self):
        self.cfg=load_config();self.cfg['frame_confirmed']=True

    def test_saved_dimensions(self):
        self.assertEqual([self.cfg[k] for k in ['mass_kg','length_m','beam_m','thruster_spacing_m']],[25.,1.1,.5,.32])

    def test_straight_ground_body_speed(self):
        samples,_,_=prepare(synthetic(self.cfg),self.cfg)
        speed=[s['u_m_s'] for s in samples if np.isfinite(s['u_m_s'])]
        self.assertGreater(len(speed),100)
        np.testing.assert_allclose(speed,1,atol=1e-6)

    def test_ned_heading_to_enu_and_wrap(self):
        samples,_,_=prepare(synthetic(self.cfg,turn=True,duration=40),self.cfg)
        selected=[s for s in samples if np.isfinite(s['r_rad_s']) and np.isfinite(s['u_m_s'])]
        np.testing.assert_allclose([s['r_rad_s'] for s in selected],.2,atol=1e-9)
        np.testing.assert_allclose([s['gyro_r_rad_s'] for s in selected],.2,atol=1e-9)
        np.testing.assert_allclose([s['u_m_s'] for s in selected],1,atol=.01)
        # Finite local-polynomial differentiation has a small truncation error.
        np.testing.assert_allclose([s['v_m_s'] for s in selected],0,atol=5e-5)

    def test_unconfirmed_frames_disable_body_and_models(self):
        self.cfg['frame_confirmed']=False
        report,samples=analyze(synthetic(self.cfg),self.cfg)
        self.assertTrue(all(math.isnan(s['u_m_s']) for s in samples))
        self.assertFalse(report['models'])

    def test_unknown_covariance_not_fake_valid_fix(self):
        rows=synthetic(self.cfg)
        for row in rows:
            if row['kind']=='gps':row['msg']['position_covariance_type']=0
        samples,rejected,_=prepare(rows,self.cfg)
        self.assertEqual(samples,[])
        self.assertGreater(rejected['gps'],0)

    def test_nonfinite_fix_rejected(self):
        rows=synthetic(self.cfg)
        rows[20]['msg']['latitude']=None
        _,rejected,_=prepare(rows,self.cfg)
        self.assertGreater(rejected['gps'],0)

    def test_missing_vectornav_fields_not_zero_heading(self):
        rows=synthetic(self.cfg)
        for row in rows:
            if row['kind']=='vn':row['msg']['group_fields']=0
        samples,rejected,_=prepare(rows,self.cfg)
        self.assertTrue(all(math.isnan(s['yaw_enu_rad']) for s in samples))
        self.assertGreater(rejected['vn'],0)

    def test_source_stamp_duplicate_rejected(self):
        rows=synthetic(self.cfg)
        rows[4]['msg']['header']=rows[2]['msg']['header']
        _,rejected,_=prepare(rows,self.cfg)
        self.assertEqual(rejected['source_stamp_nonincreasing'],1)

    def test_gap_never_interpolated(self):
        values=interpolate([0,1,4,5],[0,1,4,5],[.5,2,3,4.5],1.1)
        self.assertTrue(np.isnan(values[1:3]).all())
        out=local_derivatives(np.r_[np.arange(0,4,.1),np.arange(8,12,.1)],np.r_[np.arange(0,4,.1),np.arange(8,12,.1)],2,1)
        self.assertTrue(np.isnan(out[39]).all())
        self.assertTrue(np.isnan(out[40]).all())

    def test_stale_command_not_treated_as_neutral(self):
        rows=synthetic(self.cfg)
        rows.append({'t':0,'kind':'command','msg':{'data':json.dumps({'left_pwm_us':1500,'right_pwm_us':1500,'status':'acknowledged'})}})
        samples,_,_=prepare(rows,self.cfg)
        self.assertTrue(all(math.isnan(s['left_pwm_us']) for s in samples if s['t']>1))

    def test_singular_experiment_rejected(self):
        fit=linear_fit(np.ones((100,3)),np.ones(100),['a','b','c'])
        self.assertEqual(fit['status'],'insufficient_excitation')

    def test_repeated_inverse_parameter_recovery_30_seeds(self):
        # Synthetic 30-run numerical verification, NOT water-test evidence.
        for seed in range(30):
            rng=np.random.default_rng(seed)
            x=rng.normal(size=(500,4));x[:,-1]=1
            truth=np.array([30,8,2,.3])
            fit=linear_fit(x,x@truth+rng.normal(0,.02,500),['M','d1','d2','bias'])
            self.assertEqual(fit['status'],'estimated')
            np.testing.assert_allclose(list(fit['coefficients'].values()),truth,atol=.005)

    def test_calibration_never_extrapolates(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'cal.csv'
            p.write_text('side,pwm_us,voltage_V,thrust_N\nleft,1500,12,0\nleft,1600,12,10\n')
            c=ThrustCalibration(p)
            self.assertEqual(c.force('left',1550,12),5)
            self.assertTrue(math.isnan(c.force('left',1700,12)))
            self.assertTrue(math.isnan(c.force('left',1550,11.9)))
            self.assertTrue(math.isnan(c.force('right',1550,12)))

    def test_no_absolute_force_claim_without_table(self):
        report,_=analyze(synthetic(self.cfg),self.cfg)
        self.assertNotIn('surge_force_balance',report['models'])
        self.assertIn('Absolute thrust from PWM alone',report['not_identifiable'])

    def test_trial_default_only_prints(self):
        text=io.StringIO()
        with contextlib.redirect_stdout(text):trial_main(['surge','--repeats','3'])
        self.assertIn('DRY_RUN',text.getvalue())
        self.assertEqual(sum(s.label=='straight' for s in plan('surge',3)),6)

    def test_trial_pwm_bounds_and_repeats(self):
        with self.assertRaises(ValueError):plan('yaw',11)
        with self.assertRaises(ValueError):plan('surge',3,1500,101)
        self.assertTrue(all(1400<=s.left_pwm_us<=1600 for s in plan('zigzag',3)))

    def test_mpc_repeated_rollouts_50_scenarios(self):
        model=model_fixture()
        for i in range(50):
            advice=choose(model,[0,0,0,.1,0],np.array([3,i*.01]),[[5,2]])
            self.assertIsNotNone(advice)
            self.assertFalse(advice['deployment_ready'])
            self.assertLessEqual(advice['left_pwm_us'],1560)

    def test_mpc_blocks_hull_collision(self):
        self.assertIsNone(choose(model_fixture(),[0,0,0,0,0],np.array([3,0]),[[.1,0]]))

    def test_nan_json_is_valid_json(self):
        self.assertEqual(json.loads(json.dumps(clean_json({'a':math.nan}))),{'a':None})


if __name__=='__main__':unittest.main()
