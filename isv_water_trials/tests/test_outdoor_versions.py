from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import tempfile
import unittest
import numpy as np
from test_outdoor import CONFIG,known_model,synthetic_rows
from isv_water_trials.outdoor_config import fingerprint,load_config
from isv_water_trials.outdoor_store import OutdoorStore
from isv_water_trials.outdoor_model import fit_model,ResponseModel,score_rollouts,_grid
from isv_water_trials.outdoor_layers import LidarLayers


class VersionsTests(unittest.TestCase):
    def test_navigation_tuning_keeps_identity_but_calibration_changes_it(self):
        for key in ('percent','obstacle_margin_m','recovery_margin_m','scan_range_m','arrival_radius_m','planner_deadline_s','fit_interval_s'):
            changed=dict(CONFIG);changed[key]+=1
            self.assertEqual(fingerprint(CONFIG),fingerprint(changed),key)
        for key in ('esc_neutral_us','heading_offset_deg','mass_kg','prediction_dt_s'):
            changed=dict(CONFIG);changed[key]+=1
            self.assertNotEqual(fingerprint(CONFIG),fingerprint(changed),key)

    def test_legacy_config_records_and_models_survive_migration(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);(root/'configs').mkdir()
            legacy=hashlib.sha256(json.dumps(CONFIG,sort_keys=True).encode()).hexdigest()
            (root/'configs'/f'{legacy}.json').write_text(json.dumps(CONFIG))
            s=OutdoorStore(root,dict(CONFIG,percent=70))
            row=synthetic_rows(1)[0];s.db.execute('INSERT INTO frames(signature,segment,stamp,payload) VALUES (?,?,?,?)',
                (legacy,'original',row['t'],json.dumps(row)))
            model=known_model();model['signature']=legacy;s.save_model(model)
            self.assertEqual(len(s.rows()),1)
            self.assertEqual(s.load_compatible_model(model['id']),model)
            self.assertTrue(s.model_catalog()[0]['compatible']);s.close()
            incompatible=OutdoorStore(root,dict(CONFIG,esc_neutral_us=1490))
            with self.assertRaises(ValueError):incompatible.load_compatible_model(model['id'])
            incompatible.close()

    def test_fork_and_save_child_never_overwrite_parent(self):
        with tempfile.TemporaryDirectory() as tmp:
            s=OutdoorStore(tmp,CONFIG);parent=known_model();parent['training_end_t']=1010
            path=s.save_model(parent);before=path.read_bytes()
            branch=s.fork_learning(parent,1020)
            for stamp,bid in [(1001,None),(1015,'other'),(1021,branch['id']),(1022,'other')]:
                row=synthetic_rows(1)[0];row.update(t=stamp,branch_id=bid);s.add(row)
            self.assertEqual([r['t'] for r in s.training_rows(branch)],[1001,1021])
            child=deepcopy(parent);child.update(id='abcdef0123456789',parent_model_id=parent['id'],lineage=branch)
            s.save_model(child);s.close()
            self.assertEqual(path.read_bytes(),before)
            s=OutdoorStore(tmp,CONFIG);self.assertEqual(len(s.model_catalog()),2)
            self.assertEqual(s.load_compatible_model(parent['id'])['id'],parent['id']);s.close()

    def test_branch_compares_parent_using_only_new_heldout_data(self):
        old=synthetic_rows(400);parent=known_model();parent['training_end_t']=old[-1]['t']
        before=deepcopy(parent);lineage={'id':'branch','started_t':2000.,'data_cutoff_t':parent['training_end_t'],'parent_model_id':parent['id']}
        fresh=synthetic_rows(1100)
        for row in fresh:
            row['t']+=1100;row['commands']=[[t+1100,l,r] for t,l,r in row['commands']]
            row.update(branch_id='branch',segment='new')
        unrelated=deepcopy(fresh[:10])
        for row in unrelated:row.update(branch_id='other',velocity=[100,100,100])
        fitted=fit_model(old+fresh+unrelated,CONFIG,parent,lineage)
        self.assertEqual(parent,before);self.assertEqual(fitted['parent_model_id'],parent['id'])
        self.assertNotEqual(fitted['id'],parent['id']);self.assertEqual(fitted['schema'],'outdoor-response-v1')
        self.assertIn('parent_comparison',fitted);self.assertEqual(fitted['native_samples'],len(fresh))
        self.assertGreater(fitted['parent_comparison']['windows'],0)
        self.assertTrue(any('기준 보존' in r['label'] for r in fitted['ranking']))
        self.assertEqual(fitted['name'],fitted['candidate_name'])
        self.assertNotIn('%',fitted['name'])
        self.assertEqual(len(fitted['candidate_catalog']),fitted['candidate_count'])
        selected=[c for c in fitted['candidate_catalog'] if c['selected']]
        self.assertEqual(len(selected),1);self.assertEqual(selected[0]['name'],fitted['name'])
        self.assertEqual(fitted['recording_segments'],2)
        self.assertEqual(fitted['training_samples'],len(old)+len(fresh))
        runs=_grid(old+fresh,CONFIG['prediction_dt_s'])
        expected=np.mean(np.concatenate([np.linalg.norm(z[g,:2],axis=1) for _,z,_,g in runs]))
        self.assertAlmostEqual(fitted['speed_coverage']['mean_m_s'],expected)
        self.assertGreater(fitted['output_coverage']['total_seconds'],0)
        self.assertEqual(fitted['output_coverage']['definition'],'max(abs(left),abs(right))')
        with tempfile.TemporaryDirectory() as tmp:
            store=OutdoorStore(tmp,CONFIG);store.save_model(fitted)
            self.assertEqual(store.model_catalog()[0]['label'],fitted['name'])
            store.close()

    def test_live_selection_restores_independent_models_and_pause(self):
        with tempfile.TemporaryDirectory() as tmp:
            store=OutdoorStore(tmp,CONFIG);old=known_model();new=deepcopy(old);new['id']='abcdef0123456789'
            store.finalize_model(old);before=store.model_path(old['id']).read_bytes();store.save_model(new)
            store.save_live_selection(new,old,True);store.close()
            store=OutdoorStore(tmp,CONFIG);current,validation,learning=store.live_selection()
            self.assertEqual(current['id'],new['id']);self.assertEqual(validation['id'],old['id']);self.assertFalse(learning)
            self.assertEqual(store.model_path(old['id']).read_bytes(),before);store.close()

    def test_archive_sampler_retains_old_and_recent_without_deleting(self):
        with tempfile.TemporaryDirectory() as tmp:
            store=OutdoorStore(tmp,CONFIG)
            values=[(store.signature,'record',i*.2,json.dumps({'t':i*.2})) for i in range(32000)]
            store.db.executemany('INSERT INTO frames(signature,segment,stamp,payload) VALUES (?,?,?,?)',values)
            rows=store.training_rows();self.assertEqual(rows[0]['t'],0.);self.assertEqual(rows[-1]['t'],31999*.2)
            self.assertLessEqual(len(rows),30000);self.assertEqual(store.db.execute('SELECT COUNT(*) FROM frames').fetchone()[0],32000)
            store.close()

    def test_continuous_comparison_requires_unseen_data(self):
        old=synthetic_rows(500);parent=known_model();parent['training_end_t']=old[-1]['t']
        with self.assertRaises(ValueError):fit_model(old,CONFIG,parent)
        fresh=synthetic_rows(1100)
        for row in fresh:
            row['t']+=1000;row['commands']=[[t+1000,l,r] for t,l,r in row['commands']];row['segment']='fresh'
        fitted=fit_model(old+fresh,CONFIG,parent)
        self.assertEqual(fitted['native_samples'],1100);self.assertEqual(fitted['parent_model_id'],parent['id'])
        self.assertGreater(fitted['parent_comparison']['windows'],0)

    def test_output_bands_require_coverage_and_ignore_training_windows(self):
        rows=synthetic_rows(1000,False);runs=_grid(rows,.2)
        score=score_rollouts(ResponseModel(known_model()),runs,.7,1.)
        self.assertEqual(len(score['output_bands']),10)
        self.assertLessEqual(sum(b['windows'] for b in score['output_bands']),21)
        if score['best_output_band']:
            self.assertGreaterEqual(score['best_output_band']['windows'],5)
        small=score_rollouts(ResponseModel(known_model()),_grid(rows[:50],.2))
        self.assertIsNone(small['best_output_band'])

    def test_speed_bands_use_heldout_nonoverlapping_windows_and_planar_speed(self):
        model=ResponseModel(known_model());n=600;t=np.arange(n)*.2
        z=np.tile([-.3,.4,0.],(n,1));q=np.zeros((n,2));good=np.ones(n,dtype=bool)
        score=score_rollouts(model,[(t,z,q,good)],.7,1.)
        band=score['best_speed_band'];self.assertIsNotNone(band)
        self.assertEqual((band['low_m_s'],band['high_m_s']),(.4,.6))
        self.assertGreaterEqual(band['windows'],5);self.assertLessEqual(band['windows'],12)
        z[:int(n*.7)]=[99,99,99]
        self.assertEqual(score,score_rollouts(model,[(t,z,q,good)],.7,1.))
        small=score_rollouts(model,[(t[:50],z[:50]*0,q[:50],good[:50])])
        self.assertIsNone(small['best_speed_band'])

    def test_record_counts_and_legacy_names_preserve_saved_payloads(self):
        with tempfile.TemporaryDirectory() as tmp:
            store=OutdoorStore(tmp,CONFIG)
            for row in synthetic_rows(3):store.add(row)
            store.new_segment('resume');store.add(synthetic_rows(1)[0])
            self.assertEqual(store.recording_summary(),{'segments':2,'samples':4})
            parent=known_model();parent.update(name='35%',variant='전체·비선형')
            path=store.save_model(parent);before=path.read_bytes()
            child=deepcopy(parent);child.update(id='abcdef0123456789',variant='기준 보존',parent_model_id=parent['id'])
            store.save_model(child)
            self.assertEqual(store.model_display(child)['display_name'],'전체·비선형')
            self.assertEqual(store.model_catalog()[0]['name'],'전체·비선형')
            self.assertNotIn('display_name',child);self.assertEqual(path.read_bytes(),before)
            store.close()

    def test_real_store_and_planner_reject_synthetic_data(self):
        from isv_water_trials.outdoor_planner import OutdoorPlanner
        with tempfile.TemporaryDirectory() as tmp:
            store=OutdoorStore(tmp,CONFIG)
            row=synthetic_rows(1)[0];row['synthetic']=True
            with self.assertRaises(ValueError):store.add(row)
            model=known_model();model['synthetic']=True
            store.save_model(model)
            with self.assertRaises(ValueError):store.load_compatible_model(model['id'])
            with self.assertRaises(ValueError):OutdoorPlanner(model,CONFIG,20)
            store.close()

    def test_lidar_overlay_clears_free_space_expires_and_stays_bounded(self):
        layers=LidarLayers(limit=30)
        a=np.linspace(-math.pi,math.pi,720,endpoint=False)
        pose={'position':[10,20],'yaw':0.}
        scan={'angles':a,'ranges':np.full(720,5.),'range_min':.1,'range_max':40.,'stamp':100.}
        layers.update(scan,pose,CONFIG);first=layers.snapshot(100.)
        self.assertEqual(len(first['hits']),720);self.assertLessEqual(len(first['accumulated']),30)
        scan.update(ranges=np.full(720,np.inf),stamp=100.2);layers.update(scan,pose,CONFIG)
        self.assertEqual(layers.snapshot(100.2)['accumulated'],[])
        scan.update(ranges=np.full(720,5.),stamp=100.4);layers.update(scan,pose,CONFIG)
        self.assertEqual(layers.snapshot(101.)['hits'],[])
        self.assertEqual(layers.snapshot(121.)['accumulated'],[])


if __name__=='__main__':unittest.main()
