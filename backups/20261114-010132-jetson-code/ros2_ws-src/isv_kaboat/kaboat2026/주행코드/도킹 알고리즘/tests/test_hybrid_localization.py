from copy import deepcopy
from unittest.mock import patch
from types import SimpleNamespace as NS
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from docking_mapping.geometry import apply,transform
from docking_mapping.hybrid_localization import BestMapMatcher,HybridLocalization
from test_mapping import config,structure

def reading(stamp,x=0.,yaw=0.,accepted=True,rmse=.05):
    rotation=Rotation.from_euler('z',yaw,degrees=True).as_matrix()
    return dict(stamp=stamp,pose=transform(rotation,[x,0.,0.]),rotation=rotation,accepted=accepted,
                reason='CONVERGED' if accepted else 'HIGH_RMSE',rmse=rmse,inlier_ratio=1.,information_ratio=.1)

class LocalizationTests(unittest.TestCase):
    def test_three_consistent_matches_lock_and_one_failure_does_not_erase_last_valid_pose(self):
        loc=HybridLocalization(config())
        for i in range(3):
            self.assertTrue(loc.accept(reading(1+i*.1,i*.1),1+i*.1))
            self.assertEqual(loc.locked,i==2)
        last=loc.poses[-1][0]
        self.assertFalse(loc.accept(reading(1.3,accepted=False,rmse=9.),1.3))
        pose,source,uncertainty=loc.predict(1.5,np.eye(3))
        self.assertIsNotNone(pose);self.assertEqual(source,'LIDAR_PREDICTING')
        self.assertAlmostEqual(pose[0,3],.5);self.assertEqual(loc.poses[-1][0],last)
        self.assertLess(uncertainty,.5)
    def test_bad_or_inconsistent_initial_matches_do_not_lock(self):
        loc=HybridLocalization(config());loc.accept(reading(1),1)
        self.assertFalse(loc.accept(reading(1.1,x=20),1.1));self.assertFalse(loc.locked)
        self.assertEqual(loc.confirmed,0)
        self.assertFalse(loc.accept(reading(2,accepted=False),2));self.assertFalse(loc.locked)
    def test_stale_result_cannot_lock_or_refresh_prediction(self):
        loc=HybridLocalization(config())
        self.assertFalse(loc.accept(reading(1),3));self.assertEqual(loc.metrics['reason'],'STALE_REGISTRATION')
        for i in range(3):loc.accept(reading(4+i*.1),4+i*.1)
        self.assertIsNone(loc.predict(6,np.eye(3))[0])
    def test_prediction_has_absolute_time_distance_and_uncertainty_limits(self):
        loc=HybridLocalization(config())
        for i in range(3):loc.accept(reading(1+i*.1,i*.1),1+i*.1)
        self.assertIsNone(loc.predict(2.1,np.eye(3))[0])
        self.assertIsNone(loc.predict(2.3,np.eye(3))[0])
        self.assertEqual(loc.poses[-1][0],1.2)
    def test_imu_turn_updates_prediction_in_map_frame(self):
        loc=HybridLocalization(config())
        for i in range(3):loc.accept(reading(1+i*.1),1+i*.1)
        pose,_,_=loc.predict(1.3,Rotation.from_euler('z',10,degrees=True).as_matrix())
        self.assertAlmostEqual(np.degrees(np.arctan2(pose[1,0],pose[0,0])),10.)
    def test_out_of_order_result_is_ignored(self):
        loc=HybridLocalization(config());loc.accept(reading(1),1)
        self.assertFalse(loc.accept(reading(.9,x=20),1));self.assertEqual(loc.confirmed,1)

class CandidateTests(unittest.TestCase):
    def test_real_gicp_search_corrects_offset_and_large_initial_yaw_without_changing_map(self):
        c=config();points=structure();matcher=BestMapMatcher(points,np.eye(4),c)
        old=matcher.points_map.copy();seed=transform(Rotation.from_euler('z',45,degrees=True).as_matrix(),[1.5,0,0])
        result=matcher.match(points,np.eye(3),seed,1.)
        self.assertTrue(result['accepted'],{k:v for k,v in result.items() if k not in ('pose','rotation')})
        np.testing.assert_allclose(result['pose'],np.eye(4),atol=.04)
        self.assertGreater(result['tested_candidates'],10);np.testing.assert_array_equal(matcher.points_map,old)
    def test_no_overlap_and_cancellation_do_not_choose_least_bad_result(self):
        matcher=BestMapMatcher(structure(),np.eye(4),config())
        bad=matcher.match(structure(),np.eye(3),transform(translation=[100,100,100]),1.)
        self.assertFalse(bad['accepted'])
        cancelled=matcher.match(structure(),np.eye(3),np.eye(4),1.,cancelled=lambda:True)
        self.assertEqual(cancelled['reason'],'CANCELLED')
    def test_only_valid_candidates_are_ranked_and_ambiguous_valid_poses_are_rejected(self):
        matcher=BestMapMatcher(structure(),np.eye(4),config());count=[0]
        def result(source,target,initial,cfg):
            count[0]+=1;pose=np.eye(4);pose[0,3]=float(count[0])
            return NS(matrix=pose,metrics=lambda:dict(accepted=True,reason='CONVERGED',rmse=.05,
                         inlier_ratio=.9,information_ratio=.1))
        with patch('docking_mapping.hybrid_localization.align',side_effect=result):
            r=matcher.match(structure(),np.eye(3),np.eye(4),1.)
        self.assertFalse(r['accepted']);self.assertEqual(r['reason'],'AMBIGUOUS_INITIAL_ALIGNMENT')
    def test_tracking_uses_existing_longer_range_with_survey_front_and_height_convention(self):
        c=config();c['preprocess'].update(min_range_m=.1,max_range_m=2.,range_mode='horizontal',front_only=True,
            height_filter_enabled=False,cluster_enabled=False)
        matcher=BestMapMatcher(structure(),np.eye(4),c)
        source=matcher.source([[1,0,10],[-1,0,0],[3,0,0],[1,0,-10]],np.eye(3))
        np.testing.assert_array_equal(source,[[1,0,-10],[1,0,10],[3,0,0]])
        self.assertEqual(c['preprocess']['max_range_m'],2.)

if __name__=='__main__':unittest.main()
