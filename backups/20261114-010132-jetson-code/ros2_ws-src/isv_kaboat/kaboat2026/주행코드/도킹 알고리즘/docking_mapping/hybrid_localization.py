"""Best valid GICP candidate, stable handoff, and bounded pose prediction."""
from collections import deque
from copy import deepcopy
import math
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from .geometry import apply,rigid,transform,voxel
from .mapper import Mapper
from .registration import align

DEFAULTS=dict(confirm_scans=3,initial_search_xy_m=3.,initial_search_step_m=1.5,
    initial_yaw_step_deg=45.,refine_candidates=3,registration_hz=2.,max_match_age_s=1.,
    max_prediction_age_s=1.,max_prediction_uncertainty_m=.5,max_prediction_distance_m=.75,
    gps_approach_stop_m=3.,confirmation_position_m=.4,confirmation_yaw_deg=10.,
    ambiguous_rmse_m=.02,ambiguous_position_m=.5,ambiguous_yaw_deg=12.)

def settings(config):
    result=dict(DEFAULTS,**config.get('hybrid_navigation',{}))
    for name,value in result.items():
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
            raise ValueError('Invalid hybrid_navigation.'+name)
    for key in ('confirm_scans','refine_candidates'):
        if type(result[key]) is not int:raise ValueError(key+' must be integer')
    if result['max_prediction_age_s']>config['autonomy']['tracking']['reacquire_gap_s']:
        raise ValueError('Prediction must fit existing reacquisition gap')
    return result

def angle_between(a,b):
    return math.degrees(Rotation.from_matrix(a[:3,:3]@b[:3,:3].T).magnitude())

class BestMapMatcher:
    def __init__(self,points_map,extrinsic,config):
        self.config=deepcopy(config);self.options=settings(config);self.extrinsic=rigid(extrinsic).copy()
        self.points_map=voxel(points_map,config['autonomy']['tracking']['voxel_m'],config['mapping']['max_voxels'])
        if len(self.points_map)<config['registration']['min_correspondences']:raise ValueError('INSUFFICIENT_REFERENCE_POINTS')
        self.points_map.flags.writeable=False;self.tree=cKDTree(self.points_map)
        tracking_config=deepcopy(config)
        tracking_config['preprocess'].update(min_range_m=config['autonomy']['tracking']['min_range_m'],
            max_range_m=config['autonomy']['tracking']['max_range_m'])
        self.preprocessor=Mapper(tracking_config)

    def source(self,cloud,rotation):
        # Survey front/height/cluster convention, existing localization range.
        # The survey's 2m radius remains unchanged: structures can be recognized
        # from farther away, using only the saved-map ROI for each candidate.
        points=self.preprocessor.preprocess(cloud,transform(rotation))
        return voxel(points,self.config['autonomy']['tracking']['voxel_m'],2500)

    def candidates(self,seed,tracking):
        o=self.options
        if tracking:
            offsets=[(0.,0.),(.2,0.),(-.2,0.),(0.,.2),(0.,-.2)]
            turns=[0.,-5.,5.]
        else:
            offsets=[(x,y) for x in np.arange(-o['initial_search_xy_m'],o['initial_search_xy_m']+.01,o['initial_search_step_m'])
                     for y in np.arange(-o['initial_search_xy_m'],o['initial_search_xy_m']+.01,o['initial_search_step_m'])]
            turns=list(np.arange(-180.,180.,o['initial_yaw_step_deg']))
        poses=[rigid(seed).copy()]
        for dx,dy in offsets:
            for yaw in turns:
                pose=seed.copy();pose[:3,3]+=[dx,dy,0.]
                pose[:3,:3]=Rotation.from_euler('z',yaw,degrees=True).as_matrix()@seed[:3,:3]
                poses.append(pose)
        return poses

    def match(self,cloud,rotation,seed,stamp,tracking=False,cancelled=lambda:False):
        source=self.source(cloud,rotation);minimum=self.config['registration']['min_correspondences']
        if len(source)<minimum:
            return dict(accepted=False,reason='INSUFFICIENT_SCAN',scan_points=len(source),stamp=stamp)
        config=deepcopy(self.config['autonomy']['tracking']['registration'] if tracking else self.config['registration'])
        config['method']='gicp';ranked=[];margin=self.config['autonomy']['tracking']['roi_margin_m']
        low=self.points_map.min(0)-margin;high=self.points_map.max(0)+margin
        for pose in self.candidates(seed,tracking):
            if cancelled():return dict(accepted=False,reason='CANCELLED',stamp=stamp)
            world=apply(source,pose@self.extrinsic);mask=np.all((world>=low)&(world<=high),axis=1)
            if np.count_nonzero(mask)<minimum:continue
            crop=world[mask];sample=crop[::max(1,len(crop)//250)]
            distances=self.tree.query(sample)[0]
            score=float(np.mean(np.minimum(distances,config['max_correspondence_m'])**2))
            ranked.append((score,pose))
        if not ranked:return dict(accepted=False,reason='NO_OVERLAP',stamp=stamp,scan_points=len(source),tested_candidates=0)
        ranked.sort(key=lambda row:row[0]);selected=[]
        for score,pose in ranked:
            if any(np.linalg.norm(pose[:3,3]-old[:3,3])<.3 and angle_between(pose,old)<8 for old in selected):continue
            selected.append(pose)
            if len(selected)>=self.options['refine_candidates']:break
        results=[]
        for pose in selected:
            if cancelled():return dict(accepted=False,reason='CANCELLED',stamp=stamp)
            # Bound target cost, retaining all geometry in the observed region.
            expected=apply(source,pose@self.extrinsic)
            selected_source=voxel(source[np.all((expected>=low)&(expected<=high),axis=1)],self.config['autonomy']['tracking']['voxel_m'],600)
            expected=apply(selected_source,pose@self.extrinsic);margin=config['max_correspondence_m']
            mask=np.all((self.points_map>=expected.min(0)-margin)&(self.points_map<=expected.max(0)+margin),axis=1)
            target=voxel(self.points_map[mask],self.config['autonomy']['tracking']['voxel_m'],1500)
            result=align(selected_source,target,pose@self.extrinsic,config)
            results.append(dict(result.metrics(),pose=result.matrix@np.linalg.inv(self.extrinsic),scan_points=len(selected_source)))
        valid=sorted((r for r in results if r['accepted']),key=lambda r:(r['rmse'],-r['inlier_ratio'],-r['information_ratio']))
        winner=valid[0] if valid else min(results,key=lambda r:(not math.isfinite(r['rmse']),r['rmse']))
        if not tracking and len(valid)>1:
            for other in valid[1:]:
                if (abs(other['rmse']-winner['rmse'])<=self.options['ambiguous_rmse_m']
                    and (np.linalg.norm(other['pose'][:3,3]-winner['pose'][:3,3])>self.options['ambiguous_position_m']
                         or angle_between(other['pose'],winner['pose'])>self.options['ambiguous_yaw_deg'])):
                    winner=dict(winner,accepted=False,reason='AMBIGUOUS_INITIAL_ALIGNMENT');break
        return dict(winner,stamp=stamp,rotation=rotation.copy(),input_points=len(source),reference_points=len(self.points_map),
                    tested_candidates=len(ranked),refined_candidates=len(results),valid_candidates=len(valid),method='gicp')

class HybridLocalization:
    def __init__(self,config):
        self.config=config;self.options=settings(config);self.poses=deque(maxlen=2)
        self.rotation=None;self.locked=False;self.confirmed=0;self.metrics={};self.last_result_stamp=-math.inf
        self.velocity=np.zeros(3);self.has_velocity=False
        self.last_rmse=.3

    def accept(self,result,now):
        stamp=result['stamp']
        if stamp<=self.last_result_stamp:return False
        self.last_result_stamp=stamp
        self.metrics={k:v for k,v in result.items() if k not in ('pose','rotation')}
        if not result.get('accepted') or not -.1<=now-stamp<=self.options['max_match_age_s']:
            if result.get('accepted'):self.metrics.update(accepted=False,reason='STALE_REGISTRATION')
            if not self.locked:
                self.confirmed=0;self.poses.clear();self.velocity=np.zeros(3);self.has_velocity=False
            return False
        pose=rigid(result['pose']).copy();rotation=result['rotation']
        if self.poses:
            old_stamp,old=self.poses[-1];dt=stamp-old_stamp
            predicted=old.copy();predicted[:3,3]+=self.velocity*dt
            predicted[:3,:3]=old[:3,:3]@self.rotation.T@rotation
            margin=self.options['confirmation_position_m']
            if (dt<=0 or dt>self.config['autonomy']['tracking']['reacquire_gap_s']
                or np.linalg.norm(pose[:3,3]-old[:3,3])>margin+self.config['mapping']['max_boat_speed_mps']*dt
                or angle_between(pose,predicted)>self.options['confirmation_yaw_deg']):
                self.metrics.update(accepted=False,reason='INCONSISTENT_POSE_SEQUENCE')
                if not self.locked:
                    self.confirmed=0;self.poses.clear();self.velocity=np.zeros(3);self.has_velocity=False
                return False
            self.velocity=(pose[:3,3]-old[:3,3])/dt
            self.has_velocity=True
        self.poses.append((stamp,pose));self.rotation=rotation.copy();self.confirmed+=1
        self.last_rmse=float(result['rmse'])
        if self.confirmed>=self.options['confirm_scans']:self.locked=True
        return True

    def predict(self,stamp,rotation):
        if not self.poses:return None,'WAIT_INITIAL_ALIGNMENT',None
        last,pose=self.poses[-1];dt=stamp-last;o=self.options
        if not -.1<=dt<=o['max_prediction_age_s']:return None,'PREDICTION_TIME_LIMIT',None
        dt=max(0.,dt)
        speed=float(np.linalg.norm(self.velocity))
        if speed>self.config['mapping']['max_boat_speed_mps']:return None,'PREDICTION_SPEED_LIMIT',None
        displacement=self.velocity*dt
        # A bound, not a measured covariance. It grows from the last accepted
        # match; predictions never refresh its timestamp or reset uncertainty.
        last_rmse=self.last_rmse
        if not math.isfinite(last_rmse):last_rmse=.3
        uncertainty=last_rmse+(.2 if self.has_velocity else self.config['mapping']['max_boat_speed_mps'])*dt+.5*dt*dt
        if np.linalg.norm(displacement)>o['max_prediction_distance_m'] or uncertainty>o['max_prediction_uncertainty_m']:
            return None,'PREDICTION_UNCERTAINTY_LIMIT',uncertainty
        result=pose.copy();result[:3,3]+=displacement;result[:3,:3]=pose[:3,:3]@self.rotation.T@rotation
        return result,'LIDAR_TRACKING' if dt<.15 else 'LIDAR_PREDICTING',uncertainty
