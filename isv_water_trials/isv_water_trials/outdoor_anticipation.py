"""Measured-model early avoidance, independent of ROS and asynchronous route jobs.

Evaluate the real surge, sway, yaw momentum, measured input delay and slew.
Soft stand-off encourages an early turn without making a narrow corridor a stop.
This is a sampled local prediction, not a collision-free guarantee.
"""
import math
import numpy as np
from .codex_nav.fast_learned_rollout import command_motion,warmup_command_motion
from .codex_nav.gpu_geometry import trajectory_hulls,warmup_geometry
from .outdoor_follow import forward_approach_metrics
from .outdoor_obstacles import ObstacleTracker
from .outdoor_regions import OperatingRegion


def reaction_profile(model,config):
    rho=float(np.max(abs(np.linalg.eigvals(model.coefficients[:3]))))
    tau=-model.dt/math.log(max(1e-9,min(rho,.999999)))
    delay=model.delay*model.dt
    maximum=float(np.clip(config.get('early_avoidance_horizon_max_s',6.),3.,8.))
    horizon=float(np.clip(delay+2*tau+float(config.get('early_avoidance_lead_s',.6)),3.,maximum))
    steps=max(math.ceil(3./model.dt-1e-9),min(math.ceil(horizon/model.dt-1e-9),math.floor(maximum/model.dt+1e-9)))
    horizon=round(steps*model.dt,9)
    metrics=model.data.get('metrics',{}).get('horizons',{})
    error=float(metrics.get('3',metrics.get('2',{})).get('integrated_position_rmse_m',0.))
    if not math.isfinite(error) or error<0:raise ValueError('Model prediction uncertainty invalid')
    return dict(model_id=model.data.get('id'),model_delay_s=delay,inertia_tau_s=tau,
                prediction_mass_scale=model.inertia_scales[0],prediction_yaw_inertia_scale=model.inertia_scales[2],
                horizon_s=horizon,horizon_capped=delay+2*tau>maximum,uncertainty_m=min(error,.75))


def delayed_history(model,commands,now):
    if not model.delay:return np.empty((0,2))
    rows=np.asarray(commands,dtype=float)
    if rows.ndim!=2 or rows.shape[1]!=3 or not len(rows) or not np.isfinite(rows).all():
        raise ValueError('추진 지연 이력 수신 대기')
    if np.any(np.diff(rows[:,0])<0):raise ValueError('추진 이력 시각 역전')
    times=now+np.arange(-model.delay,0)*model.dt
    ids=np.searchsorted(rows[:,0],times,side='right')-1
    if np.any(ids<0) or np.any(times-rows[np.maximum(ids,0),0]>.35):raise ValueError('추진 지연 이력 수신 대기')
    return np.ascontiguousarray(rows[ids,1:])


class EarlyAvoidance:
    def __init__(self):
        self.key=None;self.profile=None
        self.tracker=ObstacleTracker();self.region_key=None;self.region=None
        warmup_command_motion()
        warmup_geometry()

    def adjust(self,model,state,goal,scan,commands,config,percent,nominal,previous,now,dt=.1):
        if model.data.get('synthetic'):raise ValueError('실측 학습 모델이 필요합니다')
        velocity=np.asarray(state.get('velocity'),dtype=float)
        position=np.asarray(state['position'],dtype=float);yaw=float(state['yaw'])
        nominal=np.asarray(nominal,dtype=float);previous=np.asarray(previous,dtype=float)
        if velocity.shape!=(3,) or position.shape!=(2,) or nominal.shape!=(2,) or previous.shape!=(2,) or not np.isfinite(np.r_[velocity,position,yaw,nominal,previous]).all():
            raise ValueError('실측 속도·위치·명령 수신 대기')
        key=(model.data.get('id'),repr(config))
        if key!=self.key:
            self.prediction_model=model.for_navigation(config)
            self.profile=reaction_profile(self.prediction_model,config);self.key=key
        model=self.prediction_model
        profile=self.profile
        cap=float(percent)/100
        if not 0<cap<=1:raise ValueError('출력 상한 오류')
        lo=np.maximum(-cap,model.data['command_min']);hi=np.minimum(cap,model.data['command_max'])
        if np.any(lo>0) or np.any(hi<0) or np.any(lo>hi) or np.any(abs(previous)>cap+1e-6):
            raise ValueError('학습 모델 출력 범위 확인 필요')
        if np.any(nominal<lo-1e-6) or np.any(nominal>hi+1e-6):raise ValueError('명령이 모델 학습 범위를 벗어났습니다')
        if np.any(previous<lo-1e-6) or np.any(previous>hi+1e-6):raise ValueError('현재 출력이 모델 학습 범위를 벗어났습니다')
        history=delayed_history(model,commands,now)
        angles=np.asarray(scan['angles'],dtype=float);ranges=np.asarray(scan['ranges'],dtype=float)
        if len(angles)<180 or ranges.shape!=angles.shape or not np.isfinite(angles).all():raise ValueError('360도 라이다 수신 대기')
        # The scan contract is a uniformly binned full circle. Do not misindex
        # an arbitrary/nonuniform scan as though it had that angular geometry.
        if not np.allclose(np.diff(angles),2*math.pi/len(angles),atol=1e-5):raise ValueError('라이다 각도 간격 확인 필요')
        minimum=max(0.,float(scan.get('range_min',0.)))
        valid=(ranges>minimum)&(np.isfinite(ranges)|np.isposinf(ranges))
        if valid.mean()<.8:raise ValueError('라이다 유효 관측 부족')
        limit=float(scan.get('range_max',config.get('scan_range_m') or 20.))
        if config.get('scan_range_m',0)>0:limit=min(limit,float(config['scan_range_m']))
        if not math.isfinite(limit) or limit<=0:raise ValueError('라이다 최대 거리 오류')
        pose=scan.get('pose',state);scan_yaw=float(pose['yaw']);scan_position=np.asarray(pose['position'],dtype=float)
        lever=np.asarray(config.get('lidar_lever_arm_m',[0.,0.]));c,s=math.cos(scan_yaw),math.sin(scan_yaw)
        sensor=scan_position+np.array([c*lever[0]-s*lever[1],s*lever[0]+c*lever[1]])
        bearing=angles+scan_yaw+math.radians(config.get('lidar_yaw_deg',0.))
        hit=valid&np.isfinite(ranges)&(ranges<limit-1e-4)
        points=sensor+ranges[hit,None]*np.c_[np.cos(bearing[hit]),np.sin(bearing[hit])]
        # Self returns are classified in the hull pose at scan acquisition.
        delta=points-scan_position
        own=(abs(delta[:,0]*c+delta[:,1]*s)<config['length_m']/2)&(abs(-delta[:,0]*s+delta[:,1]*c)<config['width_m']/2)
        points=points[~own];obstacles=np.c_[points,np.zeros(len(points))]
        tracked_ranges=ranges.copy();tracked_ranges[np.flatnonzero(hit)[own]]=np.inf
        self.tracker.update(angles,tracked_ranges,sensor,scan_yaw+math.radians(config.get('lidar_yaw_deg',0.)),float(scan.get('stamp',now)),limit)
        moving,_=self.tracker.prediction(now,np.zeros(2))
        regions=state.get('test_regions')
        if regions is None and state.get('test_area') is not None:
            regions=[dict(kind='include',points=state['test_area'])]
        region_key=repr(regions)
        if region_key!=self.region_key:
            self.region=OperatingRegion(regions) if regions else None;self.region_key=region_key
        input_age=max(0.,float(state.get('age_s',0.)))
        if not math.isfinite(input_age) or input_age>config.get('gps_max_age_s',1.):raise ValueError('실측 속도 시각 확인 필요')
        # Conservative extra committed motion covers state/scan age and the
        # control/compute pipeline; the model's input delay remains additional.
        scan_age=max(0.,now-float(scan.get('stamp',now)))
        if scan_age>config.get('scan_max_age_s',.5):raise ValueError('라이다 최신값 수신 대기')
        latency=max(input_age,scan_age)+float(config.get('early_avoidance_latency_s',.15))
        committed=max(0,math.ceil(latency/model.dt-1e-9))
        horizon=max(committed+2,math.ceil(profile['horizon_s']/model.dt))
        # No neutral target: blocked-space ranking still considers reverse and
        # both turn directions. Caps and empirical asymmetry apply to each side.
        maneuver_cap=min(cap,float(state.get('early_recovery_percent',percent))/100)
        mc=maneuver_cap
        candidates=np.clip(np.array([nominal,[-mc,-mc],[-mc,mc],[mc,-mc],
            [-mc,.35*mc],[.35*mc,-mc],[0.,mc],[mc,0.],nominal*.45]),lo,hi)
        targets=np.repeat(candidates[:,None,:],horizon,axis=1)
        initial=np.r_[position,yaw,velocity,previous]
        slew=max(.01,float(config.get('command_slew_per_s',.35)))
        result=command_motion(initial,targets,history,model.coefficients,lo,hi,model.dt,slew,min(dt,model.dt),committed)
        before=np.concatenate((np.broadcast_to(initial,(len(candidates),1,8)),result[:,:-1]),axis=1)
        probes=np.concatenate((result,.5*(result+before)),axis=1)
        swept=np.linalg.norm(result[:,:,:2]-before[:,:,:2],axis=2)/2
        swept+=abs(result[:,:,2]-before[:,:,2])*math.hypot(config['length_m'],config['width_m'])/4
        times=np.r_[(np.arange(horizon)+1)*model.dt,(np.arange(horizon)+.5)*model.dt]
        flat=probes.reshape(-1,8)
        distances=trajectory_hulls(flat,np.tile(times,len(candidates)),obstacles,moving,config['length_m'],config['width_m'],model.dt)
        gap=np.minimum(distances[:,0],distances[:,1]).reshape(len(candidates),-1)
        if self.region is not None:
            gap=np.minimum(gap,self.region.hull_clearance(flat,config['length_m'],config['width_m']).reshape(gap.shape))
        gap-=np.tile(swept,(1,2))
        # A future hull corner outside observed free rays is not assumed safe.
        observed=np.where(valid,np.minimum(ranges,limit),0.)
        visibility=np.full(gap.shape,np.inf)
        for sx in (-.5,.5):
            for sy in (-.5,.5):
                a=probes[:,:,2]
                x=probes[:,:,0]+np.cos(a)*sx*config['length_m']-np.sin(a)*sy*config['width_m']-sensor[0]
                y=probes[:,:,1]+np.sin(a)*sx*config['length_m']+np.cos(a)*sy*config['width_m']-sensor[1]
                relative=(np.arctan2(y,x)-scan_yaw-math.radians(config.get('lidar_yaw_deg',0.))-angles[0])%(2*math.pi)
                bins=np.rint(relative/(2*math.pi/len(angles))).astype(int)%len(angles)
                visibility=np.minimum(visibility,observed[bins]-np.hypot(x,y))
        gap=np.minimum(gap,visibility)
        uncertainty=profile['uncertainty_m']
        gap-=uncertainty
        current=float(trajectory_hulls(initial[None],np.zeros(1),obstacles,moving,config['length_m'],config['width_m'],model.dt).min())-uncertainty
        if self.region is not None:current=min(current,float(self.region.hull_clearance(initial[None],config['length_m'],config['width_m'])[0])-uncertainty)
        stand_off=max(0.,float(config.get('obstacle_margin_m',0.)))+float(config.get('early_avoidance_clearance_m',.15))
        stand_off+=np.linalg.norm(velocity[:2])*float(config.get('early_avoidance_lead_s',.6))
        minimum_gap=gap.min(axis=1)
        invalid_arrival,_,_,_=forward_approach_metrics(initial,result,state.get('arrival_goal',goal),config.get('arrival_radius_m',1.))
        approaching=minimum_gap[0]<min(stand_off,current-.02) or minimum_gap[0]<0.
        first_collision=np.flatnonzero(gap[0]<0)
        ttc=float(times[first_collision].min()) if len(first_collision) else None
        selected=0
        if approaching:
            risk=np.mean(np.maximum(stand_off-gap,0.)**2/(1.+times[None,:]),axis=1)
            progress=np.linalg.norm(position-np.asarray(goal))-np.linalg.norm(result[:,-1,:2]-goal,axis=1)
            cost=25*risk-.2*progress+.2*np.linalg.norm((candidates-nominal)/max(cap,.01),axis=1)
            feasible=(minimum_gap>=0.)&~invalid_arrival
            if feasible.any():selected=int(np.argmin(np.where(feasible,cost,np.inf)))
            else:
                permitted=~invalid_arrival if np.any(~invalid_arrival) else np.ones(len(candidates),dtype=bool)
                selected=int(np.lexsort((risk,np.where(permitted,-minimum_gap,np.inf)))[0])
        q=result[selected,committed,6:8]
        info=dict(available=True,**profile,intervened=selected!=0,input_age_s=input_age,
            committed_motion_s=committed*model.dt,stand_off_m=float(stand_off),
            current_clearance_m=current if math.isfinite(current) else None,
            nominal_clearance_m=float(minimum_gap[0]) if math.isfinite(minimum_gap[0]) else None,
            selected_clearance_m=float(minimum_gap[selected]) if math.isfinite(minimum_gap[selected]) else None,
            predicted_collision_s=ttc,candidates=len(candidates),selected=selected,
            all_candidates_blocked=bool(np.all(minimum_gap<0.)),moving_tracks=len(moving),region_checked=self.region is not None)
        return q.tolist(),info
