"""Offline sampled MPC candidate. No ROS clients, publishers, or actuation."""
import argparse
import json
import math
from pathlib import Path
import numpy as np
from .common import dump_json
from .identification import analyze


def coefficients(model):
    m=model['identified_models']
    surge=m['surge_command_response']
    yaw=m['empirical_yaw_response']
    if surge['status']!='estimated' or yaw['status']!='estimated':
        raise ValueError('Both identified command-response models must be accepted')
    a=surge['coefficients'];b=yaw['coefficients']
    return a,b


def rollout(model,state,left,right,horizon_s=3.,dt=.1):
    a,b=coefficients(model)
    neutral=model['config']['neutral_pwm_us']
    x,y,psi,u,r=map(float,state)
    trajectory=[]
    for _ in range(round(horizon_s/dt)):
        du=a['u_decay_per_s']*u+a['quadratic_decay_per_m']*abs(u)*u+a['pwm_sum_gain_m_s2_per_us']*(left+right-2*neutral)+a['bias_m_s2']
        dr=b['r_decay_per_s']*r+b['pwm_difference_gain_rad_s2_per_us']*(left-right)+b['bias_rad_s2']
        u+=dt*du;r+=dt*dr;psi+=dt*r
        x+=dt*u*math.cos(psi);y+=dt*u*math.sin(psi)
        if not np.isfinite([x,y,psi,u,r]).all() or abs(u)>8 or abs(r)>2:
            return None
        trajectory.append([x,y,psi,u,r])
    return np.array(trajectory)


def choose(model,state,goal,obstacles,previous=(1500,1500),margin=.3):
    neutral=model['config']['neutral_pwm_us']
    candidates=[neutral,neutral+20,neutral+40,neutral+60]
    radius=math.hypot(model['boat']['length_m']/2,model['boat']['beam_m']/2)+margin
    obstacles=np.asarray(obstacles,float).reshape((-1,2))
    best=None
    for left in candidates:
        for right in candidates:
            tr=rollout(model,state,left,right)
            if tr is None: continue
            # Include initial hull footprint. Small integration steps approximate swept clearance.
            xy=np.vstack([np.asarray(state[:2]),tr[:,:2]])
            if len(obstacles) and np.min(np.linalg.norm(xy[:,None,:]-obstacles[None,:,:],axis=2))<=radius:
                continue
            cost=float(np.linalg.norm(tr[-1,:2]-goal)**2+.002*((left-previous[0])**2+(right-previous[1])**2)
                       +.05*np.mean(tr[:,4]**2))
            if best is None or cost<best['cost']:
                best={'left_pwm_us':left,'right_pwm_us':right,'cost':cost,'trajectory':tr.tolist(),
                      'mode':'OFFLINE_ADVICE_ONLY','deployment_ready':False}
    return best


def main(argv=None):
    parser=argparse.ArgumentParser(description='Hold-out model validation and offline LiDAR MPC suggestions; NO motor output')
    parser.add_argument('--model',required=True)
    parser.add_argument('--session',required=True,help='An independent session, NOT the training session')
    parser.add_argument('--goal-east',type=float,required=True)
    parser.add_argument('--goal-north',type=float,required=True)
    parser.add_argument('--config',help='Validated frame/extrinsic configuration for this held-out run')
    args=parser.parse_args(argv)
    model=json.loads(Path(args.model).read_text())
    session=Path(args.session).resolve()
    if session==Path(model['source_session']).resolve():
        parser.error('Training session cannot be used as held-out validation')
    meta=json.loads((session/'metadata.json').read_text())
    cfg=meta['config']
    if args.config:
        from .common import load_config
        cfg=load_config(args.config)
    if not cfg['frame_confirmed'] or not cfg.get('lidar_frame_confirmed'):
        parser.error('Frame and LiDAR extrinsic must be confirmed; defaults are intentionally unconfirmed')
    coefficients(model)
    rows=[json.loads(line) for line in (session/'raw.jsonl').read_text().splitlines() if line.strip()]
    _,samples=analyze(rows,cfg)
    scans=[row for row in rows if row['kind']=='scan']
    results=[]
    for s in samples[::10]:
        if not np.isfinite([s[k] for k in ['east_m','north_m','yaw_enu_rad','u_m_s','r_rad_s']]).all(): continue
        recent=[row for row in scans if 0<=s['t']-row['t']<.3]
        if not recent: continue
        scan=recent[-1]['msg']
        values=np.array([float(v) if v is not None else np.nan for v in scan['ranges']])
        # Unknown/no-return values are NOT silently treated as free space.
        if not len(values) or np.isfinite(values).mean()<.95:
            continue
        if abs(scan['angle_increment'])*len(values)<math.radians(350): continue
        angles=scan['angle_min']+np.arange(len(values))*scan['angle_increment']+math.radians(cfg['lidar_yaw_deg'])
        valid=(values>=scan['range_min'])&(values<=scan['range_max'])
        body=np.c_[values[valid]*np.cos(angles[valid])+cfg['lidar_x_m'],values[valid]*np.sin(angles[valid])+cfg['lidar_y_m']]
        c,sn=math.cos(s['yaw_enu_rad']),math.sin(s['yaw_enu_rad'])
        rotation=np.array([[c,-sn],[sn,c]])
        obstacles=body@rotation.T+np.array([s['east_m'],s['north_m']])
        advice=choose(model,[s['east_m'],s['north_m'],s['yaw_enu_rad'],s['u_m_s'],s['r_rad_s']],
                      np.array([args.goal_east,args.goal_north]),obstacles)
        results.append({'t':s['t'],'advice':advice})
    a,b=coefficients(model)
    residuals=[]
    for s in samples:
        vals=[s[k] for k in ('u_m_s','udot_m_s2','r_rad_s','rdot_rad_s2','left_pwm_us','right_pwm_us')]
        if not np.isfinite(vals).all() or not s['interior_segment']: continue
        u,du,r,dr,l,rp=vals
        pu=a['u_decay_per_s']*u+a['quadratic_decay_per_m']*abs(u)*u+a['pwm_sum_gain_m_s2_per_us']*(l+rp-2*model['config']['neutral_pwm_us'])+a['bias_m_s2']
        pr=b['r_decay_per_s']*r+b['pwm_difference_gain_rad_s2_per_us']*(l-rp)+b['bias_rad_s2']
        residuals.append([du-pu,dr-pr])
    report={'deployment_ready':False,'mode':'offline independent-session replay',
            'derivative_validation_samples':len(residuals),
            'derivative_rmse':[float(v) for v in np.sqrt(np.mean(np.square(residuals),axis=0))] if residuals else None,
            'derivative_rmse_units':['m/s^2','rad/s^2'], 'suggestions':results,
            'limitations':['Reduced model neglects sway/coupling, moving obstacles, roll/pitch, actuator delay and current changes.',
                           'No suggestion is emitted for incomplete/stale scans; no deployment approval is inferred.']}
    dump_json(session/'mpc_replay.json',report)
    print('REPLAY='+str(session/'mpc_replay.json'), 'SUGGESTIONS='+str(len(results)))
