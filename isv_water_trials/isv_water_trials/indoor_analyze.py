"""Fit PWM-to-motion response from indoor trials; retain a chronological holdout."""
import argparse
import csv
import html
import json
import math
from pathlib import Path
import numpy as np
from scipy.signal import savgol_filter


def fit_channel(t, velocity, command):
    """dv/dt = -decay*v + PWM_gain*q + bias; units retained in output."""
    dt = np.diff(t)
    good = (dt>.025) & (dt<.3)
    if len(t)<80 or np.count_nonzero(good)<60:
        return {'qualified':False,'reason':'유효한 AUTO 구간을 8초 이상 기록해 주세요.'}, []
    # Smooth each uninterrupted section independently; never filter across gaps.
    v = velocity.copy()
    split = np.flatnonzero(good)[int(np.count_nonzero(good)*.7)]
    splits = np.unique(np.r_[0,np.where(~good)[0]+1,split,len(t)])
    for start,end in zip(splits[:-1],splits[1:]):
        n = end-start; window = min(11,n if n%2 else n-1)
        if window>=5: v[start:end] = savgol_filter(v[start:end],window,2)
    x = np.column_stack((-v[:-1],command[:-1],np.ones(len(t)-1)))
    y = np.diff(v)/dt
    indices = np.flatnonzero(good & np.isfinite(y) & np.all(np.isfinite(x),axis=1))
    cut = max(1,int(len(indices)*.7)); train,test = indices[:cut],indices[cut:]
    if len(test)<15 or np.ptp(command[train])<.08 or np.std(v[train])<.01:
        return {'qualified':False,'reason':'정지·가속·감속을 포함해 서로 다른 출력 구간이 필요합니다.'}, []
    scale = np.std(x[train,:2],axis=0)
    design = np.column_stack((x[train,:2]/scale,np.ones(len(train))))
    condition = float(np.linalg.cond(design))
    if not math.isfinite(condition) or condition>300:
        return {'qualified':False,'reason':'속도와 명령 변화가 부족해 계수를 구분할 수 없습니다.',
                'condition':condition if math.isfinite(condition) else None}, []
    beta,*_ = np.linalg.lstsq(x[train],y[train],rcond=None)
    predicted = v[test]+dt[test]*(x[test]@beta)
    persistence_rmse = float(np.sqrt(np.mean((v[test+1]-v[test])**2)))
    rmse = float(np.sqrt(np.mean((v[test+1]-predicted)**2)))
    # A free rollout on every uninterrupted validation section exposes drift.
    rollout = []; estimate = None; previous = None
    for i in test:
        if previous is None or i != previous+1: estimate = v[i]
        estimate += dt[i]*(-beta[0]*estimate+beta[1]*command[i]+beta[2])
        if not math.isfinite(estimate) or abs(estimate)>max(10.,10*float(np.max(np.abs(v)))):
            return {'qualified':False,'reason':'검증 구간에서 예측이 발산했습니다. 출력·감속 구간을 더 기록해 주세요.'}, []
        rollout.append([float(t[i+1]),float(v[i+1]),float(estimate)])
        previous = i
    rollout_rmse = float(np.sqrt(np.mean([(a-b)**2 for _,a,b in rollout])))
    qualified = bool(beta[0]>0 and rmse < persistence_rmse and rollout_rmse < max(.05,np.std(v[test])*1.5))
    result = {'qualified':qualified,'model':'dv/dt = -decay*v + pwm_gain*q + bias',
              'decay_per_s':float(beta[0]),'pwm_gain':float(beta[1]),'bias':float(beta[2]),
              'time_constant_s':float(1/beta[0]) if beta[0]>0 else None,
              'steady_velocity_per_unit_pwm':float(beta[1]/beta[0]) if beta[0]>0 else None,
              'holdout_one_step_rmse':rmse,'holdout_persistence_rmse':persistence_rmse,
              'holdout_rollout_rmse':rollout_rmse,'train_samples':len(train),'test_samples':len(test),
              'condition':condition,'reason':'검증 구간 응답 모델' if qualified else '검증 오차 또는 계수 조건 미달: 참고값'}
    return result,rollout


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session',help='Directory printed by indoor_trial')
    args = parser.parse_args(argv); root = Path(args.session).expanduser()
    metadata = json.loads((root/'metadata.json').read_text(encoding='utf-8'))
    axis = math.radians(metadata['args'].get('forward_axis_deg') or 0.)
    rows = []
    for line in (root/'samples.jsonl').read_text(encoding='utf-8').splitlines():
        row = json.loads(line); mcu = row.get('mcu') or {}; odom = row.get('odom') or {}
        if not (row.get('execute') and row.get('ack') and mcu.get('receiving') and mcu.get('mode')==2
                and mcu.get('armed') and odom.get('age_s',9)<.4): continue
        if not all(isinstance(odom.get(k),(int,float)) and math.isfinite(odom[k]) for k in ('stamp_s','x','y','yaw_rad')): continue
        if rows and odom['stamp_s'] <= rows[-1]['odom']['stamp_s']: continue
        rows.append(row)
    report = {'samples':len(rows),'mass_kg':metadata['mass_kg'],'thruster_spacing_m':metadata['thruster_spacing_m'],
              'limits':['LiDAR-relative motion; symmetric walls, reflections and scan matching drift affect accuracy.',
                        'PWM fraction is not thrust fraction. Absolute drag, added mass and inertia require thrust calibration.',
                        'Pose is at the LiDAR, not automatically at the centre of mass.',
                        'No excitation manoeuvres are executed by this analyzer.']}
    traces = {}
    if len(rows)>=5:
        t = np.array([r['odom']['stamp_s'] for r in rows]); t -= t[0]
        x = np.array([r['odom']['x'] for r in rows]); y = np.array([r['odom']['y'] for r in rows])
        heading = np.unwrap([r['odom']['yaw_rad']+axis for r in rows])
        vx,vy = np.gradient(x,t),np.gradient(y,t)
        surge = vx*np.cos(heading)+vy*np.sin(heading)
        yaw_rate = np.gradient(heading,t)
        l = np.array([(r['mcu']['left']-1500)/400 for r in rows])
        r = np.array([(r['mcu']['right']-1500)/400 for r in rows])
        report['surge'],traces['surge'] = fit_channel(t,surge,(l+r)/2)
        report['yaw'],traces['yaw'] = fit_channel(t,yaw_rate,(r-l)/2)
        report['speed_abs_95_percentile_m_s'] = float(np.quantile(np.abs(surge),.95))
        report['yaw_rate_abs_95_percentile_deg_s'] = float(np.degrees(np.quantile(np.abs(yaw_rate),.95)))
        report['path_length_m'] = float(np.sum(np.hypot(np.diff(x),np.diff(y))))
        report['units'] = {'surge':'m/s; pwm_gain in m/s^2 per normalized PWM','yaw':'rad/s; pwm_gain in rad/s^2 per normalized PWM'}
    else: report['reason'] = '실제 AUTO 응답·오도메트리 기록이 부족합니다. 미리보기 기록은 모델 학습에서 제외합니다.'
    (root/'response_model.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    charts = []
    for name,trace in traces.items():
        if not trace: continue
        a = np.array(trace)
        unit = 'deg/s' if name=='yaw' else 'm/s'
        if name=='yaw': a[:,1:] = np.degrees(a[:,1:])
        with (root/(name+'_prediction.csv')).open('w',newline='',encoding='utf-8') as stream:
            writer = csv.writer(stream); writer.writerow(['time_s','observed_'+unit,'holdout_prediction_'+unit]); writer.writerows(a.tolist())
        minimum,maximum = float(a[:,1:].min()),float(a[:,1:].max()); span = max(.01,maximum-minimum)
        def line(column):
            return ' '.join(f'{40+720*(p[0]-a[0,0])/max(.1,a[-1,0]-a[0,0]):.1f},{220-180*(p[column]-minimum)/span:.1f}' for p in a)
        charts.append(f'<h2>{name} ({unit}): observed / holdout prediction</h2><svg viewBox="0 0 800 260"><path d="M40 20V220H780" fill="none" stroke="#aaa"/><text x="40" y="255">{a[0,0]:.1f} s</text><text x="700" y="255">{a[-1,0]:.1f} s</text><text x="45" y="20">{maximum:.3f}</text><text x="45" y="240">{minimum:.3f}</text><polyline points="{line(1)}" fill="none" stroke="#007c83" stroke-width="2"/><polyline points="{line(2)}" fill="none" stroke="#bd7421" stroke-width="2"/></svg>')
    page = '<!doctype html><meta charset="utf-8"><title>수조 응답 분석</title><style>body{font:16px sans-serif;max-width:1000px;margin:30px auto;padding:16px}svg{width:100%}pre{white-space:pre-wrap}</style><h1>수조 주행 응답</h1><p>청록: 관측값 / 주황: 시간순으로 분리한 검증 구간 예측</p>'+''.join(charts)+'<pre>'+html.escape(json.dumps(report,ensure_ascii=False,indent=2))+'</pre>'
    (root/'analysis.html').write_text(page,encoding='utf-8')
    print(str(root/'analysis.html')); print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__ == '__main__': main()
