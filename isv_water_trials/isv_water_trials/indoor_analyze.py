"""Fit PWM-to-motion response from indoor trials; retain a chronological holdout."""
import argparse
import csv
import html
import json
import math
from pathlib import Path
from collections import deque
import time
import numpy as np
from scipy.signal import savgol_filter
from .tank_storage import read_rows, atomic_json


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
    horizons = {}
    for seconds in (1.,2.,3.):
        errors = []; baseline = []
        for i in test[::5]:
            j = int(np.searchsorted(t,t[i]+seconds))
            if j>=len(t) or not np.all(good[i:j]) or j==i: continue
            estimate = float(v[i])
            for k in range(i,j): estimate += dt[k]*(-beta[0]*estimate+beta[1]*command[k]+beta[2])
            errors.append((estimate-v[j])**2); baseline.append((v[i]-v[j])**2)
        if errors:
            horizons[str(int(seconds))] = {'rmse':float(np.sqrt(np.mean(errors))),
                'persistence_rmse':float(np.sqrt(np.mean(baseline))),'windows':len(errors)}
    rng = np.random.default_rng(42); bootstrap = []
    # Block resampling keeps short-term sample correlation in the coefficient interval.
    for _ in range(48):
        starts = rng.integers(0,max(1,len(train)-10),size=math.ceil(len(train)/10))
        chosen = train[np.concatenate([np.arange(s,min(s+10,len(train))) for s in starts])[:len(train)]]
        if np.linalg.matrix_rank(x[chosen])==3:
            bootstrap.append(np.linalg.lstsq(x[chosen],y[chosen],rcond=None)[0])
    confidence = np.quantile(bootstrap,[.025,.975],axis=0).T.tolist() if bootstrap else []
    qualified = bool(beta[0]>0 and beta[1]>0 and rmse < persistence_rmse and rollout_rmse < max(.05,np.std(v[test])*1.5))
    qualified = qualified and bool('2' in horizons and horizons['2']['rmse']<horizons['2']['persistence_rmse'])
    result = {'qualified':qualified,'model':'dv/dt = -decay*v + pwm_gain*q + bias',
              'decay_per_s':float(beta[0]),'pwm_gain':float(beta[1]),'bias':float(beta[2]),
              'time_constant_s':float(1/beta[0]) if beta[0]>0 else None,
              'steady_velocity_per_unit_pwm':float(beta[1]/beta[0]) if beta[0]>0 else None,
              'holdout_one_step_rmse':rmse,'holdout_persistence_rmse':persistence_rmse,
              'holdout_rollout_rmse':rollout_rmse,'train_samples':len(train),'test_samples':len(test),
              'condition':condition,'reason':'검증 구간 응답 모델' if qualified else '검증 오차 또는 계수 조건 미달: 참고값'}
    result.update(coefficient_ci95=dict(zip(('decay_per_s','pwm_gain','bias'),confidence)),
                  horizon_validation=horizons,observed_command_range=[float(command[train].min()),float(command[train].max())],
                  observed_velocity_range=[float(v[train].min()),float(v[train].max())])
    return result,rollout



def chart(title, t, series, labels, unit):
    colors = ['#007c83','#c1771b','#7958ab']
    keep = np.linspace(0,len(t)-1,min(len(t),1600),dtype=int)
    values = np.concatenate([np.asarray(a)[np.isfinite(a)] for a in series])
    if not len(values): return ''
    lo,hi = float(values.min()),float(values.max()); span = max(.01,hi-lo)
    shapes = []
    for a,color in zip(series,colors):
        parts = []; previous = None
        for i in keep:
            if not math.isfinite(a[i]): previous = None; continue
            op = 'M' if previous is None or t[i]-t[previous]>2 else 'L'
            parts.append(f'{op}{50+720*(t[i]-t[0])/max(.1,t[-1]-t[0]):.1f},{215-180*(a[i]-lo)/span:.1f}')
            previous = i
        shapes.append(f'<path d="{" ".join(parts)}" fill="none" stroke="{color}" stroke-width="1.6"/>')
    return f'<h2>{html.escape(title)} ({unit})</h2><p>'+ ' / '.join(f'<span style="color:{c}">{html.escape(label)}</span>' for label,c in zip(labels,colors))+f'</p><svg viewBox="0 0 800 255"><path d="M50 25V215H780" fill="none" stroke="#aaa"/><text x="50" y="20">{hi:.3f}</text><text x="50" y="232">{lo:.3f}</text><text x="50" y="250">{t[0]:.1f}s</text><text x="690" y="250">{t[-1]:.1f}s</text>'+''.join(shapes)+'</svg>'


def derivative(values,t):
    result = np.full(len(t),np.nan)
    cuts = np.r_[0,np.flatnonzero((np.diff(t)>.3)|(np.diff(t)<.025))+1,len(t)]
    for a,b in zip(cuts[:-1],cuts[1:]):
        if b-a>=3: result[a:b] = np.gradient(values[a:b],t[a:b])
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('sessions',nargs='+',help='One or more trial directories')
    parser.add_argument('--output',help='Output directory; default is the last session')
    args = parser.parse_args(argv)
    roots = [Path(v).expanduser() for v in args.sessions]; root = Path(args.output).expanduser() if args.output else roots[-1]
    root.mkdir(parents=True,exist_ok=True)
    rows = deque(maxlen=120000); offset = 0.; all_count = 0; metadata = None
    for session_root in roots:
        meta = json.loads((session_root/'metadata.json').read_text(encoding='utf-8'))
        if metadata and any(meta.get(k)!=metadata.get(k) for k in ('mass_kg','length_m','width_m','thruster_spacing_m')):
            raise ValueError('Hull metadata differs across sessions')
        if metadata and meta['args'].get('forward_axis_deg',0)!=metadata['args'].get('forward_axis_deg',0):
            raise ValueError('Forward axis differs across sessions')
        metadata = meta; first = None; last = None
        for row in read_rows(session_root):
            all_count += 1; o = row.get('odom') or {}
            if not all(isinstance(o.get(k),(int,float)) and math.isfinite(o[k]) for k in ('stamp_s','x','y','yaw_rad')): continue
            if last is not None and o['stamp_s']<=last: continue
            if o.get('age_s',9)>.4: continue
            if first is None: first = o['stamp_s']
            last = o['stamp_s']
            # Keep only analysis inputs in RAM; raw records stay on disk.
            mcu = row.get('mcu') or {}; imu = row.get('imu') or {}
            rows.append({'analysis_t':offset+last-first,'odom':o,
                'execute':row.get('execute'),'ack':row.get('ack'),'state':row.get('state'),
                'mcu':{k:mcu.get(k) for k in ('receiving','mode','armed','age_s','left','right') if k in mcu},
                'imu':{k:imu.get(k) for k in ('gyro_sensor_z_rad_s','age_s','accel_m_s2') if k in imu}})
        if first is not None: offset += last-first+2.
    rows = list(rows); traces = {}; charts = []
    report = {'samples':len(rows),'total_records':all_count,'fit_window_max_rows':120000,
        'sessions':[str(r) for r in roots],'mass_kg':metadata['mass_kg'],'length_m':metadata['length_m'],
        'width_m':metadata['width_m'],'thruster_spacing_m':metadata['thruster_spacing_m'],
        'limits':['PWM-based local surge/yaw response at <=20%; no extrapolation to other loads/speeds/waves.',
                  'Absolute drag, added mass and inertia require independently measured thrust.',
                  'LiDAR reference point; lateral velocity includes sensor offset during turning.',
                  'Bootstrap intervals describe sampled trials; unobserved disturbances are not covered.']}
    if len(rows)>=5:
        t = np.array([r['analysis_t'] for r in rows]); t -= t[0]
        x = np.array([r['odom']['x'] for r in rows]); y = np.array([r['odom']['y'] for r in rows])
        axis = math.radians(metadata['args'].get('forward_axis_deg') or 0.)
        heading = np.unwrap([r['odom']['yaw_rad']+axis for r in rows])
        vx,vy = derivative(x,t),derivative(y,t)
        surge = vx*np.cos(heading)+vy*np.sin(heading); sway = -vx*np.sin(heading)+vy*np.cos(heading)
        rate = derivative(heading,t)
        valid = np.array([bool(r.get('execute') and r.get('ack') and (r.get('mcu') or {}).get('receiving')
            and (r.get('mcu') or {}).get('mode')==2 and (r.get('mcu') or {}).get('armed')
            and (r.get('mcu') or {}).get('age_s',9)<.4 and r.get('state') in ('RUNNING','DWELL')) for r in rows])
        def accepted(r,k):
            value = (r.get('mcu') or {}).get(k)
            return (value-1500)/400 if isinstance(value,(int,float)) and 1100<=value<=1900 else np.nan
        l = np.array([accepted(r,'left') for r in rows]); rr = np.array([accepted(r,'right') for r in rows])
        valid &= np.isfinite(surge)&np.isfinite(rate)&np.isfinite(l)&np.isfinite(rr)
        # Scope identification to the commanded 20% envelope, including neutral coasts.
        valid &= (np.abs(l)<=.201)&(np.abs(rr)<=.201)&(np.abs(surge)<2)&(np.abs(rate)<3)
        report['auto_samples'] = int(valid.sum())
        for name,values,command in [('surge',surge,(l+rr)/2),('yaw',rate,(rr-l)/2)]:
            report[name],traces[name] = fit_channel(t[valid],values[valid],command[valid])
        if all('decay_per_s' in report[k] for k in ('surge','yaw')):
            report['trajectory_validation'] = {}
            indices = np.flatnonzero(valid); indices = indices[int(len(indices)*.7):]
            for horizon in (1,2,3):
                errors = []; baseline = []; angle_errors = []
                for i in indices[::10]:
                    j = int(np.searchsorted(t,t[i]+horizon))
                    if j>=len(t) or j==i or not np.all(valid[i:j+1]) or np.any(np.diff(t[i:j+1])>.3): continue
                    px,py,a,u,r = x[i],y[i],heading[i],surge[i],rate[i]
                    for k in range(i,j):
                        dt = t[k+1]-t[k]; s,yaw_model = report['surge'],report['yaw']
                        u += dt*(-s['decay_per_s']*u+s['pwm_gain']*(l[k]+rr[k])/2+s['bias'])
                        r += dt*(-yaw_model['decay_per_s']*r+yaw_model['pwm_gain']*(rr[k]-l[k])/2+yaw_model['bias'])
                        a += r*dt
                        px += dt*(u*math.cos(a)-sway[i]*math.sin(a)); py += dt*(u*math.sin(a)+sway[i]*math.cos(a))
                    errors.append((px-x[j])**2+(py-y[j])**2)
                    baseline.append((x[i]+vx[i]*(t[j]-t[i])-x[j])**2+(y[i]+vy[i]*(t[j]-t[i])-y[j])**2)
                    angle_errors.append(math.atan2(math.sin(a-heading[j]),math.cos(a-heading[j]))**2)
                if errors:
                    report['trajectory_validation'][str(horizon)] = {'position_rmse_m':float(np.sqrt(np.mean(errors))),
                        'constant_velocity_rmse_m':float(np.sqrt(np.mean(baseline))),
                        'heading_rmse_deg':float(np.degrees(np.sqrt(np.mean(angle_errors)))),'windows':len(errors)}
            report['trajectory_validation_note'] = 'Held-out recorded commands; lateral drift held constant. Future commands must be supplied for prediction.'
        imu = np.array([(r.get('imu') or {}).get('gyro_sensor_z_rad_s',np.nan) if (r.get('imu') or {}).get('age_s',9)<.4 else np.nan for r in rows],dtype=float)
        report['imu_note'] = 'IMU Z is in its published sensor frame; sign and mounting require calibration before fusion.'
        both = valid & np.isfinite(imu)
        if np.count_nonzero(both)>30 and np.std(imu[both])>.005 and np.std(rate[both])>.005:
            report['lidar_imu_yaw_correlation'] = float(np.corrcoef(rate[both],imu[both])[0,1])
        report['speed_abs_95_percentile_m_s'] = float(np.nanquantile(np.abs(surge),.95)) if np.isfinite(surge).any() else None
        distance = np.hypot(np.diff(x),np.diff(y)); good = (np.diff(t)<.3)&(distance<.6)
        report['path_length_m'] = float(distance[good].sum())
        charts += [chart('좌우 입력',t,[l*100,rr*100],['좌현 MCU','우현 MCU'],'%'),
                   chart('전진·횡방향 속도',t,[surge,sway],['전진','횡방향'],'m/s'),
                   chart('회전 응답',t,[np.degrees(rate),np.degrees(imu)],['라이다 yaw 변화','IMU 센서 Z'],'°/s'),
                   chart('지도 내 위치 변화',t,[x,y],['연속 오도메트리 X','Y'],'m'),
                   chart('가속·감속',t,[derivative(surge,t)],['라이다 전진 가속도'],'m/s²')]
        accel = np.array([(r.get('imu') or {}).get('accel_m_s2',[np.nan]*3) for r in rows],dtype=float)
        charts.append(chart('IMU 가속도',t,[accel[:,i] for i in range(3)],['센서 X','Y','Z'],'m/s²'))
        with (root/'motion.csv.incoming').open('w',newline='',encoding='utf-8') as stream:
            w = csv.writer(stream); w.writerow(['t_s','x_m','y_m','surge_m_s','sway_m_s','yaw_deg_s','imu_z_deg_s','left_percent','right_percent','eligible'])
            w.writerows(zip(t,x,y,surge,sway,np.degrees(rate),np.degrees(imu),100*l,100*rr,valid.astype(int)))
        (root/'motion.csv.incoming').replace(root/'motion.csv')
    else: report['reason'] = '유효한 위치 기록 대기 중입니다.'
    atomic_json(root/'response_model.json',report)
    for name,trace in traces.items():
        if not trace: continue
        a = np.array(trace); unit = '°/s' if name=='yaw' else 'm/s'
        if name=='yaw': a[:,1:] = np.degrees(a[:,1:])
        charts.append(chart(name+' 검증 구간',a[:,0],[a[:,1],a[:,2]],['실측','자유 예측'],unit))
        with (root/(name+'_prediction.csv')).open('w',newline='',encoding='utf-8') as stream:
            writer = csv.writer(stream); writer.writerow(['time_s','observed','holdout_prediction']); writer.writerows(a.tolist())
    progress_path = root/'convergence.jsonl'
    progress = {'unix':time.time(),'samples':report.get('auto_samples',0)}
    for channel in ('surge','yaw'):
        progress[channel] = {k:report.get(channel,{}).get(k) for k in ('decay_per_s','pwm_gain','time_constant_s','qualified','coefficient_ci95','horizon_validation')}
    with progress_path.open('a',encoding='utf-8') as stream: stream.write(json.dumps(progress,ensure_ascii=False,allow_nan=False)+'\n')
    history = []
    for line in progress_path.read_text(encoding='utf-8').splitlines():
        try: history.append(json.loads(line))
        except ValueError: continue
    if len(history)>1:
        count = np.array([h['samples'] for h in history],dtype=float)
        # The horizontal axis here is analysis time; sample counts remain in the JSONL.
        ht = np.array([h['unix'] for h in history]); ht-=ht[0]
        for channel in ('surge','yaw'):
            vals = [h[channel].get('time_constant_s') for h in history]
            charts.append(chart(channel+' 시정수 추정 이력',ht,[np.array([v if v is not None else np.nan for v in vals])],['추정 시정수'],'s'))
    page = '<!doctype html><html lang="ko"><meta charset="utf-8"><title>수조 응답 분석</title><style>body{font:16px sans-serif;max-width:1000px;margin:30px auto;padding:16px;color:#193947}svg{width:100%}pre{white-space:pre-wrap}h2{margin-top:36px}text{font:12px sans-serif}</style><h1>수조 주행 응답</h1><p>입력·이동·회전·가속도 누적 기록과 시간순 70/30 검증입니다. 1·2·3초 예측 오차와 계수 신뢰 구간은 아래 결과에서 확인합니다. 계수 변화가 작아져도 관측한 수조·출력 범위에 한해 사용합니다.</p>'+''.join(charts)+'<pre>'+html.escape(json.dumps(report,ensure_ascii=False,indent=2))+'</pre></html>'
    target = root/'analysis.html'; temporary = target.with_suffix('.html.incoming')
    temporary.write_bytes(page.encode('utf-8')); temporary.replace(target)
    print(str(target))


if __name__ == '__main__': main()
