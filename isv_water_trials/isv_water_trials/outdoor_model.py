"""Timestamped RC identification. No ROS imports and no firmware writes.

Velocity models describe GNSS-referenced motion, not thrust in newtons or
uniquely identified hydrodynamic coefficients. Delay includes receipt timing.
"""
from collections import deque
from copy import deepcopy
import hashlib
import json
import math
import numpy as np
from .outdoor_config import fingerprint, learning_identity


def wrap(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def local_xy(lat, lon, origin):
    return np.array([math.radians(lon-origin[1])*6371000*math.cos(math.radians(origin[0])),
                     math.radians(lat-origin[0])*6371000])


def latlon(x, y, origin):
    return [origin[0]+math.degrees(y/6371000),
            origin[1]+math.degrees(x/(6371000*math.cos(math.radians(origin[0]))))]


def applied_command(status, config):
    """Invert *observed ESC output* using AUTO polarity, even during RC driving."""
    if not status.get('receiving') or status.get('age_s', 99) > .3:
        return None
    values = []
    for side, reverse in zip(('left', 'right'), config['auto_reversed']):
        pulse = status.get('observed_'+side+'_us')
        if pulse is None or not config['esc_min_us']-5 <= pulse <= config['esc_max_us']+5:
            return None
        delta = pulse-config['esc_neutral_us']
        span = config['esc_max_us']-config['esc_neutral_us'] if delta >= 0 else config['esc_neutral_us']-config['esc_min_us']
        values.append(float(np.clip((-1 if reverse else 1)*delta/span, -1, 1)))
    return values


def logical_pwm(command):
    """Normalized learned command -> existing bridge's 1100..1900 interface."""
    return [int(round(1500+400*float(np.clip(x, -1, 1)))) for x in command]


def features(state, command):
    state, command = np.asarray(state), np.asarray(command)
    return np.concatenate((state, command, command*np.abs(command),
                           np.ones((*state.shape[:-1], 1))), axis=-1)


class ResponseModel:
    def __init__(self, data, inertia_scales=(1., 1., 1.)):
        if data.get('schema') != 'outdoor-response-v1':
            raise ValueError('지원하지 않는 모델 형식')
        self.data = data
        self.dt = float(data['dt_s'])
        self.delay = int(data['delay_steps'])
        self.coefficients = np.array(data['coefficients'], dtype=float)
        if self.coefficients.shape != (8, 3) or not np.isfinite(self.coefficients).all():
            raise ValueError('모델 계수 오류')
        scales = np.asarray(inertia_scales, dtype=float)
        if scales.shape != (3,) or not np.isfinite(scales).all() or np.any(scales <= 0):
            raise ValueError('Invalid prediction inertia scales')
        self.inertia_scales = tuple(scales)
        # Scale acceleration, including drag, while preserving steady motion.
        # Keep the identified data unchanged for learning and model comparison.
        self.coefficients /= scales
        self.coefficients[:3] += np.diag(1.-1./scales)
        if not .05 <= self.dt <= 1 or not 0 <= self.delay <= 30:
            raise ValueError('모델 시간 간격 오류')
        if max(abs(np.linalg.eigvals(self.coefficients[:3].T))) >= 1.001:
            raise ValueError('불안정한 응답 모델')

    def for_navigation(self, config):
        mass = float(config.get('prediction_mass_scale', 1.))
        yaw = float(config.get('prediction_yaw_inertia_scale', 1.))
        scales = (mass, mass, yaw)
        return self if self.inertia_scales == scales else ResponseModel(self.data, scales)

    def step(self, state, delayed_command):
        with np.errstate(over='ignore',invalid='ignore'):
            result=features(state, delayed_command) @ self.coefficients
        if not np.isfinite(result).all() or np.max(abs(result))>20:
            raise ValueError('모델 예측 발산 · 학습 범위와 계수를 확인하세요')
        return result

    def predict(self, state, commands, history=None):
        commands = np.asarray(commands, dtype=float)
        history = np.asarray(history if history is not None else np.zeros((self.delay, 2))).reshape(-1, 2)
        if len(history) < self.delay:
            raise ValueError('명령 이력 부족')
        queued = np.concatenate((history[-self.delay:] if self.delay else history[:0], commands))
        z, result = np.array(state, dtype=float), []
        for i in range(len(commands)):
            z = self.step(z, queued[i]); result.append(z.copy())
        return np.asarray(result)


def eligible_row(row):
    if not row.get('valid') or row.get('learning') is False or row.get('mode') not in ('RC','AUTO'):
        return False
    try:
        velocity=np.asarray(row['velocity'],dtype=float)
        commands=np.asarray(row.get('commands',[]),dtype=float)
        return bool(math.isfinite(row['t']) and velocity.shape==(3,) and np.isfinite(velocity).all()
                and commands.ndim==2 and commands.shape[1]==3 and len(commands)>0
                and np.isfinite(commands).all() and np.max(abs(commands[:,1:]))<=1.001)
    except (KeyError,ValueError,TypeError):return False


def _grid(rows, dt):
    """Interpolate only bounded, continuous native GNSS intervals; never bridge outages."""
    runs, run = [], []
    for row in rows:
        valid = eligible_row(row)
        if run and (not valid or row.get('segment') != run[-1].get('segment') or row['t']-run[-1]['t'] > 1.6 or row['t'] <= run[-1]['t']):
            runs.append(run); run = []
        if valid: run.append(row)
    if run: runs.append(run)
    result = []
    for run in runs:
        if len(run) < 8: continue
        t = np.array([r['t'] for r in run]); times = np.arange(t[0], t[-1], dt)
        z = np.array([r['velocity'] for r in run])
        # Per-sample command history comes from 10 Hz MCU observations, not
        # interpolated stick values at the sparse GNSS update rate.
        ct, cq = [], []
        for r in run:
            for stamp, left, right in r.get('commands', []):
                if not ct or stamp > ct[-1]: ct.append(stamp); cq.append([left, right])
        if len(ct) < 3: continue
        ct, cq = np.asarray(ct), np.asarray(cq)
        idx = np.searchsorted(ct, times, side='right')-1
        good = (idx >= 0) & (times-ct[np.maximum(idx, 0)] < .35)
        states = np.column_stack([np.interp(times, t, z[:, j]) for j in range(3)])
        commands = cq[np.maximum(idx, 0)]
        result.append((times, states, commands, good))
    return result


def score_rollouts(model, runs, start_fraction=0., end_fraction=1.):
    errors, baseline, heading, displacement = [], [], [], []
    output_windows=[];speed_windows=[]
    horizon = max(1, round(3/model.dt))
    budget_step = max(1, math.ceil(sum(len(r[0]) for r in runs)*(end_fraction-start_fraction)/80))
    for t, z, q, good in runs:
        last_output_time=-np.inf
        lo = max(model.delay, int(len(t)*start_fraction))
        hi = int(len(t)*end_fraction)-horizon
        for i in range(lo, hi, max(budget_step, round(.6/model.dt))):
            if not good[i-model.delay:i+horizon+1].all(): continue
            try: pred = model.predict(z[i], q[i:i+horizon], q[i-model.delay:i])
            except ValueError: return {'windows':0,'unstable':True}
            truth = z[i+1:i+horizon+1]
            errors.append(pred-truth); baseline.append(z[i]-truth)
            heading.append(np.cumsum((pred[:, 2]-truth[:, 2])*model.dt))
            pp, tp = np.zeros(2), np.zeros(2); py = ty = 0.; pe = []
            for p, v in zip(pred, truth):
                py += p[2]*model.dt; ty += v[2]*model.dt
                pp += model.dt*np.array([np.cos(py)*p[0]-np.sin(py)*p[1], np.sin(py)*p[0]+np.cos(py)*p[1]])
                tp += model.dt*np.array([np.cos(ty)*v[0]-np.sin(ty)*v[1], np.sin(ty)*v[0]+np.cos(ty)*v[1]])
                pe.append(float(np.linalg.norm(pp-tp)))
            displacement.append(pe)
            # Report non-overlapping 3 s windows for output-band coverage.
            if t[i]-last_output_time>=3.-1e-6:
                level=float(np.max(abs(q[i-model.delay:i+horizon])))
                output_windows.append((min(9,int(level*10)),len(errors)-1))
                speed_windows.append((int(np.linalg.norm(z[i,:2])/.2),len(errors)-1))
                last_output_time=t[i]
    if not errors: return {'windows': 0}
    errors, baseline = np.asarray(errors), np.asarray(baseline)
    result = {'windows': len(errors), 'horizons': {}}
    for seconds in (1, 2, 3):
        i = min(horizon-1, round(seconds/model.dt)-1)
        result['horizons'][str(seconds)] = {
            'velocity_rmse': np.sqrt(np.mean(errors[:, i]**2, axis=0)).tolist(),
            'persistence_rmse': np.sqrt(np.mean(baseline[:, i]**2, axis=0)).tolist(),
            'heading_rmse_deg': float(np.degrees(np.sqrt(np.mean(np.asarray(heading)[:, i]**2)))),
            'integrated_position_rmse_m': float(np.sqrt(np.mean(np.asarray(displacement)[:, i]**2)))}
    result['output_bands']=[]
    index=min(horizon-1,round(2/model.dt)-1)
    for band in range(10):
        selected=[i for b,i in output_windows if b==band]
        item={'low_percent':band*10,'high_percent':(band+1)*10,'windows':len(selected),'score':None}
        if selected:
            rmse=np.sqrt(np.mean(errors[selected,index]**2,axis=0))
            item.update(velocity_rmse=rmse.tolist(),score=float(np.mean((rmse/[.3,.3,.2])**2)))
        result['output_bands'].append(item)
    covered=[b for b in result['output_bands'] if b['windows']>=5]
    result['best_output_band']=min(covered,key=lambda b:b['score']) if covered else None
    result['speed_bands']=[]
    for band in sorted({b for b,_ in speed_windows}):
        selected=[i for b,i in speed_windows if b==band]
        rmse=np.sqrt(np.mean(errors[selected,index]**2,axis=0))
        result['speed_bands'].append(dict(low_m_s=round(band*.2,2),high_m_s=round((band+1)*.2,2),
            windows=len(selected),velocity_rmse=rmse.tolist(),score=float(np.mean((rmse/[.3,.3,.2])**2))))
    covered=[b for b in result['speed_bands'] if b['windows']>=5]
    result['best_speed_band']=min(covered,key=lambda b:b['score']) if covered else None
    result['speed_band_basis']='initial measured planar speed; 2-second prediction; non-overlapping 3-second windows'
    return result


def fit_model(rows, config, parent=None, lineage=None):
    """Chronological fit/tune/test. Delay selection never sees the final 30%."""
    lineage=lineage or {}
    if parent and lineage:
        cutoff=float(lineage['data_cutoff_t'])
        original=[r for r in rows if r['t']<=cutoff]
        fresh=[r for r in rows if r['t']>lineage['started_t'] and r.get('branch_id')==lineage['id']]
        rows=original+fresh
    elif parent:
        # An incumbent must be compared on observations it has never seen.
        cutoff=float(parent['training_end_t'])
        original=[r for r in rows if r['t']<=cutoff]
        fresh=[r for r in rows if r['t']>cutoff]
    else: original=[];fresh=rows
    native = sum(eligible_row(r) for r in fresh)
    if native < 120: raise ValueError(f'유효 주행 속도 표본 {native}/120')
    dt = config['prediction_dt_s']; runs = _grid(fresh, dt); old_runs=_grid(original,dt)
    pairs = sum(len(t) for t, *_ in runs)
    if pairs < 300: raise ValueError('연속 주행 기록 60초 이상 필요')
    # Split each continuous run; leave 3 s plus max delay between blocks.
    gap = math.ceil(5/dt); candidates = []
    def objective(score):
        e=np.array([score['horizons'][str(h)]['velocity_rmse'] for h in (1,2,3)])/np.array([.3,.3,.2])
        return float(np.mean(e**2))
    for delay in range(0, round(1.6/dt)+1):
        xx, yy = [], []
        for run_index,(t, z, q, good) in enumerate(old_runs+runs):
            end=len(t)-1 if run_index<len(old_runs) else int(len(t)*.55)-gap
            ii = np.arange(delay, max(delay,end))
            if len(ii):
                valid = np.ones(len(ii), dtype=bool)
                for shift in range(-delay, 2): valid &= good[ii+shift]
                ii = ii[valid]
                if len(ii): xx.append(features(z[ii], q[ii-delay])); yy.append(z[ii+1])
        if not xx: continue
        x, y = np.concatenate(xx), np.concatenate(yy)
        if len(x) < 120: continue
        if len(x) > 12000:
            keep = np.linspace(0, len(x)-1, 12000, dtype=int); x, y = x[keep], y[keep]
        # Balance input regions without choosing examples by their fit error.
        bins = np.clip(((x[:, 3:5]+1)*2).astype(int), 0, 3)
        keys = bins[:, 0]*4+bins[:, 1]; groups = [np.flatnonzero(keys==k) for k in np.unique(keys)]
        quota = max(30, int(np.median([len(g) for g in groups])))
        balanced = np.concatenate([g[np.linspace(0, len(g)-1, min(quota, len(g)), dtype=int)] for g in groups])
        subsets = [('전체', np.arange(len(x))), ('최근', np.arange(max(0, len(x)-max(240, len(x)//2)), len(x))), ('균형', balanced), ('강건', np.arange(len(x)))]
        for label, indices in subsets:
            for family in ('선형','비선형'):
                columns=list(range(8)) if family=='비선형' else [0,1,2,3,4,7]
                design, target = x[indices][:, columns], y[indices]
                solved=np.linalg.solve(design.T@design+np.eye(len(columns))*.005,design.T@target)
                if label=='강건':
                    # Train-only Huber weighting; selection/test observations never set weights.
                    for _ in range(3):
                        residual=design@solved-target
                        scale=np.maximum(np.median(abs(residual),axis=0)*1.4826,[.005,.005,.003])
                        size=np.max(abs(residual)/scale,axis=1)
                        weight=np.minimum(1.,1.5/np.maximum(size,1e-9))
                        solved=np.linalg.solve(design.T@(design*weight[:,None])+np.eye(len(columns))*.005,
                                               design.T@(target*weight[:,None]))
                coefficients = np.zeros((8, 3)); coefficients[columns] = solved
                data = {'schema': 'outdoor-response-v1', 'dt_s': dt, 'delay_steps': delay,
                        'coefficients': coefficients.tolist(), 'signature': fingerprint(config),
                        'learning_settings':learning_identity(config), 'variant':label+'·'+family,'candidate_name':label+'·'+family,
                        'boat': {k: config[k] for k in ('mass_kg', 'length_m', 'width_m', 'thruster_spacing_m')}}
                try: model = ResponseModel(data)
                except ValueError: continue
                score = score_rollouts(model, runs, .55, .70)
                if not score['windows']: continue
                candidates.append((objective(score), model))
    if parent:
        baseline=deepcopy(parent)
        name=parent.get('candidate_name') or parent.get('variant')
        baseline['candidate_name']=parent.get('display_name','기존 모델') if name in (None,'기준 보존','기존 모델') else name
        baseline['variant']='기준 보존'
        model=ResponseModel(baseline)
        score=score_rollouts(model,runs,.55,.70)
        if score['windows']: candidates.append((objective(score),model))
    if not candidates: raise ValueError('안정적인 모델을 찾을 연속 기록 부족')
    model = min(candidates, key=lambda item: item[0])[1]
    ranked=sorted(candidates,key=lambda item:item[0])[:8]
    baseline_entries=[entry for entry in candidates if entry[1].data['variant']=='기준 보존']
    if baseline_entries and not any(entry[1].data['variant']=='기준 보존' for entry in ranked):
        ranked=ranked[:7]+baseline_entries[:1]
    model.data['ranking'] = [{'label': (('기준 보존 · '+m.data['candidate_name']) if m.data['variant']=='기준 보존' else m.data['variant'])+f' {m.delay*dt:.1f}s', 'score': value}
                             for value, m in ranked]
    model.data['candidate_catalog']=[dict(name=m.data.get('candidate_name',m.data['variant']),delay_s=m.delay*dt,
        score=value,incumbent=m.data['variant']=='기준 보존',selected=m is model)
        for value,m in sorted(candidates,key=lambda item:item[0])]
    model.data['candidate_count'] = len(candidates)
    metrics = score_rollouts(model, runs, .70, 1.)
    q = np.concatenate([q[g] for _, _, q, g in old_runs+runs])
    speeds=np.concatenate([np.linalg.norm(z[g,:2],axis=1) for _,z,_,g in old_runs+runs])
    levels=np.clip(np.max(abs(q),axis=1)*100,0,100)
    counts=np.bincount(np.minimum(9,(levels/10).astype(int)),minlength=10)
    coverage=[{'low_percent':i*10,'high_percent':(i+1)*10,'seconds':float(n*dt),
               'share_percent':float(100*n/len(levels))} for i,n in enumerate(counts)]
    average=float(np.mean(levels));peak=coverage[int(counts.argmax())]
    singular = np.linalg.svd(q-q.mean(axis=0), compute_uv=False)
    excitation = float(singular[-1]/max(singular[0], 1e-9))
    reasons = []
    if excitation < .1: reasons.append('좌우 명령을 독립적으로 바꾼 기록 부족')
    if metrics['windows'] < 20: reasons.append('분리한 검증 기록 부족')
    if metrics['windows']:
        m = metrics['horizons']['2']; e = m['velocity_rmse']
        if e[0] > .25 or e[1] > .25 or e[2] > .15 or m['heading_rmse_deg'] > 15:
            reasons.append('2초 예측 오차가 운용 기준 초과')
        if sum(np.array(e)**2) >= sum(np.array(m['persistence_rmse'])**2):
            reasons.append('현재 속도 유지 예측보다 개선되지 않음')
    used=[r for r in rows if eligible_row(r)]
    stamps = np.array([r['t'] for r in used])
    model.data.update(metrics=metrics, qualified=not reasons, reasons=reasons,
        synthetic=any(r.get('synthetic',False) for r in rows),
        name=model.data.get('candidate_name',model.data['variant']),
        speed_coverage={'mean_m_s':float(np.mean(speeds)),'seconds':float(len(speeds)*dt),'definition':'measured planar speed sqrt(u^2+v^2)'},
        recording_segments=len({r.get('segment') for r in used}),
        continuous_runs=len(old_runs+runs),training_samples=len(used),
        mode_samples={mode:sum(r.get('mode')==mode for r in used) for mode in ('RC','AUTO')},
        output_coverage={'mean_percent':average,'peak_band':peak,'bands':coverage,
            'total_seconds':float(len(levels)*dt),'definition':'max(abs(left),abs(right))'},
        signature=fingerprint(config), learning_settings=learning_identity(config),
        lineage=deepcopy(lineage), parent_model_id=parent['id'] if parent else None,
        command_min=q.min(axis=0).tolist(), command_max=q.max(axis=0).tolist(),
        native_samples=native, excitation_ratio=excitation,
        native_period_s=float(np.median(np.diff(stamps))),
        delay_resolution_note='수신 시각 기준 유효 지연. GPS 주기보다 짧은 지연은 구분이 어려움.',
        training_end_t=float(max(r['t'] for r in rows)),
        velocity_limit=np.max(np.abs(np.array([r['velocity'] for r in used])), axis=0).tolist())
    if parent:
        model.data['parent_comparison']=score_rollouts(ResponseModel(parent),runs,.70,1.)
    model.data.pop('id',None)
    model.data['id'] = hashlib.sha256(json.dumps(model.data, sort_keys=True).encode()).hexdigest()[:16]
    return model.data


class LiveComparison:
    """Open-loop windows fed actual subsequent commands, never reset to measured state."""
    def __init__(self): self.windows = deque(maxlen=20); self.model_id = None

    def update(self, model, stamp, velocity, command_history):
        if model is None: return None
        if self.model_id != model.data['id']:
            self.windows.clear(); self.model_id = model.data['id']
        result = None
        history = np.asarray(command_history, dtype=float).reshape(-1, 3)
        for window in list(self.windows):
            elapsed = stamp-window['t']; n = round(elapsed/model.dt)
            if n < 1 or n > round(3/model.dt): continue
            times = window['t']+np.arange(-model.delay, n)*model.dt
            indices = np.searchsorted(history[:, 0], times, side='right')-1
            if len(history) == 0 or np.any(indices < 0) or np.any(times-history[np.maximum(indices, 0), 0] > .35): continue
            q = history[indices, 1:]
            try: prediction = model.predict(window['z'], q[model.delay:], q[:model.delay])[-1]
            except ValueError: continue
            if elapsed >= .8 and (result is None or abs(elapsed-1) < abs(result['horizon_s']-1)):
                result = {'horizon_s': elapsed, 'predicted': prediction.tolist(),
                          'measured': list(velocity), 'model_id': self.model_id}
        self.windows.append({'t': stamp, 'z': list(velocity)})
        return result
