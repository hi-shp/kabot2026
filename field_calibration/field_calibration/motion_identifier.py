"""Offline least squares. Independent experiments are mandatory for holdout."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from .model import Commands, predict
from .core import wrap


def read_experiment(path):
    path = Path(path).resolve()
    if path.is_dir():
        path = path/'states.csv'
    with path.open(newline='', encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    states = []
    for row in rows:
        if row['valid'].lower() != 'true':
            states.append(dict(valid=False))
            continue
        s = {k: float(row[k]) for k in ('stamp', 'x', 'y', 'heading', 'u', 'v', 'r')}
        s.update(valid=True, epoch=int(row['epoch']), frame=row['frame'],
                 experiment_id=row['experiment_id'])
        if all(math.isfinite(s[k]) for k in ('stamp', 'x', 'y', 'heading', 'u', 'v', 'r')):
            states.append(s)
    commands = Commands(maxlen=None)
    with (path.parent/'commands.csv').open(newline='', encoding='utf-8') as f:
        for row in csv.DictReader(f):
            commands.add(row['kind'], float(row['stamp']), float(row['value']))
    ids = {r['experiment_id'] for r in rows}
    return dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                ids=sorted(ids), states=states, commands=commands)


def samples(datasets, delay, channel, neutral, servo_neutral):
    X, Y = [], []
    for dataset in datasets:
        previous = None
        for s in dataset['states']:
            old, previous = previous, s if s['valid'] else None
            if not s['valid'] or not old or s['epoch'] != old['epoch'] or s['frame'] != old['frame']:
                continue
            dt = s['stamp']-old['stamp']
            if not 0.01 <= dt <= 0.3:
                continue
            t = old['stamp']-delay
            thr = dataset['commands'].at('throttle', t)
            steer = dataset['commands'].at('steering', t)
            if not thr or not steer or max(t-thr[0], t-steer[0]) > 2.0:
                continue
            if channel == 'surge':
                X.append([old['u'], thr[1]-neutral, 1.0])
                Y.append((s['u']-old['u'])/dt)
            else:
                delta = math.radians(steer[1]-servo_neutral)
                X.append([old['r'], old['u']*max(delta, 0), old['u']*min(delta, 0), 1.0])
                Y.append((s['r']-old['r'])/dt)
    return np.asarray(X), np.asarray(Y)


def fit_channel(datasets, channel, neutral, servo_neutral, minimum):
    best = None
    for delay in np.arange(0, 1.01, 0.1):
        X, Y = samples(datasets, float(delay), channel, neutral, servo_neutral)
        if len(Y) < minimum:
            continue
        split = int(len(Y)*0.8)
        scale = np.maximum(np.linalg.norm(X[:split], axis=0), 1e-9)
        norm = X[:split]/scale
        if np.linalg.matrix_rank(norm) < X.shape[1] or np.linalg.cond(norm) > 1e5:
            continue
        coef = np.linalg.lstsq(norm, Y[:split], rcond=None)[0]/scale
        if coef[0] >= 0:
            continue
        loss = float(np.mean((X[split:]@coef-Y[split:])**2))
        if best is None or loss < best[0]:
            best = loss, float(delay), X, Y
    if best is None:
        raise ValueError(f'{channel}: insufficient samples/excitation or nondissipative fit')
    loss, delay, X, Y = best
    scale = np.maximum(np.linalg.norm(X, axis=0), 1e-9)
    coef = np.linalg.lstsq(X/scale, Y, rcond=None)[0]/scale
    if coef[0] >= 0:
        raise ValueError('nondissipative final fit')
    return coef.tolist(), delay, dict(samples=len(Y), tuning_derivative_rmse=math.sqrt(loss))


def validation_metrics(datasets, model):
    from .core import ErrorMatcher
    all_errors = {'model': [], 'baseline': []}
    for dataset in datasets:
        matchers = {key: ErrorMatcher() for key in all_errors}
        last = None
        for s in dataset['states']:
            for key, matcher in matchers.items():
                all_errors[key].extend(matcher.observe(s))
            if not s['valid']:
                last = None
                continue
            if last is None or s['stamp']-last >= 0.25:
                p = predict(s, dataset['commands'], model)
                if p and p['model_status'] == 'CALIBRATED':
                    matchers['model'].add(p)
                    matchers['baseline'].add(predict(s, dataset['commands']))
                    last = s['stamp']
    results = {}
    for kind, errors in all_errors.items():
        results[kind] = {}
        for horizon in (0.5, 1.0, 2.0):
            e = [v for v in errors if v['horizon'] == horizon]
            results[kind][str(horizon)] = dict(count=len(e), **{
                key+'_rmse': math.sqrt(sum(v[key]**2 for v in e)/len(e)) if e else None
                for key in ('position_error', 'heading_error', 'yaw_rate_error', 'surge_error')})
    return results


def identify(train, holdout, neutral=0.0, servo_neutral=90.0, minimum=100):
    train_paths, val_paths = {d['path'] for d in train}, {d['path'] for d in holdout}
    train_ids = {v for d in train for v in d['ids']}
    val_ids = {v for d in holdout for v in d['ids']}
    if train_paths & val_paths or train_ids & val_ids:
        raise ValueError('training and validation experiments overlap')
    if {d['sha256'] for d in train} & {d['sha256'] for d in holdout}:
        raise ValueError('training and validation data are identical')
    m = dict(schema=1, status='CALIBRATED', throttle_neutral=neutral, servo_neutral=servo_neutral,
             structure='du=a*u+b*T+c; dr=ar*r+bp*u*positive_delta+bn*u*negative_delta+c',
             units='seconds, m/s, rad/s, percent, steering degrees converted to radians',
             training=[{k: d[k] for k in ('path', 'sha256', 'ids')} for d in train],
             holdout=[{k: d[k] for k in ('path', 'sha256', 'ids')} for d in holdout],
             limitations='local input-response fit; unmodeled current, waves and sway; held future commands')
    stats = {}
    for key in ('surge', 'yaw'):
        coef, delay, stats[key] = fit_channel(train, key, neutral, servo_neutral, minimum)
        m[key] = coef
        m['throttle_delay' if key == 'surge' else 'steering_delay'] = delay
        X, Y = samples(holdout, delay, key, neutral, servo_neutral)
        if len(Y) < minimum//2:
            raise ValueError(f'{key}: insufficient independent validation')
        stats[key]['holdout_derivative_rmse'] = float(np.sqrt(np.mean((X@np.asarray(coef)-Y)**2)))
    m['fit_statistics'] = stats
    m['command_bounds'] = {}
    for key in ('throttle', 'steering'):
        values = [v for d in train for _, v in d['commands'].history[key]]
        m['command_bounds'][key] = [min(values), max(values)]
    if m['command_bounds']['throttle'][1]-m['command_bounds']['throttle'][0] < 3:
        raise ValueError('throttle excitation too small')
    lo, hi = m['command_bounds']['steering']
    if not lo < servo_neutral-3 < servo_neutral+3 < hi:
        raise ValueError('both steering directions must be observed')
    m['version'] = 'ls-'+hashlib.sha256(json.dumps(m, sort_keys=True).encode()).hexdigest()[:12]
    m['validation'] = validation_metrics(holdout, m)
    if any(m['validation']['model'][str(h)]['count'] < 10 for h in (0.5, 1.0, 2.0)):
        raise ValueError('insufficient continuous horizon validation')
    return m


def summarize(path):
    path = Path(path)
    if path.is_dir():
        path = path/'states.csv'
    with path.open(newline='', encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    valid = [r for r in rows if r['valid'].lower() == 'true']
    rates = [float(r['imu_r']) for r in rows if r.get('imu_r')]
    out = dict(rows=len(rows), valid=len(valid), invalid=len(rows)-len(valid),
               imu_r_mean=float(np.mean(rates)) if rates else None,
               imu_r_std=float(np.std(rates)) if rates else None)
    if valid:
        first, last = valid[0], valid[-1]
        out['endpoint_drift_m'] = math.hypot(float(last['x'])-float(first['x']), float(last['y'])-float(first['y']))
        out['heading_change_rad'] = wrap(float(last['heading'])-float(first['heading']))
    sensor_file = path.parent/'sensors.csv'
    if sensor_file.exists():
        stamps = {}
        with sensor_file.open(newline='', encoding='utf-8') as f:
            for r in csv.DictReader(f):
                stamps.setdefault(r['topic'], []).append((float(r['received']), float(r['header_stamp'])))
        out['sensors'] = {}
        for topic, times in stamps.items():
            periods = np.diff([v[0] for v in times])
            out['sensors'][topic] = dict(count=len(times),
                mean_receive_period_s=float(np.mean(periods)) if len(periods) else None,
                mean_header_age_s=float(np.mean([a-b for a, b in times])))
    return out


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--train', nargs='+')
    parser.add_argument('--validate', nargs='+')
    parser.add_argument('--output', default='motion_model.json')
    parser.add_argument('--summarize')
    parser.add_argument('--throttle-neutral', type=float, default=0.0)
    parser.add_argument('--servo-neutral', type=float, default=90.0)
    opts = parser.parse_args(args)
    if opts.summarize:
        print(json.dumps(summarize(opts.summarize), indent=2))
        return
    if not opts.train or not opts.validate:
        parser.error('--train and --validate require separate experiments')
    try:
        model = identify([read_experiment(p) for p in opts.train], [read_experiment(p) for p in opts.validate],
                         opts.throttle_neutral, opts.servo_neutral)
    except (ValueError, OSError) as e:
        model = dict(schema=1, status='UNCALIBRATED', reason=str(e))
    path = Path(opts.output)
    temp = path.with_suffix(path.suffix+'.tmp')
    temp.write_text(json.dumps(model, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    temp.replace(path)
    print(f"{model['status']}: {path}")
    if model['status'] == 'UNCALIBRATED':
        print(model['reason'])
        raise SystemExit(2)
