"""Measured input-response model; no assumed mass, inertia or thrust constants."""
import json
import math
from collections import deque
from pathlib import Path
from .core import wrap


def load_model(path):
    if not path:
        return None
    with open(Path(path).expanduser(), encoding='utf-8') as f:
        m = json.load(f)
    if m.get('status') != 'CALIBRATED' or m.get('schema') != 1:
        return None
    for key, count in [('surge', 3), ('yaw', 4)]:
        if len(m[key]) != count or not all(math.isfinite(v) for v in m[key]):
            raise ValueError('invalid model coefficients')
        if m[key][0] >= 0:
            raise ValueError('model must have dissipative response')
    if not all(0 <= m[k] <= 2 for k in ('throttle_delay', 'steering_delay')):
        raise ValueError('invalid response delay')
    if not m.get('version') or not m.get('validation'):
        raise ValueError('model missing provenance or validation')
    return m


class Commands:
    def __init__(self, maxlen=5000):
        self.history = {'throttle': deque(maxlen=maxlen), 'steering': deque(maxlen=maxlen)}

    def add(self, kind, t, value):
        if not math.isfinite(value):
            return
        if self.history[kind] and t < self.history[kind][-1][0]:
            self.history[kind].clear()
        self.history[kind].append((t, value))

    def at(self, kind, t):
        return next(((time, v) for time, v in reversed(self.history[kind]) if time <= t), None)


def rates(model, u, r, throttle, steering):
    a, b, c = model['surge']
    ar, bp, bn, cr = model['yaw']
    delta = math.radians(steering-model['servo_neutral'])
    return (a*u+b*(throttle-model['throttle_neutral'])+c,
            ar*r+bp*u*max(delta, 0)+bn*u*min(delta, 0)+cr)


def predict(state, commands, model=None, horizons=(0.5, 1.0, 2.0), step=0.02, command_max_age=2.0):
    if not state['valid']:
        return None
    start = state['stamp']
    calibrated = model is not None
    latest = {k: commands.at(k, start) for k in ('throttle', 'steering')}
    if calibrated:
        if any(v is None or start-v[0] > command_max_age for v in latest.values()):
            calibrated = False
        else:
            for key, value in latest.items():
                low, high = model['command_bounds'][key]
                if not low <= value[1] <= high:
                    calibrated = False
    label = model['version'] if calibrated else 'BASELINE_CONSTANT_BODY_VELOCITY_UNCALIBRATED'
    x, y, h, u, v, r = (state[k] for k in ('x', 'y', 'heading', 'u', 'v', 'r'))
    points, path, elapsed = [], [dict(x=x, y=y, heading=h, t=0.0)], 0.0
    for horizon in sorted(horizons):
        while elapsed < horizon-1e-9:
            dt = min(step, horizon-elapsed)
            du = dr = 0.0
            if calibrated:
                inputs = []
                for kind, delay in [('throttle', model['throttle_delay']), ('steering', model['steering_delay'])]:
                    at = start+elapsed-delay
                    item = commands.at(kind, min(start, at))
                    if item is None:
                        return predict(state, commands, None, horizons, step, command_max_age)
                    inputs.append(item[1])
                du, dr = rates(model, u, r, *inputs)
            mid_u, mid_r = u+du*dt/2, r+dr*dt/2
            mid_h = h+mid_r*dt/2
            x += (mid_u*math.cos(mid_h)-v*math.sin(mid_h))*dt
            y += (mid_u*math.sin(mid_h)+v*math.cos(mid_h))*dt
            h = wrap(h+mid_r*dt)
            u += du*dt
            r += dr*dt
            elapsed += dt
            if not all(math.isfinite(z) for z in (x, y, h, u, r)) or abs(u) > 10 or abs(r) > 5:
                return None
            path.append(dict(x=x, y=y, heading=h, t=elapsed))
        points.append(dict(prediction_stamp=start, target_stamp=start+horizon, horizon=horizon,
                           frame=state['frame'], epoch=state['epoch'], model_version=label,
                           x=x, y=y, heading=h, r=r, u=u))
    return dict(schema=1, stamp=start, frame=state['frame'], initial_state=state,
                model_status='CALIBRATED' if calibrated else 'UNCALIBRATED',
                model_version=label, input_assumption='latest command held; sway held constant',
                commands={k: v[1] if v else None for k, v in latest.items()}, points=points, path=path)
