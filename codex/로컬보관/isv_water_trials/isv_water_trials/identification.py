"""Offline math. Ground kinematics and conditional hydrodynamics stay separate."""
import csv
import json
import math
from pathlib import Path

import numpy as np
from scipy.signal import periodogram, find_peaks

EARTH_RADIUS_M = 6378137.0


def number(value):
    try:
        return float(value) if value is not None else float('nan')
    except (TypeError, ValueError):
        return float('nan')


def vector(msg, field, component):
    return number(msg.get(field, {}).get(component))


def source_stamp(msg):
    stamp = msg.get('header', {}).get('stamp', {})
    return stamp.get('sec', 0) * 1000000000 + stamp.get('nanosec', 0)


def local_derivatives(t, y, window, max_gap, degree=2):
    """Local polynomial derivatives, no fitting across gaps or one-sided endpoints."""
    t, y = np.asarray(t, float), np.asarray(y, float)
    out = np.full((len(t), degree + 1), np.nan)
    if len(t) < degree + 3:
        return out
    breaks = np.r_[0, np.flatnonzero(np.diff(t) > max_gap) + 1, len(t)]
    for lo, hi in zip(breaks[:-1], breaks[1:]):
        for i in range(lo, hi):
            begin=max(lo,int(np.searchsorted(t,t[i]-window/2)))
            end=min(hi,int(np.searchsorted(t,t[i]+window/2,side='right')))
            ix = np.arange(begin,end)
            if len(ix) < degree + 3 or t[i] - t[ix[0]] < window * .3 or t[ix[-1]] - t[i] < window * .3:
                continue
            coef = np.polynomial.polynomial.polyfit(t[ix] - t[i], y[ix], degree)
            out[i] = [coef[j] * math.factorial(j) for j in range(degree + 1)]
    return out


def interpolate(t, y, targets, max_gap):
    t, y, targets = np.asarray(t), np.asarray(y), np.asarray(targets)
    out = np.full(len(targets), np.nan)
    if len(t) < 2:
        return out
    for i, x in enumerate(targets):
        hi = int(np.searchsorted(t, x, side='right'))
        if hi == len(t) and x == t[-1]:
            out[i] = y[-1]
        elif 0 < hi < len(t) and t[hi] - t[hi - 1] <= max_gap:
            out[i] = np.interp(x, t[hi-1:hi+1], y[hi-1:hi+1])
    return out


def linear_fit(x, target, names):
    x, target = np.asarray(x, float), np.asarray(target, float)
    finite = np.isfinite(x).all(axis=1) & np.isfinite(target)
    x, target = x[finite], target[finite]
    result = {'samples': len(target), 'status': 'insufficient_excitation'}
    if len(target) < max(25, 5 * len(names)):
        return result
    scale = np.linalg.norm(x, axis=0)
    if np.any(scale < 1e-10):
        return result
    xs = x / scale
    if np.linalg.matrix_rank(xs) < len(names):
        return result
    condition = float(np.linalg.cond(xs))
    result['scaled_condition_number'] = condition
    if condition > 1e4:
        return result
    coefficients = np.linalg.lstsq(xs, target, rcond=None)[0] / scale
    residual = target - x @ coefficients
    variance = float(residual @ residual / max(1, len(target) - len(names)))
    covariance = variance * np.linalg.inv(xs.T @ xs) / np.outer(scale, scale)
    result.update(status='estimated', coefficients=dict(zip(names, coefficients)),
                  approximate_95pct_halfwidth=dict(zip(names, 1.96 * np.sqrt(np.maximum(0, covariance.diagonal())))),
                  rmse=float(np.sqrt(np.mean(residual**2))),
                  uncertainty_note='OLS only; overlapping derivative windows and environmental bias are not independent. Repeat runs required.')
    return result


class ThrustCalibration:
    """Measured side/PWM/voltage table; interpolation only, never extrapolation."""
    def __init__(self, path):
        with Path(path).open(newline='') as f:
            self.rows = list(csv.DictReader(f))
        for row in self.rows:
            if row['side'] not in ('left', 'right'):
                raise ValueError('Calibration side must be left or right')
            for field in ('pwm_us', 'voltage_V', 'thrust_N'):
                row[field] = float(row[field])
                if not math.isfinite(row[field]):
                    raise ValueError('Nonfinite calibration')
        keys = [(r['side'], r['voltage_V'], r['pwm_us']) for r in self.rows]
        if len(set(keys)) != len(keys):
            raise ValueError('Duplicate calibration point')
        if not self.rows:
            raise ValueError('Calibration has no measured data')

    def force(self, side, pwm, voltage):
        if not math.isfinite(voltage):
            return float('nan')
        rows = [r for r in self.rows if r['side'] == side]
        volts = sorted(set(r['voltage_V'] for r in rows))
        if not volts or not volts[0] <= voltage <= volts[-1]:
            return float('nan')
        if len(volts) == 1:
            selected = volts
        else:
            hi = min(max(1, int(np.searchsorted(volts, voltage))), len(volts)-1)
            selected = volts[hi-1:hi+1]
        forces = []
        for v in selected:
            group = sorted((r for r in rows if r['voltage_V'] == v), key=lambda r: r['pwm_us'])
            if len(group) < 2 or not group[0]['pwm_us'] <= pwm <= group[-1]['pwm_us']:
                return float('nan')
            forces.append(np.interp(pwm, [r['pwm_us'] for r in group], [r['thrust_N'] for r in group]))
        return float(np.interp(voltage, selected, forces))


def parse_events(rows, kind):
    events = []
    for row in rows:
        if row['kind'] == kind:
            try:
                msg = json.loads(row['msg']['data'])
                events.append((row['t'], msg))
            except (ValueError, KeyError, TypeError):
                continue
    return sorted(events, key=lambda e: e[0])


def segment_at(events, t):
    # Events are already sorted; avoid quadratic history scans in long sessions.
    import bisect
    index=bisect.bisect_right(events,t,key=lambda event:event[0])-1
    return events[index][1].get('label','unmarked') if index>=0 else 'unmarked'


def prepare(rows, cfg):
    rejected = {'gps': 0, 'vn': 0, 'source_stamp_nonincreasing': 0}
    gps, vn = [], []
    last_stamp = {'gps': 0, 'vn': 0}
    for row in sorted(rows, key=lambda r: r['t']):
        kind, m, t = row['kind'], row['msg'], row['t']
        if kind not in ('gps', 'vn'):
            continue
        stamp = source_stamp(m)
        if stamp and last_stamp[kind] and stamp <= last_stamp[kind]:
            rejected['source_stamp_nonincreasing'] += 1
            continue
        if stamp:
            last_stamp[kind] = stamp
        if kind == 'gps':
            lat, lon = number(m.get('latitude')), number(m.get('longitude'))
            cov = m.get('position_covariance') or [None] * 9
            unknown = not m.get('position_covariance_type', 0)
            valid = (m.get('status', {}).get('status', -1) >= 0 and math.isfinite(lat) and math.isfinite(lon)
                     and -90 <= lat <= 90 and -180 <= lon <= 180 and abs(lat) < 89)
            if unknown:
                valid = valid and cfg['allow_unknown_gps_covariance']
            else:
                valid = valid and all(math.isfinite(number(cov[i])) and 0 <= number(cov[i]) <= cfg['max_gps_variance_m2'] for i in (0, 4))
            if not valid or (gps and t <= gps[-1][0]):
                rejected['gps'] += 1
                continue
            gps.append((t, lat, lon))
        else:
            fields = m.get('group_fields', 0)
            yaw, roll, pitch = vector(m, 'yawpitchroll', 'x'), vector(m, 'yawpitchroll', 'z'), vector(m, 'yawpitchroll', 'y')
            if not fields & 0x8 or not np.isfinite([yaw, roll, pitch]).all() or (vn and t <= vn[-1][0]):
                rejected['vn'] += 1
                continue
            rate = vector(m, 'angularrate', 'z') * cfg['vectornav_gyro_z_sign'] if fields & 0x20 else float('nan')
            vn.append((t, math.pi/2 - math.radians(yaw + cfg['vectornav_yaw_offset_deg']), math.radians(roll), math.radians(pitch), rate))
    if len(gps) < 5:
        return [], rejected, vn
    gps = np.array(gps)
    origin = gps[0, 1:]
    east = EARTH_RADIUS_M * np.cos(np.deg2rad(origin[0])) * np.deg2rad(gps[:, 2] - origin[1])
    north = EARTH_RADIUS_M * np.deg2rad(gps[:, 1] - origin[0])
    # Remove both endpoints of impossible jumps, preventing a bad outlier poisoning derivatives.
    jump = np.flatnonzero(np.hypot(np.diff(east), np.diff(north)) / np.diff(gps[:, 0]) > cfg['max_ground_speed_m_s'])
    keep = np.ones(len(gps), bool)
    keep[np.unique(np.r_[jump, jump+1])] = False
    rejected['gps'] += int((~keep).sum())
    gps, east, north = gps[keep], east[keep], north[keep]
    t = gps[:, 0]
    e = local_derivatives(t, east, cfg['derivative_window_s'], cfg['max_sensor_gap_s'])
    n = local_derivatives(t, north, cfg['derivative_window_s'], cfg['max_sensor_gap_s'])
    heading, r, rdot = [np.full(len(t), np.nan) for _ in range(3)]
    gyro = np.full(len(t), np.nan)
    if len(vn) >= 5:
        vna = np.array(vn)
        yaw = np.unwrap(vna[:, 1])
        d = local_derivatives(vna[:, 0], yaw, cfg['derivative_window_s'], cfg['max_sensor_gap_s'])
        heading = interpolate(vna[:, 0], yaw, t, cfg['max_sensor_gap_s'])
        r = interpolate(vna[:, 0], d[:, 1], t, cfg['max_sensor_gap_s'])
        rdot = interpolate(vna[:, 0], d[:, 2], t, cfg['max_sensor_gap_s'])
        gyro = interpolate(vna[:, 0], vna[:, 4], t, cfg['max_sensor_gap_s'])
    commands = parse_events(rows, 'command')
    command_times=[event[0] for event in commands]
    marks = parse_events(rows, 'mark')
    battery = sorted((row['t'], number(row['msg'].get('data'))) for row in rows if row['kind'] == 'battery')
    battery_times=[event[0] for event in battery]
    samples = []
    for i, ti in enumerate(t):
        # Derivative windows must not straddle trial labels or PWM changes.
        label = segment_at(marks, ti)
        same_label = all(segment_at(marks, ti + d) == label for d in (-cfg['derivative_window_s']/2, cfg['derivative_window_s']/2))
        c, s = math.cos(heading[i]), math.sin(heading[i])
        ve = e[i, 1] - cfg['current_east_m_s']
        vnorth = n[i, 1] - cfg['current_north_m_s']
        u, v = c*ve + s*vnorth, -s*ve + c*vnorth
        udot = c*e[i, 2] + s*n[i, 2] + r[i]*v
        if not cfg['frame_confirmed']:
            u = v = udot = float('nan')
        ci=int(np.searchsorted(command_times,ti,side='right'))-1
        command = commands[ci] if ci>=0 else None
        left = right = float('nan')
        if command and ti-command[0] <= cfg['command_max_age_s'] and command[1].get('status') == 'acknowledged':
            left, right = number(command[1].get('left_pwm_us')), number(command[1].get('right_pwm_us'))
        bi=int(np.searchsorted(battery_times,ti,side='right'))-1
        volts=battery[bi][1] if bi>=0 and ti-battery[bi][0]<=2 else float('nan')
        samples.append(dict(t=ti, east_m=e[i, 0], north_m=n[i, 0], ground_speed_m_s=float(np.hypot(e[i, 1], n[i, 1])),
                            east_velocity_m_s=e[i, 1], north_velocity_m_s=n[i, 1], u_m_s=u, v_m_s=v, udot_m_s2=udot,
                            yaw_enu_rad=heading[i], r_rad_s=r[i], rdot_rad_s2=rdot[i], gyro_r_rad_s=gyro[i],
                            left_pwm_us=left, right_pwm_us=right, label=label, interior_segment=same_label,
                            voltage_V=volts))
    return samples, rejected, vn


def describe(values):
    a = np.array(values, float)
    a = a[np.isfinite(a)]
    return {'n': len(a), 'mean': float(np.mean(a)), 'std': float(np.std(a)), 'p95': float(np.percentile(a, 95))} if len(a) else {'n': 0}


def analyze(rows, cfg):
    samples, rejected, vn = prepare(rows, cfg)
    report = {'schema_version': 1, 'boat': {k: cfg[k] for k in ('mass_kg','length_m','beam_m','thruster_spacing_m')},
              'quality': {'rejected': rejected, 'frame_confirmed': cfg['frame_confirmed']},
              'kinematics': {}, 'segments': {}, 'models': {}, 'warnings': [],
              'not_identifiable': ['Rigid-body yaw inertia from dimensions alone',
                                   'Absolute thrust from PWM alone', 'Current or water speed from GPS alone',
                                   'GM and restoring stiffness without independent hydrostatic measurements',
                                   'Full coupled 3-DOF added mass and damping from these reduced tests']}
    if not cfg['frame_confirmed']:
        report['warnings'].append('Mounting/frame unconfirmed: body velocities and hydrodynamic fits disabled. Yaw values are provisional sensor-axis observations.')
    if not samples:
        report['warnings'].append('Insufficient valid GPS. No kinematic/model result generated.')
        return report, samples
    report['kinematics'] = {'ground_speed_m_s': describe([s['ground_speed_m_s'] for s in samples]),
                            'abs_yaw_rate_rad_s': describe([abs(s['r_rad_s']) for s in samples])}
    report['quality']['gyro_minus_heading_rate_rad_s'] = describe([s['gyro_r_rad_s']-s['r_rad_s'] for s in samples])
    for label in sorted(set(s['label'] for s in samples)):
        group = [s for s in samples if s['label'] == label and s['interior_segment']]
        report['segments'][label] = {'speed_m_s': describe([s['ground_speed_m_s'] for s in group]),
                                     'yaw_rate_rad_s': describe([s['r_rad_s'] for s in group]),
                                     'turn_radius_proxy_m': describe([s['ground_speed_m_s']/abs(s['r_rad_s']) for s in group
                                                                     if label.startswith('turn') and abs(s['r_rad_s']) > .03]),
                                     'note': 'Radius is v_ground/|r|, valid only in approximately steady turns; not hull turning-circle diameter.'}
    dynamic = [s for s in samples if s['interior_segment'] and s['label'] not in ('unmarked','end')]
    if cfg['frame_confirmed']:
        surge=[s for s in dynamic if s['label'] in ('straight','coast') and abs(s['v_m_s'])<.15 and abs(s['r_rad_s'])<.05]
        if surge:
            fit=linear_fit([[s['u_m_s'],abs(s['u_m_s'])*s['u_m_s'],
                             s['left_pwm_us']+s['right_pwm_us']-2*cfg['neutral_pwm_us'],1] for s in surge],
                           [s['udot_m_s2'] for s in surge],['u_decay_per_s','quadratic_decay_per_m','pwm_sum_gain_m_s2_per_us','bias_m_s2'])
            if fit['status']=='estimated':
                c=fit['coefficients']
                if c['u_decay_per_s']>=0 or c['quadratic_decay_per_m']>0 or c['pwm_sum_gain_m_s2_per_us']<=0:
                    fit['status']='rejected_nonphysical_command_response'
            fit['model']='Empirical udot=a*u+b*|u|u+c*(PWM_left+PWM_right-2*neutral)+d; not mass or thrust. Current/wind changes invalidate transfer.'
            report['models']['surge_command_response']=fit
        yaw = [s for s in dynamic if s['label'] in ('turn_left','turn_right','zigzag')]
        if yaw:
            fit = linear_fit([[s['r_rad_s'], s['left_pwm_us']-s['right_pwm_us'], 1] for s in yaw],
                             [s['rdot_rad_s2'] for s in yaw], ['r_decay_per_s','pwm_difference_gain_rad_s2_per_us','bias_rad_s2'])
            if fit['status'] == 'estimated':
                decay = fit['coefficients']['r_decay_per_s']
                if decay < 0:
                    fit['time_constant_s'] = -1/decay
                    fit['steady_gain_rad_s_per_us'] = -fit['coefficients']['pwm_difference_gain_rad_s2_per_us']/decay
                else:
                    fit['status'] = 'rejected_nonpositive_time_constant'
            fit['model'] = 'Empirical rdot=a*r+b*(PWM_left-PWM_right)+c. PWM is not rudder angle or thrust.'
            report['models']['empirical_yaw_response'] = fit
    if not (cfg['still_water_confirmed'] and cfg['frame_confirmed']):
        report['warnings'].append('Hydrodynamic fits disabled until frame and still-water/current-correction assumptions are explicitly confirmed.')
    else:
        coast = [s for s in dynamic if s['label']=='coast' and s['left_pwm_us']==cfg['neutral_pwm_us']
                 and s['right_pwm_us']==cfg['neutral_pwm_us'] and abs(s['v_m_s']) < .15 and abs(s['r_rad_s']) < .05 and s['u_m_s'] > .15]
        if coast:
            fit = linear_fit([[s['u_m_s'], abs(s['u_m_s'])*s['u_m_s']] for s in coast],
                             [-s['udot_m_s2'] for s in coast], ['d1_over_M_per_s','d2_over_M_per_m'])
            if fit['status']=='estimated' and min(fit['coefficients'].values()) < 0:
                fit['status']='rejected_negative_damping'
            fit['model']='Straight coast: -udot=(d1/M_eff)*u+(d2/M_eff)*|u|u; does not separate M_eff and damping.'
            report['models']['normalized_surge_coast'] = fit
        if cfg.get('thrust_calibration_csv'):
            cal = ThrustCalibration(cfg['thrust_calibration_csv'])
            enriched=[]
            for s in dynamic:
                fl=cal.force('left', s['left_pwm_us'], s['voltage_V'])
                fr=cal.force('right', s['right_pwm_us'], s['voltage_V'])
                enriched.append((s, fl+fr, cfg['thruster_spacing_m']/2*(fr-fl)))
            surge=[v for v in enriched if v[0]['label'] in ('straight','coast') and abs(v[0]['v_m_s'])<.15 and abs(v[0]['r_rad_s'])<.05]
            yaw=[v for v in enriched if v[0]['label'] in ('turn_left','turn_right','zigzag')]
            for name, data, vel, acc, coefficient_names in [
                ('surge_force_balance', surge, 'u_m_s','udot_m_s2',['M_eff_kg','d1_N_s_m','d2_N_s2_m2','bias_N']),
                ('yaw_moment_balance', yaw, 'r_rad_s','rdot_rad_s2',['Iz_eff_kg_m2','d1_Nm_s','d2_Nm_s2','bias_Nm'])]:
                if not data:
                    continue
                fit=linear_fit([[s[acc],s[vel],abs(s[vel])*s[vel],1] for s,_,_ in data],
                               [f if name.startswith('surge') else moment for _,f,moment in data], coefficient_names)
                if fit['status']=='estimated' and any(fit['coefficients'][k] <= 0 for k in coefficient_names[:3]):
                    fit['status']='rejected_nonpositive_inertia_or_damping'
                fit['model']='Reduced single-axis model; effective inertia includes added inertia. Static thrust calibration may differ underway. Coupling ignored; compare repeat runs.'
                report['models'][name]=fit
        else:
            report['warnings'].append('No measured thrust table: absolute inertia and force/moment damping are not estimated.')
    # Roll/pitch oscillation spectrum: observable period only, no invented GM.
    if len(vn)>40:
        a=np.array(vn)
        if np.max(np.diff(a[:,0])) <= cfg['max_sensor_gap_s']:
            dt=float(np.median(np.diff(a[:,0])))
            grid=np.arange(a[0,0],a[-1,0],dt)
            for col,name in [(2,'roll'),(3,'pitch')]:
                vals=np.interp(grid,a[:,0],a[:,col])
                freq,power=periodogram(vals,fs=1/dt,detrend='linear')
                band=(freq>=.1)&(freq<=min(3.,.4/dt))
                if np.any(band) and np.std(vals) > math.radians(.1):
                    peak=int(np.argmax(power[band])); f=float(freq[band][peak])
                    report['kinematics'][name+'_dominant_period_s']=1/f
                    report['warnings'].append(name+' spectral period mixes waves/forcing; not an identified natural period or GM.')
    return report, samples
