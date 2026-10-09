"""ROS-independent validity gates. Never extrapolate a missing position."""
import math
from collections import deque


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw(q):
    values = (q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('nonfinite quaternion')
    norm = sum(v*v for v in values)
    if abs(norm - 1.0) > 0.05:
        raise ValueError('invalid quaternion norm')
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))


class StateGate:
    def __init__(self, config):
        self.c = config
        self.scan = None
        self.imus = deque(maxlen=300)
        self.odom = None
        self.previous = None
        self.epoch = 0
        self.last_stamp = None
        self.imu_origin = None
        self.last_result = None

    def set_scan(self, sample):
        self.scan = sample

    def set_imu(self, sample):
        if self.imus and sample['stamp'] <= self.imus[-1]['stamp']:
            self.imus.clear()
            self.imu_origin = None
            self.epoch += 1
            sample = dict(sample, clock_error=True)
        self.imus.append(sample)
        if self.imu_origin is None and sample.get('heading') is not None:
            self.imu_origin = sample['heading']

    def set_odom(self, sample):
        self.previous, self.odom = self.odom, sample

    def evaluate(self, now):
        c, od, sc = self.c, self.odom, self.scan
        reasons = []
        latest_imu = self.imus[-1] if self.imus else None
        im = min(self.imus, key=lambda s: abs(s['stamp']-od['stamp'])) if od and self.imus else None
        ages = {}
        for name, s in [('lidar', sc), ('imu', latest_imu), ('rf2o', od)]:
            ages[name] = now-s['stamp'] if s and math.isfinite(s['stamp']) else None
            if not s:
                reasons.append(name+'_missing')
            elif (s['stamp'] <= 0 or ages[name] is None or not -c['future_tolerance'] <= ages[name] <= c['max_age']
                  or not -c['future_tolerance'] <= now-s['received'] <= c['max_age']):
                reasons.append(name+'_timestamp_or_age')
        if sc and (sc['frame'] != c['scan_frame'] or sc['fraction'] < c['min_scan_fraction']
                   or sc['sectors'] < c['min_scan_sectors']):
            reasons.append('scan_frame_or_features')
        if im and od:
            if abs(im['stamp']-od['stamp']) > c['max_sensor_skew']:
                reasons.append('imu_odom_skew')
            if im['frame'] != c['imu_frame'] or not im.get('available', True):
                reasons.append('imu_frame_or_unavailable')
            if im.get('clock_error') or not math.isfinite(im['r']):
                reasons.append('imu_invalid')
        quality = 'UNKNOWN'
        u = v = odom_r = acceleration = None
        if od:
            if od['frame'] != c['odom_frame'] or od['child'] != c['base_frame']:
                reasons.append('odom_frame_mismatch')
            if not all(math.isfinite(od[k]) for k in ('x', 'y', 'heading')):
                reasons.append('odom_nonfinite')
            cov = od['covariance']
            if not all(math.isfinite(vv) and vv >= 0 for vv in cov):
                reasons.append('covariance_invalid')
            elif max(cov[:2]) > c['max_position_variance'] or cov[2] > c['max_heading_variance']:
                reasons.append('covariance_large')
            elif all(vv == 0 for vv in cov):
                quality = 'LIMITED_UNKNOWN_COVARIANCE'
                if not c['allow_unknown_covariance']:
                    reasons.append('covariance_unknown')
            else:
                quality = 'COVARIANCE_GATED'
            prev = self.previous
            dt = od['stamp']-prev['stamp'] if prev else 0
            if not c['min_dt'] <= dt <= c['max_dt']:
                reasons.append('odom_interval')
            else:
                dx, dy = (od['x']-prev['x'])/dt, (od['y']-prev['y'])/dt
                mid = prev['heading'] + wrap(od['heading']-prev['heading'])/2
                u, v = dx*math.cos(mid)+dy*math.sin(mid), -dx*math.sin(mid)+dy*math.cos(mid)
                odom_r = wrap(od['heading']-prev['heading'])/dt
                if math.hypot(u, v) > c['max_speed'] or abs(odom_r) > c['max_yaw_rate']:
                    reasons.append('odom_jump')
                if im and abs(odom_r-im['r']) > c['max_yaw_disagreement']:
                    reasons.append('yaw_disagreement')
                if im and self.last_result and self.last_result['valid'] and self.last_result['stamp'] < od['stamp']:
                    acceleration = (im['r']-self.last_result['r'])/dt
        # A gap invalidates all predictions across it, including later recovery.
        new = od and od['stamp'] != self.last_stamp
        if new and reasons:
            self.epoch += 1
        if new:
            self.last_stamp = od['stamp']
        valid = not reasons
        result = dict(schema=1, stamp=od['stamp'] if od else now, received=now,
                      valid=valid, status='VALID' if valid else 'INVALID', reasons=reasons,
                      epoch=self.epoch, frame=c['odom_frame'], base_frame=c['base_frame'],
                      x=od['x'] if valid else None, y=od['y'] if valid else None,
                      heading=od['heading'] if valid else None, u=u if valid else None,
                      v=v if valid else None, r=im['r'] if valid else None,
                      yaw_acceleration=acceleration if valid else None,
                      imu_r=latest_imu['r'] if latest_imu and math.isfinite(latest_imu['r']) else None,
                      imu_stamp=latest_imu['stamp'] if latest_imu else None,
                      imu_heading_relative=wrap(latest_imu['heading']-self.imu_origin)
                      if latest_imu and latest_imu.get('heading') is not None and self.imu_origin is not None else None,
                      odom_r=odom_r if valid else None, ages=ages, quality=quality,
                      covariance=od['covariance'] if od else None)
        if new:
            self.last_result = result
        return result


class ErrorMatcher:
    """Score at the prediction target time, only within continuous valid segments."""
    def __init__(self, max_gap=0.3):
        self.pending = deque(maxlen=2000)
        self.previous = None
        self.max_gap = max_gap

    def add(self, prediction):
        self.pending.extend(prediction['points'])

    def observe(self, state):
        if not state['valid']:
            self.pending.clear()
            self.previous = None
            return []
        old = self.previous
        self.previous = state
        if old is None:
            return []
        dt = state['stamp']-old['stamp']
        if dt <= 0:
            return []
        errors, keep = [], deque(maxlen=2000)
        for p in self.pending:
            if p['target_stamp'] > state['stamp']:
                keep.append(p)
                continue
            if (old['stamp'] <= p['target_stamp'] and dt <= self.max_gap
                    and old['epoch'] == state['epoch'] == p['epoch']
                    and old['frame'] == state['frame'] == p['frame']):
                f = (p['target_stamp']-old['stamp'])/dt
                actual = {k: old[k]+f*(state[k]-old[k]) for k in ('x', 'y', 'r', 'u')}
                actual['heading'] = wrap(old['heading']+f*wrap(state['heading']-old['heading']))
                errors.append(dict(p, actual=actual,
                                   position_error=math.hypot(p['x']-actual['x'], p['y']-actual['y']),
                                   heading_error=wrap(p['heading']-actual['heading']),
                                   yaw_rate_error=p['r']-actual['r'], surge_error=p['u']-actual['u']))
        self.pending = keep
        return errors
