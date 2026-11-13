"""GNSS ENU velocity + timestamp-aligned VectorNav NED heading/rate.

Position is held at the last valid GNSS fix; IMU heading updates independently.
Heading is VectorNav's absolute heading, with explicit mount/declination offsets.
"""
from collections import deque
import math
import numpy as np
from .outdoor_model import local_xy, wrap


class OutdoorState:
    def __init__(self, config):
        self.config = config; self.origin = None
        self.imus = deque(maxlen=1000); self.fixes = deque(maxlen=100)
        self.poses = deque(maxlen=300)
        self.latest = None; self.reason = 'GPS 속도·IMU 수신 대기'
        self.last_velocity_t = -math.inf

    def reset(self):
        self.imus.clear(); self.fixes.clear(); self.latest = None
        self.poses.clear()
        self.last_velocity_t = -math.inf; self.reason = '새 센서 구간 수신 대기'

    def imu(self, stamp, received, yaw_ned_deg, rate_ned_z):
        if not all(math.isfinite(x) for x in (stamp, received, yaw_ned_deg, rate_ned_z)): return
        heading = math.radians(yaw_ned_deg+self.config['heading_offset_deg']+self.config['magnetic_declination_deg'])
        yaw = float(wrap(math.pi/2-heading))
        if self.imus and stamp <= self.imus[-1][0]:
            return
        self.imus.append((stamp, received, yaw, -rate_ned_z))

    def heading(self, now):
        """The same configured north reference used by navigation and projection."""
        if not self.imus or not 0 <= now-self.imus[-1][1] <= self.config['imu_max_age_s']:
            return None
        stamp,_,yaw,rate = self.imus[-1]
        return {'heading_deg': (90-math.degrees(yaw)) % 360,'yaw_rate_rad_s':rate,
                'stamp_s': stamp, 'reference': 'magnetic_north' if self.config.get('imu_heading_mode')=='absolute_magnetic_raw' else 'true_north',
                'declination_deg': self.config['magnetic_declination_deg'],
                'mount_offset_deg': self.config['heading_offset_deg'],
                'imu_heading_mode': self.config.get('imu_heading_mode','unknown')}

    def fix(self, stamp, received, lat, lon, status, covariance, covariance_type):
        if not all(math.isfinite(x) for x in (stamp, received, lat, lon)): return
        if not (-85 < lat < 85 and -180 <= lon <= 180) or status < 0:
            self.reason = 'GPS 위치 고정 대기'; return
        sigma = math.sqrt(max(covariance[0], covariance[4], 0))
        if not covariance_type or not math.isfinite(sigma) or sigma > self.config['gps_position_sigma_limit_m']:
            self.reason = 'GPS 위치 정확도 확인 대기'; return
        if self.fixes and stamp <= self.fixes[-1][0]: return
        if self.origin is None: self.origin = [lat, lon]
        position = local_xy(lat, lon, self.origin)
        arm_x, arm_y = self.config['gps_lever_arm_m']
        if arm_x or arm_y:
            yaw = self.yaw_at(stamp)
            if yaw is None:
                self.reason = 'GPS 안테나 위치·IMU 방향 수신 대기'; return
            c, s = math.cos(yaw), math.sin(yaw)
            position -= np.array([c*arm_x-s*arm_y, s*arm_x+c*arm_y])
        self.fixes.append((stamp, received, position, sigma, lat, lon))

    def velocity(self, stamp, received, east, north, covariance, sample_age_s=0.):
        if stamp <= self.last_velocity_t: return None
        self.last_velocity_t = stamp
        if not self.imus or not self.fixes:
            self.reason = 'GPS 위치·IMU 수신 대기'; return None
        sigma = math.sqrt(max(covariance[0], covariance[7], 0))
        if min(covariance[0], covariance[7]) <= 0 or not math.isfinite(sigma) or sigma > self.config['gps_velocity_sigma_limit_m_s']:
            self.reason = 'GPS 속도 정확도 확인 대기'; return None
        if not all(math.isfinite(v) for v in (stamp, east, north)) or math.hypot(east, north) > 5:
            self.reason = 'GPS 속도 이상값'; return None
        imu = min(self.imus, key=lambda x: abs(x[0]-stamp))
        fix = next((p for p in reversed(self.fixes) if p[0] <= stamp), None)
        if fix is None:
            self.reason = 'GPS 위치·속도 시각 정렬 대기'; return None
        if abs(imu[0]-stamp) > .15 or abs(fix[0]-stamp) > self.config['gps_max_age_s']:
            self.reason = 'GPS·IMU 시각 정렬 대기'; return None
        yaw, rate = imu[2:]; c, s = math.cos(yaw), math.sin(yaw)
        arm_x, arm_y = self.config['gps_lever_arm_m']
        u = c*east+s*north+rate*arm_y
        v = -s*east+c*north-rate*arm_x
        position = fix[2]
        self.latest = {'t': stamp, 'received': received, 'position': position.tolist(),
            'yaw': yaw, 'velocity': [u, v, rate], 'lat': fix[4], 'lon': fix[5],
            'position_sigma_m': fix[3], 'velocity_sigma_m_s': sigma,
            'imu_skew_s': abs(imu[0]-stamp), 'velocity_enu_m_s': [east, north],
            'sample_age_s': max(0., float(sample_age_s))}
        self.poses.append(dict(self.latest))
        self.reason = '정상'; return dict(self.latest)

    def pose_at(self, stamp):
        """Hold the GPS and IMU samples already measured at scan acquisition."""
        fix = next((p for p in reversed(self.fixes) if p[0] <= stamp), None)
        imu = next((p for p in reversed(self.imus) if p[0] <= stamp), None)
        if fix is None or imu is None: return None
        if stamp-imu[0] > self.config['imu_max_age_s'] or stamp-fix[0] > self.config['gps_max_age_s']: return None
        return {'position': fix[2].tolist(), 'yaw': imu[2],
                'position_source': 'gps', 'heading_source': 'imu',
                'position_stamp_s': fix[0], 'heading_stamp_s': imu[0]}

    def yaw_at(self, stamp, nearest=None):
        before = next((p for p in reversed(self.imus) if p[0] <= stamp), None)
        return before[2] if before else None

    def project_position(self, pose, dt, yaw):
        """Keep compatibility for callers without extrapolating measured pose."""
        return np.array(pose['position'], dtype=float)

    def snapshot(self, now):
        if self.latest is None or not self.fixes: return None
        receipt_age = now-self.latest['received']
        age = receipt_age+self.latest.get('sample_age_s', 0.)
        held = self.display_pose(now)
        if receipt_age < 0 or age > self.config['gps_max_age_s'] or held is None or not held['pose_fresh']:
            self.reason = 'GPS·IMU 새 데이터 대기'; return None
        value = dict(self.latest); yaw, rate = self.imus[-1][2:]
        value['position'] = held['position']
        value['lat'], value['lon'] = held['lat'], held['lon']
        value['position_sigma_m'] = held['position_sigma_m']
        east, north = value['velocity_enu_m_s']; c, s = math.cos(yaw), math.sin(yaw)
        arm_x, arm_y = self.config['gps_lever_arm_m']
        value['yaw'] = yaw
        value['velocity'] = [c*east+s*north+rate*arm_y, -s*east+c*north-rate*arm_x, rate]
        value['age_s'] = age; value['receipt_age_s'] = receipt_age
        value.update(position_compensation_s=0., position_source='gps', heading_source='imu',
                     position_stamp_s=held['position_stamp_s'], heading_stamp_s=held['heading_stamp_s'],
                     pose_fresh=True)
        return value

    def display_pose(self, now):
        """Location display does not depend on a velocity solution or learning."""
        if not self.fixes: return None
        fix = self.fixes[-1]
        yaw = self.imus[-1][2] if self.imus else None
        fresh = (0 <= now-fix[1] <= self.config['gps_max_age_s'] and bool(self.imus) and
                 0 <= now-self.imus[-1][1] <= self.config['imu_max_age_s'])
        return {'t': fix[0], 'position': fix[2].tolist(), 'lat': fix[4], 'lon': fix[5],
                'yaw': yaw, 'velocity': None, 'position_sigma_m': fix[3], 'age_s': now-fix[1],
                'position_compensation_s': 0., 'position_source': 'gps', 'heading_source': 'imu',
                'position_stamp_s': fix[0], 'heading_stamp_s': self.imus[-1][0] if self.imus else None,
                'pose_fresh': bool(fresh)}
