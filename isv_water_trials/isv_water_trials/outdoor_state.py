"""GNSS ENU velocity + timestamp-aligned VectorNav NED heading/rate.

No acceleration double integration; GNSS position anchors every fix.
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
            self.reset()
        self.imus.append((stamp, received, yaw, -rate_ned_z))

    def fix(self, stamp, received, lat, lon, status, covariance, covariance_type):
        if not (-85 < lat < 85 and -180 <= lon <= 180) or status < 0:
            self.reason = 'GPS 위치 고정 대기'; self.fixes.clear(); return
        sigma = math.sqrt(max(covariance[0], covariance[4], 0))
        if not covariance_type or not math.isfinite(sigma) or sigma > self.config['gps_position_sigma_limit_m']:
            self.reason = 'GPS 위치 정확도 확인 대기'; self.fixes.clear(); return
        if self.origin is None: self.origin = [lat, lon]
        position = local_xy(lat, lon, self.origin)
        if self.fixes and stamp <= self.fixes[-1][0]: self.reset()
        self.fixes.append((stamp, received, position, sigma, lat, lon))

    def velocity(self, stamp, received, east, north, covariance):
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
        fix = min(self.fixes, key=lambda x: abs(x[0]-stamp))
        if abs(imu[0]-stamp) > .15 or abs(fix[0]-stamp) > self.config['gps_max_age_s']:
            self.reason = 'GPS·IMU 시각 정렬 대기'; return None
        yaw, rate = imu[2:]; c, s = math.cos(yaw), math.sin(yaw)
        arm_x, arm_y = self.config['gps_lever_arm_m']
        u = c*east+s*north+rate*arm_y
        v = -s*east+c*north-rate*arm_x
        position = fix[2]+np.array([east, north])*(stamp-fix[0])-np.array([c*arm_x-s*arm_y, s*arm_x+c*arm_y])
        self.latest = {'t': stamp, 'received': received, 'position': position.tolist(),
            'yaw': yaw, 'velocity': [u, v, rate], 'lat': fix[4], 'lon': fix[5],
            'position_sigma_m': fix[3], 'velocity_sigma_m_s': sigma,
            'imu_skew_s': abs(imu[0]-stamp)}
        self.poses.append(dict(self.latest))
        self.reason = '정상'; return dict(self.latest)

    def pose_at(self, stamp):
        """Pose at the scan timestamp, rather than the latest display pose."""
        if not self.imus or not self.poses: return None
        imu = min(self.imus, key=lambda p: abs(p[0]-stamp))
        pose = min(self.poses, key=lambda p: abs(p['t']-stamp))
        dt = stamp-pose['t']
        if abs(imu[0]-stamp) > .15 or abs(dt) > self.config['gps_max_age_s']: return None
        yaw = imu[2]; u,v,_ = pose['velocity']
        xy = np.array(pose['position'])+dt*np.array([math.cos(yaw)*u-math.sin(yaw)*v, math.sin(yaw)*u+math.cos(yaw)*v])
        return {'position': xy.tolist(), 'yaw': yaw}

    def snapshot(self, now):
        if self.latest is None: return None
        age = now-self.latest['received']
        if age < 0 or age > self.config['gps_max_age_s'] or not self.imus or now-self.imus[-1][1] > self.config['imu_max_age_s']:
            self.reason = 'GPS·IMU 새 데이터 대기'; return None
        value = dict(self.latest); u, v, r = value['velocity']; yaw = self.imus[-1][2]
        value['position'] = (np.array(value['position'])+age*np.array([math.cos(yaw)*u-math.sin(yaw)*v, math.sin(yaw)*u+math.cos(yaw)*v])).tolist()
        value['yaw'] = yaw; value['velocity'] = [u, v, self.imus[-1][3]]
        value['age_s'] = age; return value

    def display_pose(self, now):
        """Location display does not depend on a velocity solution or learning."""
        if not self.fixes or now-self.fixes[-1][1] > self.config['gps_max_age_s']: return None
        fix = self.fixes[-1]
        yaw = self.imus[-1][2] if self.imus and now-self.imus[-1][1] < self.config['imu_max_age_s'] else None
        return {'t': fix[0], 'position': fix[2].tolist(), 'lat': fix[4], 'lon': fix[5],
                'yaw': yaw, 'velocity': None, 'position_sigma_m': fix[3], 'age_s': now-fix[1]}
