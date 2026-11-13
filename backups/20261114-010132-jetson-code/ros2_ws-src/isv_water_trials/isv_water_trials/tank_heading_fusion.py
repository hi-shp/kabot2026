"""Timestamp-aligned gyro/LiDAR tracking after a saved-map pose is acquired."""
from collections import deque
import math


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def ned_yaw_rate(roll_deg, pitch_deg, wx, wy, wz):
    """VectorNav body NED rates -> map CCW yaw rate, without magnetic yaw."""
    if not all(math.isfinite(v) for v in (roll_deg, pitch_deg, wx, wy, wz)):
        return None
    roll, pitch = math.radians(roll_deg), math.radians(pitch_deg)
    if abs(math.cos(pitch)) < .25:
        return None
    return -(math.sin(roll)*wy + math.cos(roll)*wz)/math.cos(pitch)


class ImuLidarFusion:
    def __init__(self):
        self.gyro = deque(maxlen=4096)
        self.active = False
        self.source = 'lidar_odometry'
        self.correction = 0.
        self.imu_used = False

    def add_gyro(self, stamp, rate):
        if not all(math.isfinite(v) for v in (stamp, rate)):
            return False
        if self.gyro and stamp <= self.gyro[-1][0]:
            if stamp < self.gyro[-1][0]-1.:
                self.gyro.clear()
            else:
                return False
        self.gyro.append((stamp, rate))
        while len(self.gyro) > 2 and stamp-self.gyro[0][0] > 6.:
            self.gyro.popleft()
        return True

    def integral(self, start, end):
        if end <= start or end-start > 1. or not self.gyro:
            return None
        samples = list(self.gyro)
        before = next((i for i in range(len(samples)-1, -1, -1) if samples[i][0] <= start), None)
        if before is None or start-samples[before][0] > .15 or end-samples[-1][0] > .15:
            return None
        total = 0.
        left = start
        for i in range(before, len(samples)):
            t, rate = samples[i]
            if i+1 < len(samples):
                nt, nr = samples[i+1]
                if nt-t > .25:
                    return None
                right = min(end, nt)
                r0 = rate+(nr-rate)*(left-t)/(nt-t)
                r1 = rate+(nr-rate)*(right-t)/(nt-t)
            else:
                right, r0, r1 = end, rate, rate
            if right > left:
                total += (r0+r1)*.5*(right-left)
                left = right
            if left >= end:
                return total
        return None

    def start(self, stamp, raw, pose=None):
        self.active = True
        self.stamp, self.raw, self.pose = stamp, tuple(raw), tuple(pose or raw)
        self.yaw_offset = wrap(self.pose[2]-self.raw[2])
        self.source = 'lidar_odometry'
        self.correction = 0.
        self.imu_used = False

    def stop(self):
        self.active = False
        self.source = 'lidar_odometry'

    def update(self, stamp, raw, rebase=False):
        raw = tuple(raw)
        if not self.active:
            return raw
        dt = stamp-self.stamp
        if dt <= 0:
            return self.pose
        if rebase or dt > 1.:
            # A LiDAR odometry reset must not move the already localized boat.
            self.start(stamp, raw, self.pose)
            return self.pose
        delta = self.integral(self.stamp, stamp)
        lidar_delta = wrap(raw[2]-self.raw[2])
        self.correction = 0.
        if delta is None:
            angle = wrap(self.pose[2]+lidar_delta)
            self.yaw_offset = wrap(angle-raw[2])
            self.source = 'lidar_odometry_fallback'
        else:
            if not self.imu_used:
                self.yaw_offset = wrap(self.pose[2]-self.raw[2])
                self.imu_used = True
            predicted = wrap(self.pose[2]+delta)
            innovation = wrap(raw[2]+self.yaw_offset-predicted)
            # Gyro carries fast turns. Consistent LiDAR slowly removes gyro bias;
            # an ambiguous scan cannot snap the heading to a different structure.
            if abs(innovation) <= math.radians(25):
                limit = math.radians(3)*dt
                self.correction = max(-limit, min(limit, (1-math.exp(-dt/12.))*innovation))
            angle = wrap(predicted+self.correction)
            self.source = 'imu_lidar'
        raw_mid = self.raw[2]+lidar_delta*.5
        fused_mid = self.pose[2]+wrap(angle-self.pose[2])*.5
        rotation = wrap(fused_mid-raw_mid)
        co, si = math.cos(rotation), math.sin(rotation)
        dx, dy = raw[0]-self.raw[0], raw[1]-self.raw[1]
        self.pose = (self.pose[0]+co*dx-si*dy, self.pose[1]+si*dx+co*dy, angle)
        self.stamp, self.raw = stamp, raw
        return self.pose
