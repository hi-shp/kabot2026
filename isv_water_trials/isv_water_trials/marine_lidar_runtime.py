"""Latest-only 3D cloud worker for the existing outdoor ROS owner."""
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
import json
import multiprocessing
import time

from .lidar_self_shape import cloud_arrays
from .marine_lidar import AttitudeHistory, MarineConfig, MarineObstacles, merge_obstacles
from .outdoor_route import waypoint_kind


_worker_engine = None
_worker_epoch = None


def process_cloud(xyz, stamp, pose, attitude, settings, boat, epoch):
    """Only arrays and configuration cross into this process; no ROS owner."""
    global _worker_engine, _worker_epoch
    if _worker_engine is None or _worker_epoch != epoch or _worker_engine.cfg != settings:
        _worker_engine = MarineObstacles(settings, boat)
        _worker_epoch = epoch
    return _worker_engine.update(xyz, stamp, pose, attitude)


class MarineLidarRuntime:
    def __init__(self, node, boat, root):
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import PointCloud2

        self.node, self.boat = node, boat
        self.path = root/'marine_lidar.json'
        self.settings = MarineConfig(**boat.get('marine_lidar', {}))
        self.epoch = 0
        self.active = False
        self.attitudes = AttitudeHistory()
        self.pool = ProcessPoolExecutor(max_workers=1, mp_context=multiprocessing.get_context('spawn'))
        self.pending = None
        self.latest = None
        self.result = None
        self.last_submit = -float('inf')
        self.settings_at = -float('inf')
        self.settings_version = None
        self.error = ''
        self.config_error = ''
        self.frame_id = None
        self.last_stamp = None
        self.origin = None
        self.submitted = 0
        self.completed = 0
        self.pose_waits = 0
        self.merged_cells = 0
        self.merge_reason = 'waiting'
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.subscription = node.create_subscription(PointCloud2, '/velodyne_points', self.cloud, qos)
        self.timer = node.create_timer(.05, self.poll)

    def imu(self, msg):
        if msg.group_fields & 0x0008:
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            self.attitudes.add(stamp, msg.yawpitchroll.z, msg.yawpitchroll.y)

    def cloud(self, msg):
        # Replacing this single slot bounds work while planning is busy.
        if self.needed():
            self.latest = msg

    def needed(self):
        return bool(getattr(self.node, 'execute', False) and
                    waypoint_kind(getattr(self.node, 'route', None), getattr(self.node, 'goal', 0)) in ('path', 'orbit'))

    def reload_settings(self, now):
        if now-self.settings_at < 1.:
            return
        self.settings_at = now
        try:
            version = self.path.read_text(encoding='utf-8') if self.path.exists() else None
            if version == self.settings_version:
                self.config_error = ''
                return
            values = json.loads(version) if version is not None else self.boat.get('marine_lidar', {})
            settings = MarineConfig(**values)
            self.settings = settings
            self.epoch += 1
            self.result = None
            self.settings_version = version
            self.config_error = ''
        except (OSError, ValueError, TypeError) as exc:
            self.config_error = 'marine_lidar.json: '+str(exc)

    def poll(self):
        if self.pending is not None:
            if not self.pending.done():
                return
            try:
                self.result = self.pending.result()
                self.completed += 1
                self.error = ''
            except (ValueError, TypeError, KeyError, RuntimeError) as exc:
                self.error = str(exc)
                self.result = None
            self.pending = None
        now = time.monotonic()
        self.reload_settings(now)
        if not self.needed():
            if self.active:
                self.epoch += 1
            self.active = False
            self.latest = None
            self.result = None
            return
        self.active = True
        if not self.settings.enabled or self.latest is None or now-self.last_submit < 1./self.settings.max_hz:
            return
        msg, self.latest = self.latest, None
        try:
            stamp = self.node.stamp(msg, self.settings.cloud_max_age_s)
            if stamp == self.last_stamp:
                return
            pose = self.node.state.pose_at(stamp)
            attitude = self.attitudes.at(stamp, self.settings.imu_max_age_s)
            if pose is None or attitude is None:
                self.error = 'cloud_time_gps_imu_wait'
                self.pose_waits += 1
                return
            origin = tuple(self.node.state.origin or ())
            changed = (self.frame_id is not None and self.frame_id != msg.header.frame_id)
            changed = changed or (self.origin is not None and origin != self.origin)
            changed = changed or (self.last_stamp is not None and stamp < self.last_stamp)
            if changed:
                self.epoch += 1
                self.result = None
            self.frame_id, self.last_stamp, self.origin = msg.header.frame_id, stamp, origin
            if msg.width*msg.height > self.settings.max_input_points:
                raise ValueError('cloud_point_budget_exceeded')
            xyz, _ = cloud_arrays(msg)
            self.last_submit = now
            self.pending = self.pool.submit(process_cloud, xyz, stamp, pose, attitude,
                                            self.settings, self.boat, self.epoch)
            self.submitted += 1
        except (ValueError, TypeError, KeyError) as exc:
            self.error = str(exc)

    def merge(self, scan, pose):
        # A frame mismatch must not put points into a different laser frame.
        self.merged_cells = 0
        self.merge_reason = 'waiting'
        if scan.get('frame_id') != self.frame_id:
            self.merge_reason = 'frame_mismatch'
            return scan
        if self.origin != tuple(self.node.state.origin or ()):
            self.merge_reason = 'origin_changed'
            return scan
        merged = merge_obstacles(scan, pose, self.result, self.boat, self.settings)
        self.merged_cells = merged.get('marine_cells', 0)
        self.merge_reason = 'merged' if self.merged_cells else 'no_confirmed_evidence'
        if pose is None:
            self.merge_reason = 'scan_pose_wait'
        elif self.result and scan['stamp']-self.result['stamp'] > self.settings.memory_s:
            self.merge_reason = 'memory_expired'
        return merged

    def diagnostic(self):
        value = dict(self.result['status']) if self.result else {}
        value.update(submitted=self.submitted, completed=self.completed, pose_waits=self.pose_waits,
                     merged_cells=self.merged_cells, merge_reason=self.merge_reason,
                     error=self.error, active=self.active)
        if self.result:
            value['cloud_stamp'] = self.result['stamp']
            value['confirmed_points'] = self.result['points'][:64]
            value['connector_points'] = self.result.get('connector_points', [])[:64]
        return value

    def status(self):
        value = dict(self.result['status']) if self.result else {}
        age = self.node.ros_now()-self.result['stamp'] if self.result else None
        value.update(enabled=self.settings.enabled, active=self.active, worker='process',
                     error=self.error, config_error=self.config_error,
                     cloud_age_s=age, fresh=age is not None and 0 <= age <= self.settings.cloud_max_age_s,
                     busy=self.pending is not None, settings=asdict(self.settings))
        value.update(submitted=self.submitted, completed=self.completed, pose_waits=self.pose_waits,
                     merged_cells=self.merged_cells, merge_reason=self.merge_reason)
        return value

    def close(self):
        self.timer.cancel()
        self.pool.shutdown(wait=True, cancel_futures=True)
