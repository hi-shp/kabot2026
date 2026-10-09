"""Line-buffered CSV and periodic fsync; partial experiments remain inspectable."""
import csv
import json
import os
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path

STATE_COLUMNS = ['experiment_id', 'test_mode', 'stamp', 'received', 'valid', 'status', 'epoch',
                 'frame', 'base_frame', 'x', 'y', 'heading', 'u', 'v', 'r', 'imu_r', 'imu_stamp',
                 'imu_heading_relative', 'yaw_acceleration', 'odom_r', 'quality',
                 'throttle', 'steering', 'throttle_stamp', 'steering_stamp',
                 'lidar_age', 'imu_age', 'rf2o_age', 'covariance', 'reasons', 'model_version']


class Experiment:
    def __init__(self, root, mode, profile, config):
        self.id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid.uuid4().hex[:8]
        self.path = Path(root).expanduser()/self.id
        self.path.mkdir(parents=True, exist_ok=False)
        self.mode = mode
        self.files, self.writers = {}, {}
        for name, columns in [('states', STATE_COLUMNS),
                              ('commands', ['experiment_id', 'stamp', 'kind', 'value']),
                              ('predictions', ['experiment_id', 'prediction_stamp', 'target_stamp', 'horizon',
                                               'frame', 'epoch', 'model_version', 'x', 'y', 'heading', 'r', 'u',
                                               'initial_state', 'commands']),
                              ('errors', ['experiment_id', 'prediction_stamp', 'target_stamp', 'horizon',
                                          'model_version', 'position_error', 'heading_error', 'yaw_rate_error',
                                          'surge_error', 'actual']),
                              ('sensors', ['experiment_id', 'received', 'topic', 'header_stamp', 'frame'])]:
            f = open(self.path/(name+'.csv'), 'w', newline='', encoding='utf-8', buffering=1)
            writer = csv.DictWriter(f, columns, extrasaction='ignore')
            writer.writeheader()
            self.files[name], self.writers[name] = f, writer
        self.meta = dict(schema=1, experiment_id=self.id, created_utc=datetime.now(timezone.utc).isoformat(),
                         mode=mode, profile=profile, motor_control='PASSIVE', config=config,
                         units=dict(position='m', heading='rad CCW', velocity='m/s body', yaw_rate='rad/s',
                                    throttle='percent command', steering='degree command'),
                         closed=False, discovered_topics={})
        self.last_sync = time.monotonic()
        self.closed = False
        self.save_meta()

    def save_meta(self):
        temp = self.path/'metadata.json.tmp'
        with open(temp, 'w', encoding='utf-8') as f:
            json.dump(self.meta, f, indent=2, allow_nan=False)
            f.flush()
            os.fsync(f.fileno())
        temp.replace(self.path/'metadata.json')

    def row(self, name, data):
        self.writers[name].writerow(dict(experiment_id=self.id, **{
            k: json.dumps(v, allow_nan=False) if isinstance(v, (dict, list)) else v
            for k, v in data.items()}))

    def sync(self):
        for f in self.files.values():
            f.flush()
            os.fsync(f.fileno())
        self.save_meta()
        self.last_sync = time.monotonic()

    def close(self):
        if self.closed:
            return
        self.meta['closed'] = True
        self.sync()
        for f in self.files.values():
            f.close()
        self.closed = True
