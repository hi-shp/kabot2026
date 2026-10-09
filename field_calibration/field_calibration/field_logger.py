"""On-boat raw rosbag2 writer and CSV. No external connection required."""
import math
import os
import time
import shutil
from pathlib import Path
import rosbag2_py
from rclpy.serialization import serialize_message
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String
from .common import FieldNode, MODES, spin, stamp
from .storage import Experiment
from .bag import topic_metadata

RAW_TOPICS = ['/scan', '/imu', '/scan/odom', '/odom_rf2o', '/odom', '/tf', '/tf_static',
              '/actuator/thruster/percentage', '/actuator/key/degree', '/sensor/emo/status']
FIELD_TOPICS = ['/field/state', '/field/prediction', '/field/prediction_error', '/field/test_profile', '/field/command']


class FieldLogger(FieldNode):
    def __init__(self):
        super().__init__('field_logger')
        self.param('output_root', '~/field_data')
        mode = self.param('test_mode', 'manual_log')
        if mode not in MODES:
            raise ValueError('unsupported mode')
        profile = self.param('profile', 'indoor')
        self.topics = self.param('record_topics', RAW_TOPICS)
        for topic in (self.param('scan_topic', '/scan'), self.param('imu_topic', '/imu'),
                      self.param('odom_topic', '/scan/odom')):
            if topic not in self.topics:
                self.topics.append(topic)
        self.throttle_topic = self.param('throttle_topic', '/actuator/thruster/percentage')
        self.steering_topic = self.param('steering_topic', '/actuator/key/degree')
        for topic in (self.throttle_topic, self.steering_topic):
            if topic not in self.topics:
                self.topics.append(topic)
        self.param('fsync_interval', 1.0)
        self.max_size = self.param('bag_split_bytes', 268435456)
        config = {p: self.get_parameter(p).value for p in self.list_parameters([], 0).names}
        config['ros_distro'] = os.environ.get('ROS_DISTRO', 'unknown')
        self.experiment = Experiment(self.get_parameter('output_root').value, mode, profile, config)
        model_path = self.param('model_path', '')
        if model_path:
            try:
                shutil.copy2(Path(model_path).expanduser(), self.experiment.path/'model_snapshot.json')
            except OSError as e:
                self.experiment.meta['model_snapshot_error'] = str(e)
                self.get_logger().warning(f'recording continues without model snapshot: {e}')
        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(rosbag2_py.StorageOptions(uri=str(self.experiment.path/'bag'), storage_id='sqlite3',
                                                  max_bagfile_size=self.max_size,
                                                  storage_preset_profile='resilient', max_cache_size=0),
                         rosbag2_py.ConverterOptions('', ''))
        self.subscriptions_by_topic = {}
        self.counts = {}
        self.commands = {'throttle': None, 'steering': None, 'throttle_stamp': None, 'steering_stamp': None}
        self.version = 'UNCALIBRATED'
        self.pub = self.json_publisher('/field/recording_status')
        self.command_pub = self.json_publisher('/field/command')
        self.bag_error = None
        self.create_timer(0.5, self.discover)
        self.create_timer(1.0, self.tick)
        self.get_logger().info(f'PASSIVE recording: {self.experiment.path}')

    def discover(self):
        allowed = set(self.topics+FIELD_TOPICS)
        for topic, types in self.get_topic_names_and_types():
            if topic not in allowed or topic in self.subscriptions_by_topic or len(types) != 1:
                continue
            try:
                typ = types[0]
                cls = get_message(typ)
                profiles = [p.qos_profile for p in self.get_publishers_info_by_topic(topic)]
                if not profiles:
                    continue
                meta = topic_metadata(topic, typ, profiles)
                self.writer.create_topic(meta)
                qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE if topic == '/tf_static'
                                 else ReliabilityPolicy.BEST_EFFORT,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL if topic == '/tf_static'
                                 else DurabilityPolicy.VOLATILE)
                sub = self.create_subscription(cls, topic, lambda m, t=topic: self.receive(t, m), qos)
                self.subscriptions_by_topic[topic] = sub
                self.experiment.meta['discovered_topics'][topic] = typ
            except (ImportError, RuntimeError, TypeError) as e:
                self.get_logger().error(f'record discovery {topic}: {e}')

    def receive(self, topic, msg):
        now = self.now()
        if self.get_parameter('use_sim_time').value and now <= 0:
            return
        try:
            self.writer.write(topic, serialize_message(msg), int(now*1e9))
        except RuntimeError as e:
            self.bag_error = str(e)
            self.get_logger().error(f'raw bag write failed: {e}')
        self.counts[topic] = self.counts.get(topic, 0)+1
        if hasattr(msg, 'header'):
            self.experiment.row('sensors', dict(received=now, topic=topic,
                                               header_stamp=stamp(msg), frame=msg.header.frame_id))
        if topic in (self.throttle_topic, self.steering_topic) and hasattr(msg, 'data'):
            kind = 'throttle' if topic == self.throttle_topic else 'steering'
            value = float(msg.data)
            if math.isfinite(value):
                self.commands[kind], self.commands[kind+'_stamp'] = value, now
                self.experiment.row('commands', dict(stamp=now, kind=kind, value=value))
                self.publish_json(self.command_pub, dict(stamp=now, kind=kind, value=value))
        if topic in FIELD_TOPICS:
            import json
            try:
                data = json.loads(msg.data)
            except (ValueError, TypeError) as e:
                self.get_logger().error(f'invalid derived message on {topic}: {e}')
                return
            if topic == '/field/state':
                data.update(self.commands)
                data.update(test_mode=self.experiment.mode, model_version=self.version)
                data.update({k+'_age': v for k, v in data['ages'].items()})
                self.experiment.row('states', data)
            elif topic == '/field/prediction':
                self.version = data['model_version']
                for point in data['points']:
                    self.experiment.row('predictions', dict(point, initial_state=data['initial_state'], commands=data['commands']))
            elif topic == '/field/prediction_error':
                self.experiment.row('errors', data)
            elif topic == '/field/test_profile':
                self.experiment.mode = data['mode']

    def tick(self):
        self.experiment.meta['message_counts'] = self.counts
        if time.monotonic()-self.experiment.last_sync >= self.get_parameter('fsync_interval').value:
            self.experiment.sync()
        self.publish_json(self.pub, dict(stamp=self.now(), experiment_id=self.experiment.id,
                                        path=str(self.experiment.path), recording=self.bag_error is None,
                                        csv_recording=True, bag_error=self.bag_error, counts=self.counts,
                                        motor_control='PASSIVE', missing_topics=sorted(set(self.topics)-set(self.counts))))

    def destroy_node(self):
        # Close the writer before marking the experiment complete.
        if hasattr(self, 'writer'):
            close = getattr(self.writer, 'close', None)
            if close:
                close()
            self.writer = None
        if hasattr(self, 'experiment'):
            self.experiment.meta['message_counts'] = self.counts
            self.experiment.close()
        return super().destroy_node()


def main(args=None):
    spin(FieldLogger, args)
