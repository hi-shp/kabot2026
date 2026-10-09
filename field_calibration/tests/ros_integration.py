"""Real DDS, launch, rosbag2 and replay checks, no vessel domain or actuators."""
import csv
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time

# This script deliberately replaces inherited hardware-domain configuration.
os.environ['ROS_DOMAIN_ID'] = '191'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy
import rosbag2_py
import yaml
from ament_index_python.packages import get_package_share_directory


def main():
    ros2 = [sys.executable, shutil.which('ros2')]
    root = Path(tempfile.mkdtemp(prefix='field-integration-'))
    share = Path(get_package_share_directory('field_calibration'))
    cfg = yaml.safe_load((share/'config/field_test.yaml').read_text())
    params = cfg['/**']['ros__parameters']
    params.update(output_root=str(root/'data'), throttle_topic='/field/mock/throttle',
                  steering_topic='/field/mock/steering',
                  record_topics=['/scan', '/imu', '/scan/odom', '/tf_static', '/field/mock/throttle', '/field/mock/steering'])
    config = root/'mock.yaml'
    config.write_text(yaml.safe_dump(cfg))
    processes, logs = [], []
    stopped = set()

    def start(args, label):
        f = open(root/(label+'.log'), 'w')
        logs.append(f)
        p = subprocess.Popen(ros2+args, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(p)
        return p

    def stop(p):
        stopped.add(p)
        if p.poll() is None:
            p.send_signal(signal.SIGINT)
            try:
                p.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGTERM)
                p.wait(timeout=5)

    rclpy.init()
    node = Node('field_integration_assertions')
    states, predictions, errors, statuses, replay = [], [], [], [], []
    for topic, target in [('/field/state', states), ('/field/prediction', predictions),
                          ('/field/prediction_error', errors), ('/field/recording_status', statuses)]:
        node.create_subscription(String, topic, lambda m, out=target: out.append(json.loads(m.data)),
                                 QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))
    node.create_subscription(LaserScan, '/field/replay_scan', replay.append, 10)

    def wait_until(condition, seconds=8):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
            if condition():
                return
            if any(p not in stopped and p.poll() is not None and p.returncode != 0 for p in processes):
                raise AssertionError('subprocess failed; inspect '+str(root))
        raise AssertionError('condition timed out; inspect '+str(root))

    def param(node_name, name, value):
        client = node.create_client(SetParameters, '/'+node_name+'/set_parameters')
        assert client.wait_for_service(timeout_sec=5)
        pv = (ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
              if isinstance(value, bool) else ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value))
        future = client.call_async(SetParameters.Request(parameters=[Parameter(name=name, value=pv)]))
        wait_until(future.done)
        result = future.result().results[0]
        node.destroy_client(client)
        return result

    checks = {}
    try:
        launch = start(['launch', 'field_calibration', 'field_test.launch.py', 'config:='+str(config)], 'launch')
        mock = start(['run', 'field_calibration', 'mock_sensors'], 'mock')
        wait_until(lambda: any(s['valid'] for s in states) and len(errors) >= 6 and statuses)
        checks['launch_mock_subscription_baseline_predictions_errors'] = True
        assert all(p['model_status'] == 'UNCALIBRATED' for p in predictions)
        for topic in ('/actuator/thruster/percentage', '/actuator/key/degree'):
            assert not node.get_publishers_info_by_topic(topic)
        checks['passive_zero_actuator_publishers'] = True
        assert not param('calibration_node', 'automatic_drive', True).successful
        checks['automatic_drive_rejected'] = True
        for fault, value, reason in [('drop_scan', True, 'lidar_timestamp_or_age'),
                                     ('drop_imu', True, 'imu_timestamp_or_age'),
                                     ('imu_stamp_offset', .8, 'imu_odom_skew'),
                                     ('drop_odom', True, 'rf2o_timestamp_or_age'),
                                     ('zero_covariance', True, 'covariance_unknown')]:
            assert param('field_mock_sensors', fault, value).successful
            offset = len(states)
            wait_until(lambda: any(not s['valid'] and reason in s['reasons'] for s in states[offset:]))
            invalid = next(s for s in states[offset:] if reason in s['reasons'])
            assert invalid['x'] is None and invalid['y'] is None
            assert param('field_mock_sensors', fault, False if isinstance(value, bool) else 0.).successful
            offset = len(states)
            wait_until(lambda: any(s['valid'] for s in states[offset:]))
            checks[fault+'_invalid_and_recovery'] = True
        # Viewer renders actual received numeric data, with no scan subscription.
        viewer = start(['run', 'field_calibration', 'trajectory_monitor', '--snapshot', str(root/'viewer.png'),
                        '--seconds', '2'], 'viewer')
        wait_until(lambda: viewer.poll() is not None, 12)
        assert viewer.returncode == 0 and (root/'viewer.png').stat().st_size > 10000
        checks['numeric_viewer_snapshot'] = True
        count_before = statuses[-1]['counts'].get('/scan', 0)
        wait_until(lambda: statuses[-1]['counts'].get('/scan', 0) >= count_before+5)
        checks['recording_continues_after_viewer_exit'] = True
        stop(mock)
        stop(launch)
        experiment = next((root/'data').iterdir())
        metadata = json.loads((experiment/'metadata.json').read_text())
        assert metadata['closed']
        assert metadata['message_counts']['/tf_static'] >= 1
        bag_meta = yaml.safe_load((experiment/'bag/metadata.yaml').read_text())
        tf_meta = next(t['topic_metadata'] for t in bag_meta['rosbag2_bagfile_information']['topics_with_message_count']
                       if t['topic_metadata']['name'] == '/tf_static')
        assert tf_meta['offered_qos_profiles']
        checks['latched_tf_recorded_with_qos'] = True
        for name in ('states', 'commands', 'sensors', 'predictions', 'errors'):
            with (experiment/(name+'.csv')).open() as f:
                assert len(list(csv.DictReader(f))) > 0
        assert (experiment/'bag/metadata.yaml').exists()
        checks['csv_flush_bag_finalize'] = True
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(experiment/'bag'), storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions('', ''))
        count = 0
        while reader.has_next():
            reader.read_next()
            count += 1
        assert count > 100
        checks['bag_read_count'] = count
        player = start(['bag', 'play', str(experiment/'bag'), '--topics', '/scan',
                        '--remap', '/scan:=/field/replay_scan', '--rate', '5.0'], 'replay')
        wait_until(lambda: len(replay) >= 10, 12)
        stop(player)
        checks['bag_play_received_scan'] = len(replay)
        # Recompute the raw bag under /clock with a synthetic calibrated model.
        # The fixture is explicitly not a real-vessel calibration artifact.
        model_path = root/'synthetic_model.json'
        model_path.write_text(json.dumps(dict(schema=1, status='CALIBRATED',
            version='SYNTHETIC_INTEGRATION_ONLY', validation={'synthetic_fixture': True},
            surge=[-.7, .045, 0.], yaw=[-1.2, 1.8, 1.8, 0.],
            servo_neutral=90., throttle_neutral=0., throttle_delay=0., steering_delay=0.,
            command_bounds=dict(throttle=[0., 20.], steering=[75., 105.]))))
        params.update(output_root=str(root/'recomputed'), model_path=str(model_path))
        config.write_text(yaml.safe_dump(cfg))
        offset = len(predictions)
        recompute = start(['launch', 'field_calibration', 'field_test.launch.py',
                           'config:='+str(config), 'use_sim_time:=true'], 'recompute')
        player = start(['bag', 'play', str(experiment/'bag'), '--clock', '--delay', '1', '--topics',
                        '/scan', '/imu', '/scan/odom', '/tf_static', '/field/mock/throttle', '/field/mock/steering'],
                       'recompute_play')
        wait_until(lambda: any(p['model_status'] == 'CALIBRATED' for p in predictions[offset:]), 15)
        wait_until(lambda: player.poll() is not None, 20)
        stop(recompute)
        result_path = next((root/'recomputed').iterdir())
        result_meta = json.loads((result_path/'metadata.json').read_text())
        assert result_meta['closed']
        with (result_path/'predictions.csv').open() as f:
            predicted_rows = list(csv.DictReader(f))
        assert any(r['model_version'] == 'SYNTHETIC_INTEGRATION_ONLY' for r in predicted_rows)
        with (result_path/'errors.csv').open() as f:
            assert len(list(csv.DictReader(f))) > 0
        checks['sim_time_raw_replay_calibrated_model_and_csv'] = True
        (root/'result.json').write_text(json.dumps(checks, indent=2))
        print(json.dumps(dict(artifacts=str(root), checks=checks), indent=2))
    finally:
        for p in reversed(processes):
            stop(p)
        for f in logs:
            f.close()
        node.destroy_node()
        rclpy.shutdown()
        print('Artifacts:', root)


if __name__ == '__main__':
    main()
