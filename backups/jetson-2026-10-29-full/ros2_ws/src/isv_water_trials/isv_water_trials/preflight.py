"""Read-only sensor availability/quality check. Returns nonzero when incomplete."""
import argparse
import math
import time
from .common import load_config


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--config')
    parser.add_argument('--seconds', type=float, default=8)
    args = parser.parse_args(argv)
    if not 1 <= args.seconds <= 60:
        parser.error('seconds must be 1..60')
    cfg = load_config(args.config)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import NavSatFix, LaserScan, PointCloud2
    from vectornav_msgs.msg import CommonGroup
    from std_msgs.msg import Bool
    rclpy.init()
    node = Node('water_trial_preflight')
    seen = {}
    def callback(key):
        def receive(msg):
            seen[key] = (time.monotonic(), msg)
        return receive
    specs = [(NavSatFix,cfg['gps_topic'],'gps'),(CommonGroup,cfg['vectornav_topic'],'vn'),
             (Bool,cfg['estop_topic'],'estop'), (LaserScan,'/scan','scan'),
             (PointCloud2,'/velodyne_points','cloud')]
    subscriptions=[node.create_subscription(typ,topic,callback(key),qos_profile_sensor_data) for typ,topic,key in specs]
    until=time.monotonic()+args.seconds
    while time.monotonic()<until:
        rclpy.spin_once(node,timeout_sec=.1)
    failures=[]
    for key in ('gps','vn','estop','scan','cloud'):
        age=time.monotonic()-seen[key][0] if key in seen else math.inf
        print(f'{key}: age_s={age:.3f}')
        if age>cfg['max_sensor_gap_s']:
            failures.append(key+' missing/stale')
    if 'gps' in seen:
        m=seen['gps'][1]
        if m.status.status<0 or not math.isfinite(m.latitude) or not math.isfinite(m.longitude):
            failures.append('GPS has no valid fix')
        if m.position_covariance_type==0 and not cfg['allow_unknown_gps_covariance']:
            failures.append('GPS covariance unknown')
    if 'vn' in seen and not seen['vn'][1].group_fields & 0x8:
        failures.append('VectorNav heading not enabled')
    if 'estop' in seen and seen['estop'][1].data:
        failures.append('Emergency switch pressed')
    for info in node.get_publishers_info_by_topic(cfg['gps_topic']):
        if 'spoof' in info.node_name.lower():
            failures.append('GPS spoofer publisher present')
    if node.count_publishers(cfg['gps_topic'])>1:
        failures.append('Multiple GPS publishers: mixed data risk')
    for topic in (cfg['gps_topic'],cfg['vectornav_topic'],'/scan','/velodyne_points'):
        print('publishers',topic,node.count_publishers(topic))
    print('frame_confirmed:',cfg['frame_confirmed'])
    node.destroy_node()
    rclpy.shutdown()
    if failures:
        print('NOT_READY:', '; '.join(failures))
        raise SystemExit(2)
    print('SENSOR_READY (does not validate motor direction, E-stop wiring, or mounting)')
