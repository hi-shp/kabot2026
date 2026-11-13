#!/usr/bin/env python3
"""Observe real sensor streams for eight seconds; never send commands."""
from collections import defaultdict
import json
import math
from pathlib import Path
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message

def main():
    rclpy.init()
    node=rclpy.create_node('kaboat2026_sensor_check')
    specs={
        '/gps/fix':'sensor_msgs/msg/NavSatFix',
        '/vectornav/raw/common':'vectornav_msgs/msg/CommonGroup',
        '/vectornav/imu':'sensor_msgs/msg/Imu',
        '/velodyne_packets':'velodyne_msgs/msg/VelodyneScan',
        '/velodyne_points':'sensor_msgs/msg/PointCloud2',
        '/scan':'sensor_msgs/msg/LaserScan',
        '/actuators/status':'std_msgs/msg/String'}
    samples=defaultdict(list);latest={};subscriptions=[]
    def receive(topic,msg):
        samples[topic].append(time.monotonic());latest[topic]=msg
    for topic,kind in specs.items():
        subscriptions.append(node.create_subscription(get_message(kind),topic,
            lambda msg,topic=topic:receive(topic,msg),qos_profile_sensor_data))
    result={'sensor_monitor':{'topics':{}}}
    try:
        deadline=time.monotonic()+8
        while rclpy.ok() and time.monotonic()<deadline: rclpy.spin_once(node,timeout_sec=.1)
        now=time.monotonic()
        for topic in specs:
            stamps=samples[topic];msg=latest.get(topic)
            entry={'receiving':bool(stamps and now-stamps[-1]<3),
                'messages':len(stamps),'age_s':round(now-stamps[-1],2) if stamps else None,
                'hz':round((len(stamps)-1)/(stamps[-1]-stamps[0]),2) if len(stamps)>1 else 0}
            if topic=='/gps/fix' and msg is not None:
                entry.update(fix_status=int(msg.status.status),valid_position=bool(msg.status.status>=0 and math.isfinite(msg.latitude) and math.isfinite(msg.longitude)))
            if topic=='/actuators/status' and msg is not None:
                try: result['actuator']=json.loads(msg.data)
                except ValueError: result['actuator']={}
                entry['receiving']=entry['receiving'] and result['actuator'].get('receiving') is True
            result['sensor_monitor']['topics'][topic]=entry
        result['checked_at_utc']=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime())
        print(json.dumps(result,indent=2))
        target=Path(__file__).resolve().parent.parent/'기록/sensor_status_latest.json'
        target.write_text(json.dumps(result,indent=2)+'\n')
        return 0 if all(v['receiving'] for v in result['sensor_monitor']['topics'].values()) else 2
    finally:
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__': raise SystemExit(main())
