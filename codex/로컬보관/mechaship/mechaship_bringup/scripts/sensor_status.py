#!/usr/bin/env python3
"""Report actual recent sensor receipt, never simulated values or coordinates."""
from collections import deque
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterDescriptor


class SensorStatus(Node):
    def __init__(self):
        super().__init__('sensor_status')
        self.start=time.monotonic()
        self.receipts={};self.last={};self.subscriptions_owned=[]
        self.last_missing=None
        self.publisher=self.create_publisher(String,'/sensor_status',10)
        specs={
            '/gps/fix':'sensor_msgs/msg/NavSatFix',
            '/vectornav/raw/common':'vectornav_msgs/msg/CommonGroup',
            '/vectornav/imu':'sensor_msgs/msg/Imu',
            '/camera/image_raw':'sensor_msgs/msg/Image',
            '/velodyne_packets':'velodyne_msgs/msg/VelodyneScan',
            '/velodyne_points':'sensor_msgs/msg/PointCloud2',
            '/scan':'sensor_msgs/msg/LaserScan',
            '/sensor/battery/voltage':'std_msgs/msg/Float32',
            '/sensor/emo/status':'std_msgs/msg/Bool',
            '/smc_2000/fix':'sensor_msgs/msg/NavSatFix',
            '/actuators/status':'std_msgs/msg/String',
        }
        expected=self.declare_parameter('expected_topics',list(specs),
                                        ParameterDescriptor(dynamic_typing=True)).value
        for topic,type_name in specs.items():
            if topic not in expected:continue
            self.receipts[topic]=deque(maxlen=500)
            def receive(msg,topic=topic):
                self.receipts[topic].append(time.monotonic());self.last[topic]=msg
            self.subscriptions_owned.append(self.create_subscription(get_message(type_name),topic,receive,qos_profile_sensor_data))
        self.create_timer(5.,self.report)

    def report(self):
        now=time.monotonic();topics={}
        for topic,times in self.receipts.items():
            recent=[t for t in times if now-t<=10]
            age=now-times[-1] if times else None
            hz=(len(recent)-1)/(recent[-1]-recent[0]) if len(recent)>1 and recent[-1]>recent[0] else 0.
            item={'receiving':age is not None and age<3.,'age_s':round(age,2) if age is not None else None,
                  'hz':round(hz,2),'publishers':self.count_publishers(topic)}
            msg=self.last.get(topic)
            if topic=='/actuators/status' and msg is not None:
                try:
                    actuator=json.loads(msg.data)
                    item['receiving']=item['receiving'] and actuator.get('receiving') is True
                except (ValueError,AttributeError):item['receiving']=False
            if topic.endswith('/fix') and msg is not None:
                item['fix_status']=int(msg.status.status)
                item['valid_position']=msg.status.status>=0 and math.isfinite(msg.latitude) and math.isfinite(msg.longitude)
            topics[topic]=item
        self.publisher.publish(String(data=json.dumps({'uptime_s':round(now-self.start,1),'topics':topics})))
        missing=[topic for topic,item in topics.items() if not item['receiving']]
        state=tuple(missing)
        if state!=self.last_missing:
            self.get_logger().info('Receiving: '+', '.join(topic for topic,item in topics.items() if item['receiving'])+
                                   ('; no recent data: '+', '.join(missing) if missing else '; all enabled streams receiving'))
            self.last_missing=state


def main():
    rclpy.init();node=SensorStatus()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
