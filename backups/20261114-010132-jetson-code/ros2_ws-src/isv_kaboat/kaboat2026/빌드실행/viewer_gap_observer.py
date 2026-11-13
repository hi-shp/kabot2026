"""GUI-only LiDAR observations. No publishers, output clients, or motion state."""
from dataclasses import fields
import math
from pathlib import Path
import threading
import time

import numpy as np
import yaml
from isv_shape_navigation.core import Scan, select_navigation_scan
from isv_shape_navigation.follow import FollowConfig
from isv_shape_navigation.route_mission import RouteConfig
from isv_shape_navigation.gap_preview import GapPreview


class GapObserver:
    def __init__(self, config_path, clock=time.monotonic):
        values=yaml.safe_load(Path(config_path).read_text(encoding='utf-8'))['course_mission']['ros__parameters']
        self.route=RouteConfig(**{f.name:values[f.name] for f in fields(RouteConfig) if f.name in values})
        self.follow=FollowConfig(**{f.name:values[f.name] for f in fields(FollowConfig) if f.name in values})
        self.frame=values.get('scan_frame','velodyne')
        self.yaw=math.radians(values.get('scan_yaw_deg',0.))
        self.x,self.y=values.get('scan_x',0.),values.get('scan_y',0.)
        if not all(math.isfinite(v) for v in (self.yaw,self.x,self.y)):
            raise ValueError('Invalid scan display transform')
        self.primary_topic=values.get('scan_topic','/scan_preprocessed')
        self.fallback_topic=values.get('scan_fallback_topic','/scan')
        self.clock=clock;self.lock=threading.Lock()
        self.primary=self.raw=None
        self.preview=GapPreview();self.requested=-math.inf;self.received=None
        self.value=dict(status='WAIT_SCAN',observation_only=True,independent=True,
            gaps=[],obstacles=[],selected_gap=None,command_deg=None,reason='라이다 관측 대기')

    def observe(self,message,raw=False):
        stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9
        if (message.header.frame_id!=self.frame or not len(message.ranges)
                or not all(math.isfinite(v) for v in (stamp,message.angle_min,message.angle_increment,
                                                       message.range_min,message.range_max))
                or message.angle_increment==0 or message.range_max<=message.range_min):
            return
        scan=Scan(np.asarray(message.ranges),message.angle_min,message.angle_increment,
            message.range_min,message.range_max,stamp,self.yaw,self.x,self.y)
        with self.lock:
            if raw:self.raw=scan
            else:self.primary=scan

    def update(self,ros_now):
        if self.clock()-self.requested>3:
            return
        with self.lock:
            primary,raw=self.primary,self.raw
        scan,source=select_navigation_scan(primary,raw,ros_now,self.follow.scan_timeout)
        value=self.preview.snapshot(ros_now,scan,raw,self.route,self.follow,'WAIT_GPS')
        value.update(independent=True,source='GUI_LIDAR_OBSERVATION',
            scan_source=self.primary_topic if source=='primary' else self.fallback_topic if source=='fallback' else None,
            reason=('LiDAR 관측 후보 '+str(len(value['gaps']))+'개 · 운항 프로그램 대기 · 표시 전용'
                    if value['status']=='LIVE' else '라이다 최신 관측 대기 · 표시 전용'))
        with self.lock:
            self.value=value;self.received=self.clock()

    def snapshot(self):
        now=self.clock()
        with self.lock:
            self.requested=now
            value=dict(self.value)
            received_age=None if self.received is None else max(0.,now-self.received)
            if value.get('age_s') is not None:
                value['age_s']+=received_age or 0.
            value['receive_age']=received_age
            if (received_age is None or received_age>.8 or value.get('age_s',math.inf)>self.follow.scan_timeout):
                value.update(status='WAIT_SCAN',gaps=[],reason='라이다 최신 관측 대기 · 표시 전용')
            return value

    def alignment(self):
        return dict(valid=True,frame_id=self.frame,yaw_deg=math.degrees(self.yaw),
                    x_m=self.x,y_m=self.y,age_s=0.,source='configured_scan_transform')


def attach_gap_observer(node,config_path):
    from sensor_msgs.msg import LaserScan
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    observer=GapObserver(config_path)
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
    observer.subscriptions=[node.create_subscription(LaserScan,observer.primary_topic,observer.observe,qos)]
    if observer.fallback_topic!=observer.primary_topic:
        observer.subscriptions.append(node.create_subscription(LaserScan,observer.fallback_topic,
            lambda m:observer.observe(m,raw=True),qos))
    observer.timer=node.create_timer(.2,lambda:observer.update(node.get_clock().now().nanoseconds/1e9))
    return observer
