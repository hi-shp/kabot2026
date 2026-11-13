#!/usr/bin/env python3
"""Read-only live avoidance diagnostics. Does not arm, start, or publish commands."""
import json,math,time,inspect
from dataclasses import fields
from pathlib import Path
import numpy as np
import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from isv_shape_navigation.core import Scan
from isv_shape_navigation.route_mission import RouteConfig,limit_course_scan,choose_corridor

def main():
 rclpy.init();node=rclpy.create_node('avoidance_readonly_debug')
 client=node.create_client(GetParameters,'/course_mission/get_parameters')
 if not client.wait_for_service(timeout_sec=5.):
  print('ERROR: /course_mission parameter service unavailable',flush=True)
  node.destroy_node();rclpy.shutdown();return
 keys=['footprint_radius','clearance','avoidance_lidar_min_range','avoidance_lidar_range',
       'avoidance_range','avoidance_horizon','avoidance_fov_deg','avoidance_distance_weight',
       'avoidance_angle_weight','avoidance_distance_epsilon','scan_x','scan_y','scan_yaw_deg','scan_frame','scan_timeout']
 future=client.call_async(GetParameters.Request(names=keys));rclpy.spin_until_future_complete(node,future,timeout_sec=5.)
 if not future.done() or future.result() is None:raise RuntimeError('Runtime parameter read timed out')
 attrs={1:'bool_value',2:'integer_value',3:'double_value',4:'string_value'}
 params={k:getattr(v,attrs[v.type]) for k,v in zip(keys,future.result().values) if v.type in attrs}
 missing=set(keys)-set(params)
 if missing:raise RuntimeError('Missing live parameters: '+str(missing))
 cfg=RouteConfig(**{k:v for k,v in params.items() if k in {f.name for f in fields(RouteConfig)}})
 radius=params['footprint_radius']+params['clearance']
 print('[AVOID_CONFIG] '+json.dumps(dict(controller_source=inspect.getsourcefile(choose_corridor),controller_parameters=params,effective_radius=radius),ensure_ascii=False),flush=True)
 latest={}
 def on_scan(msg):latest['scan']=(msg,time.monotonic())
 def on_status(msg):
  try:latest['status']=(json.loads(msg.data),time.monotonic())
  except ValueError:pass
 subscriptions=[node.create_subscription(LaserScan,'/scan',on_scan,qos_profile_sensor_data),
                node.create_subscription(String,'/course/status',on_status,qos_profile_sensor_data)]
 def report():
  if 'scan' not in latest or 'status' not in latest:
   print('[AVOID_DEBUG] WAIT_INPUT scan='+str('scan' in latest)+' status='+str('status' in latest),flush=True);return
  msg,scan_at=latest['scan'];status,status_at=latest['status'];raw=np.asarray(msg.ranges,dtype=float)
  scan=Scan(raw,msg.angle_min,msg.angle_increment,max(params['avoidance_lidar_min_range'],msg.range_min),msg.range_max,
      msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9, math.radians(params['scan_yaw_deg']),params['scan_x'],params['scan_y'])
  scan=limit_course_scan(scan,params['avoidance_lidar_range'])
  desired=status.get('heading_error_deg')
  if desired is None or not math.isfinite(desired):
   print('[AVOID_DEBUG] WAIT_HEADING state='+str(status.get('state')),flush=True);return
  angle,result=choose_corridor(scan,math.radians(desired),cfg,radius)
  data=dict(live_state=status.get('state'),scan_age=round(time.monotonic()-scan_at,3),
      status_age=round(time.monotonic()-status_at,3),frame=msg.header.frame_id,frame_match=msg.header.frame_id==params['scan_frame'],raw_samples=int(len(raw)),
      raw_far_to_zero=int(np.count_nonzero(raw>=cfg.avoidance_lidar_range)),
      goal_deg=round(desired,3),recomputed_reason=result['reason'],
      checks=result['diagnostics'],selected_deg=math.degrees(angle) if angle is not None else None,
      selected_score=result.get('objective',{}).get('selected_score'),
      live_pwm=status.get('intended_pwm'),feedback=status.get('feedback_pwm'),
      active=status.get('output',{}).get('active'))
  print('[AVOID_DEBUG] '+json.dumps(data,ensure_ascii=False,allow_nan=False),flush=True)
 timer=node.create_timer(1.,report)
 try:rclpy.spin(node)
 except KeyboardInterrupt:pass
 finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
