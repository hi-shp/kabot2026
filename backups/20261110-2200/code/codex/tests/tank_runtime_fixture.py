"""Isolated DDS domain 178. Fake sensors and fake actuators; no real hardware."""
import os
assert os.environ.get('ROS_DOMAIN_ID')=='178' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
import json,math,signal,subprocess,sys,tempfile,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid,Odometry
from sensor_msgs.msg import LaserScan,Imu
from std_msgs.msg import String
from std_srvs.srv import Trigger,SetBool
from mechaship_interfaces.srv import ThrottlePulseWidth

rclpy.init(); node=Node('isolated_tank_fixture'); started=time.monotonic()
qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
publishers={k:node.create_publisher(cls,topic,qos if k=='map' else 10) for k,cls,topic in [
 ('map',OccupancyGrid,'/map'),('pose',PoseWithCovarianceStamped,'/tank/pose'),('odom',Odometry,'/tank/odom'),
 ('scan',LaserScan,'/scan'),('imu',Imu,'/vectornav/imu'),('mcu',String,'/actuators/status'),('session',String,'/tank/session')]}
values=[1500,1500]; received=[]; states=[]; stop_ack=[]
def throttle(side):
 def callback(req,res):
  assert 1420<=req.pulse_width<=1580,req.pulse_width
  values[side]=req.pulse_width;received.append((time.monotonic()-started,side,req.pulse_width));res.status=True;return res
 return callback
def arm(req,res):res.success=True;return res
def stop(req,res):values[:]=[1500,1500];stop_ack.append(True);res.success=True;return res
for i,side in enumerate(('left','right')):node.create_service(ThrottlePulseWidth,'/actuators/throttle_'+side+'/set_pulse_width',throttle(i))
node.create_service(SetBool,'/actuators/arm',arm);node.create_service(Trigger,'/actuators/stop',stop)
node.create_subscription(String,'/tank/status',lambda m:states.append((time.monotonic()-started,json.loads(m.data))),10)
def publish():
 stamp=node.get_clock().now().to_msg();elapsed=time.monotonic()-started
 grid=OccupancyGrid();grid.header.frame_id='tank_map';grid.header.stamp=stamp
 grid.info.width=grid.info.height=120;grid.info.resolution=.1;grid.info.origin.position.x=2.;grid.info.origin.position.y=-1.;grid.info.origin.orientation.w=1.;grid.data=[0]*14400
 publishers['map'].publish(grid)
 pose=PoseWithCovarianceStamped();pose.header.stamp=stamp;pose.header.frame_id='tank_map';pose.pose.pose.position.x=6.;pose.pose.pose.position.y=5.;pose.pose.pose.orientation.w=1.
 publishers['pose'].publish(pose)
 odom=Odometry();odom.header.stamp=stamp;odom.header.frame_id='tank_odom';odom.pose=pose.pose;publishers['odom'].publish(odom)
 imu=Imu();imu.header.stamp=stamp;imu.header.frame_id='vectornav';imu.orientation.w=1.;imu.linear_acceleration.z=9.8;publishers['imu'].publish(imu)
 scan=LaserScan();scan.header.stamp=stamp;scan.header.frame_id='velodyne';scan.angle_min=-math.pi;scan.angle_max=math.pi;scan.angle_increment=2*math.pi/180;scan.range_min=.1;scan.range_max=30.
 for i in range(180):
  a=scan.angle_min+i*scan.angle_increment;dx,dy=math.cos(a),math.sin(a)
  distances=[(b-6)/dx for b in (2,14) if abs(dx)>1e-8]+[(b-5)/dy for b in (-1,11) if abs(dy)>1e-8]
  scan.ranges.append(min(d for d in distances if d>0))
 if not 7<elapsed<8.5:publishers['scan'].publish(scan)
 publishers['mcu'].publish(String(data=json.dumps(dict(receiving=True,mode=1 if 4<elapsed<5.5 else 2,armed=True,age_s=.01,left=values[0],right=values[1]))))
 publishers['session'].publish(String(data=json.dumps(dict(session_id='fixture',mode='mapping'))))
node.create_timer(.1,publish)
with tempfile.TemporaryDirectory(prefix='tank-isolated-') as folder:
 root=Path(folder);route=root/'route.json';route.write_text(json.dumps(dict(frame_id='tank_map',session_id='fixture',a=dict(x=11,y=5),b=dict(x=11,y=8))))
 with (root/'run.log').open('w') as log:
  child=subprocess.Popen([sys.executable,'-m','isv_water_trials.indoor_trial','--route',str(route),'--execute','--output',str(root/'logs')],stdout=log,stderr=subprocess.STDOUT)
  try:
   while time.monotonic()-started<12 and child.poll() is None:rclpy.spin_once(node,timeout_sec=.02)
   child.send_signal(signal.SIGINT)
   deadline=time.monotonic()+8
   while child.poll() is None and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
   assert child.poll()==0,(child.poll(),(root/'run.log').read_text())
  finally:
   if child.poll() is None:child.kill();child.wait()
 assert stop_ack and values==[1500,1500]
 assert any(v!=1500 for _,_,v in received), (root/'run.log').read_text()
 assert any(s['state']=='RC' for _,s in states)
 assert any(7.7<t<8.5 and s['state']=='WAITING' for t,s in states)
 assert any(t>9 and s['state']=='RUNNING' for t,s in states)
 session=next((root/'logs').iterdir());assert (session/'checkpoint.json').exists()
 print('ISOLATED_ROS_OK: AUTO <=20%, RC priority, stale scan neutral, recovery, Ctrl+C neutral ACK, durable logs',flush=True)
 # Test recording options on the same isolated domain, with no hardware topics.
 bag=subprocess.Popen(['ros2','bag','record','-o',str(root/'bag'),'--max-bag-duration','2','--max-bag-size','268435456','--storage-preset-profile','resilient','/tank/pose'],stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
 until=time.monotonic()+4
 while time.monotonic()<until and bag.poll() is None:rclpy.spin_once(node,timeout_sec=.05)
 bag.send_signal(signal.SIGINT);_,err=bag.communicate(timeout=15)
 assert bag.returncode==0,err.decode()
 assert (root/'bag/metadata.yaml').exists()
 print('RESILIENT_SPLIT_BAG_OK',flush=True)
 # A detached supervisor with a fake child survives SIGHUP and forwards SIGINT.
 worker_code='''import sys,os,signal,time
from pathlib import Path
from isv_water_trials import tank_session as s
s.ROOT=Path(sys.argv[1]);s.ROOT.mkdir(exist_ok=True)
s.command_for=lambda *args:[sys.executable,'-c',"import signal,time,sys;signal.signal(signal.SIGINT,lambda *a:sys.exit(0));time.sleep(60)"]
s.worker('trial',[])
'''
 worker=subprocess.Popen([sys.executable,'-c',worker_code,str(root/'runtime')],start_new_session=True,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
 time.sleep(1);worker.send_signal(signal.SIGHUP);time.sleep(.3);assert worker.poll() is None
 worker.send_signal(signal.SIGINT);_,err=worker.communicate(timeout=8);assert worker.returncode==0,err.decode()
 assert not (root/'runtime/trial.json').exists()
 print('DETACHED_SESSION_HUP_AND_STOP_OK',flush=True)
node.destroy_node();rclpy.shutdown()
