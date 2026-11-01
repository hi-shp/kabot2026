"""Seed RF2O with a valid quaternion; publish current LiDAR pose in tank_map."""
import math
import json
import time
from pathlib import Path


def known_position(info,cells,x,y):
    """Check the saved grid without clamping an invalid estimate onto the map."""
    if not all(math.isfinite(v) for v in (x,y)):return False
    qx,qy,qz,qw=info['orientation'];angle=math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
    dx=x-info['position'][0];dy=y-info['position'][1];co=math.cos(angle);si=math.sin(angle)
    ix=math.floor((co*dx+si*dy)/info['resolution']);iy=math.floor((-si*dx+co*dy)/info['resolution'])
    return 0<=ix<info['width'] and 0<=iy<info['height'] and 0<=cells[iy*info['width']+ix]<35


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String,Empty
    from tf2_ros import Buffer, TransformListener, TransformException
    rclpy.init()

    class TankPose(Node):
        def __init__(self):
            super().__init__('tank_pose')
            self.tf = Buffer(); self.listener = TransformListener(self.tf, self)
            self.seed = self.create_publisher(Odometry, '/tank/init_odom', 1)
            self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/tank/pose', 10)
            self.declare_parameter('session_id','unset'); self.declare_parameter('mode','mapping')
            self.declare_parameter('fixed_map',False);self.declare_parameter('map_file','')
            self.ready=not self.get_parameter('fixed_map').value
            self.reason='' if self.ready else '현재 위치와 방향 지정 필요'
            self.last_slam=None;self.map_info=None;self.map_cells=None
            if self.get_parameter('fixed_map').value:
                from .tank_map_file import read_bundle
                _,_,files=read_bundle(self.get_parameter('map_file').value)
                self.map_info=json.loads(files['grid.json']);self.map_cells=files['grid.bin']
            self.requested=None;self.matches=0;self.last_match=0;self.saved=0.
            self.create_subscription(PoseWithCovarianceStamped,'/initialpose',self.initial,1)
            self.create_subscription(PoseWithCovarianceStamped,'/pose',self.matched,10)
            self.create_subscription(Empty,'/tank/localize/request',self.relocalize,1)
            self.session_pub = self.create_publisher(String,'/tank/session',1)
            self.create_timer(1.,lambda:self.session_pub.publish(String(data=json.dumps({
                'session_id':self.get_parameter('session_id').value,'mode':self.get_parameter('mode').value,
                'fixed_map':self.get_parameter('fixed_map').value,'map_file':self.get_parameter('map_file').value,
                'localization_ready':self.ready,'localization_reason':self.reason}))))
            self.create_timer(.1, self.update)

        def relocalize(self,_):
            if self.get_parameter('fixed_map').value:
                self.ready=False;self.requested=None;self.matches=0;self.reason='자동 위치·방향 탐색 중'

        def initial(self,msg):
            if not self.get_parameter('fixed_map').value or msg.header.frame_id!='tank_map':return
            from .tank_core import yaw
            p=msg.pose.pose
            self.requested=(self.get_clock().now().nanoseconds,p.position.x,p.position.y,yaw(p.orientation))
            self.matches=0;self.last_match=0;self.ready=False;self.reason='지정 위치에서 라이다 정합 확인 중'

        def matched(self,msg):
            if msg.header.frame_id!='tank_map':return
            self.last_slam=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            if not self.requested:return
            from .tank_core import yaw,wrap
            stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec;p=msg.pose.pose
            when,x,y,angle=self.requested
            if stamp<=max(self.last_match,when+200_000_000):return
            self.last_match=stamp
            if math.hypot(p.position.x-x,p.position.y-y)<1. and abs(wrap(yaw(p.orientation)-angle))<.6:
                self.matches+=1
                if self.matches>=3:self.ready=True;self.requested=None;self.reason=''
            else:self.matches=0

        def update(self):
            seed = Odometry(); seed.header.stamp = self.get_clock().now().to_msg()
            seed.header.frame_id = 'tank_odom'; seed.child_frame_id = 'velodyne'
            seed.pose.pose.orientation.w = 1.0
            # RF2O consumes only its first seed; repeating also permits a node restart.
            self.seed.publish(seed)
            try:
                tf = self.tf.lookup_transform('tank_map', 'velodyne', rclpy.time.Time())
            except TransformException:
                return
            age = (self.get_clock().now() - rclpy.time.Time.from_msg(tf.header.stamp)).nanoseconds / 1e9
            if age < -.1 or age > .5:
                return
            q = tf.transform.rotation
            if not .9 < math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w) < 1.1:
                return
            msg = PoseWithCovarianceStamped(); msg.header = tf.header
            msg.pose.pose.position.x = tf.transform.translation.x
            msg.pose.pose.position.y = tf.transform.translation.y
            msg.pose.pose.orientation = q
            if self.get_parameter('fixed_map').value:
                if not self.ready:return
                fresh=self.last_slam is not None and -.1<=(self.get_clock().now().nanoseconds-self.last_slam)/1e9<1.
                valid=known_position(self.map_info,self.map_cells,msg.pose.pose.position.x,msg.pose.pose.position.y)
                if not fresh or not valid:
                    self.ready=False;self.reason='라이다 정합 수신 지연' if not fresh else '추정 위치가 지도 내 관측된 빈 공간을 벗어남'
                    return
            # SLAM provides no calibrated pose covariance here. Do not invent one.
            self.pose_pub.publish(msg)
            if self.ready and self.get_parameter('fixed_map').value and time.monotonic()-self.saved>2:
                from .tank_core import yaw
                from .tank_storage import atomic_json
                try:
                    atomic_json(Path.home()/'water_trials/tank/last_pose.json',{
                        'map_id':self.get_parameter('session_id').value,
                        'pose':[msg.pose.pose.position.x,msg.pose.pose.position.y,yaw(msg.pose.pose.orientation)]})
                    self.saved=time.monotonic()
                except OSError as exc:self.get_logger().warning('Pose checkpoint: '+str(exc))

    node = TankPose()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
