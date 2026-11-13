"""Shared filtered cloud upstream of every scan/obstacle consumer."""
import copy
import json
import math
from pathlib import Path
import time
from .lidar_self_shape import ShapeCapture, ShapeMask, save_shape, cloud_arrays
from .lidar_motion import MotionDeskew


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import String
    from vectornav_msgs.msg import CommonGroup
    from geometry_msgs.msg import TwistWithCovarianceStamped

    class Filter(Node):
        def __init__(self):
            super().__init__('lidar_self_mask')
            self.root=Path.home()/'water_trials/lidar_shapes';self.mask=None;self.capture=None
            from ament_index_python.packages import get_package_share_directory
            from .outdoor_config import load_config
            config=load_config(Path(get_package_share_directory('isv_water_trials'))/'config/outdoor.json')
            self.motion=MotionDeskew(config.get('heading_offset_deg',0)+config.get('magnetic_declination_deg',0),config.get('lidar_yaw_deg',0))
            self.motion_status=dict(applied=False,reason='IMU 시각 이력 대기')
            self.yaw=None;self.yaw_at=0.;self.cloud_at=0.;self.error='';self.ack={};self.removed=0;self.total=0;self.header=None
            try:
                active=json.loads((self.root/'active.json').read_text())
                if active.get('file'):
                    path=self.root/Path(active['file']).name
                    self.mask=ShapeMask(json.loads(path.read_text()))
            except FileNotFoundError:pass
            except (ValueError,KeyError,OSError) as exc:self.error=str(exc)
            # Match the original transform publisher: laserscan requests RELIABLE.
            self.pub=self.create_publisher(PointCloud2,'/velodyne_points',10)
            self.preview=self.create_publisher(PointCloud2,'/lidar/self_shape',qos_profile_sensor_data)
            self.status=self.create_publisher(String,'/lidar/self_mask/status',10)
            latest_cloud=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,history=HistoryPolicy.KEEP_LAST)
            self.motion_callbacks=MutuallyExclusiveCallbackGroup()
            self.create_subscription(PointCloud2,'/velodyne_points_raw',self.cloud,latest_cloud)
            self.create_subscription(String,'/navigation/heading',self.heading,10)
            self.create_subscription(CommonGroup,'/vectornav/raw/common',self.raw_imu,qos_profile_sensor_data,callback_group=self.motion_callbacks)
            self.create_subscription(TwistWithCovarianceStamped,'/gps/fix_velocity',self.velocity,qos_profile_sensor_data,callback_group=self.motion_callbacks)
            self.create_subscription(String,'/lidar/self_mask/request',self.request,10)
            self.create_timer(.5,self.report)

        def raw_imu(self,msg):
            if msg.group_fields & 0x0008:
                self.motion.imu(msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,msg.yawpitchroll.x,
                                msg.angularrate.z if msg.group_fields & 0x0020 else None)

        def velocity(self,msg):
            self.motion.velocity(msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
                                 msg.twist.twist.linear.x,msg.twist.twist.linear.y,
                                 math.sqrt(max(0.,msg.twist.covariance[0],msg.twist.covariance[7])))

        def heading(self,msg):
            try:self.yaw=math.radians(float(json.loads(msg.data)['heading_deg']));self.yaw_at=time.monotonic()
            except (ValueError,KeyError,TypeError):self.yaw=None

        def request(self,msg):
            data={};ok=False
            try:
                data=json.loads(msg.data);action=data['action']
                if action=='hull_start':self.capture=ShapeCapture()
                elif action=='hull_stop':self.capture=None
                elif action=='hull_save':
                    if not self.capture:raise ValueError('배 형상 기록을 먼저 시작하세요')
                    if time.monotonic()-self.cloud_at>1 or time.monotonic()-self.yaw_at>1:
                        raise ValueError('라이다·IMU 최신 수신을 기다려 주세요')
                    self.mask=save_shape(self.root,data.get('name'),self.capture);self.capture=None
                elif action=='hull_disable':
                    self.root.mkdir(parents=True,exist_ok=True)
                    temp=self.root/'active.incoming';temp.write_text('{"file":null}');temp.replace(self.root/'active.json');self.mask=None
                else:raise ValueError('알 수 없는 배 형상 요청')
                self.error='';ok=True
            except (ValueError,KeyError,TypeError,OSError) as exc:self.error=str(exc)
            self.ack=dict(id=data.get('request_id'),ok=ok,error=self.error)
            self.report()

        def cloud(self,msg):
            self.header=msg.header;self.cloud_at=time.monotonic();self.total=msg.height*msg.width;self.removed=0
            try:
                xyz,records=cloud_arrays(msg)
                if self.capture:
                    yaw=self.yaw if time.monotonic()-self.yaw_at<1 else None
                    self.capture.update(xyz,msg.header.frame_id,self.cloud_at,yaw)
                if self.mask:
                    keep=self.mask.keep(xyz,msg.header.frame_id);self.removed=int((~keep).sum())
                    xyz,records=xyz[keep],records[keep].copy()
                records,self.motion_status=self.motion.apply(xyz,records,msg.fields,
                    msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,msg.is_bigendian)
                if self.mask or self.motion_status['applied']:
                    out=copy.copy(msg);out.height=1;out.width=len(records);out.row_step=out.width*out.point_step
                    out.data=records.tobytes();out.is_dense=False;self.pub.publish(out)
                else:self.pub.publish(msg)
            except (ValueError,TypeError,KeyError) as exc:
                self.error=str(exc);self.pub.publish(msg)  # An invalid mask must not hide real obstacles.

        def report(self):
            import numpy as np
            now=time.monotonic();capture=self.capture.status() if self.capture else {}
            self.status.publish(String(data=json.dumps(dict(capturing=self.capture is not None,active=self.mask is not None,
                name=self.mask.name if self.mask else '',capture=capture,removed=self.removed,input_points=self.total,
                cloud_age_s=now-self.cloud_at,motion_deskew=self.motion_status,error=self.error,ack=self.ack),allow_nan=False)))
            if self.header is None:return
            points=self.capture.preview() if self.capture else self.mask.points if self.mask else np.empty((0,3))
            msg=PointCloud2();msg.header=copy.copy(self.header);msg.header.stamp=self.get_clock().now().to_msg()
            msg.height=1;msg.width=len(points);msg.fields=[PointField(name=k,offset=i*4,datatype=7,count=1) for i,k in enumerate(('x','y','z'))]
            msg.point_step=12;msg.row_step=12*len(points);msg.is_dense=True;msg.data=np.asarray(points,dtype='<f4').tobytes()
            self.preview.publish(msg)

    rclpy.init();node=Filter();executor=MultiThreadedExecutor(num_threads=2);executor.add_node(node)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:executor.shutdown();node.destroy_node();rclpy.try_shutdown()
