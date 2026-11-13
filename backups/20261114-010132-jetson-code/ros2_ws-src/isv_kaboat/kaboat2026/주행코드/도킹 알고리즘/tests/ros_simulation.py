"""Full mapping message path with synthetic sensors in an isolated ROS domain.

Use ROS_DOMAIN_ID=151, never the boat's domain 50. No propulsion interfaces.
"""
import json
import os
from pathlib import Path
import tempfile
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String, Header
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as RosPath
import yaml

from docking_mapping.node import DockingMapping
from test_mapping import config, structure


def main():
    assert os.environ.get('ROS_DOMAIN_ID')=='151','Use isolated ROS_DOMAIN_ID=151'
    with tempfile.TemporaryDirectory(prefix='mapping_ros_sim_') as directory:
        c=config()
        c['scan_motion']['fallback_duration_s']=.1  # Known synthetic scan duration.
        c['sensors'].update(gps_topic='/mapping_sim/fix',imu_topic='/mapping_sim/imu',
                            lidar_topic='/mapping_sim/cloud',rc_status_topic='/mapping_sim/rc')
        filename=Path(directory)/'config.yaml'
        filename.write_text(yaml.safe_dump(c))
        rclpy.init(args=['--ros-args','-p','config_file:='+str(filename)])
        mapper=DockingMapping();fixture=Node('mapping_sim_sensors')
        mapper.maps_dir=Path(directory)/'maps'
        executor=SingleThreadedExecutor();executor.add_node(mapper);executor.add_node(fixture)
        latest={};seen={}
        def cloud_cb(message):
            if message.width*message.height:
                points=point_cloud2.read_points_numpy(message,field_names=('x','y','z'))
                assert np.isfinite(points).all()
                seen['map_points']=message.width*message.height
        subs=[fixture.create_subscription(String,'/docking/mapping_status',lambda m:latest.update(json.loads(m.data)),1),
              fixture.create_subscription(PointCloud2,'/docking/lidar_map',cloud_cb,1),
              fixture.create_subscription(PoseStamped,'/docking/corrected_pose',lambda m:seen.update(pose=True),1),
              fixture.create_subscription(NavSatFix,'/docking/map_origin',lambda m:seen.update(origin=[m.latitude,m.longitude,m.altitude]),1),
              fixture.create_subscription(RosPath,'/docking/rc_path',lambda m:seen.update(path=len(m.poses)),1)]
        gps=fixture.create_publisher(NavSatFix,'/mapping_sim/fix',1)
        imu=fixture.create_publisher(Imu,'/mapping_sim/imu',1)
        cloud=fixture.create_publisher(PointCloud2,'/mapping_sim/cloud',1)
        rc=fixture.create_publisher(String,'/mapping_sim/rc',1)
        client=fixture.create_client(Trigger,'/docking/mapping/start')
        assert client.wait_for_service(timeout_sec=5)
        future=None
        try:
            started=time.monotonic();last_publish=-1.
            while time.monotonic()-started<15:
                now=time.monotonic()
                if now-last_publish>.15:
                    stamp=fixture.get_clock().now().to_msg()
                    fix=NavSatFix(header=Header(stamp=stamp,frame_id='gps'))
                    fix.status.status=0
                    fix.latitude,fix.longitude,fix.altitude=35.12,129.10,10.
                    fix.position_covariance=[.01,0.,0.,0.,.01,0.,0.,0.,.01]
                    fix.position_covariance_type=2
                    attitude=Imu(header=Header(stamp=stamp,frame_id='vectornav'))
                    attitude.orientation.w=1.
                    gps.publish(fix);imu.publish(attitude)
                    rc.publish(String(data=json.dumps(dict(mode=1,receiving=True,estop_active=False))))
                    cloud.publish(point_cloud2.create_cloud_xyz32(Header(stamp=stamp,frame_id='velodyne'),structure().astype(np.float32)))
                    last_publish=now
                executor.spin_once(timeout_sec=.02)
                if future is None and mapper.rc_ready():
                    future=client.call_async(Trigger.Request())
                if (latest.get('state')=='CONVERGED' and seen.get('map_points',0)>100
                        and seen.get('pose') and seen.get('path',0)>0 and 'origin' in seen):break
            assert future is not None and future.done() and future.result().success
            assert latest.get('state')=='CONVERGED',latest
            assert seen.get('map_points',0)>100 and seen.get('pose') and seen.get('path',0)>0,seen
            np.testing.assert_allclose(seen['origin'],[35.12,129.10,10.])
            print('ISOLATED_ROS_SIMULATION_OK',json.dumps({'status':latest,'outputs':seen},sort_keys=True))
        finally:
            mapper.close();executor.remove_node(mapper);executor.remove_node(fixture)
            mapper.destroy_node();fixture.destroy_node();executor.shutdown();rclpy.shutdown()


if __name__=='__main__':main()
