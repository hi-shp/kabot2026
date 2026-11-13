"""RC start/loss/resume/finish and reference save/load in isolated domain 151."""
import json
import os
from pathlib import Path
import tempfile
import time
import uuid
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix,Imu,PointCloud2,PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String,Header
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.parameter import Parameter
import yaml
from docking_mapping.node import DockingMapping
from test_mapping import config,structure


def main():
    assert os.environ.get('ROS_DOMAIN_ID')=='151','Use isolated ROS_DOMAIN_ID=151'
    with tempfile.TemporaryDirectory(prefix='reference_workflow_') as directory:
        c=config()
        c['sensors'].update(gps_topic='/mapping_sim/fix',imu_topic='/mapping_sim/imu',
                            lidar_topic='/mapping_sim/cloud',rc_status_topic='/mapping_sim/rc')
        filename=Path(directory)/'config.yaml';filename.write_text(yaml.safe_dump(c))
        rclpy.init(args=['--ros-args','-p','config_file:='+str(filename)])
        mapper=DockingMapping();fixture=Node('reference_sim_sensors')
        mapper.maps_dir=Path(directory)/'maps'
        executor=SingleThreadedExecutor();executor.add_node(mapper);executor.add_node(fixture)
        latest={};seen={};preview={};rc_mode=1;last_publish=-1.
        def status(message):latest.clear();latest.update(json.loads(message.data))
        def reference_cb(message):
            if message.width*message.height:
                assert message.header.frame_id=='dock_ref'
                seen['reference_points']=message.width*message.height
        subs=[fixture.create_subscription(String,'/docking/mapping_status',status,1),
              fixture.create_subscription(String,'/docking/map_preview',lambda m:preview.update(json.loads(m.data)),1),
              fixture.create_subscription(PointCloud2,'/docking/reference_map',reference_cb,1),
              fixture.create_subscription(PoseStamped,'/docking/map_pose',lambda m:seen.update(map_pose=True),1)]
        gps=fixture.create_publisher(NavSatFix,'/mapping_sim/fix',1)
        imu=fixture.create_publisher(Imu,'/mapping_sim/imu',1)
        cloud=fixture.create_publisher(PointCloud2,'/mapping_sim/cloud',1)
        rc=fixture.create_publisher(String,'/mapping_sim/rc',1)
        points=structure().astype(np.float32)
        timed=np.column_stack([points,np.linspace(0.,.1,len(points),dtype=np.float32)])
        fields=[PointField(name=name,offset=4*i,datatype=PointField.FLOAT32,count=1) for i,name in enumerate(('x','y','z','time'))]
        def spin_until(predicate,seconds=12):
            nonlocal last_publish
            deadline=time.monotonic()+seconds
            while time.monotonic()<deadline:
                now=time.monotonic()
                if now-last_publish>.12:
                    stamp=fixture.get_clock().now().to_msg()
                    fix=NavSatFix(header=Header(stamp=stamp,frame_id='gps'))
                    fix.status.status=0;fix.latitude,fix.longitude,fix.altitude=35.12,129.10,10.
                    fix.position_covariance=[.01,0.,0.,0.,.01,0.,0.,0.,.01];fix.position_covariance_type=2
                    attitude=Imu(header=Header(stamp=stamp,frame_id='vectornav'));attitude.orientation.w=1.
                    gps.publish(fix);imu.publish(attitude)
                    rc.publish(String(data=json.dumps(dict(mode=rc_mode,receiving=True,estop_active=False))))
                    cloud.publish(point_cloud2.create_cloud(Header(stamp=stamp,frame_id='velodyne'),fields,timed))
                    last_publish=now
                executor.spin_once(timeout_sec=.02)
                if predicate():return
            raise AssertionError('Timed out: '+json.dumps(latest))
        def command(action,expected=True):
            client=fixture.create_client(Trigger,'/docking/mapping/'+action)
            assert client.wait_for_service(timeout_sec=5)
            future=client.call_async(Trigger.Request());spin_until(future.done)
            result=future.result();assert result.success is expected,result.message
            fixture.destroy_client(client)
            return result
        def select_structure(expected):
            client=fixture.create_client(SetParametersAtomically,'/docking_mapping/set_parameters_atomically')
            assert client.wait_for_service(timeout_sec=5)
            request=SetParametersAtomically.Request(parameters=[
                Parameter('reference_roi_min_enu_m',value=[-4.,-4.,-2.]).to_parameter_msg(),
                Parameter('reference_roi_max_enu_m',value=[5.,6.,4.]).to_parameter_msg(),
                Parameter('reference_yaw_enu_deg',value=20.).to_parameter_msg()])
            future=client.call_async(request);spin_until(future.done)
            result=future.result().result
            assert result.successful is expected,result.reason
            fixture.destroy_client(client)
        def gui_command(action,expected=True,**fields):
            client=fixture.create_client(SetParametersAtomically,'/docking/mapping/gui_command')
            assert client.wait_for_service(timeout_sec=5)
            identity=uuid.uuid4().hex
            payload=dict(action=action,request_id=identity,**fields)
            request=SetParametersAtomically.Request(parameters=[Parameter('ui_request',value=json.dumps(payload)).to_parameter_msg()])
            future=client.call_async(request);spin_until(future.done)
            assert future.result().result.successful,future.result().result.reason
            spin_until(lambda:(latest.get('last_gui_result') or {}).get('request_id')==identity)
            assert latest['last_gui_result']['success'] is expected,latest['last_gui_result']
            fixture.destroy_client(client)
        try:
            spin_until(lambda:mapper.rc_ready() and bool(latest))
            assert mapper.session.state=='IDLE' and not mapper.mapper.map.cells
            command('start')
            spin_until(lambda:latest.get('state')=='CONVERGED')
            select_structure(False)
            rc_mode=2
            spin_until(lambda:latest.get('initial_mapping_state')=='PAUSED_RC_LOST')
            paused_points=mapper.mapper.map.points().copy()
            rc_mode=1
            began=time.monotonic();spin_until(lambda:time.monotonic()-began>1.2)
            assert not mapper.session.active
            np.testing.assert_array_equal(mapper.mapper.map.points(),paused_points)
            command('resume');spin_until(lambda:latest.get('state')=='CONVERGED' and latest.get('mode')=='ACTIVE')
            command('stop');spin_until(lambda:latest.get('initial_mapping_state')=='FINISHED')
            select_structure(True)
            command('save')
            spin_until(lambda:latest.get('phase')=='REFERENCE_READY' and seen.get('reference_points',0)>100 and seen.get('map_pose'))
            identity=mapper.reference.reference_id;before=mapper.reference.points.copy()
            assert mapper.reference.points.flags.writeable is False
            select_structure(False)
            command('load');spin_until(lambda:latest.get('command_result','').startswith('Loaded reference'))
            assert mapper.reference.reference_id==identity and mapper.reference.version==1
            np.testing.assert_array_equal(mapper.reference.points,before)
            assert not mapper.session.active
            binding=dict(reference_id=identity,reference_version=1)
            point=mapper.reference.enu_from_dock[:3,3]+[1.,1.,0.]
            ll=mapper.mapper.enu.to_lla(point)
            gui_command('set_hopping',**binding,latitude=float(ll[0]),longitude=float(ll[1]),height_enu_m=float(point[2]),
                name='test_hop',radius_m=1.,hold_seconds=5.,heading_dock_deg=None)
            spin_until(lambda:preview.get('hopping') is not None)
            hop=mapper.hopping
            assert hop.reference_id==identity and mapper.hopping_dirty
            gui_command('save_hopping',**binding)
            assert (mapper.reference_folder/'hopping_point.yaml').is_file() and not mapper.hopping_dirty
            gui_command('load');assert mapper.hopping==hop
            gui_command('delete_hopping',expected=False,reference_id='stale',reference_version=1)
            assert mapper.hopping==hop
            gui_command('delete_hopping',**binding);assert mapper.hopping is None
            gui_command('save_hopping',**binding);gui_command('load');assert mapper.hopping is None
            print('ISOLATED_REFERENCE_WORKFLOW_OK',json.dumps({'status':latest,'outputs':seen},sort_keys=True))
        finally:
            mapper.close();executor.remove_node(mapper);executor.remove_node(fixture)
            mapper.destroy_node();fixture.destroy_node();executor.shutdown();rclpy.shutdown()


if __name__=='__main__':main()
