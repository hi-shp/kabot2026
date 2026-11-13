"""Domain 151 only: synthetic AUTO -> registration -> hop -> GPS jump -> loss.

Never creates actuator clients. Production domain/configuration are not changed.
"""
import os
import json
import tempfile
import time
from pathlib import Path
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String,Header
from sensor_msgs.msg import NavSatFix,Imu,PointCloud2,PointField
from sensor_msgs_py import point_cloud2
from std_srvs.srv import Trigger
from docking_mapping.node import DockingMapping
from docking_mapping.geometry import ENU,apply
from docking_mapping.hopping import HoppingPoint,write_hopping
from docking_mapping.reference import ReferenceMap,digest
from test_reference import ReferenceTests
from test_mapping import config,structure


def main():
    assert os.environ.get('ROS_DOMAIN_ID')=='151','Synthetic inputs require isolated domain 151'
    with tempfile.TemporaryDirectory(prefix='docking_auto_test_') as directory:
        c=config();c['sensors'].update(gps_topic='/auto_sim/fix',imu_topic='/auto_sim/imu',
            lidar_topic='/auto_sim/cloud',rc_status_topic='/auto_sim/rc')
        # Exercise actual GICP but cap synthetic workload, not production quality gates.
        c['autonomy']['tracking']['max_points']=900
        c['autonomy']['tracking_timeout_s']=2.
        filename=Path(directory)/'config.yaml';filename.write_text(yaml.safe_dump(c))
        rclpy.init(args=['--ros-args','-p','config_file:='+str(filename)])
        mapper=DockingMapping();fixture=Node('docking_auto_test_sensors')
        executor=SingleThreadedExecutor();executor.add_node(mapper);executor.add_node(fixture)
        base,roi=ReferenceTests().fixture();base.config['sensors']=c['sensors']
        ref=ReferenceMap.from_mapper(base,roi)
        original_hash=digest(ref.points)
        hop=HoppingPoint(ref.reference_id,ref.version,tuple(apply([[0.,0.,0.]],np.linalg.inv(ref.enu_from_dock))[0]),
            radius_m=1.,hold_seconds=60.,heading_dock_deg=-27.)
        mapper.maps_dir=Path(directory)/'maps';mapper.maps_dir.mkdir()
        ref.save(mapper.maps_dir,companion_writer=lambda folder:write_hopping(folder,ref,hop))
        try:mapper.execute_command('load')
        except Exception:
            mapper.close();mapper.destroy_node();fixture.destroy_node();rclpy.shutdown();raise
        mode=2;jump=np.zeros(3);empty=False;fix_valid=True;last_publish=-1.;latest={}
        pubs=[fixture.create_publisher(kind,topic,1) for kind,topic in [(NavSatFix,'/auto_sim/fix'),
            (Imu,'/auto_sim/imu'),(PointCloud2,'/auto_sim/cloud'),(String,'/auto_sim/rc')]]
        sub=fixture.create_subscription(String,'/docking/map_preview',lambda m:latest.update(json.loads(m.data)),1)
        points=structure().astype(np.float32)
        fields=[PointField(name=name,offset=i*4,datatype=PointField.FLOAT32,count=1)
                for i,name in enumerate(('x','y','z','time'))]

        def spin_until(predicate,seconds=15.):
            nonlocal last_publish
            deadline=time.monotonic()+seconds
            while time.monotonic()<deadline:
                now=time.monotonic()
                if now-last_publish>.1:
                    stamp=fixture.get_clock().now().to_msg()
                    lla=ENU(ref.origin_lla).to_lla(jump)
                    fix=NavSatFix(header=Header(stamp=stamp,frame_id='gps'))
                    fix.status.status=0 if fix_valid else -1;fix.latitude,fix.longitude,fix.altitude=map(float,lla)
                    fix.position_covariance_type=2;fix.position_covariance=[.01,0.,0.,0.,.01,0.,0.,0.,.01]
                    imu=Imu(header=Header(stamp=stamp,frame_id='vectornav'));imu.orientation.w=1.
                    data=np.empty((0,4),np.float32) if empty else np.column_stack([
                        points,np.linspace(0.,.1,len(points),dtype=np.float32)])
                    pubs[0].publish(fix);pubs[1].publish(imu)
                    pubs[2].publish(point_cloud2.create_cloud(Header(stamp=stamp,frame_id='velodyne'),fields,data))
                    pubs[3].publish(String(data=json.dumps(dict(mode=mode,receiving=True,estop_active=False))))
                    last_publish=now
                executor.spin_once(timeout_sec=.01)
                if predicate():return
            raise AssertionError('Timed out: '+json.dumps(mapper.autonomy.snapshot()))

        def service(action):
            client=fixture.create_client(Trigger,'/docking/autonomy/'+action)
            assert client.wait_for_service(timeout_sec=3)
            future=client.call_async(Trigger.Request());spin_until(future.done)
            assert future.result().success,future.result().message
            fixture.destroy_client(client)

        try:
            spin_until(lambda:mapper.autonomy.snapshot()['auto_ready'])
            service('start')
            spin_until(lambda:mapper.autonomy.snapshot()['state']=='HOPING')
            first=mapper.autonomy.snapshot()['relative_goal']['point_base']
            spin_until(lambda:bool(latest.get('current_scan')))
            assert latest['status']['mode']=='DRY_RUN'
            with mapper.autonomy.lock:world=mapper.autonomy.world_from_dock.copy()
            jump[:]=[2.,3.,0.]
            spin_until(lambda:mapper.autonomy.raw_pose is not None and np.linalg.norm(mapper.autonomy.raw_pose[:3,3]-jump)<.05)
            after=mapper.autonomy.snapshot()['relative_goal']['point_base']
            np.testing.assert_allclose(after,first,atol=.03)
            with mapper.autonomy.lock:new_world=mapper.autonomy.world_from_dock.copy()
            np.testing.assert_allclose(new_world[:3,3]-world[:3,3],jump,atol=.04)
            assert digest(mapper.reference.points)==original_hash
            assert mapper.autonomy.output.clients==[] and not mapper.autonomy.output.enabled
            fixed_world=new_world.copy();anchor_stamp=mapper.autonomy.world_stamp
            fix_valid=False
            spin_until(lambda:mapper.autonomy.sample is not None and mapper.autonomy.sample.stamp>anchor_stamp+1.)
            np.testing.assert_allclose(mapper.autonomy.world_from_dock,fixed_world,atol=.04)
            assert mapper.autonomy.world_stamp<=anchor_stamp+.4
            assert mapper.autonomy.snapshot()['state']=='HOPING'
            fix_valid=True
            mode=1;spin_until(lambda:mapper.autonomy.snapshot()['state']=='ABORT')
            assert mapper.autonomy.snapshot()['pwm']==[1500,1500]
            mode=2;spin_until(lambda:mapper.autonomy.snapshot()['auto_ready'])
            assert mapper.autonomy.snapshot()['state']=='ABORT','AUTO return must not resume navigation'
            # Restart needs an explicit operator command. Restore the original GPS
            # anchor because a new run must seed correspondence from GPS again.
            jump[:]=0.;spin_until(lambda:mapper.gps and abs(mapper.gps[-1].latitude-ref.origin_lla[0])<1e-9)
            service('start');spin_until(lambda:mapper.autonomy.snapshot()['state']=='HOPING')
            empty=True
            spin_until(lambda:mapper.autonomy.snapshot()['state']=='TRACKING_LOST')
            assert mapper.autonomy.snapshot()['pwm']==[1500,1500]
            empty=False
            spin_until(lambda:bool(latest.get('status',{}).get('autonomy')))
            assert mapper.autonomy.snapshot()['state']=='TRACKING_LOST'
            service('emergency');assert mapper.autonomy.snapshot()['state']=='EMERGENCY'
            mode=1;spin_until(lambda:not mapper.autonomy.snapshot()['auto_ready'])
            generation=mapper.autonomy.generation
            mapper.autonomy.stop('cancel queued start')
            mode=2;spin_until(lambda:mapper.autonomy.snapshot()['auto_ready'])
            try:mapper.autonomy.start(generation)
            except ValueError as exc:assert 'cancelled' in str(exc)
            else:raise AssertionError('Cancelled start must not restart AUTO')
            print('ROS_AUTO_OK: GICP -> HOPING; GPS +2E/+3N display shift; relative goal unchanged; immutable reference; lost -> neutral; emergency; stale start cancelled; zero actuator clients',flush=True)
        finally:
            mapper.close();executor.remove_node(mapper);executor.remove_node(fixture)
            fixture.destroy_node();mapper.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
