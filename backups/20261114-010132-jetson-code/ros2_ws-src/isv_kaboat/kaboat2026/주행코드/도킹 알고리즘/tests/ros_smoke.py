"""Indoor smoke check: mapping start/pause services; no sensor or actuator writes.

Run only with the mapping node already started and no valid GPS expected.
"""
import json
import time
import rclpy
from std_msgs.msg import String
from std_srvs.srv import Trigger


def main():
    rclpy.init();node=rclpy.create_node('docking_mapping_smoke')
    latest={}
    sub=node.create_subscription(String,'/docking/mapping_status',
        lambda message:latest.update(json.loads(message.data)),1)
    def until(predicate,seconds=5):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.1)
            if predicate():return
        raise AssertionError('Timed out; latest='+json.dumps(latest))
    def command(action,require_success=True):
        client=node.create_client(Trigger,'/docking/mapping/'+action)
        assert client.wait_for_service(timeout_sec=5),action
        future=client.call_async(Trigger.Request())
        until(future.done)
        result=future.result()
        if require_success:assert result.success,result.message
        node.destroy_client(client)
        return result
    try:
        until(lambda:bool(latest))
        assert latest['mode']=='PAUSED','Smoke test requires a fresh paused mapper'
        result=command('start',require_success=False)
        if result.success:
            until(lambda:latest.get('mode')=='ACTIVE')
            until(lambda:latest.get('state') in ('WAIT_VALID_GPS','WAIT_RC','WAIT_GPS_COVARIANCE','WAIT_EXTRINSICS'))
        else:
            assert 'RC_NOT_READY' in result.message
            until(lambda:latest.get('mode')=='PAUSED')
            print('NON_RC_START_REJECTED:',result.message)
        assert latest.get('origin_lla') is None
        assert latest.get('map_points')==0
        print('INDOOR_MAPPING_STATE:',json.dumps(latest,sort_keys=True))
        command('pause')
        until(lambda:latest.get('mode')=='PAUSED')
        topics=dict(node.get_topic_names_and_types())
        services=dict(node.get_service_names_and_types())
        for name,kind in [('lidar_map','sensor_msgs/msg/PointCloud2'),
                          ('current_scan_map','sensor_msgs/msg/PointCloud2'),
                          ('gps_pose','geometry_msgs/msg/PoseStamped'),
                          ('corrected_pose','geometry_msgs/msg/PoseStamped'),
                          ('rc_path','nav_msgs/msg/Path'),('map_origin','sensor_msgs/msg/NavSatFix'),
                          ('mapping_status','std_msgs/msg/String'),
                          ('mapping_diagnostics','diagnostic_msgs/msg/DiagnosticArray')]:
            assert kind in topics.get('/docking/'+name,[]),(name,topics)
        for action in ('start','pause','resume','stop','save','load','reset'):
            assert 'std_srvs/srv/Trigger' in services.get('/docking/mapping/'+action,[])
        print('ROS_TOPICS_AND_SERVICES_OK; mapper left PAUSED; no map fabricated')
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
