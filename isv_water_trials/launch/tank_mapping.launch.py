"""LiDAR-only odometry and 2D tank mapping/localization. No actuator node."""
import os
import hashlib
import uuid
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def nodes(context):
    import fcntl
    global runtime_lock
    runtime_lock = open('/tmp/kabot_tank_slam_'+os.environ.get('ROS_DOMAIN_ID','0')+'.lock','w')
    try: fcntl.flock(runtime_lock,fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError: raise RuntimeError('tank_mapping is already running in this ROS domain')
    mode = LaunchConfiguration('mode').perform(context)
    filename = os.path.expanduser(LaunchConfiguration('map_file').perform(context))
    if mode not in ('mapping', 'localization'):
        raise ValueError('mode must be mapping or localization')
    if mode == 'localization' and not os.path.isfile(filename + '.posegraph'):
        raise ValueError('Saved .posegraph is required: ' + filename)
    params = os.path.join(get_package_share_directory('isv_water_trials'), 'config', 'tank_slam.yaml')
    settings = {'mode': mode, 'use_sim_time': False}
    if mode == 'localization':
        settings.update(map_file_name=filename, map_start_at_dock=True)
    session_id = hashlib.sha256(Path(filename+'.posegraph').read_bytes()).hexdigest() if mode=='localization' else uuid.uuid4().hex
    return [
        Node(package='isv_water_trials', executable='tank_pose', output='screen',
             parameters=[{'session_id':session_id,'mode':mode}]),
        Node(package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node',
             name='tank_laser_odometry', output='log', arguments=['--ros-args','--log-level','warn'], parameters=[{
                 'laser_scan_topic': '/scan', 'odom_topic': '/tank/odom', 'publish_tf': True,
                 'base_frame_id': 'velodyne', 'odom_frame_id': 'tank_odom',
                 'init_pose_from_topic': '/tank/init_odom', 'freq': 10.0}]),
        Node(package='slam_toolbox', name='slam_toolbox',
             executable='async_slam_toolbox_node' if mode == 'mapping' else 'localization_slam_toolbox_node',
             output='log', parameters=[params, settings])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mapping'),
        DeclareLaunchArgument('map_file', default_value='~/water_trials/tank_maps/tank'),
        OpaqueFunction(function=nodes)])
