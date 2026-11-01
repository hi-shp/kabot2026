"""LiDAR-only odometry and 2D tank mapping/localization. No actuator node."""
import os
import hashlib
import uuid
import json
import math
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
    from isv_water_trials.tank_map_file import ACTIVE,materialize
    fixed_file=None;fixed_id=None;seed=None
    if mode=='auto':
        if filename:
            mode='localization'
        elif ACTIVE.exists():
            filename=json.loads(ACTIVE.read_text(encoding='utf-8'))['file'];mode='localization'
        else:mode='mapping'
    if mode=='localization' and not filename and ACTIVE.exists():
        filename=json.loads(ACTIVE.read_text(encoding='utf-8'))['file']
    if mode=='localization' and filename.endswith('.tankmap'):
        fixed_file=filename;filename,fixed_id,manifest=materialize(fixed_file);seed=manifest.get('seed')
        last_pose=Path.home()/'water_trials/tank/last_pose.json'
        if last_pose.exists():
            try:
                previous=json.loads(last_pose.read_text(encoding='utf-8'))
                pose=previous.get('pose')
                if previous.get('map_id')==fixed_id and isinstance(pose,list) and len(pose)==3 and all(isinstance(v,(int,float)) and math.isfinite(v) for v in pose):seed=pose
            except (OSError,ValueError,TypeError):
                pass
    if mode not in ('mapping', 'localization'):
        raise ValueError('mode must be mapping or localization')
    if mode == 'localization' and not os.path.isfile(filename + '.posegraph'):
        raise ValueError('Saved .posegraph is required: ' + filename)
    params = os.path.join(get_package_share_directory('isv_water_trials'), 'config', 'tank_slam.yaml')
    settings = {'mode': mode, 'use_sim_time': False}
    if mode == 'localization':
        settings.update(map_file_name=filename,map_start_at_dock=False,map_start_pose=seed or [0.,0.,0.],
                        minimum_travel_distance=0.0,minimum_travel_heading=0.0)
    session_id = fixed_id or (hashlib.sha256(Path(filename+'.posegraph').read_bytes()).hexdigest() if mode=='localization' else uuid.uuid4().hex)
    result = [
        Node(package='isv_water_trials', executable='tank_pose', output='screen',
             parameters=[{'session_id':session_id,'mode':mode,'fixed_map':bool(fixed_file),'map_file':fixed_file or filename}]),
        Node(package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node',
             name='tank_laser_odometry', output='log', arguments=['--ros-args','--log-level','warn'], parameters=[{
                 'laser_scan_topic': '/scan', 'odom_topic': '/tank/odom', 'publish_tf': True,
                 'base_frame_id': 'velodyne', 'odom_frame_id': 'tank_odom',
                 'init_pose_from_topic': '/tank/init_odom', 'freq': 10.0}]),
        Node(package='slam_toolbox', name='slam_toolbox',
             executable='async_slam_toolbox_node' if mode == 'mapping' else 'localization_slam_toolbox_node',
             output='log', parameters=[params, settings],remappings=[('/map','/tank/localization_map')] if fixed_file else [])]
    if fixed_file:
        result.append(Node(package='isv_water_trials',executable='tank_fixed_map',output='log',parameters=[{'file':fixed_file}]))
        result.append(Node(package='isv_water_trials',executable='tank_localize',output='log',
                           parameters=[{'file':fixed_file,'automatic':True,
                                        'accept_score':float(LaunchConfiguration('localization_score').perform(context)),
                                        'accept_margin':float(LaunchConfiguration('localization_margin').perform(context))}],
                           additional_env={'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1'}))
    if mode=='mapping' and LaunchConfiguration('autosave').perform(context).lower()=='true':
        result.append(Node(package='isv_water_trials',executable='tank_autosave',output='log'))
    return result


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='auto'),
        DeclareLaunchArgument('map_file', default_value=''),
        DeclareLaunchArgument('autosave', default_value='true'),
        DeclareLaunchArgument('localization_score', default_value='0.80'),
        DeclareLaunchArgument('localization_margin', default_value='0.05'),
        OpaqueFunction(function=nodes)])
