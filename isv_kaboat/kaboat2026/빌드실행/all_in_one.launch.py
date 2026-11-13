"""Full or split launch: equipment/perception and a separately owned mission.

The full stack may auto-start; a split mission always waits for operator start.
Ctrl+C shuts down this launch's own
processes; it does not use pkill or edit any existing configuration files.
"""
import os
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction, ExecuteProcess, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from isv_water_trials.launch_once import node as Node, once, GPS_EXECUTABLES


def assemble(context):
    from launcher_check import settings,validate
    root=Path(__file__).resolve().parent.parent
    mode=LaunchConfiguration('mode').perform(context)
    components=LaunchConfiguration('components').perform(context)
    external_sensors=LaunchConfiguration('external_sensors').perform(context).lower()=='true'
    if components=='bringup':mode='preview'
    gui=LaunchConfiguration('gui').perform(context)
    windows_gui=os.environ.get('KABOAT_GUI')=='windows'
    options=settings(root)
    validate(options,mode,devices=False)
    params=Path(options['params'])
    data=yaml.safe_load(params.read_text())['course_mission']['ros__parameters']
    def include(package,name,args):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory(package))/'launch'/name)),launch_arguments=args.items())
    sensors=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(root/'빌드실행/sensors_one_command.launch.py')),launch_arguments={
        'camera_device':options['camera'],'gps_device':options['gps'],
        'imu_device':options['imu'],'arduino_device':options['arduino'],
        'velodyne_model':options['lidar_model'],'velodyne_ip':options['lidar_ip']}.items())
    yolo=include('yolo_bringup','yolo.launch.py',{
        'model':options['model'],'input_image_topic':'/camera/image_raw/compressed',
        'image_reliability':'2','device':'cuda:0','use_tracking':'True',
        'use_3d':'False','use_debug':'True','threshold':str(data.get('score_threshold',.65))})
    fusion=include('fusion_viewer','fusion_viewer.launch.py',{
        'image_topic':'/camera/image_raw','detections_topic':'/yolo/tracking',
        'cloud_topic':'/velodyne_points','calib_file':options['calibration'],
        'enable_projection':'true','force_projection_without_time_match':'false',
        'max_age_ms':'250'})
    if windows_gui or components=='bringup':
        fusion=ExecuteProcess(cmd=['/usr/bin/python3',str(root/'빌드실행/fusion_headless.py'),'--ros-args',
            '-p','image_topic:=/camera/image_raw','-p','detections_topic:=/yolo/tracking',
            '-p','cloud_topic:=/velodyne_points','-p','calib_file:='+options['calibration'],
            '-p','enable_projection:=true','-p','force_projection_without_time_match:=false',
            '-p','max_age_ms:=250'],output='screen')
    fusion=once('camera_lidar_fusion',('fusion_headless.py','fusion_viewer'),[fusion])
    from launcher_check import measured_calibration
    calibrated=measured_calibration(options)
    docking=Node(package='isv_shape_navigation',executable='docking_camera',
        name='docking_camera',output='screen',
        parameters=[str(root/'설정/docking_camera.yaml')])
    mission=Node(package='isv_shape_navigation',executable='course_mission',name='course_mission',output='screen',
        parameters=[str(params),dict(enable_output=mode=='autonomy',gui_backend='web' if gui=='web' else 'desktop',
                                   gui_enabled=(not windows_gui if gui=='auto' else gui!='none'),
                                   dock_calibration_verified=calibrated)])
    if components=='bringup':
        actions=[LogInfo(msg='Equipment bringup only: YOLO + fusion. No mission or auto-start.' if external_sensors else 'Equipment bringup only: sensors + YOLO + fusion. No mission or auto-start.')]
        if not external_sensors:actions.append(sensors)
        actions.extend([TimerAction(period=3.,actions=[yolo]),TimerAction(period=5.,actions=[fusion]),
                        TimerAction(period=6.,actions=[docking])])
        return actions
    if components=='mission':
        return [LogInfo(msg='Mission only: reusing equipment topics. GUI start required; mode='+mode),
                mission,ExecuteProcess(cmd=['/usr/bin/python3',str(root/'빌드실행/launcher_check.py'),'--monitor'],output='screen')]
    # Delays stagger GPU/device startup; they are not sensor-readiness proofs.
    # The mission controller's existing data/RC/AUTO/GUI start gates still apply.
    return [LogInfo(msg=('Windows GUI: waiting for operator start. ' if windows_gui else '')+'All-in-one mode='+mode+('; GUI start required. Dock calibration=' if windows_gui else '; automatic start once RC AUTO and navigation inputs are ready. Dock calibration=')+str(calibrated)),
            sensors,TimerAction(period=3.,actions=[yolo]),TimerAction(period=5.,actions=[fusion]),
            TimerAction(period=6.,actions=[docking]),
            TimerAction(period=7.,actions=[mission]),
            ExecuteProcess(cmd=['/usr/bin/python3',str(root/'빌드실행/launcher_check.py'),
                                '--auto-start' if mode=='autonomy' and not windows_gui else '--monitor'],output='screen')]


def generate_launch_description():
    # Import sibling helper without changing the installed underlay packages.
    import sys
    root=str(Path(__file__).resolve().parent)
    if root not in sys.path:sys.path.insert(0,root)
    return LaunchDescription([DeclareLaunchArgument('mode',default_value='autonomy',choices=['preview','autonomy']),
                              DeclareLaunchArgument('components',default_value='all',choices=['all','bringup','mission']),
                              DeclareLaunchArgument('gui',default_value='auto',choices=['auto','desktop','web','none']),
                              DeclareLaunchArgument('external_sensors',default_value='false',choices=['true','false']),
                              OpaqueFunction(function=assemble)])
