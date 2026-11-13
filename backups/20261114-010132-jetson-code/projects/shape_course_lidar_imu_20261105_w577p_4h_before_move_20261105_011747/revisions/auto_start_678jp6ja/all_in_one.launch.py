"""One supervised launch: sensors, perception, live checks, mission GUI.

Startup does not arm or start the vessel. Ctrl+C shuts down this launch's own
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


def assemble(context):
    from launcher_check import settings,validate
    root=Path(__file__).resolve().parent
    mode=LaunchConfiguration('mode').perform(context)
    options=settings(root)
    validate(options,mode,devices=False)
    params=Path(options['params'])
    data=yaml.safe_load(params.read_text())['course_mission']['ros__parameters']
    def include(package,name,args):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory(package))/'launch'/name)),launch_arguments=args.items())
    sensors=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(root/'sensors_one_command.launch.py')),launch_arguments={
        'camera_device':options['camera'],'gps_device':options['gps'],
        'imu_device':options['imu'],'arduino_device':options['arduino'],
        'velodyne_model':options['lidar_model'],'velodyne_ip':options['lidar_ip']}.items())
    yolo=include('yolo_bringup','yolo.launch.py',{
        'model':options['model'],'input_image_topic':'/camera/image_raw',
        'image_reliability':'2','device':'cuda:0','use_tracking':'True',
        'use_3d':'False','use_debug':'False','threshold':str(data.get('score_threshold',.65))})
    fusion=include('fusion_viewer','fusion_viewer.launch.py',{
        'image_topic':'/camera/image_raw','detections_topic':'/yolo/tracking',
        'cloud_topic':'/velodyne_points','calib_file':options['calibration'],
        'enable_projection':'true','force_projection_without_time_match':'false',
        'max_age_ms':'250'})
    mission=include('isv_shape_navigation','course_mission.launch.py',{
        'params_file':str(params),'enable_output':'true' if mode=='autonomy' else 'false',
        'gui_backend':'desktop','target_label':str(data.get('target_label','red triangle'))})
    # Delays stagger GPU/device startup; they are not sensor-readiness proofs.
    # The mission controller's existing data/RC/AUTO/GUI start gates still apply.
    return [LogInfo(msg='All-in-one mode='+mode+'; RC AUTO + GUI Start required for motion.'),
            sensors,TimerAction(period=3.,actions=[yolo]),TimerAction(period=5.,actions=[fusion]),
            TimerAction(period=7.,actions=[mission]),
            ExecuteProcess(cmd=['/usr/bin/python3',str(root/'launcher_check.py'),'--monitor'],output='screen')]


def generate_launch_description():
    # Import sibling helper without changing the installed underlay packages.
    import sys
    root=str(Path(__file__).resolve().parent)
    if root not in sys.path:sys.path.insert(0,root)
    return LaunchDescription([DeclareLaunchArgument('mode',default_value='preview',choices=['preview','autonomy']),
                              OpaqueFunction(function=assemble)])
