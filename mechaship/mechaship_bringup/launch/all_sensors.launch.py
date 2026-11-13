"""Sensors and passive outdoor recording. Driving requires an explicit start."""
import os
import sys
import signal
import runpy
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, LogInfo, EmitEvent, RegisterEventHandler, ExecuteProcess
from launch.events import Shutdown
from launch.event_handlers import OnProcessStart, OnShutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from isv_water_trials.launch_once import node as Node, once, GPS_EXECUTABLES



_session = None


def nodes(context):
    global _session
    session_class = runpy.run_path(str(Path(__file__).with_name('process_lifecycle.py')))['BringupSession']
    _session = session_class()
    try:
        _session.acquire()
    except RuntimeError as exc:
        return [LogInfo(msg=str(exc)), EmitEvent(event=Shutdown(reason=str(exc)))]
    value=lambda name: LaunchConfiguration(name).perform(context)
    bringup=get_package_share_directory('mechaship_bringup')
    vectornav=os.path.join(get_package_share_directory('vectornav'),'config','vectornav.yaml')
    expected=[]
    for option,topics in (
        ('use_gps',['/gps/fix']),
        ('use_imu',['/vectornav/raw/common','/vectornav/imu']),
        ('use_lidar',['/velodyne_packets','/velodyne_points','/scan']),
        ('use_arduino',['/actuators/status'])):
        if value(option).lower()=='true':expected.extend(topics)
    if value('use_mcu').lower()=='true' and value('use_arduino').lower()!='true':
        expected.extend(['/sensor/battery/voltage','/sensor/emo/status','/smc_2000/fix'])
    common={'output':'screen','respawn':True,'respawn_delay':4.0,
            'sigterm_timeout':'2','sigkill_timeout':'1'}
    return [
        ExecuteProcess(cmd=[sys.executable,'-m','isv_water_trials.tank_record'],name='onboard_sensor_record',
             condition=IfCondition(LaunchConfiguration('use_record')),output='log',respawn=True,respawn_delay=4.,
             sigterm_timeout='30',sigkill_timeout='3'),
        RegisterEventHandler(OnProcessStart(on_start=_session.started)),
        RegisterEventHandler(OnShutdown(on_shutdown=_session.shutdown)),
        ExecuteProcess(cmd=[sys.executable,'-m','isv_water_trials.tank_session','start','mapping'],
             name='start_slam_session',condition=IfCondition(LaunchConfiguration('use_slam')),
             output='screen'),
        Node(package='ublox_gps',executable='ublox_gps_node',name='ublox_gps_node',
             condition=IfCondition(LaunchConfiguration('use_gps')),
             parameters=[{'device':value('gps_device'),'frame_id':'gps_link',
                          'config_on_startup':False,'save_on_shutdown':False,'rate':1.0,
                          'nav_rate':1,'tmode3':0,'debug':0,'publish.nmea':True}],
             remappings=[('~/fix','/gps/fix'),('~/fix_velocity','/gps/fix_velocity'),
                         ('diagnostics','/gps/diagnostics')],**common),
        # Camera capture follows dashboard selection; VisionRuntime owns its lifetime.
        Node(package='vectornav',executable='vectornav',name='vectornav',
             condition=IfCondition(LaunchConfiguration('use_imu')),
             parameters=[vectornav,{'port':value('imu_device'),'baud':int(value('imu_baud'))}],**common),
        Node(package='vectornav',executable='vn_sensor_msgs',name='vn_sensor_msgs',
             condition=IfCondition(LaunchConfiguration('use_imu')),parameters=[vectornav],**common),
        Node(package='micro_ros_agent',executable='micro_ros_agent',name='sensor_micro_ros_agent',
             condition=IfCondition('true' if value('use_mcu').lower()=='true' and value('use_arduino').lower()!='true' else 'false'),
             arguments=['serial','--dev',value('mcu_device'),'-b',value('mcu_baud'),'-v','4'],**common),
        Node(package='isv_water_trials',executable='arduino_bridge',name='arduino_actuator_bridge',
             condition=IfCondition(LaunchConfiguration('use_arduino')),
             arguments=['--port',value('arduino_device')],**common),
        Node(package='isv_water_trials',executable='outdoor',name='outdoor_water_trial',
             condition=IfCondition(LaunchConfiguration('use_outdoor')),
             output='screen',respawn=True,respawn_delay=5.0,
             sigterm_timeout='25',sigkill_timeout='3'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(bringup,'launch','velodyne_water_trial.launch.py')),
             condition=IfCondition(LaunchConfiguration('use_lidar')),
             launch_arguments={name:LaunchConfiguration(name) for name in ('velodyne_model','velodyne_ip','velodyne_ring')}.items()),
        Node(package='mechaship_bringup',executable='sensor_status.py',name='sensor_status',
             parameters=[{'expected_topics':expected}],**common),
    ]


def generate_launch_description():
    # Terminal/SSH loss is not a request to shut down sensors. Ctrl+C still is.
    signal.signal(signal.SIGHUP, signal.SIG_IGN)
    from isv_water_trials.shutdown import protect_console
    protect_console()
    gps='/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00'
    imu='/dev/serial/by-id/usb-FTDI_USB-RS232-WE_AV0LG0SE-if00-port0'
    defaults={
        'camera_backend':'d555','use_gps':'true','use_camera':'true','use_imu':'true','use_mcu':'true','use_lidar':'true','use_outdoor':'true','use_record':'true',
        'use_slam':'true',
        # Keep device identities even when unplugged; tty numbers belong to other devices after reconnects.
        'gps_device':gps,
        'camera_device':'/dev/v4l/by-id/usb-046d_Logitech_Webcam_C930e_8FFCDE5E-video-index0',
        'imu_device':imu,'imu_baud':'115200',
        'mcu_device':'/dev/ttyTHS1','mcu_baud':'115200',
        'use_arduino':'true','arduino_device':'/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
        'velodyne_model':'VLP16','velodyne_ip':'192.168.1.201','velodyne_ring':'8',
    }
    return LaunchDescription([*(DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()),
                              OpaqueFunction(function=nodes)])
