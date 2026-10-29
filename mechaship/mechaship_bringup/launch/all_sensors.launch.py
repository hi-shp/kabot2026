"""Sensor-only bringup. No actuator enable/configuration or control nodes."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    value=lambda name: LaunchConfiguration(name).perform(context)
    bringup=get_package_share_directory('mechaship_bringup')
    vectornav=os.path.join(get_package_share_directory('vectornav'),'config','vectornav.yaml')
    common={'output':'screen','respawn':True,'respawn_delay':4.0}
    return [
        Node(package='ublox_gps',executable='ublox_gps_node',name='ublox_gps_node',
             condition=IfCondition(LaunchConfiguration('use_gps')),
             parameters=[{'device':value('gps_device'),'frame_id':'gps_link',
                          'config_on_startup':False,'save_on_shutdown':False,'rate':1.0,
                          'nav_rate':1,'tmode3':0,'debug':0,'publish.nmea':True}],
             remappings=[('~/fix','/gps/fix'),('~/fix_velocity','/gps/fix_velocity'),
                         ('diagnostics','/gps/diagnostics')],**common),
        Node(package='usb_cam',executable='usb_cam_node_exe',name='usb_cam',namespace='camera',
             condition=IfCondition(LaunchConfiguration('use_camera')),
             parameters=[{'video_device':value('camera_device'),'framerate':15.0,'io_method':'mmap',
                          'pixel_format':'yuyv2rgb','image_width':640,'image_height':480,
                          'frame_id':'camera_frame','camera_name':'logitech_c930e','camera_info_url':'',
                          'brightness':-1,'contrast':-1,'saturation':-1,'sharpness':-1,'gain':-1,
                          'auto_white_balance':True,'white_balance':4000,'autoexposure':True,
                          'exposure':100,'autofocus':False,'focus':-1}],**common),
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
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(bringup,'launch','velodyne_water_trial.launch.py')),
             condition=IfCondition(LaunchConfiguration('use_lidar')),
             launch_arguments={name:LaunchConfiguration(name) for name in ('velodyne_model','velodyne_ip','velodyne_ring')}.items()),
        Node(package='mechaship_bringup',executable='sensor_status.py',name='sensor_status',output='screen'),
    ]


def generate_launch_description():
    gps='/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00'
    imu='/dev/serial/by-id/usb-FTDI_USB-RS232-WE_AV0LG0SE-if00-port0'
    defaults={
        'use_gps':'true','use_camera':'false','use_imu':'true','use_mcu':'true','use_lidar':'true',
        'gps_device':gps if Path(gps).exists() else '/dev/ttyACM0',
        'camera_device':'/dev/video0','imu_device':imu if Path(imu).exists() else '/dev/ttyUSB1','imu_baud':'115200',
        'mcu_device':'/dev/ttyTHS1','mcu_baud':'115200',
        'use_arduino':'false','arduino_device':'/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
        'velodyne_model':'VLP16','velodyne_ip':'192.168.1.201','velodyne_ring':'-1',
    }
    return LaunchDescription([*(DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()),
                              OpaqueFunction(function=nodes)])
