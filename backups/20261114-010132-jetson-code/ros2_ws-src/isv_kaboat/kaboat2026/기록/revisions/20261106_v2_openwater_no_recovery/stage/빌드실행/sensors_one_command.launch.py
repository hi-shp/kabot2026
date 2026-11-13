"""Sensor-only launch copied into the new workspace, no legacy lifecycle edits.

Uses existing installed executables and parameters; starts neither outdoor
navigation nor a second actuator agent. ROS launch owns only its children.
"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,OpaqueFunction,ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def sensors(context):
    root=Path(__file__).resolve().parent.parent
    val=lambda key:LaunchConfiguration(key).perform(context)
    common=dict(output='screen',respawn=True,respawn_delay=4.,sigterm_timeout='3',sigkill_timeout='1')
    vectornav=str(Path(get_package_share_directory('vectornav'))/'config/vectornav.yaml')
    nodes=[
        Node(package='ublox_gps',executable='ublox_gps_node',name='ublox_gps_node',
             parameters=[dict(device=val('gps_device'),frame_id='gps_link',config_on_startup=False,
                              save_on_shutdown=False,rate=1.,nav_rate=1,tmode3=0,debug=0,
                              **{'publish.nmea':True,'publish.nav.pvt':True,'publish.rxm.rtcm':True})],
             remappings=[('~/fix','/gps/fix'),('~/fix_velocity','/gps/fix_velocity'),('diagnostics','/gps/diagnostics')],**common),
        Node(package='usb_cam',executable='usb_cam_node_exe',name='usb_cam',namespace='camera',
             parameters=[dict(video_device=val('camera_device'),framerate=15.,io_method='mmap',
                pixel_format='yuyv2rgb',image_width=640,image_height=480,frame_id='camera_frame',
                camera_name='logitech_c930e',camera_info_url='',brightness=-1,contrast=-1,saturation=-1,
                sharpness=-1,gain=-1,auto_white_balance=True,white_balance=4000,
                autoexposure=True,exposure=100,autofocus=False,focus=-1)],**common),
        Node(package='vectornav',executable='vectornav',name='vectornav',
             parameters=[vectornav,dict(port=val('imu_device'),baud=115200)],**common),
        Node(package='vectornav',executable='vn_sensor_msgs',name='vn_sensor_msgs',parameters=[vectornav],**common),
        Node(package='isv_water_trials',executable='arduino_bridge',name='arduino_actuator_bridge',
             arguments=['--port',val('arduino_device')],**common),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(root/'빌드실행/velodyne_course.launch.py')),
             launch_arguments={**{key:val(key) for key in ('velodyne_model','velodyne_ip')},
                               'min_range':val('lidar_min_range')}.items()),
    ]
    nodes.append(ExecuteProcess(cmd=['/usr/bin/python3',str(root/'빌드실행/lidar_preprocess.py'),
        '--ros-args','--params-file',str(root/'설정/lidar_preprocess.yaml')],output='screen',
        respawn=True,respawn_delay=4.,sigterm_timeout='3',sigkill_timeout='1'))
    if (Path.home()/'.config/kabot/ntrip.json').is_file():
        nodes.append(Node(package='isv_water_trials',executable='ntrip_bridge',name='ntrip_bridge',**common))
    return nodes


def generate_launch_description():
    defaults=dict(camera_device='/dev/video0',gps_device='/dev/ttyACM0',imu_device='/dev/ttyUSB1',
                  arduino_device='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
                  velodyne_model='VLP16',velodyne_ip='192.168.1.201',lidar_min_range='0.1')
    return LaunchDescription([*[DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()],OpaqueFunction(function=sensors)])
