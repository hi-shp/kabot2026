"""Sensor-only launch copied into the new workspace, no legacy lifecycle edits.

Uses existing installed executables and parameters; starts neither outdoor
navigation nor a second actuator agent. ROS launch owns only its children.
"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,OpaqueFunction,ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from isv_water_trials.launch_once import node as Node, once, GPS_EXECUTABLES
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def sensors(context):
    root=Path(__file__).resolve().parent.parent
    val=lambda key:LaunchConfiguration(key).perform(context)
    common=dict(output='screen',respawn=True,respawn_delay=4.,sigterm_timeout='3',sigkill_timeout='1')
    vectornav=str(Path(get_package_share_directory('vectornav'))/'config/vectornav.yaml')
    nodes=[
        once('gps_receiver',GPS_EXECUTABLES,[IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory('mechaship_bringup'))/'launch/smc_2000.launch.py')),
            launch_arguments={'gps_device':val('gps_device')}.items())]),
        Node(package='usb_cam',executable='usb_cam_node_exe',name='usb_cam',namespace='camera',
             condition=IfCondition('true' if val('camera_backend')=='usb' else 'false'),
             parameters=[dict(video_device=val('camera_device'),framerate=15.,io_method='mmap',
                pixel_format='yuyv2rgb',image_width=640,image_height=480,frame_id='camera_frame',
                camera_name='logitech_c930e',camera_info_url='',brightness=-1,contrast=-1,saturation=-1,
                sharpness=-1,gain=-1,auto_white_balance=True,white_balance=4000,
                autoexposure=True,exposure=100,autofocus=False,focus=-1)],**common),
        once('d555_camera', ('d555_camera_bridge.py',), [
            ExecuteProcess(cmd=['/usr/bin/python3', str(root/'빌드실행/d555_camera_bridge.py')],
                condition=IfCondition('true' if val('camera_backend')=='d555' else 'false'),
                output='screen', respawn=True, respawn_delay=4.,
                sigterm_timeout='3', sigkill_timeout='1')]),
        Node(package='vectornav',executable='vectornav',name='vectornav',
             parameters=[vectornav,dict(port=val('imu_device'),baud=115200)],**common),
        Node(package='vectornav',executable='vn_sensor_msgs',name='vn_sensor_msgs',parameters=[vectornav],**common),
        Node(package='isv_water_trials',executable='arduino_bridge',name='arduino_actuator_bridge',
             arguments=['--port',val('arduino_device')],**common),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(root/'빌드실행/velodyne_course.launch.py')),
             launch_arguments={**{key:val(key) for key in ('velodyne_model','velodyne_ip')},
                               'min_range':val('lidar_min_range')}.items()),
    ]
    nodes.append(once('lidar_preprocess',('lidar_preprocess.py',),[ExecuteProcess(cmd=['/usr/bin/python3',str(root/'빌드실행/lidar_preprocess.py'),
        '--ros-args','--params-file',str(root/'설정/lidar_preprocess.yaml')],output='screen',
        respawn=True,respawn_delay=4.,sigterm_timeout='3',sigkill_timeout='1')]))
    if (Path.home()/'.config/kabot/ntrip.json').is_file():
        nodes.append(Node(package='isv_water_trials',executable='ntrip_bridge',name='ntrip_bridge',
                          remappings=[('/rtcm','/smc_2000/rtcm')],**common))
    return nodes


def generate_launch_description():
    defaults=dict(camera_backend='d555',camera_device='/dev/video0',gps_device='/dev/ttyACM0',imu_device='/dev/ttyUSB1',
                  arduino_device='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
                  velodyne_model='VLP16',velodyne_ip='192.168.1.201',lidar_min_range='0.1')
    return LaunchDescription([*[DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()],OpaqueFunction(function=sensors)])
