"""Read-only launcher checks; never publishes propulsion or arm commands."""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time


def settings(root):
    def value(name,default):return os.path.expandvars(os.path.expanduser(os.environ.get('COURSE_'+name,default)))
    return dict(params=str(root/'src/isv_shape_navigation/config/course_mission.yaml'),
        camera=value('CAMERA_DEVICE','/dev/video0'),
        gps=value('GPS_DEVICE','/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00'),
        imu=value('IMU_DEVICE','/dev/serial/by-id/usb-FTDI_USB-RS232-WE_AV0LG0SE-if00-port0'),
        arduino=value('ARDUINO_DEVICE','/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'),
        model=value('YOLO_MODEL','~/ros2_ws/weights/best3.pt'),
        calibration=value('CALIBRATION','calib_example.yaml'),
        lidar_ip=value('LIDAR_IP','192.168.1.201'),lidar_model=value('LIDAR_MODEL','VLP16'))


def calibration_path(value):
    p=Path(value)
    if not p.is_absolute():
        from ament_index_python.packages import get_package_share_directory
        p=Path(get_package_share_directory('fusion_viewer'))/'config'/p
    return p


def validate(options,mode,devices=True):
    import yaml
    if mode not in ('preview','autonomy'):raise ValueError('Unknown mode')
    for key in ('params','model'):
        if not Path(options[key]).is_file():raise ValueError('Missing '+key+': '+options[key])
    calib=calibration_path(options['calibration'])
    if not calib.is_file():raise ValueError('Missing camera/LiDAR calibration: '+str(calib))
    data=yaml.safe_load(calib.read_text())
    camera=data.get('camera',{});extrinsics=data.get('extrinsics',{})
    if len(camera.get('K',[]))!=9 or len(extrinsics.get('T',[]))!=16:
        raise ValueError('Calibration must contain camera.K[9], camera.D and extrinsics.T[16]')
    if not camera.get('D') or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in camera['K']+camera['D']+extrinsics['T']):
        raise ValueError('Calibration numbers must be finite')
    if camera['K'][0]<=0 or camera['K'][4]<=0:raise ValueError('Invalid camera focal length')
    if mode=='autonomy':
        if calib.name=='calib_example.yaml' or (camera.get('width'),camera.get('height'))!=(640,480):
            raise ValueError('AUTONOMY needs actual 640x480 camera/LiDAR calibration. Edit COURSE_CALIBRATION in all_in_one.env. Preview remains available.')
        example=calibration_path('calib_example.yaml')
        if example.exists() and data==yaml.safe_load(example.read_text()):
            raise ValueError('A renamed example calibration is still an example; provide measured calibration.')
    missing=[key+': '+options[key] for key in ('camera','gps','imu','arduino') if not Path(options[key]).exists()]
    if devices and missing:
        if mode=='autonomy':raise ValueError('Devices not connected:\n'+'\n'.join(missing))
        print('PREVIEW: disconnected devices (nodes will wait/retry):\n'+'\n'.join(missing),flush=True)
    print('Mode:',mode,'| model:',options['model'],'| calibration:',calib,flush=True)
    if mode=='preview':print('PREVIEW: no thruster output. Example calibration is for connection checks only.',flush=True)
    return calib


def duplicates():
    import rclpy
    rclpy.init();node=rclpy.create_node('isv_one_command_duplicate_check')
    try:
        end=time.monotonic()+2.
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
        names=set(node.get_node_names())
        conflicts=names&{'course_mission','outdoor_water_trial','arduino_actuator_bridge','usb_cam',
                         'velodyne_driver_node','yolo_node','tracking_node','fusion_viewer','vectornav'}
        if conflicts:raise ValueError('Already running: '+', '.join(sorted(conflicts))+'. Stop their launch terminal first; no existing process was stopped.')
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


def monitor():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import NavSatFix,Image,LaserScan
    from std_msgs.msg import String,Float32MultiArray
    from vectornav_msgs.msg import CommonGroup
    from yolo_msgs.msg import DetectionArray
    rclpy.init();node=rclpy.create_node('isv_one_command_receive_check');seen={};details={}
    def accept(key,message):
        seen[key]=time.monotonic()
        if key=='GPS':
            details[key]='fix='+str(message.status.status)
        elif key=='MCU':
            try:
                value=json.loads(message.data)
                details[key]='mode='+str(value.get('mode_name',value.get('mode')))
            except (ValueError,TypeError):details[key]='invalid JSON'
        elif key=='FUSION':details[key]='rows='+str(len(message.data)//8)+' (zero is normal until a target is visible)'
    streams=[('GPS','/gps/fix',NavSatFix),('IMU','/vectornav/raw/common',CommonGroup),
             ('LIDAR','/scan',LaserScan),('CAMERA','/camera/image_raw',Image),
             ('MCU','/actuators/status',String),('YOLO','/yolo/tracking',DetectionArray),
             ('FUSION','/fusion/roi_center',Float32MultiArray),('GUI','/course/status',String)]
    subscriptions=[node.create_subscription(kind,topic,lambda msg,k=key:accept(k,msg),qos_profile_sensor_data)
                   for key,topic,kind in streams]
    start=time.monotonic();next_print=start
    try:
        while rclpy.ok() and time.monotonic()-start<60:
            rclpy.spin_once(node,timeout_sec=.2);now=time.monotonic()
            if now>=next_print:
                text=' | '.join(key+': '+('RX' if key in seen and now-seen[key]<3 else 'WAIT') for key,_,_ in streams)
                print('[RECEIVE CHECK] '+text,flush=True)
                if details:print('[DETAIL] '+json.dumps(details,ensure_ascii=False),flush=True)
                next_print=now+5
        print('[RECEIVE CHECK] Finished. RX means messages arrived, not verified calibration or safe navigation. Check live GUI/target data before Start.',flush=True)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


def main():
    parser=argparse.ArgumentParser()
    group=parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--plan',choices=['preview','autonomy'])
    group.add_argument('--preflight',choices=['preview','autonomy'])
    group.add_argument('--duplicates',action='store_true');group.add_argument('--monitor',action='store_true')
    args=parser.parse_args()
    try:
        if args.plan:
            print(json.dumps(dict(mode=args.plan,output_enabled=args.plan=='autonomy',options=settings(Path(__file__).resolve().parent)),ensure_ascii=False,indent=2))
            print('No equipment was started. Sensors -> YOLO -> fusion -> GUI; RC AUTO + GUI Start remain required.')
        elif args.preflight:validate(settings(Path(__file__).resolve().parent),args.preflight)
        elif args.duplicates:duplicates()
        else:monitor()
    except (ValueError,FileNotFoundError,ImportError) as exc:
        print('STARTUP CHECK FAILED: '+str(exc),file=sys.stderr);return 2
    return 0


if __name__=='__main__':raise SystemExit(main())
