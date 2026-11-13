"""Launcher checks and optional one-shot /course/start after RC/data gates."""
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
        if not measured_calibration(options):
            print('GPS navigation enabled. Docking will stop at DOCK_CALIBRATION_REQUIRED until actual camera/LiDAR calibration is provided.',flush=True)
    missing=[key+': '+options[key] for key in ('camera','gps','imu','arduino') if not Path(options[key]).exists()]
    if devices and missing:
        if mode=='autonomy':raise ValueError('Devices not connected:\n'+'\n'.join(missing))
        print('PREVIEW: disconnected devices (nodes will wait/retry):\n'+'\n'.join(missing),flush=True)
    print('Mode:',mode,'| model:',options['model'],'| calibration:',calib,flush=True)
    if mode=='preview':print('PREVIEW: no thruster output. Example calibration is for connection checks only.',flush=True)
    return calib


def measured_calibration(options):
    """Reject known example/dimension mismatch; accuracy needs field validation."""
    import yaml
    path=calibration_path(options['calibration']);data=yaml.safe_load(path.read_text())
    camera=data.get('camera',{})
    if path.name=='calib_example.yaml' or (camera.get('width'),camera.get('height'))!=(640,480):return False
    example=calibration_path('calib_example.yaml')
    return not (example.exists() and data==yaml.safe_load(example.read_text()))


def may_auto_start(report,telemetry,ages):
    if not isinstance(report,dict) or not isinstance(telemetry,dict):return False
    out=report.get('output') or {}
    if out.get('enabled') is not True or out.get('active') or out.get('error'):return False
    if report.get('stage')!='TO_WP1' or report.get('state') not in ('TO_WP1','WAYPOINT_ARRIVAL'):return False
    if telemetry.get('receiving') is not True or telemetry.get('mode')!=2 or telemetry.get('estop_active') is not False or telemetry.get('stop_latched',False):return False
    for key,limit in {'GUI':.5,'MCU':.5,'GPS':2.,'IMU':.5,'LIDAR':.5}.items():
        age=ages.get(key,math.inf)
        if not isinstance(age,(int,float)) or not math.isfinite(age) or not 0<=age<=limit:return False
    return True


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


def monitor(auto_start=False):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import NavSatFix,Image,LaserScan
    from std_msgs.msg import String,Float32MultiArray
    from vectornav_msgs.msg import CommonGroup
    from yolo_msgs.msg import DetectionArray
    from std_srvs.srv import Trigger
    rclpy.init();node=rclpy.create_node('isv_one_command_receive_check');seen={};details={}
    latest={};issued=False;cancelled=False;request=None
    client=node.create_client(Trigger,'/course/start') if auto_start else None
    def accept(key,message):
        nonlocal cancelled
        seen[key]=time.monotonic()
        if key=='GPS':
            details[key]='fix='+str(message.status.status)
        elif key=='MCU':
            try:
                value=json.loads(message.data)
                latest['MCU']=value
                details[key]='mode='+str(value.get('mode_name',value.get('mode')))
            except (ValueError,TypeError):details[key]='invalid JSON'
        elif key=='FUSION':details[key]='rows='+str(len(message.data)//8)+' (zero is normal until a target is visible)'
        elif key=='GUI':
            try:
                latest['GUI']=json.loads(message.data)
                command=latest['GUI'].get('last_command') or {}
                if command.get('action') in ('stop','reset'):cancelled=True
            except (ValueError,TypeError):latest['GUI']={}
    streams=[('GPS','/gps/fix',NavSatFix),('IMU','/vectornav/raw/common',CommonGroup),
             ('LIDAR','/scan',LaserScan),('CAMERA','/camera/image_raw',Image),
             ('MCU','/actuators/status',String),('YOLO','/yolo/tracking',DetectionArray),
             ('FUSION','/fusion/roi_center',Float32MultiArray),('GUI','/course/status',String)]
    subscriptions=[node.create_subscription(kind,topic,lambda msg,k=key:accept(k,msg),qos_profile_sensor_data)
                   for key,topic,kind in streams]
    start=time.monotonic();next_print=start
    try:
        while rclpy.ok() and (time.monotonic()-start<60 or (auto_start and not issued and not cancelled)):
            rclpy.spin_once(node,timeout_sec=.2);now=time.monotonic()
            if auto_start and not issued and not cancelled:
                report=latest.get('GUI',{})
                if (report.get('output') or {}).get('active'):
                    issued=True
                    print('[AUTO START] Already started by operator; no second request.',flush=True)
                elif client.service_is_ready() and may_auto_start(report,latest.get('MCU'),{k:now-v for k,v in seen.items()}):
                    issued=True
                    request=client.call_async(Trigger.Request())
                    print('[AUTO START] RC AUTO and fresh navigation data confirmed. One start request sent.',flush=True)
            if request is not None and request.done():
                try:
                    response=request.result()
                    print('[AUTO START] '+str(response.success)+' '+response.message+'. No automatic restart after stop/fault.',flush=True)
                except Exception as exc:print('[AUTO START] Failed: '+str(exc)+'. No automatic retry.',flush=True)
                request=None
            if now>=next_print:
                text=' | '.join(key+': '+('RX' if key in seen and now-seen[key]<3 else 'WAIT') for key,_,_ in streams)
                print('[RECEIVE CHECK] '+text,flush=True)
                if details:print('[DETAIL] '+json.dumps(details,ensure_ascii=False),flush=True)
                next_print=now+5
        print('[RECEIVE CHECK] Finished. RX means message arrival, not verified calibration. Auto-start is one-shot; stop/fault never triggers a restart.',flush=True)
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
    group.add_argument('--auto-start',action='store_true')
    args=parser.parse_args()
    try:
        if args.plan:
            print(json.dumps(dict(mode=args.plan,output_enabled=args.plan=='autonomy',options=settings(Path(__file__).resolve().parent)),ensure_ascii=False,indent=2))
            print('No equipment was started. Sensors -> YOLO -> fusion -> GUI; autonomy sends one start request when RC AUTO/navigation are ready.')
        elif args.preflight:validate(settings(Path(__file__).resolve().parent),args.preflight)
        elif args.duplicates:duplicates()
        else:monitor(auto_start=args.auto_start)
    except (ValueError,FileNotFoundError,ImportError) as exc:
        print('STARTUP CHECK FAILED: '+str(exc),file=sys.stderr);return 2
    return 0


if __name__=='__main__':raise SystemExit(main())
