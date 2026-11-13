"""Persistent, independent camera and navigation-consumer switches."""
import json
import math
from pathlib import Path
import time
import uuid

LOGITECH_DEVICE = '/dev/v4l/by-id/usb-046d_Logitech_Webcam_C930e_8FFCDE5E-video-index0'

DEFAULTS = dict(camera=False, corridor=False, objects=False, device=LOGITECH_DEVICE,
                camera_backend='logitech', depth_serial='262422303049', depth_source_domain=0,
                width=320, height=240, fps=5., jpeg_quality=40, hfov_deg=70.,
                camera_yaw_deg=0., camera_xy_m=[0., 0.], max_range_m=25.,
                track_ttl_s=6., sync_s=.25, margin_m=.35,
                image_topic='/camera/image_raw', compressed_topic='/camera/image_raw/compressed',
                camera_info_topic='/camera/camera_info', camera_node='usb_cam',
                camera_namespace='/camera', frame_id='camera_frame',
                capture_width=640, capture_height=480, capture_fps=5.,
                yolo_topic='/yolo/tracking',
                yolo_weights='', yolo_device='cpu', yolo_fps=2., yolo_imgsz=320,
                docking_class_routes={})


def directory():
    return Path.home()/'water_trials/vision'


def read_json(path, default=None):
    try:return json.loads(Path(path).read_text(encoding='utf-8'))
    except (OSError, ValueError):return default


def atomic_json(path, value):
    path=Path(path);path.parent.mkdir(parents=True, exist_ok=True)
    temporary=path.with_name(path.name+'.'+uuid.uuid4().hex+'.tmp')
    try:
        temporary.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False), encoding='utf-8')
        for attempt in range(5):
            try:temporary.replace(path);break
            except PermissionError:
                if attempt==4:raise
                time.sleep(.01)
    finally:temporary.unlink(missing_ok=True)


def settings(root=None):
    result=dict(DEFAULTS);stored=read_json((root or directory())/'control.json', {})
    if isinstance(stored, dict):result.update(stored)
    backend=result.get('camera_backend','logitech')
    if backend not in ('logitech','depth'):backend='logitech'
    result['camera_backend']=backend
    prefix='/camera/logitech' if backend=='logitech' else '/camera/depth/color'
    result.update(image_topic=prefix+'/image_raw',compressed_topic=prefix+'/image_raw/compressed',
                  camera_info_topic=prefix+'/camera_info',camera_namespace=prefix,
                  camera_node='usb_cam' if backend=='logitech' else 'd555_camera_bridge',
                  frame_id='logitech_camera_frame' if backend=='logitech' else 'd555_color_optical_frame')
    if backend=='logitech':
        devices=sorted(Path('/dev/v4l/by-id').glob('*Logitech*-video-index0'))
        preferred=result.get('logitech_device') or LOGITECH_DEVICE
        result['device']=preferred if Path(preferred).exists() or len(devices)!=1 else str(devices[0])
    return result


def select_camera(backend,enabled,root=None):
    if backend not in ('logitech','depth') or type(enabled) is not bool:
        raise ValueError('로지텍 또는 뎁스 카메라와 켜짐 상태를 지정하세요')
    root=root or directory();result=settings(root)
    result.update(camera_backend=backend,camera=enabled,revision=uuid.uuid4().hex,updated_unix=time.time())
    atomic_json(root/'control.json',result)
    return settings(root)


def set_camera_fps(value,root=None):
    if type(value) not in (int,float) or not math.isfinite(value) or not 1<=value<=30:
        raise ValueError('카메라 최대 FPS는 1~30으로 입력하세요')
    root=root or directory();result=settings(root)
    result.update(fps=float(value),capture_fps=float(value),revision=uuid.uuid4().hex,updated_unix=time.time())
    atomic_json(root/'control.json',result)
    return result


def process_identity(pid):
    try:return Path('/proc',str(int(pid)),'stat').read_text().rsplit(')',1)[1].split()[19]
    except (OSError,ValueError,TypeError):return None


def navigation_camera_owner(root=None):
    value=read_json((root or directory())/'navigation_camera_owner.json',{}) or {}
    identity=process_identity(value.get('pid'))
    return value if identity is not None and identity==value.get('start_ticks') else None


def set_switch(key, enabled, root=None):
    if key not in ('camera', 'corridor', 'objects') or type(enabled) is not bool:
        raise ValueError('Expected a camera, corridor or objects boolean switch')
    root=root or directory()
    result=settings(root);result[key]=enabled;result['revision']=uuid.uuid4().hex
    result['updated_unix']=time.time();atomic_json(root/'control.json', result)
    return result


def assisted_leg(nav):
    route=nav.get('route') or {};kinds=route.get('waypoint_types', [])
    return bool(nav.get('execute') and nav.get('path') and nav.get('gps', True)
                and nav.get('goal')==2 and len(route.get('points', []))>=3
                and all((kinds[i] if i<len(kinds) else 'path')=='path' for i in (1, 2)))


def valid_stamp(stamp, now, maximum=.8):
    return isinstance(stamp, (float, int)) and math.isfinite(stamp) and -.1<=now-stamp<=maximum
