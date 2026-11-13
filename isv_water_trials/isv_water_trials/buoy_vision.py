"""Vessel-owned camera, HSV overlay and independently supplied YOLO detections."""
from concurrent.futures import ThreadPoolExecutor
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import numpy as np
from .vision_settings import settings, set_switch, directory, atomic_json, valid_stamp, process_identity, navigation_camera_owner, read_json
from .buoy_corridor import LidarCorridor


def colours(image, config, intrinsics=None):
    import cv2
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    masks={'green':cv2.inRange(hsv,(35,65,45),(90,255,255)),
           'red':cv2.inRange(hsv,(0,90,55),(12,255,255))|cv2.inRange(hsv,(168,90,55),(179,255,255))}
    h,w=image.shape[:2];fx,cx=intrinsics or (w/(2*math.tan(math.radians(config['hfov_deg'])/2)),w/2)
    result=[]
    for colour,mask in masks.items():
        mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
        mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((5,5),np.uint8))
        for contour in cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[0]:
            area=cv2.contourArea(contour);x,y,bw,bh=cv2.boundingRect(contour)
            if area<max(35,w*h*.00015) or area>w*h*.15 or not .15<=bw/max(bh,1)<=4.:
                continue
            if area/max(bw*bh,1)<.35:continue
            result.append(dict(source='HSV',colour=colour,box=[x,y,bw,bh],
                               bearing=math.atan2(cx-(x+bw/2),fx),
                               half_angle=math.atan2(bw/2,fx),area_px=area))
    return sorted(result,key=lambda d:d['area_px'],reverse=True)[:30]


def yolo_detections(message):
    """Accept existing yolo_msgs or JSON; never infer a trained class from HSV."""
    if hasattr(message,'data'):message=json.loads(message.data)
    get=lambda obj,key,default=None:obj.get(key,default) if isinstance(obj,dict) else getattr(obj,key,default)
    result=[]
    for d in get(message,'detections',[]):
        bbox=get(d,'bbox',{});center=get(bbox,'center',{});position=get(center,'position',center)
        size=get(bbox,'size',{})
        x=get(position,'x');y=get(position,'y');w=get(size,'x',get(bbox,'size_x'));h=get(size,'y',get(bbox,'size_y'))
        score=get(d,'score',0.);label=get(d,'class_name',str(get(d,'class_id','')))
        if not all(isinstance(v,(int,float)) and math.isfinite(v) for v in (x,y,w,h,score)) or min(w,h)<=0:continue
        result.append(dict(source='YOLO',label=str(label),score=float(score),box=[x-w/2,y-h/2,w,h]))
    return result[:50]


def overlay(image, hsv, objects):
    import cv2
    output=image.copy()
    for item in [*hsv,*objects]:
        x,y,w,h=map(int,item['box'])
        if item['source']=='HSV':
            colour=(60,220,80) if item['colour']=='green' else (70,70,250)
            text='HSV '+item['colour'].upper()
        else:colour=(230,150,255);text='YOLO '+item['label']+' %.2f'%item['score']
        cv2.rectangle(output,(x,y),(x+w,y+h),colour,2)
        cv2.putText(output,text,(max(0,x),max(14,y-5)),cv2.FONT_HERSHEY_SIMPLEX,.45,colour,1,cv2.LINE_AA)
    return output


class VisionRuntime:
    def __init__(self,node,boat,camera_only=False):
        self.node=node;self.boat=boat;self.root=directory();self.root.mkdir(parents=True,exist_ok=True)
        self.options=settings();self.previous_options=dict(self.options);self.previous_execute=False
        self.pool=ThreadPoolExecutor(max_workers=1);self.job=None;self.lock=threading.RLock()
        self.latest=None;self.objects=[];self.object_stamp=0.;self.intrinsics=None
        self.last_submit=0.;self.last_compressed=0.;self.last_manage=0.;self.last_report=0.;self.last_record=0.
        self.subscriptions=[];self.yolo_subscription=None;self.driver=None;self.borrowed=False;self.error=''
        self.corridor=LidarCorridor();self.corridor_data=None;self.context=None;self.active=False
        self.profile_choice=None;self.profile_votes=[];self.closed=False;self.retiring=[];self.previous_dock=False
        self.last_status={};self.owner_pid=os.getpid()
        self.yolo_model=None;self.yolo_model_file=None;self.last_inference=0.;self.inferred=[];self.inferred_at=0.;self.yolo_error=''
        self.camera_generation=0;self.camera_lease=None;self.camera_only=camera_only
        if not camera_only:
            atomic_json(self.root/'navigation_camera_owner.json',dict(pid=os.getpid(),start_ticks=process_identity(os.getpid())))

    def now(self):return self.node.get_clock().now().nanoseconds/1e9

    def manage(self,nav):
        self.context=nav;now=time.monotonic()
        if now-self.last_manage<.25:return
        self.last_manage=now;options=settings()
        started=nav.get('execute') and not self.previous_execute
        enabled_during_run=nav.get('execute') and options['objects'] and not self.previous_options['objects']
        entered_dock=bool(nav.get('execute') and nav.get('dock') and not self.previous_dock)
        relevant=bool(options['objects'] and nav.get('dock'))
        if (started or enabled_during_run or entered_dock) and relevant and not options['camera']:
            options=set_switch('camera',True)
        self.previous_dock=bool(nav.get('dock'))
        if started:self.profile_choice=None;self.profile_votes=[]
        from .vision_settings import assisted_leg
        if not assisted_leg(nav) or not options['corridor']:
            self.corridor.reset();self.corridor_data=None
        camera_keys=('camera_backend','device','depth_serial','depth_source_domain','image_topic',
                     'compressed_topic','capture_fps','capture_width','capture_height')
        changed=any(options.get(k)!=self.options.get(k) for k in camera_keys)
        if changed and self.subscriptions:self.stop_camera()
        self.previous_execute=bool(nav.get('execute'));self.previous_options=dict(options);self.options=options
        self.retiring=[p for p in self.retiring if p.poll() is None]
        if self.camera_lease and not self.subscriptions and not self.retiring:
            self.camera_lease.close();self.camera_lease=None
        if options['camera'] and not self.subscriptions:self.start_camera()
        elif not options['camera'] and self.subscriptions:self.stop_camera()
        if self.driver and self.driver.poll() is not None:
            self.error='camera driver stopped; check camera.log';self.driver=None
        if options['camera'] and self.subscriptions:
            self.ensure_yolo()
            if not self.driver and not self.borrowed:
                topic=options['image_topic']
                self.borrowed=self.node.count_publishers(topic)>0 or self.node.count_publishers(options['compressed_topic'])>0
            if not self.driver and not self.borrowed and now-getattr(self,'last_driver_attempt',0)>3:self.start_driver()
        if now-self.last_report>.5:
            if options['camera'] and not self.camera_lease and not self.error:return
            self.last_report=now;self.report()

    def start_camera(self):
        if self.retiring:return
        try:
            import fcntl
            lease=(self.root/('vision_runtime_'+os.environ.get('ROS_DOMAIN_ID','0')+'.lock')).open('a')
            try:fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
            except BlockingIOError:lease.close();return
            self.camera_lease=lease;self.camera_generation+=1
            from sensor_msgs.msg import Image,CompressedImage,CameraInfo
            from rclpy.qos import qos_profile_sensor_data as qos
            topic=self.options['image_topic']
            self.subscriptions=[self.node.create_subscription(CompressedImage,self.options['compressed_topic'],self.compressed,qos),
                                self.node.create_subscription(CameraInfo,self.options['camera_info_topic'],self.camera_info,qos)]
            if self.options['camera_backend']=='logitech':
                self.subscriptions.append(self.node.create_subscription(Image,topic,self.raw,qos))
            self.borrowed=self.node.count_publishers(topic)>0 or self.node.count_publishers(self.options['compressed_topic'])>0
            if not self.borrowed:self.start_driver()
            self.error=''
        except (ImportError,OSError,RuntimeError) as exc:self.error=str(exc)

    def start_driver(self):
        if self.retiring:return
        self.last_driver_attempt=time.monotonic()
        try:
            log=(self.root/'camera.log').open('a')
            with log:
                self.driver=subprocess.Popen([sys.executable,'-m','isv_water_trials.buoy_vision','_driver',
                                              str(self.owner_pid),json.dumps(self.options)],
                    stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        except OSError as exc:self.error=str(exc)

    def stop_camera(self):
        self.camera_generation+=1
        for sub in self.subscriptions:self.node.destroy_subscription(sub)
        self.subscriptions=[]
        if self.yolo_subscription:self.node.destroy_subscription(self.yolo_subscription);self.yolo_subscription=None
        if self.driver and self.driver.poll() is None:
            self.driver.send_signal(signal.SIGINT)
            self.retiring.append(self.driver)
        self.driver=None;self.borrowed=False;self.latest=None;self.objects=[];self.active=False
        self.inferred=[];self.inferred_at=0.;self.last_inference=0.;self.intrinsics=None
        self.last_compressed=0.;self.last_submit=0.;self.last_driver_attempt=0.

    def ensure_yolo(self):
        if self.options.get('yolo_weights'):
            if self.yolo_subscription:self.node.destroy_subscription(self.yolo_subscription);self.yolo_subscription=None
            return
        if self.yolo_subscription or time.monotonic()-getattr(self,'last_yolo_probe',0)<2:return
        self.last_yolo_probe=time.monotonic()
        from rosidl_runtime_py.utilities import get_message
        from rclpy.qos import qos_profile_sensor_data
        for topic,types in self.node.get_topic_names_and_types():
            if topic==self.options['yolo_topic'] and types:
                try:self.yolo_subscription=self.node.create_subscription(get_message(types[0]),topic,self.on_yolo,qos_profile_sensor_data)
                except (ImportError,AttributeError,RuntimeError) as exc:self.error='YOLO input: '+str(exc)
                break

    def camera_info(self,msg):
        if msg.width and msg.k[0]>0:self.intrinsics=(msg.k[0]/msg.width,msg.k[2]/msg.width)

    def on_yolo(self,msg):
        try:
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9 if hasattr(msg,'header') else self.now()
            if valid_stamp(stamp,self.now()):
                with self.lock:self.objects=yolo_detections(msg);self.object_stamp=stamp
        except (ValueError,TypeError,AttributeError):pass

    def compressed(self,msg):
        self.last_compressed=time.monotonic();self.submit(msg,True)

    def raw(self,msg):
        if time.monotonic()-self.last_compressed>.8:self.submit(msg,False)

    def infer_objects(self,image,options,stamp,generation):
        path=options.get('yolo_weights','')
        if not path:return []
        try:
            path=str(Path(path).expanduser())
            if path!=self.yolo_model_file:
                if not Path(path).is_file():raise ValueError('YOLO weights file not found')
                from ultralytics import YOLO
                self.yolo_model=YOLO(path);self.yolo_model_file=path
                import torch
                torch.set_num_threads(2)
            if time.monotonic()-self.last_inference>=1/options['yolo_fps']:
                self.last_inference=time.monotonic()
                device=options['yolo_device']
                prediction=self.yolo_model.predict(image,imgsz=int(options.get('yolo_imgsz',320)),conf=.35,max_det=20,
                    device=device,half=str(device) not in ('cpu','mps'),verbose=False)[0]
                inferred=[]
                for xyxy,score,label in zip(prediction.boxes.xyxy.cpu().tolist(),prediction.boxes.conf.cpu().tolist(),prediction.boxes.cls.cpu().tolist()):
                    x,y,x2,y2=xyxy
                    inferred.append(dict(source='YOLO',label=str(prediction.names[int(label)]),score=score,box=[x,y,x2-x,y2-y]))
                with self.lock:
                    if generation!=self.camera_generation:return []
                    self.inferred=inferred;self.inferred_at=stamp
                self.yolo_error=''
            ttl=max(options['sync_s'],1/options['yolo_fps']+.1)
            return list(self.inferred) if abs(stamp-self.inferred_at)<=ttl else []
        except Exception as exc:
            self.yolo_error=type(exc).__name__+': '+str(exc)[:120];return []

    def submit(self,msg,compressed):
        now=time.monotonic()
        if self.closed or not self.options['camera'] or now-self.last_submit<1/self.options['fps'] or (self.job and not self.job.done()):return
        self.last_submit=now;self.job=self.pool.submit(self.process,msg,compressed,dict(self.options),self.camera_generation)

    def process(self,msg,compressed,options,generation):
        try:
            import cv2
            if compressed:image=cv2.imdecode(np.frombuffer(msg.data,np.uint8),cv2.IMREAD_COLOR)
            else:
                from cv_bridge import CvBridge
                image=CvBridge().imgmsg_to_cv2(msg,desired_encoding='bgr8')
            if image is None:raise ValueError('camera decode failed')
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            if not valid_stamp(stamp,self.now()):return
            h,w=image.shape[:2]
            intrinsics=tuple(v*w for v in self.intrinsics) if self.intrinsics else None
            hsv=colours(image,options,intrinsics)
            with self.lock:objects=list(self.objects) if abs(stamp-self.object_stamp)<=options['sync_s'] else []
            if options.get('yolo_weights') or not self.yolo_subscription:objects=self.infer_objects(image,options,stamp,generation)
            output=overlay(image,hsv,objects)
            scale=min(1.,options['width']/w,options['height']/h);output=cv2.resize(output,(max(1,round(w*scale)),max(1,round(h*scale))))
            ok,jpeg=cv2.imencode('.jpg',output,[cv2.IMWRITE_JPEG_QUALITY,options['jpeg_quality']])
            if not ok:raise ValueError('JPEG encoding failed')
            with self.lock:
                if self.closed or not self.options['camera'] or generation!=self.camera_generation:return
                path=self.root/'camera.jpg';temporary=path.with_suffix('.incoming');temporary.write_bytes(jpeg.tobytes());temporary.replace(path)
                self.latest=dict(stamp_s=stamp,hsv=hsv,objects=objects,width=output.shape[1],height=output.shape[0],
                                 calibrated=self.intrinsics is not None,camera_backend=options['camera_backend'],jpeg_bytes=len(jpeg))
                self.active=True;self.error=''
                if time.monotonic()-self.last_record>=1:
                    self.last_record=time.monotonic()
                    with (self.root/('detections_'+time.strftime('%Y%m%d')+'.jsonl')).open('a',encoding='utf-8') as f:
                        f.write(json.dumps(self.latest,allow_nan=False)+'\n')
        except Exception as exc:self.error=type(exc).__name__+': '+str(exc)[:140]

    def route(self,state,start,finish,scan,scan_pose):
        from .vision_settings import assisted_leg
        nav=self.context or {}
        if not self.options['corridor'] or not assisted_leg(nav):
            self.corridor.reset();self.corridor_data=None;return None
        key=(tuple(np.round(start,3)),tuple(np.round(finish,3)))
        if self.corridor.frame!=key:
            self.corridor.reset();self.corridor.frame=key
        now=self.now()
        if scan is None or scan_pose is None or not valid_stamp(scan['stamp'],now,self.boat.get('scan_max_age_s',.8)):
            self.corridor.reset();self.corridor.reason='lidar scan/pose pending; GPS avoidance'
            self.corridor_data=None;return None
        if scan['stamp']!=self.corridor.last_scan:
            self.corridor.last_scan=scan['stamp']
            self.corridor.observe_scan(scan,scan_pose,start,finish,self.boat,self.options)
        route=self.corridor.build(start,finish,state['position'],now,self.boat['width_m'],self.options)
        self.corridor_data=route
        return route

    def guard(self,command,state,model=None,hits=()):
        from .vision_settings import assisted_leg
        if not self.options['corridor'] or not assisted_leg(self.context or {}):return list(command)
        if not valid_stamp(self.corridor.last_scan,self.now(),self.boat.get('scan_max_age_s',.8)):return list(command)
        if model is not None:model=model.for_navigation(self.boat)
        return self.corridor.guard(command,state,self.boat,model,hits)

    def docking_choice(self,map_id):
        nav=self.context or {};frame=self.latest
        if not (self.options['camera'] and self.options['objects'] and nav.get('execute') and nav.get('dock') and frame and valid_stamp(frame['stamp_s'],self.now())):
            self.profile_votes=[];return None
        if getattr(self,'choice_map',None)!=map_id:
            self.choice_map=map_id;self.profile_choice=None;self.profile_votes=[];self.voted_stamp=None
        with self.lock:
            external=self.yolo_subscription and not self.options.get('yolo_weights')
            objects=list(self.objects) if external else list(self.inferred)
            stamp=self.object_stamp if external else self.inferred_at
        if not valid_stamp(stamp,self.now()):self.profile_votes=[];return None
        if stamp==getattr(self,'voted_stamp',None):return getattr(self,'frame_choice',None)
        self.voted_stamp=stamp;self.frame_choice=None
        mapping=self.options['docking_class_routes']
        choices=[d for d in objects if d['score']>=.55 and mapping.get(d['label']) in ('left','middle','right')]
        if not choices:self.profile_votes=[];return None
        choice=mapping[max(choices,key=lambda d:d['score'])['label']]
        self.profile_votes=(self.profile_votes+[choice])[-3:]
        if len(self.profile_votes)==3 and len(set(self.profile_votes))==1:
            self.profile_choice=choice;self.frame_choice=choice;return choice
        return None

    def report(self):
        with self.lock:frame=dict(self.latest) if self.latest else {}
        self.last_status=dict(settings=self.options,camera_running=bool(self.options['camera'] and valid_stamp(frame.get('stamp_s'),self.now(),1.5)),
                              camera_backend=self.options['camera_backend'],frame_backend=frame.get('camera_backend'),
                              frame_width=frame.get('width'),frame_height=frame.get('height'),jpeg_bytes=frame.get('jpeg_bytes'),
                              runtime_owner='camera_only' if self.camera_only else 'navigation',owner_pid=os.getpid(),
                              camera_shared=self.borrowed,error=self.error,stamp_s=self.now(),written_unix=time.time(),
                              hsv=frame.get('hsv',[]),objects=frame.get('objects',[]),frame_stamp_s=frame.get('stamp_s'),
                              calibrated=frame.get('calibrated',False),corridor_active=self.corridor_data is not None,
                              corridor_locked=self.corridor.locked,corridor_reason=self.corridor.reason,corridor_source='lidar',
                              yolo_input=bool(self.yolo_subscription or self.yolo_model),yolo_error=self.yolo_error,object_route=self.profile_choice,
                              object_mapping_ready=bool(self.options['docking_class_routes']))
        try:atomic_json(self.root/'status.json',self.last_status)
        except OSError as exc:self.error='vision status storage: '+str(exc)[:100]

    def close(self):
        self.closed=True;self.stop_camera();self.corridor.reset();self.corridor_data=None
        self.pool.shutdown(wait=True,cancel_futures=True)
        for child in self.retiring:
            try:child.wait(timeout=6)
            except subprocess.TimeoutExpired:child.terminate()
        if self.camera_lease:self.camera_lease.close();self.camera_lease=None
        if not self.camera_only:
            owner=navigation_camera_owner(self.root)
            if owner and owner['pid']==os.getpid():(self.root/'navigation_camera_owner.json').unlink(missing_ok=True)
        self.report()


def driver(owner,options):
    """Own only this camera driver, survive SSH loss, exit with its vessel owner."""
    import fcntl
    root=directory();root.mkdir(parents=True,exist_ok=True)
    lock=(root/('camera_'+os.environ.get('ROS_DOMAIN_ID','0')+'.lock')).open('a')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:return
    # Reuse the bringup's per-domain USB-camera lease when available.
    try:
        from .launch_once import claim
        depth=options.get('camera_backend')=='depth'
        # A legacy bringup can retain its old D555 lease until it exits.
        owned,_=claim('vision_d555_camera' if depth else 'node_usb_cam_node_exe',
                      ('isv_water_trials.d555_camera',) if depth else ('usb_cam_node_exe',))
        if not owned:return
    except ImportError:
        # Legacy installations still get the dedicated camera lease above.
        pass
    def identity(pid):
        try:return Path('/proc',str(pid),'stat').read_text().rsplit(')',1)[1].split()[19]
        except OSError:return None
    expected=identity(owner)
    if expected is None:return
    stopped=False
    def stop(*_):
        nonlocal stopped
        stopped=True
    signal.signal(signal.SIGHUP,signal.SIG_IGN)
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    if options.get('camera_backend')=='depth':
        command=[sys.executable,'-m','isv_water_trials.d555_camera',
                 '--serial',options['depth_serial'],'--source-domain',str(options['depth_source_domain']),
                 '--topic-prefix',options['camera_namespace'],'--max-fps',str(options['capture_fps']),
                 '--max-width',str(options['capture_width']),'--max-height',str(options['capture_height']),
                 '--jpeg-quality','60']
    else:command=['ros2','run','usb_cam','usb_cam_node_exe','--ros-args',
             '-r','__ns:='+options['camera_namespace'],'-r','__node:='+options['camera_node'],
             '-r','image_raw:='+options['image_topic'],
             '-r','image_raw/compressed:='+options['compressed_topic'],
             '-r','camera_info:='+options['camera_info_topic'],
             '-p','video_device:='+options['device'],
             '-p','image_width:='+str(options['capture_width']),
             '-p','image_height:='+str(options['capture_height']),
             '-p','framerate:='+str(float(options['capture_fps'])),
             '-p','frame_id:='+options['frame_id'],'-p','io_method:=mmap',
             '-p','pixel_format:=yuyv2rgb','-p','camera_name:=logitech_c930e']
    child=subprocess.Popen(command,stdin=subprocess.DEVNULL,start_new_session=True)
    try:
        while not stopped and child.poll() is None and identity(owner)==expected:time.sleep(.2)
    finally:
        if child.poll() is None:
            os.killpg(child.pid,signal.SIGINT)
            try:child.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM)
        lock.close()


def camera_worker():
    """Display/inference only; yield camera ownership when the navigation owner starts."""
    import fcntl
    import rclpy
    from rclpy.node import Node
    root=directory();root.mkdir(parents=True,exist_ok=True)
    lease=(root/('vision_worker_'+os.environ.get('ROS_DOMAIN_ID','0')+'.lock')).open('a')
    try:fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:lease.close();return
    if navigation_camera_owner(root):lease.close();return
    rclpy.init(args=['--ros-args','--disable-rosout-logs','--disable-external-lib-logs'])
    node=Node('dashboard_camera',start_parameter_services=False)
    runtime=VisionRuntime(node,{},camera_only=True);stopped=False;idle_since=None
    def stop(*_):
        nonlocal stopped
        stopped=True
    signal.signal(signal.SIGHUP,signal.SIG_IGN)
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    try:
        while not stopped and rclpy.ok() and not navigation_camera_owner(root):
            rclpy.spin_once(node,timeout_sec=.05);runtime.manage(dict(execute=False))
            if runtime.options['camera']:idle_since=None
            elif idle_since is None:idle_since=time.monotonic()
            elif time.monotonic()-idle_since>3:break
    finally:
        runtime.close();node.destroy_node();rclpy.try_shutdown();lease.close()


if __name__=='__main__':
    if sys.argv[1:2]==['_driver']:driver(int(sys.argv[2]),json.loads(sys.argv[3]))
    elif sys.argv[1:2]==['_camera_worker']:camera_worker()
