"""Independent ROS2 docking mapper. Sensor input and mapping services only."""
from collections import deque
from copy import deepcopy
from pathlib import Path
from queue import Queue, Empty, Full
import json
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor,SetParametersResult
from rcl_interfaces.srv import SetParametersAtomically
from sensor_msgs.msg import NavSatFix, Imu, PointCloud2
from sensor_msgs_py import point_cloud2
from .pointcloud import read_scalar_fields
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as RosPath
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from builtin_interfaces.msg import Time
from std_msgs.msg import Header
from scipy.spatial.transform import Rotation
import yaml

from .geometry import ENU,voxel
from .mapper import Mapper, json_safe
from .geographic_map import GeographicMapStore
from .session import MappingSession
from .reference import ReferenceMap
from .preprocessing import scan_motion_reason
from .hopping import read_hopping,write_hopping
from .preview import mapping_preview
from .gui_commands import GUI_ACTIONS,RC_ACTIONS,execute as execute_gui_command
from .autonomy import DockingAutonomy
from .map_navigation_runtime import MapNavigationDryRun


class DockingMapping(Node):
    def __init__(self):
        super().__init__('docking_mapping')
        root=Path(__file__).resolve().parents[1]
        self.declare_parameter('config_file',str(root/'config/mapping.yaml'),ParameterDescriptor(read_only=True))
        self.config=yaml.safe_load(Path(self.get_parameter('config_file').value).read_text(encoding='utf-8'))
        self.mapper=Mapper(self.config)
        self.session=MappingSession()
        self.reference=None
        self.reference_folder=None
        self.hopping=None
        self.hopping_dirty=False
        self.last_gui_result=None
        self.rc_interrupted=threading.Event()
        self.root=root
        self.maps_dir=root/'maps'
        self.maps_dir.mkdir(exist_ok=True)
        self.geographic_maps=GeographicMapStore(self.maps_dir/'gps_maps')
        self.gps_map_loaded=False
        self.gps_map_name=None
        self.gps_target=None
        self.mode='PAUSED'
        self.state='PAUSED'
        self.error=''
        self.command_result=''
        self.data_lock=threading.Lock()
        self.map_lock=threading.Lock()
        self.stop_event=threading.Event()
        self.commands=Queue(maxsize=8)
        self.gps=deque(maxlen=100)
        self.imu=deque(maxlen=500)
        self.latest_cloud=None
        self.arrivals={}
        self.rc={}
        reference_config=self.config['reference']
        for name,key in [('reference_roi_min_enu_m','roi_min_enu_m'),('reference_roi_max_enu_m','roi_max_enu_m')]:
            self.declare_parameter(name,reference_config[key] or [],ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('reference_yaw_enu_deg',float(reference_config['yaw_enu_deg']))
        self.add_on_set_parameters_callback(self.select_reference_parameters)
        self.last_scan_stamp=-1.
        self.last_output_stamp=None
        self.processing_ms=0.
        self.mapping_services=[]
        self.mapping_publishers={}
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,history=HistoryPolicy.KEEP_LAST)
        latched=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        kinds={'lidar_map':PointCloud2,'provisional_map':PointCloud2,'current_scan_map':PointCloud2,
               'gps_pose':PoseStamped,'corrected_pose':PoseStamped,'rc_path':RosPath,
               'map_origin':NavSatFix,'mapping_status':String,'mapping_diagnostics':DiagnosticArray,
               'reference_map':PointCloud2,'map_pose':PoseStamped,'map_preview':String}
        for name,kind in kinds.items():
            self.mapping_publishers[name]=self.create_publisher(kind,'/docking/'+name,latched if name=='map_origin' else 1)
        s=self.config['sensors']
        self.subs=[self.create_subscription(NavSatFix,s['gps_topic'],self.on_gps,qos),
                   self.create_subscription(Imu,s['imu_topic'],self.on_imu,qos),
                   self.create_subscription(PointCloud2,s['lidar_topic'],self.on_cloud,qos),
                   self.create_subscription(String,s['rc_status_topic'],self.on_rc,qos)]
        for action in ('start','pause','resume','stop','save','load','reset'):
            self.mapping_services.append(self.create_service(Trigger,'/docking/mapping/'+action,
                lambda request,response,a=action:self.command(a,response)))
        self.mapping_services.append(self.create_service(SetParametersAtomically,
            '/docking/mapping/gui_command',self.gui_command))
        self.autonomy=DockingAutonomy(self)
        self.map_navigation=MapNavigationDryRun(self)
        for action in ('auto_start','auto_stop','auto_emergency'):
            self.mapping_services.append(self.create_service(Trigger,'/docking/autonomy/'+action[5:],
                lambda request,response,a=action:self.auto_command(a,response)))
        self.timer=self.create_timer(1/self.config['mapping']['gui_hz'],self.publish_snapshot)
        self.worker=threading.Thread(target=self.work,name='docking_mapping_worker',daemon=False)
        self.worker.start()
        self.get_logger().info('Mapper ready; initially PAUSED; /docking/mapping/start starts RC-only mapping')

    @staticmethod
    def stamp(message):
        return message.header.stamp.sec+message.header.stamp.nanosec*1e-9

    def now_s(self):
        return self.get_clock().now().nanoseconds*1e-9

    def fresh(self,stamp):
        return -.1<=self.now_s()-stamp<=self.config['mapping']['input_timeout_s']

    def on_gps(self,message):
        with self.data_lock:
            self.arrivals['gps']=time.monotonic()
            if self.fresh(self.stamp(message)):
                self.gps.append(message)

    def on_imu(self,message):
        with self.data_lock:
            self.arrivals['imu']=time.monotonic()
            if self.fresh(self.stamp(message)) and message.orientation_covariance[0]>=0:
                self.imu.append(message)

    def on_cloud(self,message):
        with self.data_lock:
            self.arrivals['lidar']=time.monotonic()
            if self.fresh(self.stamp(message)):
                self.latest_cloud=message

    def on_rc(self,message):
        try:
            data=json.loads(message.data)
            if not isinstance(data,dict):return
        except (ValueError,TypeError):return
        with self.data_lock:
            self.rc=data
            self.arrivals['rc']=time.monotonic()
            if not (data.get('mode')==1 and data.get('receiving') is True and data.get('estop_active') is False):
                self.rc_interrupted.set()
        if hasattr(self,'autonomy'):self.autonomy.on_telemetry(data)
        if hasattr(self,'map_navigation'):self.map_navigation.on_telemetry(data)

    def auto_command(self,action,response):
        try:
            if action=='auto_start':
                self.commands.put_nowait(dict(action=action,request_id='ros_auto_start',auto_generation=self.autonomy.generation))
            else:
                self.map_navigation.stop('operator '+action)
                self.autonomy.stop('operator '+action,action=='auto_emergency')
            response.success=True;response.message='Queued AUTO start' if action=='auto_start' else 'AUTO stopped'
        except (ValueError,Full) as exc:response.success=False;response.message=str(exc)
        return response

    def command(self,action,response):
        if action in ('start','resume') and not self.rc_ready():
            response.success=False;response.message='RC_NOT_READY: mapping requires fresh RC telemetry'
            return response
        try:
            self.commands.put_nowait(action)
            response.success=True
            response.message='Queued '+action+'; completion is reported on /docking/mapping_status'
        except Full:
            response.success=False;response.message='Mapping command queue full'
        return response

    def gui_command(self,request,response):
        try:
            if (len(request.parameters)!=1 or request.parameters[0].name!='ui_request'
                    or request.parameters[0].value.type!=4):raise ValueError('Expected one JSON ui_request string')
            raw=request.parameters[0].value.string_value
            if len(raw)>8192:raise ValueError('Mapping request too large')
            payload=json.loads(raw)
            if not isinstance(payload,dict) or payload.get('action') not in GUI_ACTIONS:
                raise ValueError('Unsupported mapping-only action')
            identity=payload.get('request_id')
            if not isinstance(identity,str) or not 1<=len(identity)<=64:raise ValueError('Missing request ID')
            if payload['action'] in RC_ACTIONS and not self.rc_ready():raise ValueError('RC_NOT_READY')
            if payload['action']=='map_dry_run_stop':
                self.map_navigation.stop('GUI stop')
                self.last_gui_result=dict(request_id=identity,success=True,message='Map Dry-run stopped; output disabled')
                response.result.successful=True;response.result.reason='Map Dry-run stopped'
                return response
            if payload['action'] in ('map_dry_run_start','map_drive_start'):payload['dry_generation']=self.map_navigation.generation
            if payload['action'] in ('auto_stop','auto_emergency'):
                self.map_navigation.stop('GUI '+payload['action'])
                self.autonomy.stop('GUI '+payload['action'],payload['action']=='auto_emergency')
                self.last_gui_result=dict(request_id=identity,success=True,message='AUTO stopped')
                response.result.successful=True;response.result.reason='AUTO stopped'
                return response
            if payload['action']=='auto_start':payload['auto_generation']=self.autonomy.generation
            self.commands.put_nowait(payload)
            response.result.successful=True;response.result.reason='Queued mapping command '+identity
        except (ValueError,TypeError,Full) as exc:
            response.result.successful=False;response.result.reason=str(exc)
        return response

    def select_reference_parameters(self,parameters):
        names={'reference_roi_min_enu_m':'roi_min_enu_m','reference_roi_max_enu_m':'roi_max_enu_m',
               'reference_yaw_enu_deg':'yaw_enu_deg'}
        selected=[p for p in parameters if p.name in names]
        if not selected:return SetParametersResult(successful=True)
        if not self.map_lock.acquire(blocking=False):
            return SetParametersResult(successful=False,reason='Mapping busy; retry selection')
        try:
            if self.session.active or self.reference is not None or self.map_navigation.active:
                raise ValueError('Select structure only while stopped and before reference freeze')
            changed=deepcopy(self.config['reference'])
            for parameter in selected:
                key=names[parameter.name]
                if key=='yaw_enu_deg':
                    if isinstance(parameter.value,bool) or not np.isfinite(parameter.value):
                        raise ValueError('Invalid dock_ref yaw')
                    changed[key]=float(parameter.value)
                else:
                    value=np.asarray(parameter.value,dtype=float)
                    if value.shape!=(3,) or not np.isfinite(value).all():
                        raise ValueError('Reference ROI must have three finite ENU values')
                    changed[key]=value.tolist()
            low,high=changed['roi_min_enu_m'],changed['roi_max_enu_m']
            if low is not None and high is not None and np.any(np.asarray(low)>=high):
                raise ValueError('Reference ROI min must be below max')
            self.config['reference']=changed
            self.mapper.config['reference']=deepcopy(changed)
            return SetParametersResult(successful=True)
        except (ValueError,TypeError) as exc:
            return SetParametersResult(successful=False,reason=str(exc))
        finally:self.map_lock.release()

    def execute_command(self,action):
        with self.map_lock:
            if getattr(self,'map_navigation',None) is not None and self.map_navigation.active:raise ValueError('Stop map Dry-run before mapping/editing commands')
            if self.autonomy.active:raise ValueError('Stop AUTO docking before mapping/editing commands')
            with self.autonomy.lock:
                self.autonomy.controller.state='IDLE';self.autonomy.world_from_dock=None
            if action in ('start','resume'):
                self.rc_interrupted.clear()
                if action=='start':self.session.start(self.rc_ready())
                else:self.session.resume(self.rc_ready())
                self.mode='ACTIVE';self.state='WAIT_SENSORS'
            elif action in ('pause','stop'):
                if action=='pause':self.session.pause()
                else:self.session.finish()
                self.mode='PAUSED' if action=='pause' else 'STOPPED'
                self.state='MAPPING_'+self.session.state
                self.mapper.current_scan=np.empty((0,3))
            elif action in ('save_gps_map','load_gps_map'):
                if self.session.active:raise ValueError('Finish mapping before saving/loading a GPS map')
                if action=='save_gps_map':
                    folder=self.geographic_maps.save(self.mapper)
                    old_target=getattr(self,'gps_target',None)
                    if old_target is not None:
                        self.gps_target=self.geographic_maps.write_target(folder.name,old_target['latitude'],old_target['longitude'])
                    self.gps_map_name=folder.name
                    self.state='GPS_MAP_SAVED'
                    self.command_result='GPS 맵을 저장했습니다. 맵 불러오기를 누르면 위성지도에 표시됩니다.'
                else:
                    mapper,folder=self.geographic_maps.load(self.config)
                    target=self.geographic_maps.upgrade_target(folder.name)
                    self.mapper=mapper
                    self.gps_target=target
                    self.reference=None;self.reference_folder=None;self.hopping=None;self.hopping_dirty=False
                    self.session.reset();self.session.state='FINISHED'
                    self.mode='PAUSED';self.state='GPS_MAP_LOADED'
                    self.gps_map_loaded=True;self.gps_map_name=folder.name
                    self.last_output_stamp=None
                    self.command_result='저장한 GPS 맵을 불러왔습니다.'
            elif action=='save':
                if self.reference is None:
                    self.session.can_freeze()
                    reference=ReferenceMap.from_mapper(self.mapper,self.config['reference'])
                else:reference=self.reference
                folder=reference.save(self.maps_dir,
                    companion_writer=lambda directory:write_hopping(directory,reference,self.hopping))
                self.reference=reference
                self.reference_folder=folder
                self.hopping_dirty=False
                self.session.reference_ready()
                self.mode='PAUSED';self.state='REFERENCE_SAVED'
                self.command_result='Saved reference '+str(folder)
            elif action in ('load','reset'):
                if self.mode=='ACTIVE':
                    raise ValueError('Pause mapping before '+action)
                if action=='load':
                    reference,folder=ReferenceMap.load_latest(self.maps_dir,self.config['sensors'],self.config['mapping']['max_voxels'])
                    if reference.voxel_m!=self.config['mapping']['voxel_m']:
                        raise ValueError('Saved reference voxel size differs from active YAML')
                    hopping=read_hopping(folder,reference)
                    self.mapper.reset()
                    self.mapper.enu=ENU(reference.origin_lla)
                    self.mapper.map.add(reference.points_enu())
                    self.mapper.confirmed=True
                    self.mapper.path.extend(map(tuple,reference.path_enu))
                    self.reference=reference
                    self.reference_folder=folder;self.hopping=hopping;self.hopping_dirty=False
                    self.session.reference_ready()
                    self.mapper.status=dict(state='REFERENCE_LOADED',map_points=len(self.mapper.map.cells))
                    self.command_result='Loaded reference '+str(folder)
                else:
                    self.mapper.reset();self.gps_map_loaded=False;self.gps_map_name=None;self.gps_target=None;self.command_result='Map reset; next valid GPS establishes a new origin'
                    self.reference=None;self.session.reset()
                    self.reference_folder=None;self.hopping=None;self.hopping_dirty=False
                self.state=self.mapper.status['state']
                self.last_output_stamp=None
        if action not in ('save_gps_map','load_gps_map','save','load','reset'):
            self.command_result=action+' completed'

    def work(self):
        while not self.stop_event.is_set():
            started=time.monotonic()
            try:
                try:
                    action=self.commands.get_nowait()
                except Empty:
                    action=None
                if action:
                    if isinstance(action,dict):
                        try:
                            message=execute_gui_command(self,action)
                            self.command_result=message
                            self.last_gui_result=dict(request_id=action['request_id'],success=True,message=message)
                        except Exception as exc:
                            self.command_result=str(exc)
                            self.last_gui_result=dict(request_id=action['request_id'],success=False,message=str(exc))
                    else:
                        try:self.execute_command(action)
                        except Exception as exc:self.command_result=action+' FAILED: '+str(exc)
                if self.session.active:
                    if self.rc_interrupted.is_set() or not self.rc_ready():self.pause_for_rc_loss()
                    else:self.process_latest()
                elif self.autonomy.active:self.autonomy.process_latest()
                elif self.map_navigation.active:self.map_navigation.process_latest()
            except Exception as exc:
                self.error=str(exc);self.state='PROCESSING_ERROR'
                if self.map_navigation.active:self.map_navigation.fail('PROCESSING_ERROR: '+str(exc),self.map_navigation.generation)
                self.get_logger().error(self.error,throttle_duration_sec=5.)
            rate=self.config['mapping']['processing_hz']
            if self.map_navigation.active:rate=max(rate,self.map_navigation.parameters['control_rate'])
            remaining=1/rate-(time.monotonic()-started)
            self.stop_event.wait(max(.001,remaining))

    def process_latest(self):
        m,s=self.config['mapping'],self.config['sensors']
        with self.data_lock:
            cloud,self.latest_cloud=self.latest_cloud,None
            fixes,imus=list(self.gps),list(self.imu)
            rc=dict(self.rc)
            rc_age=time.monotonic()-self.arrivals.get('rc',-float('inf'))
        if self.rc_interrupted.is_set() or not (rc_age<=m['rc_timeout_s'] and rc.get('mode')==1
                and rc.get('receiving') is True and rc.get('estop_active') is False):
            self.pause_for_rc_loss();return
        if not fixes or not any(f.status.status>=0 for f in fixes):
            self.state='WAIT_VALID_GPS';return
        if cloud is None or not self.fresh(self.stamp(cloud)):
            self.state='WAIT_LIDAR';return
        stamp=self.stamp(cloud)
        if stamp<=self.last_scan_stamp:return
        self.last_scan_stamp=stamp
        if not imus:self.state='WAIT_IMU';return
        fix=min(fixes,key=lambda message:abs(self.stamp(message)-stamp))
        imu=min(imus,key=lambda message:abs(self.stamp(message)-stamp))
        if not self.fresh(self.stamp(fix)) or not self.fresh(self.stamp(imu)):
            self.state='WAIT_SYNC';return
        gps_skew=abs(self.stamp(fix)-stamp);imu_skew=abs(self.stamp(imu)-stamp)
        if gps_skew>m['gps_sync_tolerance_s'] or imu_skew>m['imu_sync_tolerance_s']:
            self.state='WAIT_SYNC';return
        if cloud.header.frame_id!=s['lidar_frame'] or imu.header.frame_id!=s['imu_frame']:
            self.state='FRAME_MISMATCH';return
        motion=self.config.get('scan_motion',{})
        if motion.get('enabled',False):
            duration=None
            if any(field.name=='time' for field in cloud.fields):
                offsets=read_scalar_fields(cloud,('time',)).ravel()
                offsets=offsets[np.isfinite(offsets)]
                if len(offsets)>1:duration=float(np.ptp(offsets))
            if duration is None or duration<=0:duration=motion.get('fallback_duration_s')
            rate=imu.angular_velocity
            reason=scan_motion_reason(duration,float(np.linalg.norm([rate.x,rate.y,rate.z])),m['max_boat_speed_mps'],motion)
            if reason:self.state=reason;return
        if fix.position_covariance_type==NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            self.state='WAIT_GPS_COVARIANCE';return
        diagonal=np.asarray(fix.position_covariance)[[0,4,8]]
        if not np.isfinite(diagonal).all() or np.any(diagonal<0):
            self.state='INVALID_GPS_COVARIANCE';return
        sigma=float(np.sqrt(max(diagonal[0],diagonal[1])))
        q=imu.orientation
        began=time.monotonic()
        points=read_scalar_fields(cloud,('x','y','z')).reshape(-1,3)
        with self.map_lock:
            self.mapper.process(points,[fix.latitude,fix.longitude,fix.altitude],
                [q.x,q.y,q.z,q.w],stamp,sigma,fix.status.status,
                commit_guard=lambda:self.rc_ready() and not self.rc_interrupted.is_set())
            self.state=self.mapper.status['state']
            self.last_output_stamp=cloud.header.stamp
            self.error=''
            self.processing_ms=(time.monotonic()-began)*1000
            self.mapper.status.update(gps_skew_s=gps_skew,imu_skew_s=imu_skew,processing_ms=self.processing_ms)
        if self.state=='WAIT_RC':self.pause_for_rc_loss()

    def pause_for_rc_loss(self):
        with self.map_lock:
            self.session.lose_rc()
            self.mode='PAUSED';self.state='MAPPING_PAUSED_RC_LOST'
            self.mapper.current_scan=np.empty((0,3))

    def rc_ready(self):
        with self.data_lock:
            rc=self.rc
            return (time.monotonic()-self.arrivals.get('rc',-float('inf'))<=self.config['mapping']['rc_timeout_s']
                    and rc.get('mode')==1 and rc.get('receiving') is True and rc.get('estop_active') is False)

    def pose_message(self,matrix,stamp):
        message=PoseStamped()
        message.header.frame_id=self.config['sensors']['map_frame']
        message.header.stamp=stamp
        message.pose.position.x,message.pose.position.y,message.pose.position.z=map(float,matrix[:3,3])
        # Some SciPy ARM64 builds require a writable buffer, even for conversion.
        q=Rotation.from_matrix(np.array(matrix[:3,:3],copy=True)).as_quat()
        message.pose.orientation.x,message.pose.orientation.y,message.pose.orientation.z,message.pose.orientation.w=map(float,q)
        return message

    @staticmethod
    def ros_stamp(seconds):
        nanoseconds=int(round(seconds*1e9))
        return Time(sec=nanoseconds//1000000000,nanosec=nanoseconds%1000000000)

    def publish_snapshot(self):
        if not self.map_lock.acquire(blocking=False):return
        try:
            mapper=self.mapper
            now=time.monotonic()
            with self.data_lock:
                ages={key:now-value for key,value in self.arrivals.items()}
            report=dict(mapper.status,state=self.state,mode=self.mode,error=self.error,
                        time_now_s=self.now_s(),
                        phase=self.session.phase,initial_mapping_state=self.session.state,
                        reference_id=None if self.reference is None else self.reference.reference_id,
                        reference_version=None if self.reference is None else self.reference.version,
                        reference_points=0 if self.reference is None else len(self.reference.points),
                        rc_ready=self.rc_ready(),reference_selection=deepcopy(self.config['reference']),
                        hopping_dirty=self.hopping_dirty,last_gui_result=self.last_gui_result,
                        hopping_config=deepcopy(self.config['hopping']),
                        command_result=self.command_result,sensor_receive_age_s=ages,
                        origin_lla=None if mapper.enu is None else mapper.enu.origin,
                        map_quality='CONFIRMED' if mapper.confirmed else 'PROVISIONAL',
                        gps_map_available=self.geographic_maps.available(),gps_map_loaded=self.gps_map_loaded,
                        gps_map_name=self.gps_map_name,gps_target=self.gps_target,
                        processing_state=mapper.status.get('state'),
                        mapping_blockers=([] if self.config['sensors']['extrinsics_verified'] else ['WAIT_EXTRINSICS'])+([] if self.config['sensors']['heading_reference']!='magnetic' or self.config['sensors']['magnetic_declination_deg'] is not None else ['WAIT_DECLINATION']),
                        saved_reference_available=any(self.maps_dir.glob('*/docking_reference_metadata.yaml')),
                        extrinsics_verified=self.config['sensors']['extrinsics_verified'],
                        heading_reference=self.config['sensors']['heading_reference'],
                        relative_reference=self.config['sensors']['heading_reference']=='relative',
                        processing_ms=self.processing_ms,
                        sample_stamp_s=None if self.last_output_stamp is None else self.last_output_stamp.sec+self.last_output_stamp.nanosec*1e-9)
            auto=self.autonomy.snapshot()
            report['autonomy']=auto
            map_navigation=self.map_navigation.snapshot()
            report['map_navigation']=map_navigation
            with self.autonomy.lock:
                live=None if self.autonomy.world_from_dock is None else self.autonomy.world_from_dock.copy()
                raw=None if self.autonomy.raw_pose is None else self.autonomy.raw_pose.copy()
                sample=self.autonomy.sample
                live_scan=self.autonomy.world_scan.copy() if sample is not None and auto['active'] else np.empty((0,3))
                relative_pose=(None if sample is None or self.reference is None else
                    self.reference.enu_from_dock@np.linalg.inv(sample.base_from_dock))
            if auto['state']!='IDLE':
                report['phase']='AUTO';report['state']=auto['state']
                report['mode']='DRY_RUN' if auto['dry_run'] else 'AUTO'
            self.mapping_publishers['mapping_status'].publish(String(data=json.dumps(json_safe(report),allow_nan=False)))
            diagnostic=DiagnosticArray()
            diagnostic.header.stamp=self.get_clock().now().to_msg()
            diagnostic.status=[DiagnosticStatus(name='docking_mapping',hardware_id='GPS_IMU_VLP16',
                level=bytes([0 if self.state=='CONVERGED' or auto['active'] and sample is not None else 1]),message=report['state'],
                values=[KeyValue(key=k,value=str(v)) for k,v in json_safe(report).items()])]
            self.mapping_publishers['mapping_diagnostics'].publish(diagnostic)
            preview=mapping_preview(mapper,self.reference,self.hopping,self.config,report,live)
            with self.map_navigation.lock:
                nav_pose=None if self.map_navigation.pose is None else self.map_navigation.pose.copy()
                nav_stamp=self.map_navigation.pose_stamp
            if nav_pose is not None and map_navigation['pose_age_s'] is not None and map_navigation['pose_age_s']<=self.config['autonomy']['tracking_timeout_s']:
                anchor=mapper.display_anchor or dict(origin_xyz=[0.,0.,0.],yaw_deg=0.)
                relative=(nav_pose[:3,3]-np.asarray(anchor['origin_xyz']))@Rotation.from_euler('z',-anchor['yaw_deg'],degrees=True).as_matrix().T
                yaw=float(np.degrees(np.arctan2(nav_pose[1,0],nav_pose[0,0])))
                preview['corrected_position']=dict(latlon=mapper.enu.to_lla(nav_pose[:3,3])[:2].tolist(),enu=nav_pose[:3,3].tolist(),
                    yaw_enu_deg=yaw,relative_xyz=relative.tolist(),yaw_relative_deg=(yaw-anchor['yaw_deg']+180)%360-180,
                    stamp_s=nav_stamp,age_s=map_navigation['pose_age_s'])
            if auto['state']!='IDLE':
                preview['gps_position']=None
                preview['corrected_position']=None
                if raw is not None and live is not None and auto['world_anchor_stamp_s'] is not None:
                    lla=mapper.enu.to_lla(raw[:3,3])
                    preview['gps_position']=dict(latlon=lla[:2].tolist(),enu=raw[:3,3].tolist(),
                        yaw_enu_deg=float(Rotation.from_matrix(raw[:3,:3]).as_euler('xyz',degrees=True)[2]),
                        stamp_s=auto['world_anchor_stamp_s'],age_s=max(0.,self.now_s()-auto['world_anchor_stamp_s']))
                display_scan=voxel(live_scan,self.config['mapping']['gui_voxel_m'],1000)
                preview['current_scan']=[] if not len(display_scan) else mapper.enu.to_lla(display_scan)[:,:2].tolist()
                if relative_pose is not None:
                    ll=mapper.enu.to_lla(relative_pose[:3,3])
                    preview['corrected_position']=dict(latlon=ll[:2].tolist(),enu=relative_pose[:3,3].tolist(),
                        yaw_enu_deg=float(Rotation.from_matrix(relative_pose[:3,:3]).as_euler('xyz',degrees=True)[2]),
                        stamp_s=sample.stamp,age_s=max(0.,self.now_s()-sample.stamp))
                    self.mapping_publishers['corrected_pose'].publish(self.pose_message(relative_pose,self.ros_stamp(sample.stamp)))
            self.mapping_publishers['map_preview'].publish(String(data=json.dumps(json_safe(preview),allow_nan=False)))
            if mapper.enu is None:return
            stamp=self.ros_stamp(sample.stamp) if auto['active'] and sample is not None else self.last_output_stamp or self.get_clock().now().to_msg()
            header=Header(frame_id=self.config['sensors']['map_frame'],stamp=stamp)
            if self.reference is not None:
                reference_header=Header(frame_id='dock_ref',stamp=stamp)
                self.mapping_publishers['reference_map'].publish(point_cloud2.create_cloud_xyz32(reference_header,self.reference.points.astype(np.float32)))
                map_stamp=self.ros_stamp(auto['world_anchor_stamp_s']) if live is not None and auto['world_anchor_stamp_s'] is not None else stamp
                self.mapping_publishers['map_pose'].publish(self.pose_message(self.reference.enu_from_dock if live is None else live,map_stamp))
            from .geometry import apply
            points=(apply(self.reference.points,live) if self.reference is not None and live is not None
                    else mapper.map.points()).astype(np.float32)
            empty=np.empty((0,3),dtype=np.float32)
            for key,data in [('lidar_map',points if mapper.confirmed else empty),
                             ('provisional_map',empty if mapper.confirmed else points),
                             ('current_scan_map',live_scan.astype(np.float32) if auto['active'] else mapper.current_scan.astype(np.float32) if self.state=='CONVERGED' else empty)]:
                self.mapping_publishers[key].publish(point_cloud2.create_cloud_xyz32(header,data))
            origin=NavSatFix()
            origin.header=header
            origin.latitude,origin.longitude,origin.altitude=map(float,mapper.enu.origin)
            # Origin defines the map reference, not a live GPS quality measurement.
            origin.status.status=0
            self.mapping_publishers['map_origin'].publish(origin)
            if mapper.gps_pose is not None and mapper.gps_pose_stamp is not None:
                self.mapping_publishers['gps_pose'].publish(self.pose_message(mapper.gps_pose,self.ros_stamp(mapper.gps_pose_stamp)))
            if mapper.corrected_pose is not None and mapper.accepted_poses:
                self.mapping_publishers['corrected_pose'].publish(self.pose_message(mapper.corrected_pose,self.ros_stamp(mapper.accepted_poses[-1][0])))
            path=RosPath(header=header)
            for t,x,y,z in mapper.path:
                pose=PoseStamped()
                pose.header.frame_id=header.frame_id
                pose.header.stamp.sec=int(t);pose.header.stamp.nanosec=int((t-int(t))*1e9)
                pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=map(float,[x,y,z])
                pose.pose.orientation.w=1.
                path.poses.append(pose)
            self.mapping_publishers['rc_path'].publish(path)
        finally:self.map_lock.release()

    def close(self):
        self.map_navigation.stop('node shutdown')
        self.autonomy.stop('node shutdown')
        self.stop_event.set()
        self.worker.join()
        deadline=time.monotonic()+1.5
        while (self.autonomy.output.owner is not None or self.map_navigation.drive.owned) and rclpy.ok() and time.monotonic()<deadline:
            self.autonomy.output_tick()
            self.map_navigation.output_tick()
            rclpy.spin_once(self,timeout_sec=.02)
        self.map_navigation.close()


def main(args=None):
    rclpy.init(args=args)
    node=None
    try:
        node=DockingMapping();rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node is not None:node.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
