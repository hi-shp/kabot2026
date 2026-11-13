"""Standalone learned-shape following with unchanged Jetson actuator functions."""
from dataclasses import asdict, fields
import json
import math
import signal
import time
import numpy as np

from .core import Scan
from .follow import FollowConfig, Follower, fusion_rows, label_name, native_pwm
from .output import PairedOutput, telemetry_ready


def make_node(mode='follow'):
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rcl_interfaces.msg import ParameterDescriptor
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Float32MultiArray, String, Empty
    from std_srvs.srv import Trigger, SetBool
    from yolo_msgs.msg import DetectionArray
    from mechaship_interfaces.srv import ThrottlePulseWidth
    from isv_water_trials.tank_core import guidance
    from isv_water_trials.actuator_config import command_pwm
    course_mode = mode == 'course'
    gps_mode = mode in ('gps_dock', 'course')
    if mode not in ('follow', 'gps_dock', 'course'):
        raise ValueError('Unknown mission mode')
    if gps_mode:
        from sensor_msgs.msg import NavSatFix
        from vectornav_msgs.msg import CommonGroup
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        from .gps_mission import GPSConfig, GPSDockMission, Fix
        from .core import Pose
        from .node import quaternion_yaw
        from .planner import Grid
    if course_mode:
        from .route_mission import RouteConfig, RouteMission
        from .mission_dashboard import Dashboard
        from .desktop_dashboard import DesktopDashboard
        from .course_settings import apply_course_settings, handle_settings_rpc
        from rcl_interfaces.srv import SetParametersAtomically
        from .navigation_display import SlamDisplay
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge
        import cv2
        from ament_index_python.packages import get_package_share_directory
        from pathlib import Path
        from queue import Empty as QueueEmpty

    class ShapeFollower(Node):
        def __init__(self):
            super().__init__('course_mission' if course_mode else 'gps_dock' if gps_mode else 'shape_follow')
            self.prefix = '/course' if course_mode else '/gps_dock' if gps_mode else '/shape_follow'
            defaults = asdict(FollowConfig())
            defaults['class_names'] = list(defaults['class_names'])
            defaults.update(enable_output=False, fusion_topic='/fusion/roi_center',
                            yolo_topic='/yolo/tracking', scan_topic='/scan', scan_frame='velodyne',
                            camera_frame='camera_frame', actuator_status_topic='/actuators/status',
                            score_threshold=.65, scan_yaw_deg=0., scan_x=0., scan_y=0.,
                            control_rate=10.)
            if gps_mode:
                defaults.update(asdict(GPSConfig()))
                defaults.update(gps_topic='/gps/fix', heading_topic='/vectornav/raw/common',
                                heading_offset_deg=0., heading_frame='vectornav',
                                map_topic='/map', pose_topic='/tank/pose', map_frame='tank_map')
            if course_mode:
                defaults.update(asdict(RouteConfig()))
                defaults['wp_latitudes']=list(defaults['wp_latitudes'])
                defaults['wp_longitudes']=list(defaults['wp_longitudes'])
                defaults.update(gui_enabled=True,gui_backend='desktop',gui_port=8770,image_topic='/camera/image_raw',
                                goal_configured=True,slam_display=False,session_topic='/tank/session')
            for name, value in defaults.items():
                self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
            self.p = {name: self.get_parameter(name).value for name in defaults}
            self.cfg = FollowConfig(**{item.name: self.p[item.name] for item in fields(FollowConfig)})
            if not 0 < self.p['score_threshold'] <= 1:
                raise ValueError('score_threshold must be in (0,1]')
            if not 1 <= self.p['control_rate'] <= 30:
                raise ValueError('control_rate must be 1..30 Hz')
            if not all(math.isfinite(self.p[key]) for key in ('scan_yaw_deg', 'scan_x', 'scan_y')):
                raise ValueError('Invalid scan transform')
            if self.p['enable_output'] and self.get_parameter('use_sim_time').value:
                raise ValueError('Hardware output is disabled with simulated ROS time')
            self.native_command_pwm = command_pwm
            if native_pwm((0., 0.), command_pwm) != (1500, 1500):
                raise ValueError('Installed native calibration does not match the 1500us bridge protocol')
            self.controller = (GPSDockMission(self.cfg, guidance,
                GPSConfig(**{item.name: self.p[item.name] for item in fields(GPSConfig)}))
                if gps_mode else Follower(self.cfg, guidance))
            if course_mode:
                self.controller=RouteMission(self.cfg,guidance,self.controller.gps_config,
                    RouteConfig(**{item.name:self.p[item.name] for item in fields(RouteConfig)}))
            self.motion_states = self.controller.motion_states if gps_mode else ('FOLLOW_TARGET', 'ALIGN_TARGET', 'HOLD_DISTANCE')
            if gps_mode and max(self.p['gps_percent'], self.p['entry_percent'], self.p['search_percent']) > self.cfg.max_percent:
                raise ValueError('GPS/search/entry output cannot exceed max_percent')
            if gps_mode and not math.isfinite(self.p['heading_offset_deg']):
                raise ValueError('Invalid heading offset')
            self.scan = None
            self.yolo, self.yolo_stamp = [], -math.inf
            self.telemetry, self.telemetry_at = {}, -math.inf
            self.last_reason, self.previous_now = 'WAIT_DATA', -math.inf
            self.output, self.output_lock = None, None
            self.dashboard=None
            self.desktop_closed=False
            if course_mode:
                self.display_state=SlamDisplay(self.p['map_frame'],self.p['slam_pose_timeout'],
                                               self.p['sync_tolerance'])
                if self.p['slam_display']:
                    self.create_subscription(PoseWithCovarianceStamped,self.p['pose_topic'],self.on_display_pose,qos_profile_sensor_data)
                    self.create_subscription(String,self.p['session_topic'],self.on_display_session,10)
                    display_map_qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                              reliability=ReliabilityPolicy.RELIABLE)
                    self.create_subscription(OccupancyGrid,self.p['map_topic'],self.on_display_map,display_map_qos)
                self.image_bridge=CvBridge()
                self.last_camera_at=-math.inf
                self.create_subscription(Image,self.p['image_topic'],self.on_course_image,qos_profile_sensor_data)
                if self.p['gui_enabled']:
                    if self.p['gui_backend']=='desktop':
                        self.dashboard=DesktopDashboard()
                        self.get_logger().info('Native mission window opened on the Jetson desktop')
                    elif self.p['gui_backend']=='web':
                        web=Path(get_package_share_directory('isv_shape_navigation'))/'web'
                        self.dashboard=Dashboard(web,int(self.p['gui_port']))
                        self.get_logger().info('Mission GUI: http://127.0.0.1:'+str(self.dashboard.port))
                    else:raise ValueError('gui_backend must be desktop or web')
            self.status_pub = self.create_publisher(String, self.prefix+'/status', 10)
            self.command_pub = self.create_publisher(String, self.prefix+'/command', 10)
            if gps_mode:
                self.create_subscription(NavSatFix, self.p['gps_topic'], self.on_fix, qos_profile_sensor_data)
                self.create_subscription(CommonGroup, self.p['heading_topic'], self.on_heading, qos_profile_sensor_data)
                if not course_mode:
                    self.create_service(Trigger, self.prefix+'/capture_goal', self.capture_goal)
                if self.p['use_slam']:
                    self.create_subscription(PoseWithCovarianceStamped, self.p['pose_topic'], self.on_pose, qos_profile_sensor_data)
                    map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                         reliability=ReliabilityPolicy.RELIABLE)
                    self.create_subscription(OccupancyGrid, self.p['map_topic'], self.on_map, map_qos)
            self.create_subscription(Float32MultiArray, self.p['fusion_topic'], self.on_fusion, qos_profile_sensor_data)
            self.create_subscription(DetectionArray, self.p['yolo_topic'], self.on_yolo, qos_profile_sensor_data)
            self.create_subscription(LaserScan, self.p['scan_topic'], self.on_scan, qos_profile_sensor_data)
            self.create_subscription(String, self.p['actuator_status_topic'], self.on_telemetry, qos_profile_sensor_data)
            self.create_subscription(Empty, '/water_trial/stop', self.external_stop, 10)
            self.create_service(Trigger, self.prefix+'/start', self.start)
            self.create_service(Trigger, self.prefix+'/stop', self.stop)
            self.create_service(Trigger, self.prefix+'/reset', self.reset)
            if course_mode:
                self.settings_service=self.create_service(SetParametersAtomically, self.prefix+'/gui_command',
                    lambda request,response:handle_settings_rpc(self,request,response))
            if self.p['enable_output']:
                import fcntl
                import os
                self.output_lock = open('/tmp/kabot_shape_output_'+os.environ.get('ROS_DOMAIN_ID', '0')+'.lock', 'a')
                fcntl.flock(self.output_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                self.arm_client = self.create_client(SetBool, '/actuators/arm')
                self.stop_client = self.create_client(Trigger, '/actuators/stop')
                self.pwm_clients = [self.create_client(ThrottlePulseWidth,
                    '/actuators/throttle_'+side+'/set_pulse_width') for side in ('left', 'right')]
                self.output = PairedOutput(
                    lambda value: self.arm_client.call_async(SetBool.Request(data=value)),
                    lambda left, right: [client.call_async(ThrottlePulseWidth.Request(pulse_width=pulse))
                                         for client, pulse in zip(self.pwm_clients, (left, right))],
                    lambda: self.stop_client.call_async(Trigger.Request()),
                    lambda: all(client.service_is_ready() for client in
                                (self.arm_client, self.stop_client, *self.pwm_clients)),
                    stop_ready=lambda: self.stop_client.service_is_ready())
            self.timer = self.create_timer(1/self.p['control_rate'], self.update)
            self.get_logger().info('Target '+self.cfg.target_label+'; class_id='+str(self.cfg.target_class_id)
                                   +'; native isv_water_trials guidance + command_pwm; '
                                   +('OUTPUT ENABLED; start service required' if self.output else 'PREVIEW'))

        def now(self):
            return self.get_clock().now().nanoseconds/1e9

        @staticmethod
        def stamp(header):
            return header.stamp.sec+header.stamp.nanosec/1e9

        def on_fix(self, message):
            stamp = self.stamp(message.header)
            previous = self.controller.fix
            if previous and stamp <= previous.stamp:
                return
            if (message.status.status < 0 or message.position_covariance_type == 0
                    or not -.1 <= self.now()-stamp <= self.p['gps_timeout']
                    or not math.isfinite(message.latitude) or not -90 <= message.latitude <= 90
                    or not math.isfinite(message.longitude) or not -180 <= message.longitude <= 180):
                self.controller.fix = None
                return
            xx, xy, yx, yy = [message.position_covariance[index] for index in (0, 1, 3, 4)]
            if (not all(math.isfinite(value) for value in (xx, xy, yx, yy))
                    or xx < 0 or yy < 0 or abs(xy-yx) > 1e-6 or xx*yy-xy*yx < -1e-6):
                self.controller.fix = None
                return
            largest = .5*(xx+yy+math.sqrt((xx-yy)**2+4*xy*yx))
            self.controller.fix = Fix(message.latitude, message.longitude, math.sqrt(largest), stamp)

        def on_heading(self, message):
            stamp = self.stamp(message.header)
            if (stamp <= self.controller.heading_stamp
                    or message.header.frame_id != self.p['heading_frame']
                    or not -.1 <= self.now()-stamp <= self.p['heading_timeout']
                    or not math.isfinite(message.yawpitchroll.x)):
                return
            # VectorNav: north-zero clockwise -> east-zero CCW for native guidance.
            self.controller.heading = math.pi/2-math.radians(message.yawpitchroll.x+self.p['heading_offset_deg'])
            self.controller.heading_stamp = stamp

        def on_pose(self, message):
            stamp = self.stamp(message.header)
            pose = message.pose.pose
            if (message.header.frame_id != self.p['map_frame']
                    or not -.1 <= self.now()-stamp <= self.p['slam_pose_timeout']
                    or not all(math.isfinite(v) for v in (pose.position.x, pose.position.y))):
                return
            try:
                angle = quaternion_yaw(pose.orientation)+math.radians(self.p['pose_yaw_offset_deg'])
                self.controller.pose = Pose(pose.position.x, pose.position.y, angle, stamp)
            except ValueError:
                self.controller.pose = None

        def on_map(self, message):
            if (message.header.frame_id != self.p['map_frame'] or not message.data
                    or message.info.width*message.info.height != len(message.data)):
                return
            try:
                origin = message.info.origin
                self.controller.map = Grid(np.asarray(message.data).reshape(message.info.height, message.info.width),
                    message.info.resolution, (origin.position.x, origin.position.y, quaternion_yaw(origin.orientation)),
                    self.cfg.footprint_radius+self.cfg.clearance)
                self.controller.map_received = self.now()
            except (ValueError, OverflowError):
                self.controller.map = None

        def capture_goal(self, request, response):
            fix = self.controller.fix
            if self.output and (self.output.active or self.output.pending or self.output.stopping):
                response.success, response.message = False, 'Stop and wait for acknowledgment before capturing GPS'
            elif (fix is None or not -.1 <= self.now()-fix.stamp <= self.p['gps_timeout']
                  or not 0 <= fix.sigma <= self.p['max_gps_sigma']):
                response.success, response.message = False, 'Fresh valid GPS with known acceptable covariance required'
            else:
                cfg = self.controller.gps_config
                cfg.goal_latitude, cfg.goal_longitude, cfg.goal_configured = fix.latitude, fix.longitude, True
                self.controller.reset()
                self.controller.fix = fix
                response.success = True
                response.message = json.dumps(dict(goal_latitude=fix.latitude, goal_longitude=fix.longitude,
                                                    goal_configured=True, persisted=False))
            return response

        def on_display_pose(self,message):
            if message.header.frame_id!=self.p['map_frame']:return
            stamp=self.stamp(message.header);pose=message.pose.pose
            if not -.1<=self.now()-stamp<=self.p['slam_pose_timeout']:return
            try:
                yaw=quaternion_yaw(pose.orientation)+math.radians(self.p['pose_yaw_offset_deg'])
                self.display_state.pose(Pose(pose.position.x,pose.position.y,yaw,stamp))
            except (ValueError,TypeError):pass

        def on_display_map(self,message):
            if message.header.frame_id!=self.p['map_frame']:return
            try:
                origin=message.info.origin
                self.display_state.occupancy(message.data,int(message.info.width),int(message.info.height),
                    message.info.resolution,(origin.position.x,origin.position.y,quaternion_yaw(origin.orientation)))
            except (ValueError,TypeError,OverflowError):pass

        def on_display_session(self,message):
            try:
                value=json.loads(message.data)
                self.display_state.session(value['session_id'],value.get('localization_ready'),self.now())
            except (ValueError,KeyError,TypeError):pass

        def on_yolo(self, message):
            stamp = self.stamp(message.header)
            if (message.header.frame_id != self.p['camera_frame'] or stamp <= self.yolo_stamp
                    or not -.1 <= self.now()-stamp <= self.cfg.target_timeout):
                return
            accepted = []
            for detection in message.detections:
                if (label_name(detection.class_name) != self.cfg.target_label
                        or detection.class_id != self.cfg.target_class_id):
                    continue
                box = detection.bbox
                values = (detection.score, box.center.position.x, box.center.position.y, box.size.x, box.size.y)
                if (all(math.isfinite(value) for value in values)
                        and self.p['score_threshold'] <= detection.score <= 1
                        and box.size.x > 0 and box.size.y > 0):
                    accepted.append(values)
            self.yolo, self.yolo_stamp = accepted, stamp

        def on_fusion(self, message):
            now = self.now()
            if not -.1 <= now-self.yolo_stamp <= self.cfg.target_timeout:
                self.controller.observe([], now)
                return
            try:
                dimensions = message.layout.dim
                if dimensions and len(dimensions) != 2:
                    raise ValueError('Fusion layout must contain rows and cols')
                observations = fusion_rows(message.data, self.yolo_stamp,
                    rows=int(dimensions[0].size) if dimensions else None,
                    cols=int(dimensions[1].size) if dimensions else 8,
                    offset=int(message.layout.data_offset))
                # Fusion has no confidence field: verify each row against a fresh YOLO bbox.
                candidates = []
                for observation in observations:
                    if observation.class_id != self.cfg.target_class_id:
                        continue
                    matches = [box for box in self.yolo if math.hypot(observation.u-box[1], observation.v-box[2])
                               <= max(8., .35*min(box[3], box[4]))]
                    if len(matches) == 1:
                        candidates.append(observation)
                self.controller.observe(candidates, now)
            except (ValueError, TypeError, IndexError, OverflowError):
                self.controller.observe([], now)

        def on_scan(self, message):
            stamp = self.stamp(message.header)
            if (message.header.frame_id != self.p['scan_frame'] or not message.ranges
                    or not -.1 <= self.now()-stamp <= self.cfg.scan_timeout
                    or (self.scan is not None and stamp <= self.scan.stamp)
                    or not math.isfinite(message.angle_min) or not math.isfinite(message.angle_increment)
                    or message.angle_increment == 0 or not math.isfinite(message.range_min)
                    or not math.isfinite(message.range_max) or not 0 <= message.range_min < message.range_max):
                return
            self.scan = Scan(np.asarray(message.ranges), message.angle_min, message.angle_increment,
                             max(self.p['avoidance_lidar_min_range'] if course_mode else .2, message.range_min), message.range_max, stamp,
                             math.radians(self.p['scan_yaw_deg']), self.p['scan_x'], self.p['scan_y'])
            if course_mode:
                from .route_mission import limit_course_scan
                self.scan = limit_course_scan(self.scan, self.controller.route_config.avoidance_lidar_range)

        def on_telemetry(self, message):
            try:
                self.telemetry = json.loads(message.data)
                self.telemetry_at = time.monotonic()
            except (ValueError, TypeError):
                self.telemetry = {}

        def on_course_image(self,message):
            if not self.dashboard or time.monotonic()-self.last_camera_at<.35:return
            try:
                frame=self.image_bridge.imgmsg_to_cv2(message,desired_encoding='bgr8').copy()
                current_boxes=self.yolo if -.1<=self.now()-self.yolo_stamp<=self.cfg.target_timeout else []
                for score,x,y,w,h in current_boxes:
                    cv2.rectangle(frame,(int(x-w/2),int(y-h/2)),(int(x+w/2),int(y+h/2)),(70,210,255),2)
                    cv2.putText(frame,self.cfg.target_label+' '+format(score,'.2f'),(int(x-w/2),max(18,int(y-h/2)-5)),cv2.FONT_HERSHEY_SIMPLEX,.55,(70,210,255),2)
                extension=getattr(self.dashboard,'image_extension','.jpg')
                if extension=='.png':
                    frame=cv2.resize(frame,(400,round(frame.shape[0]*400/frame.shape[1])))
                ok,jpeg=cv2.imencode(extension,frame,[] if extension=='.png' else [cv2.IMWRITE_JPEG_QUALITY,75])
                if ok:self.dashboard.camera(jpeg.tobytes())
                self.last_camera_at=time.monotonic()
            except (ValueError,RuntimeError,cv2.error):pass

        def course_commands(self):
            if not self.dashboard:return
            for _ in range(8):
                try:command=self.dashboard.commands.get_nowait()
                except QueueEmpty:break
                action=command['action']
                response=Trigger.Response()
                try:
                    if action in ('start','stop','reset'):
                        getattr(self,action)(Trigger.Request(),response)
                    else:
                        response.message=apply_course_settings(self,command)
                        response.success=True
                except (ValueError,KeyError,TypeError,OverflowError) as error:
                    response.success,response.message=False,str(error)
                with self.dashboard.lock:
                    self.dashboard.last_command=dict(action=action,success=response.success,message=response.message)

        def update(self):
            if (course_mode and self.dashboard and hasattr(self.dashboard,'is_open')
                    and not self.dashboard.is_open()):
                self.desktop_closed=True
                if self.output:self.output.request_stop('Desktop window closed')
                return
            if course_mode:self.course_commands()
            now = self.now()
            if now < self.previous_now-.1:
                self.controller.reset()
                self.scan, self.yolo_stamp = None, -math.inf
                if self.output:
                    self.output.request_stop('ROS time moved backwards')
            self.previous_now = now
            if course_mode:
                self.controller.executing=bool(self.output and self.output.active and self.output.armed)
                self.controller.neutral_confirmed=(time.monotonic()-self.telemetry_at<=.5
                    and telemetry_ready(self.telemetry) and self.telemetry.get('armed') is True
                    and self.telemetry.get('left')==1500 and self.telemetry.get('right')==1500)
            try:
                reason, percentages = self.controller.step(now, self.scan)
                intended = self.controller.intended_pwm if course_mode else native_pwm(percentages, self.native_command_pwm)
                if any(type(pwm) is not int or not 1100<=pwm<=1900 for pwm in intended):
                    raise ValueError('Invalid service PWM')
            except (ValueError, RuntimeError, OverflowError):
                reason, percentages, intended = 'GUIDANCE_INVALID', (0., 0.), (1500, 1500)
            self.last_reason = reason
            if self.output:
                if reason not in self.motion_states and self.output.active:
                    self.output.request_stop(reason)
                self.output.tick(time.monotonic(), intended, self.telemetry, time.monotonic()-self.telemetry_at)
            target = self.controller.target
            report = dict(stamp=now, state=reason, target_label=self.cfg.target_label,
                          target_class_id=self.cfg.target_class_id, stable_frames=self.controller.count,
                          target=asdict(target) if target else None, intended_pwm=list(intended),
                          left_percent=percentages[0], right_percent=percentages[1],
                          native_output='isv_water_trials.actuator_config.command_pwm',
                          output=dict(enabled=self.output is not None,
                                      active=bool(self.output and self.output.active),
                                      armed=bool(self.output and self.output.armed),
                                      stop_ack=bool(self.output and self.output.stop_ack),
                                      pending=bool(self.output and self.output.pending),
                                      stopping=bool(self.output and self.output.stopping),
                                      settings_locked=bool(self.output and (self.output.active or self.output.pending or self.output.stopping or self.output.armed)),
                                      error=self.output.error if self.output else ''))
            if gps_mode:
                report.update(phase=self.controller.phase, gps_distance=self.controller.gps_distance,
                              gps_goal=dict(latitude=self.controller.gps_config.goal_latitude,
                                            longitude=self.controller.gps_config.goal_longitude,
                                            configured=self.controller.gps_config.goal_configured),
                              use_slam=self.p['use_slam'])
            if course_mode:
                fix=self.controller.fix
                route=self.controller.route_config
                report.update(stage=self.controller.stage,wp_index=self.controller.wp_index,
                    waypoints=list(zip(route.wp_latitudes,route.wp_longitudes)),
                    hold_remaining=self.controller.hold_remaining,distance_to_wp=self.controller.distance_to_wp,
                    heading_error_deg=self.controller.heading_error_deg,
                    yaw=self.controller.heading,avoidance=self.controller.avoidance,
                    position=dict(latitude=fix.latitude,longitude=fix.longitude,sigma=fix.sigma) if fix else None,
                    feedback_pwm=[self.telemetry.get('left'),self.telemetry.get('right')]
                        if time.monotonic()-self.telemetry_at<=.5 else None,
                    gps_age=now-fix.stamp if fix else None,
                    scan_age=now-self.scan.stamp if self.scan else None,
                    heading_age=now-self.controller.heading_stamp if math.isfinite(self.controller.heading_stamp) else None,
                    telemetry_age=time.monotonic()-self.telemetry_at if math.isfinite(self.telemetry_at) else None,
                    native_output='supplied PWM tables -> existing paired ThrottlePulseWidth services',
                    command_kind=self.controller.command_kind)
                report['visual_bearing_deg']=self.controller.target_bearing() if self.controller.target else None
                report['dock_geometry']=self.controller.local_dock.report
                report['wp2_brake_remaining']=max(0.,route.wp2_brake_seconds-self.controller.brake_elapsed)
                report['search_turn_direction']=self.controller.search_direction
                report['decision']=self.display_state.angles(report)
                report['slam_display']=self.display_state.snapshot(now,self.scan,self.cfg.scan_timeout)
                report['slam_display']['enabled']=self.p['slam_display']
                if self.dashboard:self.dashboard.publish(report)
            text = String(data=json.dumps(report, allow_nan=False))
            self.status_pub.publish(text)
            self.command_pub.publish(text)

        def start(self, request, response):
            if self.output is None:
                response.success, response.message = False, 'PREVIEW: relaunch with enable_output:=true'
            elif (self.last_reason not in self.motion_states
                  or not telemetry_ready(self.telemetry)
                  or time.monotonic()-self.telemetry_at > .5):
                response.success, response.message = False, 'Wait for stable target, scan and fresh AUTO telemetry'
            else:
                response.success = self.output.start()
                response.message = 'Start requested' if response.success else 'Stop/drain still pending'
            return response

        def stop(self, request, response):
            if self.output:
                self.output.request_stop('operator stop')
            response.success, response.message = True, 'Stop requested; inspect output.stop_ack'
            return response

        def reset(self, request, response):
            if self.output and (self.output.active or self.output.pending or self.output.stopping):
                response.success, response.message = False, 'Stop and wait for acknowledgment first'
            else:
                self.controller.reset()
                response.success, response.message = True, 'Target observations cleared'
            return response

        def external_stop(self, message):
            if self.output and (self.output.active or self.output.armed):
                self.output.request_stop('external /water_trial/stop')

    return ShapeFollower()


def main(args=None, mode='follow'):
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    exiting = False

    def stop_signal(*_):
        nonlocal exiting
        exiting = True

    signals = [signal.SIGINT, signal.SIGTERM]
    if hasattr(signal, 'SIGHUP'):
        signals.append(signal.SIGHUP)
    previous = {item: signal.signal(item, stop_signal) for item in signals}
    node = None
    try:
        node = make_node(mode)
        while rclpy.ok() and not exiting and not node.desktop_closed:
            rclpy.spin_once(node, timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.timer.cancel()
            if node.output and node.output.owned:
                node.output.request_stop('process exit')
                until = time.monotonic()+3
                while rclpy.ok() and time.monotonic() < until and not node.output.stop_ack:
                    node.output.tick(time.monotonic(), (1500, 1500), {}, math.inf)
                    rclpy.spin_once(node, timeout_sec=.02)
                if not node.output.stop_ack:
                    node.get_logger().error('STOP_UNCONFIRMED: MCU acknowledgment missing')
            node.destroy_node()
            if node.output_lock:
                node.output_lock.close()
            if node.dashboard:
                node.dashboard.close()
        if rclpy.ok():
            rclpy.shutdown()
        for item, handler in previous.items():
            signal.signal(item, handler)


def gps_main(args=None):
    main(args, mode='gps_dock')


def course_main(args=None):
    main(args,mode='course')


if __name__ == '__main__':
    main()
