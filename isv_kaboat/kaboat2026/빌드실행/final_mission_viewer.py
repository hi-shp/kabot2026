"""Read-only dashboard for gps_follow_stage1; never sends mission or actuator commands."""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import json
import math
import threading
import time
from mission_gps_editor import GPSConfigEditor
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent.parent/'주행코드'))
from buoy_search_circle import buoy_circle_points


def configured_buoy_orbits(stages):
    stage = stages.get(5, {})
    result = []
    for name, center in stage.get('gps_by_buoy', {}).items():
        if (not isinstance(center, (list, tuple)) or len(center) != 2
                or not all(type(v) in (int, float) and math.isfinite(v) for v in center)):
            continue
        try:
            points = buoy_circle_points(*center, stage.get('orbit_point_count', 8),
                                       radius_m=stage.get('orbit_radius_m', 2.0))
        except (ValueError, TypeError):
            continue
        result.append(dict(name=name, center=list(center), points=points,
                           radius_m=stage.get('orbit_radius_m', 2.0),
                           direction=stage.get('orbit_direction_by_buoy', {}).get(name)))
    return result

MISSIONS = ['경로 시작점 접근', '경로 추종', '호핑·장애물 회피', '도킹 시작점·장애물 회피',
            '도킹', '부표 탐색', '회피 시작점 접근', '장애물 회피 주행']


def configured_map_points(stages):
    points = []

    def add(stage, label, pair):
        if (isinstance(pair, (tuple, list)) and len(pair) == 2
                and all(type(v) in (float, int) and math.isfinite(v) for v in pair)
                and abs(pair[0]) <= 85.05112878 and abs(pair[1]) <= 180):
            points.append(dict(status=stage, label=label, gps=list(pair)))

    for stage, values in sorted(stages.items()):
        if stage == 1:
            for index, pair in enumerate(values.get('gps_points', []), 1):
                add(stage, f'1 경로 {index}', pair)
        elif stage in (4, 5):
            key = 'gps_by_shape' if stage == 4 else 'gps_by_buoy'
            prefix = '4 도킹' if stage == 4 else '5 탐색'
            for name, pair in values.get(key, {}).items():
                add(stage, f'{prefix} · {name}', pair)
        elif stage == 7:
            for index, pair in enumerate(values.get('intermediate_gps', []), 1):
                add(stage, f'7 회피 중간 {index}', pair)
            add(stage, '7 회피 최종', values.get('gps'))
        else:
            label = {0: '0 경로 시작', 2: '2 호핑', 3: '3 도킹 시작', 6: '6 회피 시작'}
            add(stage, label.get(stage, str(stage)), values.get('gps'))
    return points


class ViewerState:
    def __init__(self):
        self.lock = threading.RLock()
        self.mission = {}
        self.mission_at = -math.inf
        self.gps = None
        self.gps_at = -math.inf
        self.yaw = None
        self.imu_at = -math.inf
        self.actuator = {}
        self.actuator_at = -math.inf
        self.scan = []
        self.scan_at = -math.inf
        self.scan_source = ''
        self.jpeg = None
        self.camera_at = -math.inf
        self.configured_points = []
        self.buoy_orbits = []
        self.gps_editor = None

    def update_mission(self, value, now=None):
        if not isinstance(value, dict) or type(value.get('status')) is not int or not 0 <= value['status'] <= 7:
            return False
        with self.lock:
            self.mission = value
            self.mission_at = time.monotonic() if now is None else now
        return True

    def snapshot(self, now=None):
        now = time.monotonic() if now is None else now
        def age(at):
            return max(0., now-at) if math.isfinite(at) else None
        with self.lock:
            mission = dict(self.mission)
            target = [mission.get('goal_latitude'), mission.get('goal_longitude')]
            valid_goal = all(isinstance(v, (int, float)) and math.isfinite(v) for v in target)
            return dict(mission=mission, mission_age=age(self.mission_at), missions=MISSIONS,
                        position=self.gps, gps_age=age(self.gps_at), yaw=self.yaw, imu_age=age(self.imu_at),
                        actuator=dict(self.actuator), actuator_age=age(self.actuator_at),
                        camera_age=age(self.camera_at), scan=self.scan, scan_age=age(self.scan_at),
                        scan_source=self.scan_source, waypoints=[target] if valid_goal else [],
                        configured_points=self.configured_points, buoy_orbits=self.buoy_orbits)


def create_server(state, port, web_root=None):
    root = Path(web_root) if web_root else Path(__file__).resolve().parent/'final_view_assets'
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def send(self, code, data, kind='application/json; charset=utf-8'):
            self.send_response(code)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            path = self.path.split('?', 1)[0]
            if path == '/api/gps-config' and state.gps_editor:
                self.send(200, json.dumps(state.gps_editor.snapshot(), allow_nan=False).encode())
            elif path == '/api/state':
                self.send(200, json.dumps(state.snapshot(), allow_nan=False).encode())
            elif path == '/api/viewer':
                import os
                self.send(200, json.dumps(dict(app='isv_final_mission_dashboard', protocol=1,
                                               pid=os.getpid(), read_only=True)).encode())
            elif path == '/camera.jpg':
                with state.lock:
                    jpeg = state.jpeg
                self.send(200 if jpeg else 204, jpeg or b'', 'image/jpeg')
            elif path in ('/', '/app.js', '/style.css'):
                name = 'index.html' if path == '/' else path[1:]
                kind = {'index.html': 'text/html', 'app.js': 'application/javascript', 'style.css': 'text/css'}[name]
                self.send(200, (root/name).read_bytes(), kind+'; charset=utf-8')
            else:
                self.send(404, b'{}')

        def do_POST(self):
            from urllib.parse import urlsplit
            if self.path != '/api/gps-config' or state.gps_editor is None:
                self.send(403, b'{"error":"Mission controls are read-only"}')
                return
            origin = self.headers.get('Origin')
            if ((origin and urlsplit(origin).netloc != self.headers.get('Host'))
                    or self.headers.get('Content-Type', '').split(';')[0] != 'application/json'):
                self.send(403, b'{"error":"Same-origin JSON required"}')
                return
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 4096:
                    raise ValueError('Invalid request size')
                result = state.gps_editor.save(json.loads(self.rfile.read(length)))
                with state.lock:
                    stages = result.pop('stages')
                    state.configured_points = configured_map_points(stages)
                    state.buoy_orbits = configured_buoy_orbits(stages)
                self.send(200, json.dumps(result, allow_nan=False).encode())
            except FileExistsError:
                self.send(409, json.dumps({'error': '설정이 변경되었습니다. 목록을 다시 불러온 후 저장하세요.'}).encode())
            except (ValueError, TypeError, KeyError, SyntaxError) as exc:
                self.send(400, json.dumps({'error': str(exc)}).encode())
            except OSError:
                self.send(500, json.dumps({'error': '설정 저장 실패. 파일 권한과 저장 공간을 확인하세요.'}).encode())

    return ThreadingHTTPServer(('127.0.0.1', port), Handler)


def run(port=8771, gps_topic='/gps/fix', image_topic='/yolo/dbg_image/compressed'):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import NavSatFix, LaserScan, CompressedImage, Image
    from vectornav_msgs.msg import CommonGroup
    from std_msgs.msg import String
    from cv_bridge import CvBridge
    import cv2
    import runpy
    state = ViewerState()
    root = Path(__file__).resolve().parent.parent
    config = runpy.run_path(str(root/'주행코드/mission_config.py'))
    state.configured_points = configured_map_points(config['STATUS_CONFIG'])
    state.buoy_orbits = configured_buoy_orbits(config['STATUS_CONFIG'])
    state.gps_editor = GPSConfigEditor(root/'주행코드/mission_config.py')
    common = config['COMMON_CONFIG']
    import yaml
    v3 = yaml.safe_load(Path(common['v3_settings_file']).expanduser().read_text())['course_mission']['ros__parameters']
    rclpy.init()
    node = rclpy.create_node('isv_final_mission_viewer')
    bridge = CvBridge()
    server = None
    thread = None
    def ros_age(header):
        return node.get_clock().now().nanoseconds/1e9-header.stamp.sec-header.stamp.nanosec/1e9

    def mission(msg):
        try:
            state.update_mission(json.loads(msg.data))
        except (ValueError, TypeError):
            pass

    def gps(msg):
        if (msg.status.status < 0 or not all(math.isfinite(v) for v in (msg.latitude, msg.longitude))
                or not -90 <= msg.latitude <= 90 or not -180 <= msg.longitude <= 180
                or not -.1 <= ros_age(msg.header) <= common['gps_timeout']):
            return
        with state.lock:
            state.gps = dict(latitude=msg.latitude, longitude=msg.longitude)
            state.gps_at = time.monotonic()

    def imu(msg):
        if (msg.header.frame_id != common['heading_frame'] or not math.isfinite(msg.yawpitchroll.x)
                or not -.1 <= ros_age(msg.header) <= common['heading_timeout']):
            return
        with state.lock:
            state.yaw = math.pi/2-math.radians(msg.yawpitchroll.x+common['heading_offset_deg'])
            state.imu_at = time.monotonic()

    def feedback(msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                return
            with state.lock:
                state.actuator = value
                state.actuator_at = time.monotonic()
        except (ValueError, TypeError):
            pass

    def camera(msg):
        now = time.monotonic()
        if now-state.camera_at < .3:
            return
        try:
            if not -.1 <= ros_age(msg.header) <= 2.:
                return
            decode = bridge.compressed_imgmsg_to_cv2 if image_topic.endswith('/compressed') else bridge.imgmsg_to_cv2
            frame = decode(msg, desired_encoding='bgr8')
            h, w = frame.shape[:2]
            frame = cv2.resize(frame, (800, round(h*800/w)))
            ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if ok:
                with state.lock:
                    state.jpeg = jpeg.tobytes()
                    state.camera_at = now
        except (ValueError, RuntimeError, cv2.error):
            pass

    primary = v3.get('scan_topic', '/scan_preprocessed')
    primary_at = [-math.inf]
    def scan(msg, topic):
        now = time.monotonic()
        if (topic != primary and now-primary_at[0] < v3.get('scan_timeout', .5)):
            return
        if (msg.header.frame_id != v3.get('scan_frame', 'velodyne')
                or not -.1 <= ros_age(msg.header) <= v3.get('scan_timeout', .5)
                or not math.isfinite(msg.angle_min) or not math.isfinite(msg.angle_increment)):
            return
        points = []
        c, s = math.cos(math.radians(v3.get('scan_yaw_deg', 0.))), math.sin(math.radians(v3.get('scan_yaw_deg', 0.)))
        for i in range(0, len(msg.ranges), max(1, len(msg.ranges)//720)):
            distance = msg.ranges[i]
            if math.isfinite(distance) and msg.range_min <= distance <= min(msg.range_max, 10.):
                angle = msg.angle_min+i*msg.angle_increment
                x, y = distance*math.cos(angle), distance*math.sin(angle)
                points.append([c*x-s*y+v3.get('scan_x', 0.), s*x+c*y+v3.get('scan_y', 0.)])
        with state.lock:
            state.scan, state.scan_at, state.scan_source = points, now, topic
        if topic == primary:
            primary_at[0] = now

    try:
        server = create_server(state, port)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        subscriptions = [
            node.create_subscription(String, '/gps_stage1_follower/status', mission, 10),
            node.create_subscription(NavSatFix, gps_topic, gps, qos_profile_sensor_data),
            node.create_subscription(CommonGroup, common['heading_topic'], imu, qos_profile_sensor_data),
            node.create_subscription(String, '/actuators/status', feedback, qos_profile_sensor_data),
            node.create_subscription(CompressedImage if image_topic.endswith('/compressed') else Image,
                                     image_topic, camera, qos_profile_sensor_data),
        ]
        for topic in dict.fromkeys([primary, v3.get('scan_fallback_topic', '/scan')]):
            subscriptions.append(node.create_subscription(LaserScan, topic,
                                 lambda msg, topic=topic: scan(msg, topic), qos_profile_sensor_data))
        print(f'FINAL MISSION GUI: http://127.0.0.1:{server.server_address[1]} | read-only | status 0~7', flush=True)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if server:
            server.shutdown()
            server.server_close()
        if thread:
            thread.join(timeout=2)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
