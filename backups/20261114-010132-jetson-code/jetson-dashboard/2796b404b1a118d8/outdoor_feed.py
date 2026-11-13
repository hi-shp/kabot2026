"""Bounded browser requests -> ROS owner; acknowledgements come from the owner."""
from concurrent.futures import Future, TimeoutError
from queue import Queue, Empty, Full
import json
import math
import time
import uuid
import os
from pathlib import Path


def validated_route(data):
    from waypoint_route import normalize_route
    return normalize_route(data)


def validate_drive_limits(values):
    result={}
    for name in ('point','dock','path'):
        value=values[name]
        if isinstance(value,bool):raise ValueError('출력은 0~100%로 입력하세요')
        value=float(value)
        if not math.isfinite(value) or not 0<=value<=100:raise ValueError('출력은 0~100%로 입력하세요')
        result[name]=value
    return result


def read_drive_limits(path,default=20.):
    try:return validate_drive_limits(json.loads(Path(path).read_text(encoding='utf-8')))
    except FileNotFoundError:return dict(point=default,dock=default,path=default)


class OutdoorFeed:
    def __init__(self, node, store, route_path=None):
        from std_msgs.msg import String
        self.kind = String; self.node = node; self.store = store
        self.route_path=Path(route_path) if route_path is not None else Path.home()/'water_trials/outdoor/route.json'
        from ament_index_python.packages import get_package_share_directory
        from isv_water_trials.outdoor_config import load_config
        navigation=load_config(Path(get_package_share_directory('isv_water_trials'))/'config/outdoor.json')
        self.arrival_radius_default=float(navigation['arrival_radius_m']);self.drive_percent_default=float(navigation['percent'])
        self.publisher = node.create_publisher(String, '/outdoor/request', 10)
        self.hull_publisher = node.create_publisher(String, '/lidar/self_mask/request', 10)
        from sensor_msgs.msg import NavSatFix
        self.gps_kind = NavSatFix
        self.virtual_publisher = node.create_publisher(NavSatFix, '/gps/virtual/fix', 10)
        self.virtual_point = None
        self.virtual_sequence = 0
        self.trail_after_t = 0.
        node.create_timer(.2, self.publish_virtual_gps)
        self.virtual_status()
        self.route_status()
        node.create_timer(.5,self.route_status)
        self.queue = Queue(maxsize=16); node.create_timer(.1, self.work)

    def route_status(self):
        self.navigation_status()
        try:
            cutoff = float(json.loads((self.route_path.parent/'display.json').read_text()).get('trail_after_t',0.))
            if math.isfinite(cutoff): self.trail_after_t = cutoff
        except (OSError,ValueError,TypeError): pass
        route=None;error=None
        try:route=validated_route(json.loads(self.route_path.read_text(encoding='utf-8')))
        except FileNotFoundError:pass
        except (OSError,ValueError,TypeError,AttributeError) as exc:error=str(exc)
        self.store.value('saved_route',dict(route=route,error=error,available=True,path=str(self.route_path)))

    def navigation_status(self):
        from isv_water_trials.outdoor_config import read_arrival_radius
        try:radius=read_arrival_radius(self.route_path.parent/'gps_arrival.json',self.arrival_radius_default)
        except (OSError,ValueError,KeyError,TypeError):radius=self.arrival_radius_default
        self.store.value('gps_arrival',dict(arrival_radius_m=radius))
        try:limits=read_drive_limits(self.route_path.parent/'drive_limits.json',self.drive_percent_default)
        except (OSError,ValueError,KeyError,TypeError):limits=dict(point=self.drive_percent_default,dock=self.drive_percent_default,path=self.drive_percent_default)
        self.store.value('drive_limits',dict(limits=limits,version=2))

    def save_without_owner(self,data):
        """Persist the same route file while no navigation owner holds its lock."""
        import fcntl
        try:
            route=validated_route(data)
            self.route_path.parent.mkdir(parents=True,exist_ok=True)
            with (self.route_path.parent/'owner.lock').open('a') as owner_lock:
                try:fcntl.flock(owner_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
                except BlockingIOError:return 503,{'error':'주행 노드 전환 중입니다. 잠시 후 다시 저장하세요'}
                temporary=self.route_path.with_name('.route-'+uuid.uuid4().hex+'.incoming')
                try:
                    with temporary.open('w',encoding='utf-8') as stream:
                        json.dump(route,stream,ensure_ascii=False,allow_nan=False);stream.flush();os.fsync(stream.fileno())
                    os.replace(temporary,self.route_path)
                finally:
                    if temporary.exists():temporary.unlink()
            self.route_status()
            return 200,dict(saved=True,route=route,path=str(self.route_path),navigation_running=False)
        except (ValueError,TypeError,AttributeError) as exc:return 400,{'error':str(exc)}
        except OSError as exc:return 500,{'error':'경로 파일 저장 실패: '+str(exc)}

    def virtual_status(self):
        self.store.value('virtual_gps',dict(active=self.virtual_point is not None,
                         point=self.virtual_point,topic='/gps/virtual/fix',rate_hz=5,
                         published=self.virtual_sequence))

    def publish_virtual_gps(self):
        if self.virtual_point is not None:
            msg = self.gps_kind()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = 'virtual_gps'
            msg.latitude, msg.longitude = self.virtual_point
            msg.altitude = 0.
            msg.status.status = 0  # NavSatStatus.STATUS_FIX; synthetic source is explicit in topic/frame.
            msg.status.service = 1
            msg.position_covariance = [.25,0.,0.,0.,.25,0.,0.,0.,1.]
            msg.position_covariance_type = 2
            self.virtual_publisher.publish(msg)
            self.virtual_sequence += 1
        self.virtual_status()

    def request(self, data):
        from waypoint_route import WaypointRoutes,normalize_route
        library=WaypointRoutes(self.route_path)
        if data.get('action')=='restart_point_follow':
            try:
                route=validated_route(data.get('route'))
                index=data.get('start_index')
                if type(index) is not int or not 0<=index<len(route['points']):
                    raise ValueError('시작할 웨이포인트를 선택하세요')
                data=dict(data,route=route)
            except (ValueError,TypeError,AttributeError) as exc:return 400,{'error':str(exc)}
        if data.get('action') in ('start','start_point_follow','start_model_test'):
            selected=library.selected()
            if selected and any(p is None for p in selected['route']['points']):
                return 409,{'error':'토너먼트 지점 좌표를 지정하고 경로를 저장하세요'}
        if data.get('action')=='load_route':
            with self.store.lock:status=self.store.values.get('outdoor',{})
            if status.get('execute') and time.monotonic()-status.get('received_monotonic',0)<3:
                return 409,{'error':'주행을 종료한 뒤 저장 경로를 불러오세요'}
            try:
                profile=library.read(data.get('name'))
                if any(p is None for p in profile['route']['points']):
                    library.select(profile)
                    return 200,dict(draft_saved=True,loaded_profile={k:v for k,v in profile.items() if k!='route'},loaded_route=profile['route'])
                status,response=self.request(dict(action='route',**profile['route']))
                if status<300:
                    library.select(profile)
                    response=dict(response,loaded_profile={k:v for k,v in profile.items() if k!='route'},loaded_route=profile['route'])
                return status,response
            except (OSError,ValueError,KeyError,TypeError) as exc:return 400,{'error':str(exc)}
        if data.get('action')=='route':
            try:
                draft=normalize_route(data,allow_draft=True)
                if any(p is None for p in draft['points']):
                    selected=library.selected()
                    name=data.get('archive_name') or (selected['name'] if selected else None)
                    profile=library.save(draft,name=name)
                    library.select(profile)
                    return 200,dict(draft_saved=True,saved_profile={k:v for k,v in profile.items() if k!='route'},draft_route=draft)
            except (OSError,ValueError,TypeError,AttributeError) as exc:return 400,{'error':str(exc)}
            try:data=dict(data,**validated_route(data))
            except (ValueError,TypeError,AttributeError) as exc:return 400,{'error':str(exc)}
            try:
                # Keep the old multi-point course before a map click replaces it.
                try:previous=validated_route(json.loads(self.route_path.read_text(encoding='utf-8')))
                except (OSError,ValueError,TypeError,AttributeError):previous=None
                selected=library.selected()
                archive_name=data.get('archive_name') or (selected['name'] if selected else None)
                if previous and len(previous['points'])>1 and previous!=validated_route(data) and not (data.get('archive') is True and archive_name):
                    library.save(previous,label='이전 경로 자동 보관',source=str(self.route_path))
                profile=library.save(data,name=archive_name) if data.get('archive') is True else None
                if profile:library.select(profile)
            except (OSError,ValueError) as exc:return 500,{'error':'경로 보관 실패: '+str(exc)}
        virtual = data.get('action') in ('virtual_gps_start','virtual_gps_stop')
        if data.get('action') in ('start','start_model_test') and self.virtual_point is not None:
            return 409, {'error':'가상 GPS를 종료하고 실제 배 위치로 주행을 시작하세요'}
        if data.get('action')=='virtual_gps_start':
            with self.store.lock:
                status=self.store.values.get('outdoor',{})
                virtual_follow=(status.get('navigation_mode')=='point_follow' and status.get('point_follow_position_source')=='virtual')
                if status.get('execute') and not virtual_follow and time.monotonic()-status.get('received_monotonic',0)<3:
                    return 409, {'error':'주행 정지 후 가상 GPS를 사용할 수 있습니다'}
        if virtual and data['action']=='virtual_gps_start':
            point=data.get('point')
            if (not isinstance(point,list) or len(point)!=2 or
                any(type(v) not in (int,float) or not math.isfinite(v) for v in point) or
                not -90<=point[0]<=90 or not -180<=point[1]<=180):
                return 400, {'error':'지도에서 가상 GPS 위치를 선택하세요'}
        if not virtual and data.get('action') not in ('pause','resume','apply_model','freeze','load','route','regions','start','stop','abort_docking','clear_trail','gps_arrival','drive_limits','start_model_test','start_point_follow','restart_point_follow','point_follow_limit','hull_start','hull_stop','hull_save','hull_disable'):
            return 400, {'error': '알 수 없는 요청'}
        if not virtual and data.get('action') not in ('clear_trail','gps_arrival','drive_limits'):
            with self.store.lock:
                status = self.store.values.get('hull_mask' if data['action'].startswith('hull_') else 'outdoor', {})
                stale=time.monotonic()-status.get('received_monotonic', 0)>3
            if stale:
                if data['action']=='route':
                    code,response=self.save_without_owner(data)
                    if code<300 and profile:response=dict(response,saved_profile={k:v for k,v in profile.items() if k!='route'})
                    return code,response
                return 503, {'error': '브링업의 야외 기록 노드 연결 대기'}
            if data['action']=='restart_point_follow' and not status.get('waypoint_restart_schema'):
                return 409,{'error':'선택 지점 시작 코드를 적용한 브링업에서 사용하세요'}
            if (data['action']=='route' and any(k!='path' for k in data.get('waypoint_types',[])) and
                not status.get('waypoint_schema')):
                return 409,{'error':'현재 주행은 이전 코드로 실행 중입니다. 주행 종료 후 센서 브링업을 다시 켜면 미션 지점 저장이 적용됩니다.'}
            if data['action'] in ('start_point_follow','restart_point_follow') and 'drive_limits' in data and not isinstance(status.get('drive_limits'),dict):
                return 409,{'error':'젯슨 주행 노드에 새 출력 설정 코드를 적용한 뒤 시작하세요.'}
            if data['action'] in ('route','start_point_follow','restart_point_follow'):
                candidate=data if data['action']=='route' else data['route'] if data['action']=='restart_point_follow' else status.get('route',{})
                if 'orbit' in candidate.get('waypoint_types',[]) and not status.get('buoy_orbit_schema'):
                    return 409,{'error':'선회 미션 코드를 적용한 브링업에서 사용하세요.'}
                if any(k in ('dock_left','dock_middle','dock_right') for k in candidate.get('waypoint_types',[])) and not status.get('registered_gps_docking_schema'):
                    return 409,{'error':'GPS left·middle·right 도킹 코드를 적용한 브링업에서 사용하세요.'}
            if data['action'] in ('route','start_point_follow','restart_point_follow'):
                route=data if data['action']=='route' else data['route'] if data['action']=='restart_point_follow' else status.get('route',{})
                if any(v is not None for v in route.get('waypoint_outputs',[])) and not status.get('waypoint_output_schema'):
                    return 409,{'error':'지점별 출력 코드를 적용한 브링업에서 사용하세요.'}
        request_id = uuid.uuid4().hex
        future = Future()
        try: self.queue.put_nowait((dict(data, request_id=request_id), future))
        except Full: return 429, {'error': '요청 처리 대기 중'}
        try:
            code,response=future.result(timeout=2)
            if data['action']=='route' and code<300 and profile:response=dict(response,saved_profile={k:v for k,v in profile.items() if k!='route'})
            return code,response
        except TimeoutError:
            future.cancel(); return 503, {'error': '요청 전송 시간 초과'}

    def work(self):
        try: data, future = self.queue.get_nowait()
        except Empty: return
        if not future.set_running_or_notify_cancel(): return
        if data['action']=='drive_limits':
            try:
                from isv_water_trials.tank_storage import atomic_json
                limits=validate_drive_limits(data.get('limits',{}))
                path=self.route_path.parent/'drive_limits.json';path.parent.mkdir(parents=True,exist_ok=True)
                atomic_json(path,limits);self.navigation_status()
                future.set_result((200,dict(drive_limits_saved=True,limits=limits)))
            except (ValueError,TypeError,KeyError) as exc:future.set_result((400,{'error':str(exc)}))
            except OSError as exc:future.set_result((500,{'error':str(exc)}))
            return
        if data['action']=='gps_arrival':
            try:
                from isv_water_trials.outdoor_config import validate_arrival_radius
                from isv_water_trials.tank_storage import atomic_json
                radius=validate_arrival_radius(data.get('arrival_radius_m'))
                path=self.route_path.parent/'gps_arrival.json'
                path.parent.mkdir(parents=True,exist_ok=True)
                atomic_json(path,dict(arrival_radius_m=radius))
                self.navigation_status()
                future.set_result((200,dict(gps_arrival_saved=True,arrival_radius_m=radius)))
            except (ValueError,TypeError,KeyError) as exc:future.set_result((400,{'error':str(exc)}))
            except OSError as exc:future.set_result((500,{'error':str(exc)}))
            return
        if data['action']=='clear_trail':
            try:
                from isv_water_trials.tank_storage import atomic_json
                cutoff = self.node.get_clock().now().nanoseconds/1e9
                path = self.route_path.parent/'display.json'
                try: display = json.loads(path.read_text())
                except (OSError,ValueError): display = {}
                display['trail_after_t'] = cutoff
                atomic_json(path,display)
                self.trail_after_t = cutoff
                with self.store.lock:
                    status = self.store.values.get('outdoor')
                    if status is not None:
                        self.store.values['outdoor'] = dict(status,trail=[],trail_after_t=cutoff)
                if self.publisher.get_subscription_count():
                    self.publisher.publish(self.kind(data=json.dumps(data,allow_nan=False)))
                future.set_result((200,{'trail_cleared':True,'trail_after_t':cutoff,'request_id':data['request_id']}))
            except (OSError,ValueError,TypeError) as exc:
                future.set_result((500,{'error':str(exc)}))
            return
        if data['action'] in ('virtual_gps_start','virtual_gps_stop'):
            self.virtual_point = list(data['point']) if data['action']=='virtual_gps_start' else None
            if self.virtual_point is None:
                msg=self.gps_kind();msg.header.stamp=self.node.get_clock().now().to_msg()
                msg.header.frame_id='virtual_gps';msg.status.status=-1
                self.virtual_publisher.publish(msg)
            self.publish_virtual_gps()
            future.set_result((200,{'request_id':data['request_id'],'simulation':True,
                                   'active':self.virtual_point is not None,'topic':'/gps/virtual/fix'}))
            return
        publisher=self.hull_publisher if data['action'].startswith('hull_') else self.publisher
        if data['action'] in ('start_point_follow','restart_point_follow'):
            data['position_source']='virtual' if self.virtual_point is not None else 'gps'
        if publisher.get_subscription_count() < 1:
            future.set_result((503, {'error': '야외 노드 연결 대기'})); return
        publisher.publish(self.kind(data=json.dumps(data, allow_nan=False)))
        future.set_result((202, {'request_id': data['request_id'], 'pending': True}))
