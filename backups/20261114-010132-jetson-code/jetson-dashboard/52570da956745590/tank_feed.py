"""Map display and route/initial-pose editing. This module has no actuator API."""
from concurrent.futures import Future, TimeoutError
import json
import math
from pathlib import Path
import re
from queue import SimpleQueue, Empty
import struct
import threading
import time
import uuid


def position_edit_blocked(values):
    now=time.monotonic();trial=values.get('tank_status',{});mcu=values.get('actuators',{})
    rc=bool(mcu.get('receiving') and mcu.get('mode')==1 and now-mcu.get('received_monotonic',0)<.5)
    return bool(trial.get('execute') and now-trial.get('received_monotonic',0)<3 and not rc)


class TankFeed:
    def __init__(self, node, store):
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from std_msgs.msg import Empty
        self.node, self.store = node, store
        self.pose_type = PoseWithCovarianceStamped
        self.publisher = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        self.empty_type=Empty;self.localize_publisher=node.create_publisher(Empty,'/tank/localize/request',1)
        self.pending = SimpleQueue(); self.lock = threading.RLock()
        self.grid = None; self.body = b''; self.sequence = 0
        self.stream_id = uuid.uuid4().hex
        self.last_geometry = None; self.last_cells = None
        self.route_checked = 0.; self.route_mtime = None
        self.reset_marker = None; self.reset_after_ns = 0
        self.map_jobs = {}; self.map_jobs_lock = threading.RLock()
        self.checkpoint=None;self.checkpoint_checked=0.
        node.create_timer(.1, self.work)

    def docking_routes(self, data=None, query=None):
        from docking_routes import DockingRoutes
        library=DockingRoutes()
        if data is None:return 200,{'routes':library.files(),'applied':library.applied()}
        if data.get('action')=='save':
            return self.request('/api/route',dict(data,profile_version=data.get('version')))
        if data.get('action')=='apply':
            return self.request('/api/apply_docking_route',data)
        if data.get('action')!='load':return 400,{'error':'Unknown docking route action.'}
        try:
            profile=library.read(str(data.get('path','')))
            path=library.map_path(profile['map_file'])
        except (OSError,ValueError,KeyError,TypeError) as exc:
            return 400,{'error':str(exc)}
        with self.map_jobs_lock:
            if any(job.get('state')=='running' for job in self.map_jobs.values()):
                return 409,{'error':'A map operation is already running.'}
            job_id=uuid.uuid4().hex
            self.map_jobs[job_id]={'job_id':job_id,'action':'load_route','state':'running',
                                    'message':'도킹 경로를 불러오는 중'}
        threading.Thread(target=self._run_map_job,args=(job_id,'load_route',path,None,profile),daemon=True).start()
        return 202,{'job_id':job_id,'state':'running'}

    def maps(self, data=None, query=None):
        root=Path.home()/'water_trials/tank_maps';root.mkdir(parents=True,exist_ok=True)
        query=query or {}
        if data is None:
            job_id=query.get('job',[''])[0]
            if job_id:
                with self.map_jobs_lock: job=dict(self.map_jobs.get(job_id,{}))
                if not job:return 404,{'error':'지도 작업을 찾을 수 없습니다.'}
                return 200,job
            files=sorted(root.glob('*.tankmap'),key=lambda p:p.stat().st_mtime,reverse=True)
            active=None
            try:active=json.loads((root/'active.json').read_text(encoding='utf-8')).get('file')
            except (OSError,ValueError):pass
            active_path=Path(active).resolve() if active else None
            return 200,{'maps':[{'name':p.stem,'path':str(p.resolve()),'active':p.resolve()==active_path} for p in files]}
        action=data.get('action')
        if action not in ('create','save','load','enable','disable','erase'):return 400,{'error':'알 수 없는 지도 작업입니다.'}
        path=None;crop=None
        if action=='save' and data.get('crop') is not None:
            try:
                crop=[float(v) for v in data['crop']]
                if len(crop)!=4 or not all(math.isfinite(v) for v in crop):raise ValueError()
                if crop[0]>=crop[2] or crop[1]>=crop[3]:raise ValueError()
            except (TypeError,ValueError):return 400,{'error':'저장할 직사각형 구역을 다시 지정하세요.'}
        if action in ('save','load','erase'):
            try:
                if action=='save':
                    name=str(data.get('name','')).strip()
                    if not 1<=len(name)<=48 or not re.fullmatch(r'[\w가-힣 .-]+',name):raise ValueError()
                    path=(root/(name+'.tankmap')).resolve()
                    if path.parent!=root.resolve():raise ValueError()
                else:
                    path=Path(str(data.get('path',''))).resolve()
                    if path.parent!=root.resolve() or path.suffix!='.tankmap' or not path.is_file():raise ValueError()
            except (ValueError,OSError):return 400,{'error':'지도 이름 또는 저장 파일을 확인해 주세요.'}
        with self.map_jobs_lock:
            if any(job.get('state')=='running' for job in self.map_jobs.values()):
                return 409,{'error':'다른 지도 작업이 진행 중입니다.'}
            job_id=uuid.uuid4().hex
            self.map_jobs[job_id]={'job_id':job_id,'action':action,'state':'running','message':'지도 작업 시작'}
        threading.Thread(target=self._run_map_job,args=(job_id,action,path,crop,data if action=='erase' else None),daemon=True).start()
        return 202,{'job_id':job_id,'state':'running','message':{'create':'새 LiDAR SLAM 지도 시작 중','save':'고정 지도로 저장 중','load':'저장 지도 불러오는 중','enable':'SLAM 켜는 중','disable':'SLAM 끄는 중','erase':'지운 지도와 공통 도킹 경로 저장 중'}[action]}

    def _run_map_job(self,job_id,action,path,crop=None,profile=None):
        try:
            from isv_water_trials.tank_session import main, running
            if running('trial'):
                raise RuntimeError('왕복 주행을 종료한 뒤 지도 작업을 시작하세요.')
            with self.store.lock:
                outdoor=self.store.values.get('outdoor',{})
                priority=outdoor.get('priority_route') or {}
                if priority.get('state') in ('neutralizing','docking'):
                    raise RuntimeError('도킹을 종료한 뒤 지도 작업을 시작하세요.')
                if action=='erase' and outdoor.get('execute'):
                    raise RuntimeError('주행을 종료한 뒤 저장 지도를 지워 주세요.')
            if action=='erase':
                from docking_routes import erase_saved_map
                edited=erase_saved_map(path,profile.get('map_id'),profile.get('strokes',[]))
                previous=running('mapping')
                if edited['erased_cells'] and previous:
                    main(['stop','mapping'])
                    if running('mapping'):raise RuntimeError('지도는 저장됐습니다. SLAM 종료 상태를 확인하세요.')
                    main(['start','mapping','--',*previous.get('args',[])])
                if edited.get('route'):self.store.value('tank_route',edited['route'])
                # Read back the committed file instead of keeping the pre-edit grid.
                self.restore_saved_map(path,edited['map_id'])
            elif action in ('enable','disable'):
                if action=='enable':
                    if not self.store.sensors_running():
                        raise RuntimeError('센서 브링업을 먼저 켜 주세요.')
                    from jetson_server import runtime_status
                    if running('mapping') and runtime_status()['mapping']=='failed':main(['stop','mapping'])
                    main(['start','mapping'])
                    if not running('mapping'):raise RuntimeError('SLAM 기동 실패 · mapping.log를 확인하세요.')
                else:
                    main(['stop','mapping'])
                    if running('mapping'):raise RuntimeError('SLAM 종료 대기 중입니다.')
                    with self.lock:
                        self.grid=None;self.body=b'';self.sequence+=1
                        self.stream_id=uuid.uuid4().hex;self.last_geometry=None;self.last_cells=None
                        with self.store.lock:
                            for key in ('map','tank_pose','tank_session','tank_status','tank_localization_candidates'):
                                self.store.values.pop(key,None)
                from jetson_server import runtime_status
                self.store.value('runtime',{'services':runtime_status()})
            elif action=='load_route':
                from docking_routes import atomic_json, DockingRoutes
                from isv_water_trials.tank_map_file import ACTIVE, activate, read_bundle
                identity,_,_=read_bundle(path)
                if identity!=profile['map_id']:
                    raise ValueError('연결된 SLAM 지도 파일이 변경되었습니다. 해당 지도에서 경로를 다시 저장하세요.')
                try:active=json.loads(ACTIVE.read_text(encoding='utf-8'))
                except (OSError,ValueError):active={}
                same_map=active.get('file')==str(path) and active.get('map_id')==identity
                route=dict(profile['route'],session_id=identity,frame_id='tank_map',
                           revision=uuid.uuid4().hex,updated_unix=time.time())
                if not same_map:activate(path)
                atomic_json(Path.home()/'water_trials/tank/route.json',route)
                self.store.value('tank_route',route)
                from jetson_server import runtime_status
                if not same_map or not running('mapping') or runtime_status()['mapping']=='failed':
                    main(['stop','mapping']);main(['start','mapping'])
                applied=DockingRoutes().apply(profile)
                route=applied['route']
                atomic_json(Path.home()/'water_trials/tank/route.json',route)
                self.store.value('tank_route',route)
            elif action=='create':
                main(['stop','mapping'])
                if running('mapping'):
                    raise RuntimeError('기존 지도 프로세스 종료 대기 중입니다.')
                from isv_water_trials.tank_map_file import ACTIVE
                from isv_water_trials.tank_storage import atomic_json
                # Detach the old map and every checkpoint tied to its frame.
                for previous in (ACTIVE,Path.home()/'water_trials/tank/last_pose.json',
                                 Path.home()/'water_trials/tank/route.json'):
                    previous.unlink(missing_ok=True)
                reset={'id':uuid.uuid4().hex,'stamp_ns':time.time_ns()}
                atomic_json(Path.home()/'water_trials/runtime/tank_map_reset.json',reset)
                self.sync_reset()
                if not self.store.sensors_running():main(['start','sensors'])
                main(['start','mapping','--','mode:=mapping'])
                if not running('mapping'):
                    raise RuntimeError('새 지도 기동 실패 · mapping.log를 확인하세요.')
            elif action=='save':
                if self.checkpoint:
                    import subprocess,sys
                    command=[sys.executable,'-m','isv_water_trials.tank_save','--freeze',
                             '--from-snapshot',self.checkpoint['prefix'],'--name',str(path)]
                    if crop is not None:command+=['--crop',*map(str,crop)]
                    subprocess.run(command,check=True,timeout=90)
                    main(['stop','mapping']);main(['start','mapping'])
                else:
                    command=['freeze','mapping','--name',str(path)]
                    if crop is not None:command+=['--crop',*map(str,crop)]
                    main(command)
                from isv_water_trials.tank_map_file import read_bundle
                identity,_,_=read_bundle(path)
                route_file=Path.home()/'water_trials/tank/route.json'
                try:route=json.loads(route_file.read_text(encoding='utf-8'))
                except (OSError,ValueError):route=None
                if isinstance(route,dict) and isinstance(route.get('a'),dict) and isinstance(route.get('b'),dict):
                    route=dict(route,session_id=identity,frame_id='tank_map')
                    from isv_water_trials.tank_storage import atomic_json
                    atomic_json(path.with_suffix('.route.json'),route)
                    atomic_json(route_file,route)
            else:
                from isv_water_trials.tank_map_file import activate
                identity=activate(path)
                route_path=path.with_suffix('.route.json')
                if route_path.is_file():
                    route=json.loads(route_path.read_text(encoding='utf-8'))
                    if not all(isinstance(route.get(k),dict) and all(math.isfinite(float(route[k][axis])) for axis in ('x','y')) for k in ('a','b')):
                        raise ValueError('저장 지도에 연결된 A/B 경로를 읽을 수 없습니다.')
                    route=dict(route,session_id=identity,frame_id='tank_map')
                    from isv_water_trials.tank_storage import atomic_json
                    atomic_json(Path.home()/'water_trials/tank/route.json',route)
                main(['stop','mapping']);main(['start','mapping'])
            result={'state':'done','message':{'create':'기존 지도 초기화 완료 · 원점 (0, 0)에서 새 라이다 지도 작성 중','save':'SLAM 지도를 저장하고 고정 모드로 전환했습니다.','load':'저장 SLAM 지도를 불러왔습니다.','enable':'SLAM 켜짐','disable':'SLAM 꺼짐 · 센서와 선체 기록은 유지됩니다.','load_route':'도킹 지도와 A/B 경로를 불러왔습니다.','erase':'지운 지도 저장됨 · 도킹 경로 공통 적용'}[action]}
            if action=='load_route':result.update(route=route,applied=applied)
            if action=='erase':result.update(edited)
        except BaseException as exc:
            result={'state':'error','message':str(exc)[:240]}
        with self.map_jobs_lock:self.map_jobs[job_id].update(result)

    def sync_reset(self):
        marker=Path.home()/'water_trials/runtime/tank_map_reset.json'
        try:
            changed=marker.stat().st_mtime_ns
            if changed==getattr(self,'reset_marker',None):return
            reset=json.loads(marker.read_text(encoding='utf-8'))
        except (OSError,ValueError):return
        with self.lock:
            self.reset_marker=changed;self.reset_after_ns=int(reset['stamp_ns'])
            self.grid=None;self.body=b'';self.sequence+=1
            self.checkpoint=None
            self.stream_id=uuid.uuid4().hex
            self.last_geometry=None;self.last_cells=None;self.route_mtime=None
            with self.store.lock:
                for key in ('map','tank_pose','tank_route','tank_session','tank_status','tank_localization_candidates'):
                    self.store.values.pop(key,None)
            self.store.value('tank_map_reset',reset)

    def map(self, message,checkpoint=None):
        self.sync_reset()
        # Ignore retained or delayed grids published before the fresh session.
        if getattr(self,'reset_after_ns',0):
            stamp=message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
            if stamp<self.reset_after_ns:return
        from telemetry_core import quaternion_angles
        info = message.info; count = info.width*info.height
        if not 0<count<=8_000_000 or len(message.data)!=count: return
        if not .005<=info.resolution<=1: return
        angle = quaternion_angles(info.origin.orientation)
        metadata = {'width':info.width,'height':info.height,'resolution_m':info.resolution,
                    'origin_x_m':info.origin.position.x,'origin_y_m':info.origin.position.y,
                    'origin_yaw_deg':angle['deg_signed']['yaw'] if angle else 0.,
                    'frame_id':message.header.frame_id}
        with self.lock:
            self.checkpoint=checkpoint
            if getattr(self,'reset_after_ns',0):
                stamp=message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
                if stamp<self.reset_after_ns:return
            cells=message.data.tobytes()
            if metadata!=self.last_geometry or cells!=self.last_cells:
                self.sequence += 1
                self.body = struct.pack('<4sIII',b'KMAP',self.sequence,info.width,info.height)+cells
                self.last_geometry=dict(metadata);self.last_cells=cells
            if checkpoint:metadata.update(source='checkpoint',checkpoint_saved_unix=checkpoint['saved_unix'])
            self.grid = message
            metadata['sequence'] = self.sequence
            metadata['stream_id'] = self.stream_id
            self.store.value('map',metadata)

    def payload(self, after):
        with self.lock:
            return b'' if after==self.sequence else self.body

    def restore_checkpoint(self):
        from map_checkpoint import latest_snapshot,snapshot_grid
        from nav_msgs.msg import OccupancyGrid
        checkpoint=latest_snapshot(reset_after_ns=self.reset_after_ns)
        if not checkpoint:return
        info,cells=snapshot_grid(checkpoint['prefix'])
        message=OccupancyGrid();message.header.frame_id='tank_map'
        stamp=round(checkpoint['saved_unix']*1e9)
        message.header.stamp.sec=stamp//10**9;message.header.stamp.nanosec=stamp%10**9
        message.info.width=info['width'];message.info.height=info['height'];message.info.resolution=info['resolution']
        message.info.origin.position.x=float(info['x']);message.info.origin.position.y=float(info['y'])
        message.info.origin.orientation.z=math.sin(info['yaw']/2);message.info.origin.orientation.w=math.cos(info['yaw']/2)
        from array import array
        message.data=array('b',cells)
        self.map(message,checkpoint=checkpoint)
        self.store.errors.pop('map_checkpoint',None)

    def restore_saved_map(self,path,expected_id):
        from array import array
        from nav_msgs.msg import OccupancyGrid
        from isv_water_trials.tank_map_file import read_bundle
        identity,_,files=read_bundle(path)
        if identity!=expected_id:raise ValueError('Saved map changed before reload.')
        info=json.loads(files['grid.json']);message=OccupancyGrid()
        message.header.frame_id='tank_map'
        stamp=time.time_ns()
        message.header.stamp.sec=stamp//10**9;message.header.stamp.nanosec=stamp%10**9
        message.info.width=info['width'];message.info.height=info['height']
        message.info.resolution=info['resolution']
        for key,value in zip(('x','y','z'),info['position']):setattr(message.info.origin.position,key,value)
        for key,value in zip(('x','y','z','w'),info['orientation']):setattr(message.info.origin.orientation,key,value)
        message.data=array('b',files['grid.bin'])
        with self.lock:
            self.stream_id=uuid.uuid4().hex
            self.map(message)
        with self.store.lock:
            session=dict(self.store.values.get('tank_session',{}))
        if session.get('map_file')==str(path):
            session.update(session_id=identity,localization_ready=False)
            self.store.value('tank_session',session)

    def request(self, path, data):
        future = Future(); self.pending.put((path,data,future))
        try: return future.result(timeout=2.)
        except TimeoutError:
            future.cancel(); return 503,{'error':'ROS 응답 대기 시간이 지났습니다.'}

    def work(self):
        self.sync_reset()
        if self.grid is None and time.monotonic()-self.checkpoint_checked>5:
            self.checkpoint_checked=time.monotonic()
            try:self.restore_checkpoint()
            except (OSError,ValueError,KeyError) as exc:self.store.errors['map_checkpoint']=str(exc)
        if time.monotonic()-getattr(self,'route_checked',0)>1:
            self.route_checked = time.monotonic()
            target = Path.home()/'water_trials/tank/route.json'
            try:
                changed = target.stat().st_mtime
                if changed!=getattr(self,'route_mtime',None):
                    route = json.loads(target.read_text(encoding='utf-8'))
                    self.store.value('tank_route',route); self.route_mtime = changed
            except (OSError,ValueError): pass
        for _ in range(4):
            try: path,data,future = self.pending.get_nowait()
            except Empty: return
            if future.cancelled(): continue
            try:
                point = lambda key: tuple(float(data[key][v]) for v in ('x','y'))
                if path=='/api/route':
                    from isv_water_trials.tank_core import Grid
                    if self.grid is None: raise ValueError('지도를 먼저 만들어 주세요.')
                    grid = Grid(self.grid,.75); a,b = point('a'),point('b')
                    if not all(math.isfinite(v) for v in (*a,*b)):raise ValueError('A/B 좌표를 확인하세요.')
                    target = Path.home()/'water_trials/tank/route.json'; target.parent.mkdir(parents=True,exist_ok=True)
                    with self.store.lock: session = dict(self.store.values.get('tank_session',{}))
                    if not session.get('session_id') or time.monotonic()-session.get('received_monotonic',0)>3:
                        raise ValueError('지도 세션 수신 대기 중입니다.')
                    route = {'frame_id':grid.frame,'session_id':session['session_id'],'a':dict(zip(('x','y'),a)),'b':dict(zip(('x','y'),b)),
                             'clearance_m':.75,'map_resolution_m':grid.resolution}
                    from isv_water_trials.tank_storage import atomic_json
                    if 'profile_version' in data:
                        from docking_routes import DockingRoutes
                        if not session.get('fixed_map') or not session.get('map_file'):
                            raise ValueError('현재 SLAM 지도를 먼저 저장하거나 불러온 뒤 도킹 경로를 저장하세요.')
                        profile=DockingRoutes().save(session['map_file'],session['session_id'],data['profile_version'],route)
                        route=profile['route']
                    atomic_json(target,route)
                    if 'profile_version' not in data and session.get('fixed_map') and session.get('map_file'):
                        map_file=Path(session['map_file']).expanduser().resolve()
                        if map_file.parent==(Path.home()/'water_trials/tank_maps').resolve() and map_file.suffix=='.tankmap':
                            atomic_json(map_file.with_suffix('.route.json'),route)
                    self.store.value('tank_route',route)
                    response = {'ok':True,'route':route,'message':route.get('profile_version','기본')+' A/B 도킹 경로 저장 완료'}
                elif path=='/api/localize':
                    with self.store.lock:
                        session=dict(self.store.values.get('tank_session',{}));status=dict(self.store.values.get('tank_status',{}))
                    if not session.get('fixed_map') or time.monotonic()-session.get('received_monotonic',0)>3:
                        raise ValueError('고정 지도를 먼저 불러오세요.')
                    with self.store.lock: blocked=position_edit_blocked(self.store.values)
                    if blocked:
                        raise ValueError('RC 모드로 전환하거나 왕복을 종료한 뒤 위치를 다시 찾으세요.')
                    if not self.localize_publisher.get_subscription_count():raise ValueError('자동 위치 추정 노드 수신 대기')
                    self.localize_publisher.publish(self.empty_type())
                    response={'ok':True,'message':'현재 위치를 유지하며 주변 정합을 보정합니다.'}
                elif path=='/api/apply_docking_route':
                    from docking_routes import DockingRoutes, atomic_json
                    from isv_water_trials.tank_map_file import ACTIVE, read_bundle
                    with self.store.lock:
                        if self.store.values.get('outdoor',{}).get('execute') or self.store.values.get('tank_status',{}).get('execute'):
                            raise ValueError('주행 종료 후 도킹 경로를 적용하세요.')
                    library=DockingRoutes();profile=library.read(str(data.get('path','')))
                    active=json.loads(ACTIVE.read_text(encoding='utf-8'))
                    identity,_,_=read_bundle(profile['map_file'])
                    if identity!=profile['map_id'] or active.get('map_id')!=identity:
                        raise ValueError('이 도킹 경로를 먼저 불러온 뒤 적용하세요.')
                    selected=library.apply(profile);route=selected['route']
                    atomic_json(Path.home()/'water_trials/tank/route.json',route)
                    self.store.value('tank_route',route)
                    response={'ok':True,'applied':selected,'route':route,'message':profile['version']+' 도킹 경로 적용됨 · 다음 주행 시작부터 사용'}
                elif path=='/api/initial_pose':
                    if self.grid is None: raise ValueError('지도 수신 대기 중입니다.')
                    with self.store.lock: status = dict(self.store.values.get('tank_status',{}))
                    with self.store.lock: blocked=position_edit_blocked(self.store.values)
                    if blocked:
                        raise ValueError('RC 모드로 전환하거나 왕복을 종료한 뒤 현재 위치를 지정하세요.')
                    consumers = self.node.get_subscriptions_info_by_topic('/initialpose')
                    if not any(c.node_name=='slam_toolbox' for c in consumers):
                        raise ValueError('저장 지도의 localization 모드에서 현재 위치를 설정해 주세요.')
                    if 'candidate_id' in data:
                        with self.store.lock:
                            session=dict(self.store.values.get('tank_session',{}))
                            report=dict(self.store.values.get('tank_localization_candidates',{}))
                        if not data.get('map_id') or data['map_id']!=session.get('session_id') or data['map_id']!=report.get('map_id'):
                            raise ValueError('지도가 바뀌었습니다. 후보를 다시 선택해 주세요.')
                        if time.monotonic()-report.get('received_monotonic',0)>.5 or time.monotonic()-session.get('received_monotonic',0)>3:
                            raise ValueError('최신 위치 수신 대기 중입니다.')
                        candidate=next((c for c in report.get('live_candidates',[]) if c.get('candidate_id')==data['candidate_id']),None)
                        if candidate is None:raise ValueError('후보가 갱신되었습니다. 다시 선택해 주세요.')
                        age=self.node.get_clock().now().nanoseconds/1e9-candidate.get('stamp_s',0)
                        if not -.1<=age<.5:raise ValueError('후보 위치 수신이 지연되고 있습니다.')
                        x,y,angle=(float(candidate[k]) for k in ('x','y','yaw_deg'))
                    else:
                        x,y,angle = (float(data[k]) for k in ('x','y','yaw_deg'))
                    if not all(math.isfinite(v) for v in (x,y,angle)): raise ValueError('위치·방향 값을 확인해 주세요.')
                    msg = self.pose_type(); msg.header.frame_id = self.grid.header.frame_id
                    msg.header.stamp = self.node.get_clock().now().to_msg()
                    msg.pose.pose.position.x = x; msg.pose.pose.position.y = y
                    msg.pose.pose.orientation.z = math.sin(math.radians(angle)/2)
                    msg.pose.pose.orientation.w = math.cos(math.radians(angle)/2)
                    msg.pose.covariance[0] = msg.pose.covariance[7] = .25
                    msg.pose.covariance[35] = math.radians(15)**2
                    self.publisher.publish(msg)
                    response = {'ok':True,'message':'초기 위치·방향 발행 완료. 지도 위 실제 위치와 비교해 주세요.'}
                else: raise ValueError('Unknown map operation')
                if not future.cancelled(): future.set_result((200,response))
            except (ValueError,KeyError,TypeError,ImportError,OSError) as exc:
                if not future.cancelled(): future.set_result((400,{'error':str(exc)}))
