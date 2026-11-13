"""GUI lifecycle management. Preparing a mission never starts motion."""
from pathlib import Path
import json,os,re,signal,subprocess,time
from viewer_bringup import BringupManager
ROOT=Path(__file__).resolve().parent.parent

def processes(kind):
    found=[]
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:
            args=(proc/'cmdline').read_bytes().decode().split('\0')
            if (proc/'stat').read_text().split()[2]=='Z':continue
        except (OSError,UnicodeError):continue
        mission=any(Path(a).name=='course_mission' or a in ('isv_shape_navigation.course_mission_v3','isv_shape_navigation.course_mission') for a in args)
        mission=mission or str(ROOT/'run_V3.sh') in args or (str(ROOT/'빌드실행/all_in_one.launch.py') in args and 'components:=mission' in args)
        mapper='docking_mapping.node' in args or str(ROOT/'주행코드/도킹 알고리즘/run_mapping.sh') in args
        if (kind=='mission' and mission) or (kind=='mapper' and mapper):found.append((int(proc.name),args))
    return found

class RuntimeManager(BringupManager):
    def __init__(self,root=ROOT,mission_report=None,docking_report=None):
        super().__init__(root);self.mission_report=mission_report or (lambda:{});self.docking_report=docking_report or (lambda:{});self.children=[]
    def snapshot(self):
        missions=processes('mission');maps=processes('mapper');report=self.mission_report();dock=self.docking_report();args=[a for _,argv in missions for a in argv]
        variant='v3' if 'isv_shape_navigation.course_mission_v3' in args or str(ROOT/'run_V3.sh') in args else 'course' if missions else None
        out=report.get('output') or {};auto=(dock.get('status') or {}).get('autonomy') or {}
        import yaml
        cfg=yaml.safe_load((ROOT/'주행코드/도킹 알고리즘/config/mapping.yaml').read_text());sensors=cfg['sensors'];warnings=[]
        if not sensors['extrinsics_verified']:warnings.append('센서 장착 위치·방향 실측 보정 미확인')
        if sensors['heading_reference']=='magnetic' and sensors['magnetic_declination_deg'] is None:warnings.append('자기편각 미설정')
        return dict(mission_running=bool(missions),variant=variant,mission_pids=[p for p,_ in missions],mapper_running=bool(maps),mapper_pids=[p for p,_ in maps],output_enabled=bool(out.get('enabled')),active=bool(out.get('active')),armed=bool(out.get('armed')),mapping_connected=dock.get('connected',False),docking_active=bool(auto.get('active')),docking_output_enabled=bool(cfg['autonomy']['enable_output'] and not cfg['autonomy']['dry_run'] and cfg['autonomy']['hardware_output_authorized']),warnings=warnings)
    def spawn(self,argv,name):
        logs=ROOT/'.runtime/logs';logs.mkdir(parents=True,exist_ok=True)
        with (logs/name).open('ab',buffering=0) as log:child=subprocess.Popen(argv,cwd=ROOT,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        self.children=[p for p in self.children if p.poll() is None]+[child]
        return child
    def stop_processes(self,kind):
        for pid,_ in processes(kind):
            try:os.kill(pid,signal.SIGINT)
            except ProcessLookupError:pass
        deadline=time.monotonic()+12
        while processes(kind) and time.monotonic()<deadline:time.sleep(.1)
        if processes(kind):raise RuntimeError('종료 확인 실패. 실행 터미널을 확인하세요.')
    def persist(self,command):
        p=ROOT/'설정/course_mission.yaml';text=p.read_text();action=command.get('action');values={}
        if action=='set_route':values={'wp_latitudes':[x[0] for x in command['waypoints']],'wp_longitudes':[x[1] for x in command['waypoints']]}
        elif action=='set_target':values={'target_label':command['target_label']}
        elif action=='set_speed':values={'propulsion_pwm_scale':command['percent']/100.}
        for key,value in values.items():
            pattern=r'(?m)^    '+re.escape(key)+r':.*$';line='    '+key+': '+json.dumps(value,ensure_ascii=False)
            text=re.sub(pattern,line,text) if re.search(pattern,text) else text+line+'\n'
        temp=p.with_suffix('.yaml.gui.tmp');temp.write_text(text);temp.replace(p)
    def command(self,action):
        allowed={'v3_preview','v3_autonomy','course_preview','course_autonomy','mission_stop','mapper_start','mapper_stop','docking_prepare','docking_preview','docking_autonomy'}
        if action not in allowed:raise ValueError('지원하지 않는 실행 명령')
        with self.lock:
            report=self.mission_report();out=report.get('output') or {};auto=(self.docking_report().get('status') or {}).get('autonomy') or {}
            if processes('mission') and action not in ('mapper_start','mapper_stop') and (not report.get('stamp') or time.time()-report['stamp']>2):raise ValueError('임무 상태가 최신이 아닙니다. 연결을 확인하세요.')
            if action!='mapper_start' and (out.get('active') or out.get('armed') or auto.get('active')):raise ValueError('먼저 임무/도킹 정지 버튼으로 정지·무장 해제를 확인하세요.')
            if action in ('docking_preview','docking_autonomy'):
                import yaml
                config=ROOT/'주행코드/도킹 알고리즘/config/mapping.yaml';text=config.read_text();cfg=yaml.safe_load(text)
                if (self.docking_report().get('status') or {}).get('mode')=='ACTIVE':raise ValueError('맵 생성을 종료하고 저장한 뒤 도킹 모드를 변경하세요.')
                enabled=action=='docking_autonomy'
                if enabled and (not cfg['sensors']['extrinsics_verified'] or (cfg['sensors']['heading_reference']=='magnetic' and cfg['sensors']['magnetic_declination_deg'] is None) or not cfg['autonomy']['corridor']['envelope_verified']):raise ValueError('도킹 출력 준비 불가: 센서 장착 보정·자기편각·선체 크기 실측 설정을 먼저 완료하세요.')
                self.stop_processes('mission');self.stop_processes('mapper')
                for key,value in {'dry_run':not enabled,'enable_output':enabled,'hardware_output_authorized':enabled}.items():text=re.sub(r'(?m)^  '+key+r':.*$', '  '+key+': '+str(value).lower(),text)
                temp=config.with_suffix('.yaml.gui.tmp');temp.write_text(text);temp.replace(config)
                self.spawn(['bash',str(ROOT/'주행코드/도킹 알고리즘/run_mapping.sh')],'gui_mapping.log')
            elif action=='mapper_stop':self.stop_processes('mapper')
            elif action=='mission_stop':self.stop_processes('mission')
            elif action in ('mapper_start','docking_prepare'):
                if action=='docking_prepare':self.stop_processes('mission')
                if not processes('mapper'):self.spawn(['bash',str(ROOT/'주행코드/도킹 알고리즘/run_mapping.sh')],'gui_mapping.log')
            else:
                if report.get('waypoints'):self.persist(dict(action='set_route',waypoints=report['waypoints']))
                if report.get('target_label'):self.persist(dict(action='set_target',target_label=report['target_label']))
                if isinstance(report.get('propulsion_pwm_scale'),(int,float)):self.persist(dict(action='set_speed',percent=round(report['propulsion_pwm_scale']*100)))
                self.stop_processes('mission');variant,mode=action.split('_');flag='--autonomy' if mode=='autonomy' else '--preview'
                argv=['bash',str(ROOT/'run_V3.sh'),'--wait-start']+(['--autonomy'] if mode=='autonomy' else []) if variant=='v3' else ['bash',str(ROOT/'빌드실행/run_mission.sh'),flag,'--no-gui']
                self.spawn(argv,'gui_selected_mission.log')
            return dict(success=True,message='요청 접수. 준비되면 해당 시작 버튼을 누르세요.',**self.snapshot())
