"""Native Tk desktop UI in its own process; no HTTP server or browser.

Only telemetry crosses to the UI. Operator commands are consumed by the ROS
timer in the controller process. A closed/failed window ends the mission node.
"""
import base64
from collections import deque
import json
import math
import multiprocessing as mp
from queue import Empty, Full
import threading
import time
from .navigation_display import direction


STATE_LABELS = {
    'TO_WP1':'WP1로 이동', 'WP1_TO_WP2':'WP1 → WP2 주행', 'AVOID_OBSTACLE':'장애물 회피',
    'WAYPOINT_ARRIVAL':'WP 도착 확인', 'WP2_HOLD':'WP2 · 10초 정지',
    'WP2_TO_WP3_ALIGN':'WP3 방향 정렬', 'SEARCH_SHAPE':'목표 도형 탐색',
    'VERIFY_TARGET':'도형 확인 중', 'ALIGN_SHAPE':'도형 방향 정렬',
    'APPROACH_SHAPE':'도형 접근', 'ENTER_REGION':'도킹 구역 진입',
    'REGION_ARRIVAL_HOLD':'도킹 정지 확인', 'MISSION_COMPLETE':'임무 완료',
    'WAIT_GPS':'GPS 수신 대기', 'WAIT_SCAN':'라이다 수신 대기',
    'WAIT_HEADING':'IMU 방향 수신 대기', 'TARGET_LOST':'목표 유실 · 정지',
    'WAIT_SYNC':'센서 시각 정렬 대기', 'OBSTACLE_STOP':'근접 장애물 · 정지',
    'NO_SAFE_GAP':'통과 구간 없음 · 정지', 'GPS_UNCERTAIN':'GPS 정확도 확인',
    'WP2_DRIFT':'WP2 위치 이탈 · 정지', 'WP3_POSITION_MISMATCH':'도형 위치와 WP3 불일치',
    'WP2_BRAKE':'WP2 · 1400 역추진 0.5초', 'WP2_BRAKE_BLOCKED':'후방 확인 실패 · 역추진 중단',
    'WP2_BRAKE_ABORTED':'역추진 취소 · 초기화 필요',
    'DOCK_OBSERVE_APPROACH':'도크 관측 거리까지 접근', 'DOCK_ACQUIRE':'LiDAR 도크 형상 확인',
    'DOCK_TO_Q':'도크 입구 전방 Q로 접근', 'DOCK_ALIGN':'IMU 진입 축 정렬',
    'DOCK_ENTER':'도크 중심으로 진입', 'DOCK_HOLD':'도킹 정지 확인',
    'DOCK_REVERSE':'선수 방위 유지 · 후진 이탈', 'DOCK_GEOMETRY_UNCERTAIN':'도크 형상 불확실 · 정지',
    'DOCK_ID_CHANGED':'목표 도크 변경 의심 · 정지', 'DOCK_PATH_BLOCKED':'도킹 경로 장애물 · 정지',
}


def formatted(value, digits=1, suffix=''):
    return f'{value:.{digits}f}{suffix}' if isinstance(value,(int,float)) and math.isfinite(value) else '—'


def metrics(report):
    position=report.get('gps_observation') or report.get('position') or {}
    points=report.get('waypoints') or []
    index=report.get('wp_index',0)
    goal=points[index] if type(index) is int and 0<=index<len(points) else (None,None)
    return {
        'latitude':formatted(position.get('latitude'),8),
        'longitude':formatted(position.get('longitude'),8),
        'wp':'WP'+str(index+1),
        'distance':formatted(report.get('distance_to_wp'),1,' m'),
        'goal_latitude':formatted(goal[0],8), 'goal_longitude':formatted(goal[1],8),
        'hold':formatted(report.get('hold_remaining'),1,' 초'),
    }


def latest(queue,value):
    try:queue.put_nowait(value)
    except Full:
        try:queue.get_nowait()
        except Empty:pass
        try:queue.put_nowait(value)
        except Full:pass


class DesktopDashboard:
    image_extension='.png'

    def __init__(self):
        ctx=mp.get_context('spawn')
        self.commands=ctx.Queue(maxsize=16)
        self.states=ctx.Queue(maxsize=1)
        self.images=ctx.Queue(maxsize=1)
        ready=ctx.Queue(maxsize=1)
        self.lock=threading.Lock()
        self.last_command=None
        self.history=deque(maxlen=300)
        self.events=deque(maxlen=20)
        self.previous_state=None
        self.process=ctx.Process(target=desktop_process,args=(self.states,self.images,self.commands,ready),daemon=True)
        self.process.start()
        try:result=ready.get(timeout=10)
        except Empty:
            self.close();raise RuntimeError('Desktop GUI did not start; check python3-tk and desktop display')
        finally:ready.close()
        if not result.get('ready'):
            self.close();raise RuntimeError('Desktop GUI: '+result.get('error','Unknown error')
                +' — run in the Jetson desktop terminal with python3-tk installed')

    def is_open(self):return self.process.is_alive()

    def publish(self,report):
        self.history.append(dict(pwm=report['intended_pwm'],feedback=report.get('feedback_pwm')))
        if report['state']!=self.previous_state:
            self.events.appendleft(time.strftime('%H:%M:%S')+'  '+STATE_LABELS.get(report['state'],report['state']))
            self.previous_state=report['state']
        with self.lock:command=self.last_command
        latest(self.states,dict(report,history=list(self.history),events=list(self.events),last_command=command))

    def camera(self,png):latest(self.images,png)

    def close(self):
        if self.process.is_alive():
            latest(self.states,{'close':True})
            self.process.join(timeout=2)
            if self.process.is_alive():self.process.terminate();self.process.join(timeout=2)
        for queue in (self.commands,self.states,self.images):
            queue.cancel_join_thread();queue.close()


def desktop_process(states,images,commands,ready):
    try:
        import tkinter as tk
        from tkinter import ttk,filedialog
        root=tk.Tk()
        app=DesktopView(root,tk,ttk,filedialog,states,images,commands)
        ready.put({'ready':True})
        root.mainloop()
    except Exception as error:
        try:ready.put_nowait({'ready':False,'error':str(error)})
        except Full:pass


class DesktopView:
    BG='#11191d';PANEL='#1c282e';LINE='#344c56';TEXT='#edf5f8';MUTED='#98afba'
    CYAN='#65ddc6';ORANGE='#ffb978'

    def __init__(self,root,tk,ttk,filedialog,states,images,commands):
        self.root,self.tk,self.ttk,self.filedialog=root,tk,ttk,filedialog
        self.states,self.images,self.commands=states,images,commands
        self.report={};self.received=0.;self.photo=None;self.image_received=0.
        self.trail=deque(maxlen=1500);self.last_position=None;self.last_command=None;self.settings_loaded=False
        self.slam_background_key=None
        root.title('KABOAT · GPS 항법 / 장애물 회피 / 도형 도킹')
        root.geometry('1450x1020');root.minsize(1100,850);root.configure(bg=self.BG)
        root.option_add('*Font',('DejaVu Sans',10))
        style=ttk.Style(root);style.theme_use('clam')
        style.configure('TCombobox',fieldbackground='#273b44',background='#273b44',foreground=self.TEXT)
        style.map('TCombobox',fieldbackground=[('readonly','#273b44')],foreground=[('readonly',self.TEXT)],
                  selectbackground=[('readonly','#273b44')],selectforeground=[('readonly',self.TEXT)])
        title=tk.Frame(root,bg=self.BG);title.pack(fill='x',padx=20,pady=(15,8))
        self.label(title,'KABOAT  /  MISSION CONTROL',21,'bold').pack(side='left')
        self.mode=self.label(title,'PREVIEW',12,'bold',self.CYAN);self.mode.pack(side='right')
        self.status=self.label(root,'센서 수신 대기',22,'bold');self.status.pack(anchor='w',padx=20)
        self.progress=self.label(root,'① WP1 → WP2 회피 주행     →     ② WP2 10초 정지     →     ③ WP3 도킹',11,color=self.MUTED)
        self.progress.pack(anchor='w',padx=20,pady=(4,10))
        numbers=tk.Frame(root,bg=self.PANEL);numbers.pack(fill='x',padx=20)
        self.values={}
        for i,(name,caption) in enumerate((('latitude','현재 위도 LAT'),('longitude','현재 경도 LON'),
                                          ('wp','다음 목표'),('distance','다음 WP까지 남은 거리'),('hold','WP2 정지 남은 시간'))):
            box=tk.Frame(numbers,bg=self.PANEL);box.grid(row=0,column=i,sticky='nsew',padx=15,pady=13)
            numbers.grid_columnconfigure(i,weight=1)
            self.label(box,caption,10,color=self.MUTED,bg=self.PANEL).pack(anchor='w')
            value=self.label(box,'—',23,'bold',self.CYAN if name=='distance' else self.TEXT,bg=self.PANEL)
            value.pack(anchor='w',pady=(6,0));self.values[name]=value
        self.goal=self.label(root,'목표 좌표 · GPS 방향/정확도 수신 대기',10,color=self.MUTED)
        self.goal.pack(anchor='w',padx=20,pady=8)
        self.gps_health=self.label(root,'GPS 메시지 수신 대기',11,color=self.ORANGE)
        self.gps_health.pack(anchor='w',padx=20,pady=(0,6))
        angles=tk.Frame(root,bg=self.PANEL);angles.pack(fill='x',padx=20,pady=(0,8))
        self.angle_values={}
        for index,(key,caption,color) in enumerate((('target','목표각 · WP 상대', '#82b3ff'),
                ('command','현재 변침 명령',self.ORANGE),('actual','실제 방향 변화 · 구간 시작 기준',self.CYAN),
                ('bearing','목표 절대방위 · 북쪽 0°',self.TEXT))):
            box=tk.Frame(angles,bg=self.PANEL);box.grid(row=0,column=index,sticky='ew',padx=12,pady=7)
            angles.grid_columnconfigure(index,weight=1)
            self.label(box,caption,9,color=self.MUTED,bg=self.PANEL).pack(anchor='w')
            value=self.label(box,'—',17,'bold',color,bg=self.PANEL);value.pack(anchor='w')
            self.angle_values[key]=value
        controls=tk.Frame(root,bg=self.BG);controls.pack(fill='x',padx=20,pady=(0,8))
        self.start=self.button(controls,'▶ 시작 / 재개',lambda:self.send('start'),self.CYAN,'#102e28');self.start.pack(side='left')
        self.button(controls,'■ 정지',lambda:self.send('stop'),'#733f42').pack(side='left',padx=8)
        self.reset=self.button(controls,'초기화',lambda:self.send('reset'));self.reset.pack(side='left')
        self.notice=self.label(controls,'기본값: 미리보기 · 실제 출력 비활성',10,color=self.MUTED)
        self.notice.pack(side='left',padx=14)
        displays=tk.Frame(root,bg=self.BG);displays.pack(fill='both',expand=True,padx=20)
        self.canvases={}
        self.map_view=tk.StringVar(value='gps')
        for i,(name,caption) in enumerate((('map','GPS / SLAM 지도'),('radar','라이다 · 방향 화살표'),('camera','카메라 · 목표 도형'))):
            box=tk.Frame(displays,bg=self.PANEL,highlightbackground=self.LINE,highlightthickness=1)
            box.grid(row=0,column=i,sticky='nsew',padx=(0,10 if i<2 else 0))
            displays.grid_columnconfigure(i,weight=1)
            bar=tk.Frame(box,bg=self.PANEL);bar.pack(fill='x',padx=12,pady=8)
            self.label(bar,caption,11,'bold',bg=self.PANEL).pack(side='left')
            if name=='map':
                for value,title in (('slam','SLAM'),('gps','GPS')):
                    tk.Radiobutton(bar,text=title,variable=self.map_view,value=value,command=self.draw_map,
                        bg=self.PANEL,fg=self.TEXT,selectcolor='#29414c',activebackground=self.PANEL).pack(side='right')
            canvas=tk.Canvas(box,bg='#101b21',highlightthickness=0,height=230)
            canvas.pack(fill='both',expand=True);self.canvases[name]=canvas
        displays.grid_rowconfigure(0,weight=1)
        self.reason=self.label(root,'경로 선택 근거 · 수신 대기',10,color=self.MUTED)
        self.reason.config(wraplength=1400,justify='left');self.reason.pack(fill='x',padx=20,pady=(7,0))
        self.obstacle_info=self.label(root,'장애물 거리·SLAM 위치 수신 대기',10,color=self.ORANGE)
        self.obstacle_info.pack(anchor='w',padx=20,pady=(3,0))
        self.pwm_label=self.label(root,'좌현 요청 1500 µs  |  우현 요청 1500 µs  |  MCU 피드백 대기',15,'bold',self.CYAN)
        self.pwm_label.pack(anchor='w',padx=20,pady=(12,3))
        self.pwm_canvas=tk.Canvas(root,bg=self.PANEL,height=90,highlightthickness=0)
        self.pwm_canvas.pack(fill='x',padx=20)
        bottom=tk.Frame(root,bg=self.BG);bottom.pack(fill='x',padx=20,pady=10)
        self.entries=[]
        for i in range(3):
            self.label(bottom,'WP'+str(i+1),10,'bold').grid(row=i,column=0,padx=(0,8),pady=3)
            pair=[]
            for col in (1,2):
                entry=tk.Entry(bottom,width=17,bg='#273b44',fg=self.TEXT,insertbackground=self.TEXT,relief='flat')
                entry.grid(row=i,column=col,padx=4,ipady=5);pair.append(entry)
            self.entries.append(pair)
            self.button(bottom,'현재 GPS',lambda index=i:self.send('capture_wp',index=index)).grid(row=i,column=3,padx=6)
        actions=tk.Frame(bottom,bg=self.BG);actions.grid(row=0,column=4,rowspan=3,sticky='n',padx=12)
        self.apply_route=self.button(actions,'WP 좌표 적용',self.set_route);self.apply_route.pack(fill='x',pady=3)
        self.button(actions,'설정 파일 저장',self.export).pack(fill='x',pady=3)
        target=tk.Frame(bottom,bg=self.BG);target.grid(row=0,column=5,rowspan=3,sticky='n',padx=10)
        self.label(target,'도킹 목표',10,color=self.MUTED).pack(anchor='w')
        self.target=ttk.Combobox(target,state='readonly',width=17,
            values=[color+' '+shape for color in ('blue','green','red') for shape in ('circle','cross','triangle')])
        self.target.set('red triangle');self.target.pack(pady=5)
        self.apply_target=self.button(target,'목표 적용',lambda:self.send('set_target',target_label=self.target.get()))
        self.apply_target.pack(anchor='w')
        self.health=self.label(root,'GPS —   IMU —   LIDAR —   MCU —',10,color=self.MUTED)
        self.health.pack(anchor='w',padx=20,pady=(0,3))
        self.events=self.label(root,'상태 변경 대기',9,color=self.MUTED);self.events.pack(anchor='w',padx=20,pady=(0,10))
        root.protocol('WM_DELETE_WINDOW',self.shutdown)
        root.after(100,self.poll)

    def label(self,parent,text,size=10,weight='normal',color=None,bg=None):
        return self.tk.Label(parent,text=text,font=('DejaVu Sans',size,weight),fg=color or self.TEXT,
                             bg=bg or self.BG,anchor='w')

    def button(self,parent,text,command,bg='#2a3d46',fg=None):
        return self.tk.Button(parent,text=text,command=command,bg=bg,fg=fg or self.TEXT,
            activebackground='#3f5964',activeforeground=self.TEXT,relief='flat',padx=12,pady=7,cursor='hand2')

    def send(self,action,**extra):
        try:self.commands.put_nowait(dict(action=action,**extra));self.notice.config(text='명령 처리 중…')
        except Full:self.notice.config(text='명령 대기열이 찼습니다.')

    def set_route(self):
        try:
            waypoints=[[float(entry.get()) for entry in pair] for pair in self.entries]
            if any(not math.isfinite(lat) or not -85<lat<85 or not math.isfinite(lon) or not -180<=lon<=180 for lat,lon in waypoints):
                raise ValueError
            self.send('set_route',waypoints=waypoints)
        except ValueError:self.notice.config(text='올바른 위도·경도를 입력하세요.')

    def export(self):
        try:
            values=[[float(entry.get()) for entry in pair] for pair in self.entries]
            name=self.filedialog.asksaveasfilename(parent=self.root,defaultextension='.json',initialfile='course_settings.json')
            if name:
                with open(name,'w',encoding='utf-8') as stream:
                    json.dump(dict(wp_latitudes=[p[0] for p in values],wp_longitudes=[p[1] for p in values],
                                   target_label=self.target.get(),wp2_stop_seconds=10.),stream,ensure_ascii=False,indent=2)
                self.notice.config(text='설정 저장 완료. 다음 실행 YAML에도 반영하세요.')
        except (ValueError,OSError) as error:self.notice.config(text=str(error))

    def shutdown(self):
        try:self.commands.put_nowait({'action':'stop'})
        except Full:pass
        self.root.destroy()

    def poll(self):
        incoming=None
        try:
            while True:incoming=self.states.get_nowait()
        except Empty:pass
        if incoming is not None:
            if incoming.get('close'):self.root.destroy();return
            self.report=incoming;self.received=time.monotonic();self.render()
        try:
            png=self.images.get_nowait()
            self.photo=self.tk.PhotoImage(data=base64.b64encode(png))
            self.image_received=time.monotonic()
        except Empty:pass
        except self.tk.TclError:self.photo=None
        camera=self.canvases['camera'];camera.delete('all')
        if self.photo:
            camera.create_image(camera.winfo_width()/2,camera.winfo_height()/2,image=self.photo)
            if time.monotonic()-self.image_received>2:
                camera.create_text(12,15,anchor='w',text='영상 지연',fill=self.ORANGE)
        else:camera.create_text(camera.winfo_width()/2,camera.winfo_height()/2,text='카메라 영상 수신 대기',fill=self.MUTED)
        if self.received and time.monotonic()-self.received>1.5:
            self.mode.config(text='연결 지연',fg=self.ORANGE);self.start.config(state='disabled')
        self.root.after(100,self.poll)

    def render(self):
        report=self.report;view=metrics(report)
        for key,label in self.values.items():label.config(text=view[key])
        state=report.get('state','WAIT_DATA');out=report.get('output') or {}
        self.status.config(text=STATE_LABELS.get(state,state))
        self.mode.config(text='DEMO · 가상 센서' if report.get('demo') else '실제 출력 활성' if out.get('enabled') else 'PREVIEW · 출력 비활성',fg=self.CYAN)
        if report.get('demo') and not self.last_command:
            self.notice.config(text='DEMO · 가상 GPS/라이다/PWM · 실제 장비 연결 없음')
        self.goal.config(text=f"{view['wp']} 목표 위도 {view['goal_latitude']}   경도 {view['goal_longitude']}     GPS 1σ {formatted((report.get('position') or {}).get('sigma'),2,' m')}")
        observation=report.get('gps_observation') or {}
        self.gps_health.config(text='GPS '+report.get('gps_topic','/gps/fix')+' · '
            +observation.get('message','메시지 수신 대기')+' · 수신 경과 '
            +formatted(observation.get('receive_age'),2,' s'))
        decision=report.get('decision') or {}
        self.angle_values['target'].config(text=direction(decision.get('target_relative_deg')))
        self.angle_values['command'].config(text=direction(decision.get('turn_command_deg')))
        self.angle_values['actual'].config(text=direction(decision.get('actual_heading_change_deg')))
        self.angle_values['bearing'].config(text=formatted(decision.get('target_bearing_deg'),1,'°'))
        gap_text=[]
        for gap in decision.get('gaps',[])[:4]:
            gap_text.append(('선택 ' if gap.get('selected') else '')+f"통로{gap['index']}: "
                +direction(gap['right_deg'])+' ~ '+direction(gap['left_deg'])
                +f" / 폭 {gap['width_deg']:.1f}° (WP 기준 {direction(gap.get('wp_offset_deg'))})")
        chosen=decision.get('selected_gap')
        relation=''
        if decision.get('turn_command_deg') is not None:
            relation=' · '+('선택 통로 '+str(chosen)+' ' if chosen else '')+direction(decision['turn_command_deg'])
            relation+=' (WP 방향 대비 '+direction(decision.get('wp_offset_deg'))+')'
        self.reason.config(text='선택 근거: '+decision.get('reason','수신 대기')+relation+'\n'+'  |  '.join(gap_text))
        dock=report.get('dock_geometry') or {}
        if dock.get('m'):
            mx,my=dock['m'];qx,qy=dock['q']
            self.reason.config(text='도킹: '+dock.get('reason','')+f"\n배 기준 입구 M ({mx:.2f}, {my:.2f}) m / 정렬점 Q ({qx:.2f}, {qy:.2f}) m / 도크 폭 {dock['width']:.2f} m / IMU 축 {dock['axis_yaw_deg']:.1f}°")
        slam=report.get('slam_display') or {};closest=slam.get('closest')
        obstacle='최근접 장애물: '+formatted(closest.get('distance'),2,' m') if closest else '최근접 장애물: 관측 대기'
        if closest and closest.get('map_xy'):
            x,y=closest['map_xy'];obstacle+=f" · SLAM 위치 X {x:.2f} m / Y {y:.2f} m"
        self.obstacle_info.config(text=obstacle+'   |   SLAM '+(slam.get('status','대기') if slam.get('enabled',True) else '미사용')+'   |   파랑: 목표 / 주황: 변침 명령 / 흰색: 현재 전방')
        self.start.config(state='normal' if out.get('enabled') and not out.get('active') and report.get('stage')!='COMPLETE' else 'disabled')
        if out.get('start_ready') is False:self.start.config(state='disabled')
        for button in (self.reset,self.apply_route,self.apply_target):button.config(state='disabled' if out.get('active') else 'normal')
        requested=report.get('intended_pwm') or [1500,1500];feedback=report.get('feedback_pwm')
        self.pwm_label.config(text=f"좌현 계산 {requested[0]} µs    |    우현 계산 {requested[1]} µs    |    MCU 논리값 {feedback[0] if feedback else '—'} / {feedback[1] if feedback else '—'} µs")
        mcu=report.get('actuator') or {}
        ack=out.get('acknowledged_pwm')
        age=out.get('ack_age')
        fresh_ack=bool(ack and out.get('active') and isinstance(age,(int,float)) and 0<=age<=.5)
        ack_text=(str(ack[0])+' / '+str(ack[1])+' µs') if fresh_ack else '최신 응답 없음'
        self.health.config(text='    '.join(label+' '+formatted(report.get(key),2,'s') for label,key in
            (('GPS','gps_age'),('IMU','heading_age'),('LIDAR','scan_age'),('MCU','telemetry_age')))
            +'\n출력: '+(out.get('error') or out.get('blocker') or ('MCU arm 완료' if out.get('armed') else '시작 / arm 대기'))
            +'\nMCU 수락 '+ack_text+'  |  MCU armed '+str(mcu.get('armed','—'))
            +'  |  실제 핀 '+str(mcu.get('observed_left_us','—'))+' / '+str(mcu.get('observed_right_us','—'))+' µs · 회전 미계측')
        self.events.config(text='   ·   '.join(report.get('events',[])[:3]))
        command=report.get('last_command')
        if command and command!=self.last_command:
            self.last_command=command;self.notice.config(text=command.get('message',''))
            if command.get('success') and command.get('action') in ('set_route','capture_wp','set_target'):
                self.settings_loaded=False
        if not self.settings_loaded and report.get('waypoints'):
            for pair,point in zip(self.entries,report['waypoints']):
                for entry,value in zip(pair,point):entry.delete(0,'end');entry.insert(0,format(value,'.9f'))
            self.target.set(report.get('target_label','red triangle'));self.settings_loaded=True
        self.draw_map();self.draw_radar();self.draw_pwm()

    def draw_map(self):
        if self.map_view.get()=='slam':
            self.draw_slam();return
        self.slam_background_key=None
        canvas=self.canvases['map'];canvas.delete('all');width=max(100,canvas.winfo_width());height=max(100,canvas.winfo_height())
        points=self.report.get('waypoints') or []
        if not points:return
        lat0,lon0=points[0]
        def local(p):return ((p[1]-lon0)*111195*math.cos(math.radians(lat0)),(p[0]-lat0)*111195)
        wp=[local(point) for point in points];position=self.report.get('position')
        point=(position['latitude'],position['longitude']) if position else None
        if point and point!=self.last_position:self.trail.append(local(point));self.last_position=point
        all_points=wp+list(self.trail);xs,ys=zip(*all_points)
        x0,x1,y0,y1=min(xs)-6,max(xs)+6,min(ys)-6,max(ys)+6
        scale=min((width-50)/(x1-x0),(height-50)/(y1-y0))
        def project(p):return (25+(p[0]-x0)*scale,height-25-(p[1]-y0)*scale)
        for i in range(1,10):
            canvas.create_line(i*width/10,0,i*width/10,height,fill='#243840')
            canvas.create_line(0,i*height/10,width,i*height/10,fill='#243840')
        route=[value for point in wp for value in project(point)]
        canvas.create_line(*route,fill='#607d89',width=2,dash=(6,5))
        if len(self.trail)>1:canvas.create_line(*[value for point in self.trail for value in project(point)],fill=self.CYAN,width=2)
        for i,point in enumerate(wp):
            x,y=project(point);color=self.CYAN if i==self.report.get('wp_index') else '#7595a3'
            canvas.create_oval(x-5,y-5,x+5,y+5,fill=color,outline=color)
            canvas.create_text(x+10,y-10,text='WP'+str(i+1),anchor='w',fill=self.TEXT)
        if point is not None and position:
            x,y=project(local((position['latitude'],position['longitude'])));yaw=self.report.get('yaw') or 0.
            vertices=[(x+math.cos(yaw)*12,y-math.sin(yaw)*12),
                      (x+math.cos(yaw+2.5)*10,y-math.sin(yaw+2.5)*10),
                      (x+math.cos(yaw-2.5)*10,y-math.sin(yaw-2.5)*10)]
            canvas.create_polygon(*[v for p in vertices for v in p],fill=self.ORANGE)

    def draw_slam(self):
        canvas=self.canvases['map'];width=max(100,canvas.winfo_width());height=max(100,canvas.winfo_height())
        slam=self.report.get('slam_display') or {};grid=slam.get('map')
        if not grid:
            canvas.delete('all');self.slam_background_key=None
            canvas.create_text(width/2,height/2,text='SLAM 지도 수신 대기\n/map · /tank/pose · /tank/session',fill=self.MUTED,justify='center')
            return
        key=(grid['revision'],width,height,slam.get('session_id'))
        scale=min((width-24)/grid['width_m'],(height-24)/grid['height_m'])
        px=(width-grid['width_m']*scale)/2;py=(height-grid['height_m']*scale)/2
        if self.slam_background_key!=key:
            canvas.delete('all');self.slam_background_key=key
            for row,start,length,value in grid['runs']:
                left=px+start*grid['cell_m']*scale
                right=px+min(grid['width_m'],(start+length)*grid['cell_m'])*scale
                top=height-py-min(grid['height_m'],(row+1)*grid['cell_m'])*scale
                bottom=height-py-row*grid['cell_m']*scale
                canvas.create_rectangle(left,top,right,bottom,fill='#a6b5bb' if value>=35 else '#303940' if value<0 else '#17352f',outline='',tags='background')
        canvas.delete('overlay')
        ox,oy,yaw=grid['origin'];c,s=math.cos(yaw),math.sin(yaw)
        def project(point):
            x,y=point[0]-ox,point[1]-oy
            return px+(c*x+s*y)*scale,height-py-(-s*x+c*y)*scale
        trail=slam.get('trail') or []
        if len(trail)>1:canvas.create_line(*[value for point in trail for value in project(point)],fill=self.CYAN,width=2,tags='overlay')
        for point in slam.get('world_points',[]):
            x,y=project(point)
            canvas.create_oval(x-2,y-2,x+2,y+2,fill='#ff826a',outline='',tags='overlay')
        pose=slam.get('pose')
        if pose:
            x,y=project((pose['x'],pose['y']));angle=pose['yaw']-yaw
            vertices=[(x+math.cos(angle)*12,y-math.sin(angle)*12),
                      (x+math.cos(angle+2.5)*9,y-math.sin(angle+2.5)*9),
                      (x+math.cos(angle-2.5)*9,y-math.sin(angle-2.5)*9)]
            canvas.create_polygon(*[v for point in vertices for v in point],fill=self.ORANGE,tags='overlay')
        canvas.create_text(8,12,anchor='w',text=slam.get('frame','tank_map')+' · '+slam.get('status',''),fill=self.TEXT,tags='overlay')

    def draw_radar(self):
        canvas=self.canvases['radar'];canvas.delete('all');width=max(100,canvas.winfo_width());height=max(100,canvas.winfo_height())
        cx,cy=width/2,height-22;scale=min(width*.43,height-30)/3
        for distance in (1,2,3):
            radius=distance*scale
            canvas.create_arc(cx-radius,cy-radius,cx+radius,cy+radius,start=0,extent=180,style='arc',outline=self.LINE)
            canvas.create_text(cx+12,cy-radius+10,text=str(distance)+'m',fill=self.MUTED)
        avoidance=self.report.get('avoidance') or {}
        lidar=self.report.get('slam_display') or {}
        raw=self.report.get('lidar_display')
        points=raw['points'] if raw is not None else lidar.get('body_points',avoidance.get('obstacles',[]))
        for x,y in points:
            if x<0 or math.hypot(x,y)>4:continue
            px,py=cx-y*scale,cy-x*scale
            color=self.ORANGE if raw is None or raw['status']=='LIVE' else self.MUTED
            canvas.create_oval(px-2,py-2,px+2,py+2,fill=color,outline='')
        decision=self.report.get('decision') or {}
        dock=self.report.get('dock_geometry') or {}
        for key,color in (('m','#55d6b0'),('q','#82b3ff'),('c',self.ORANGE)):
            if key in dock:
                x,y=dock[key]
                if x>=0 and math.hypot(x,y)<=4:
                    px,py=cx-y*scale,cy-x*scale
                    canvas.create_oval(px-5,py-5,px+5,py+5,outline=color,width=2)
        for angle,color,length in ((0.,self.TEXT,1.0),
                (decision.get('target_relative_deg'),'#82b3ff',2.9),
                (decision.get('turn_command_deg'),self.ORANGE,2.4)):
            if isinstance(angle,(int,float)):
                radians=math.radians(angle)
                canvas.create_line(cx,cy,cx-math.sin(radians)*scale*length,cy-math.cos(radians)*scale*length,
                                   fill=color,width=3,arrow='last')
        canvas.create_rectangle(cx-6,cy-10,cx+6,cy+8,fill=self.TEXT,outline='')

    def draw_pwm(self):
        canvas=self.pwm_canvas;canvas.delete('all');width=max(100,canvas.winfo_width());height=max(40,canvas.winfo_height())
        def y(value):return height-10-(value-1100)/800*(height-20)
        for value in (1100,1500,1900):
            canvas.create_line(45,y(value),width-10,y(value),fill=self.LINE,dash=(3,3))
            canvas.create_text(4,y(value),text=str(value),fill=self.MUTED,anchor='w',font=('DejaVu Sans',8))
        history=self.report.get('history') or []
        if len(history)<2:return
        for key,index,color,dash in (('pwm',0,self.CYAN,()),('pwm',1,self.ORANGE,()),
                                     ('feedback',0,self.CYAN,(3,3)),('feedback',1,self.ORANGE,(3,3))):
            coordinates=[]
            for i,row in enumerate(history):
                values=row.get(key)
                if values and isinstance(values[index],(int,float)):
                    coordinates.extend((45+i/(len(history)-1)*(width-55),y(values[index])))
            if len(coordinates)>=4:canvas.create_line(*coordinates,fill=color,width=2,dash=dash)
