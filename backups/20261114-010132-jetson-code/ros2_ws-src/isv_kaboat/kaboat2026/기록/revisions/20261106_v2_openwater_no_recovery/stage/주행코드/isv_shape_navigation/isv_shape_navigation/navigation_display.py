"""Read-only SLAM/obstacle projection and explanations of existing decisions.

No planner, output command or mission state is changed by this module.
GPS ENU and the SLAM map are displayed independently without invented alignment.
"""
from collections import deque
import math
import numpy as np
from .core import wrap


def direction(degrees):
    if degrees is None or not math.isfinite(degrees):return '—'
    return ('좌 ' if degrees>0 else '우 ' if degrees<0 else '')+f'{abs(degrees):.1f}°'


class LidarDisplay:
    """Raw scan display independent of navigation; briefly retain delayed frames."""
    def __init__(self, topic='/scan', range_limit=20., hold_seconds=2.):
        self.topic,self.range_limit,self.hold_seconds=topic,range_limit,hold_seconds
        self.points=[];self.stamp=None;self.received_at=None;self.frames=0

    def observe(self,scan,received_at):
        points=scan.points(scan.yaw,scan.x,scan.y)
        if len(points):points=points[np.linalg.norm(points,axis=1)<=self.range_limit]
        points=points[::max(1,math.ceil(len(points)/600))]
        self.points=points.tolist();self.stamp=scan.stamp;self.received_at=received_at
        self.frames+=1

    def snapshot(self,ros_now,monotonic_now):
        if self.received_at is None:
            return dict(topic=self.topic,points=[],age=None,receive_age=None,status='WAIT',frames=0)
        age=ros_now-self.stamp;received=monotonic_now-self.received_at
        if not 0<=received<=self.hold_seconds:
            status='STALE';points=[]
        else:
            status='LIVE' if received<=.5 and -.1<=age<=.5 else 'DELAYED'
            points=self.points
        return dict(topic=self.topic,points=points,age=age,receive_age=received,status=status,frames=self.frames)


def decision_display(report):
    """Explain the existing chosen corridor; all body angles are CCW-left+."""
    desired=report.get('heading_error_deg')
    avoidance=report.get('avoidance') or {}
    selected=avoidance.get('selected_angle_deg')
    v2 = avoidance.get('algorithm') == 'V2_DBSCAN_GAP'
    kind=report.get('command_kind','NEUTRAL')
    command=None
    reason='정지 또는 입력 대기: 이동 방향 명령 없음'
    if report.get('state')=='NO_SAFE_GAP':
        diag=avoidance.get('diagnostics') or {}
        reason='충돌 검사 통과 방향 없음 · '+str(diag.get('free',0))+'/'+str(diag.get('candidates',0))+' 후보 · WP 목표각은 별도 표시'
    if kind=='AVOIDANCE' and isinstance(selected,(int,float)):
        command=selected
        reason = ('DBSCAN 장애물 사이에서 목표각에 가장 가까운 통로 중심 선택' if v2 else
                  '선체 충돌 검사를 통과한 후보 중 라이다 거리·목표각 오차의 가중합 비용이 최소인 방향을 선택')
    elif kind in ('HEADING','ALIGNMENT'):
        use_selected=(kind=='HEADING' and isinstance(selected,(int,float))
            and isinstance(desired,(int,float)) and (v2 or abs(desired)<=report.get('avoidance_fov_deg',75.)))
        command=selected if use_selected else desired
        reason='다음 GPS WP 방향으로 직접 진행' if kind=='HEADING' else 'WP3 방향을 향해 제자리 정렬'
    elif kind=='DOCKING':
        command=report.get('visual_bearing_deg')
        reason='안정적으로 확인한 목표 도형 방향을 사용'
        if report.get('state')=='SEARCH_SHAPE':
            command=report.get('search_turn_direction',1.)*20.
            reason='GPS 지점 이후 목표 도형을 찾기 위한 제자리 탐색'
        dock=report.get('dock_geometry') or {}
        if dock:
            command=dock.get('turn_command_deg')
            reason=dock.get('reason',reason)
    elif kind == 'RECOVERY_REVERSE':
        command = 180.
        reason = '후방 통로 확인 · 저속 후진'
    elif report.get('state') == 'RECOVERY_WAIT':
        reason = ('전진 통로 재확인 중' if avoidance.get('recovery') == 'CONFIRM_FORWARD'
                  else '후진 전 중립 대기')
    elif kind=='BRAKE':
        reason='WP2 도착 후 후방 여유 확인 · 1400 역추진 펄스'
    rows=[]
    chosen=None
    for index,gap in enumerate(avoidance.get('gaps',[]),1):
        left,right=gap['left_deg'],gap['right_deg']
        middle=gap.get('center_deg', (left+right)/2)
        contains=selected is not None and right-1e-6<=selected<=left+1e-6
        if contains:chosen=index
        rows.append(dict(index=index,left_deg=left,right_deg=right,width_deg=left-right,
                         centre_deg=middle,wp_offset_deg=math.degrees(wrap(math.radians(middle-desired)))
                         if isinstance(desired,(int,float)) else None,selected=contains and kind=='AVOIDANCE'))
    offset=math.degrees(wrap(math.radians(command-desired))) if command is not None and isinstance(desired,(int,float)) else None
    return dict(target_relative_deg=desired,turn_command_deg=command,selected_gap=chosen if kind=='AVOIDANCE' else None,
                wp_offset_deg=offset,gaps=rows,reason=reason,
                target_bearing_deg=(90-math.degrees(report['yaw'])-desired)%360
                    if isinstance(report.get('yaw'),(int,float)) and isinstance(desired,(int,float)) else None)


class SlamDisplay:
    def __init__(self,map_frame='tank_map',pose_timeout=.5,sync_tolerance=.25,
                 range_limit=20.,session_timeout=3.):
        self.map_frame=map_frame;self.pose_timeout=pose_timeout;self.sync_tolerance=sync_tolerance
        self.range_limit=range_limit;self.session_timeout=session_timeout
        self.session_id=None;self.session_ready=False;self.session_at=-math.inf
        self.poses=deque(maxlen=40);self.trail=deque(maxlen=1000)
        self.grid=None;self.revision=0
        self.heading_base=None;self.heading_key=None

    def session(self,identifier,ready,now):
        if not isinstance(identifier,str) or not identifier:raise ValueError('Invalid SLAM session ID')
        if self.session_id is not None and identifier!=self.session_id:
            self.poses.clear();self.trail.clear();self.grid=None;self.revision+=1
        self.session_id=identifier;self.session_ready=ready is True;self.session_at=now

    def pose(self,value):
        if not all(math.isfinite(v) for v in (value.x,value.y,value.yaw,value.stamp)):
            raise ValueError('Invalid display pose')
        if self.poses and value.stamp<=self.poses[-1].stamp:return
        self.poses.append(value)
        if not self.trail or math.hypot(value.x-self.trail[-1][0],value.y-self.trail[-1][1])>.03:
            self.trail.append((value.x,value.y))

    def occupancy(self,cells,width,height,resolution,origin):
        if (type(width) is not int or type(height) is not int or width<=0 or height<=0
                or width*height!=len(cells) or width*height>8_000_000
                or not math.isfinite(resolution) or resolution<=0
                or len(origin)!=3 or not all(math.isfinite(v) for v in origin)):
            raise ValueError('Invalid display map')
        array=np.asarray(cells,dtype=np.int16).reshape(height,width)
        if np.any(array < -1) or np.any(array>100):raise ValueError('Invalid occupancy value')
        # Conservative display-only pooling; preserve any occupied cell in a block.
        stride=max(1,math.ceil(max(width,height)/140))
        rows,cols=math.ceil(height/stride),math.ceil(width/stride)
        padded=np.full((rows*stride,cols*stride),-1,dtype=np.int16);padded[:height,:width]=array
        blocks=padded.reshape(rows,stride,cols,stride)
        occupied=np.any(blocks>=35,axis=(1,3));unknown=np.any(blocks<0,axis=(1,3))
        display=np.where(occupied,100,np.where(unknown,-1,0))
        runs=[]
        for row,line in enumerate(display):
            start=0
            for end in range(1,cols+1):
                if end==cols or line[end]!=line[start]:
                    runs.append([row,start,end-start,int(line[start])]);start=end
        self.revision+=1
        self.grid=dict(revision=self.revision,width_m=width*resolution,height_m=height*resolution,
                       cell_m=stride*resolution,origin=list(origin),rows=rows,cols=cols,runs=runs)

    def snapshot(self,now,scan,scan_timeout=.5):
        result=dict(frame=self.map_frame,session_id=self.session_id,map=self.grid,
                    pose=None,trail=list(self.trail),body_points=[],world_points=[],closest=None,
                    localization_ready=self.session_ready and 0<=now-self.session_at<=self.session_timeout,
                    status='WAIT_SLAM_POSE')
        latest=self.poses[-1] if self.poses else None
        if latest and -.1<=now-latest.stamp<=self.pose_timeout:
            result['pose']=dict(x=latest.x,y=latest.y,yaw=latest.yaw,stamp=latest.stamp)
        if scan is None or not -.1<=now-scan.stamp<=scan_timeout:
            result['status']='WAIT_SCAN';return result
        points=scan.points(scan.yaw,scan.x,scan.y)
        if len(points):points=points[np.linalg.norm(points,axis=1)<=self.range_limit]
        if len(points):
            closest=points[np.argmin(np.linalg.norm(points,axis=1))]
            result['closest']=dict(x=float(closest[0]),y=float(closest[1]),distance=float(np.linalg.norm(closest)))
        points=points[::max(1,math.ceil(len(points)/600))]
        result['body_points']=points.tolist()
        if not result['localization_ready']:
            result['status']='WAIT_LOCALIZATION';return result
        candidates=[p for p in self.poses if abs(p.stamp-scan.stamp)<=self.sync_tolerance
                    and -.1<=now-p.stamp<=self.pose_timeout]
        if not candidates:
            result['status']='WAIT_SLAM_SYNC';return result
        pose=min(candidates,key=lambda p:abs(p.stamp-scan.stamp))
        result['world_points']=[pose.world(float(x),float(y)).tolist() for x,y in points]
        if result['closest']:
            result['closest']['map_xy']=pose.world(result['closest']['x'],result['closest']['y']).tolist()
        result['status']='LIVE' if self.grid else 'WAIT_MAP'
        return result

    def angles(self,report):
        result=decision_display(report)
        yaw=report.get('yaw');key=(report.get('wp_index'),report.get('stage'))
        if key!=self.heading_key:
            self.heading_key=key;self.heading_base=yaw
        if self.heading_base is None:self.heading_base=yaw
        result['actual_heading_change_deg']=math.degrees(wrap(yaw-self.heading_base)) if yaw is not None and self.heading_base is not None else None
        return result
