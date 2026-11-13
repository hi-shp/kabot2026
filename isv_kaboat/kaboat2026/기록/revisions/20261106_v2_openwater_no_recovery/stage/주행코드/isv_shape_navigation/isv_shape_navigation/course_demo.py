"""Explicitly simulated dashboard preview; no ROS/SSH or actuator connection."""
import argparse
import math
from pathlib import Path
import time
import webbrowser
from queue import Empty

from .mission_dashboard import Dashboard
from .route_mission import RouteConfig
from .follow import MODEL_CLASSES
from .navigation_display import SlamDisplay
from .core import Pose,Scan
import numpy as np


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',type=int,default=8771)
    parser.add_argument('--no-browser',action='store_true')
    parser.add_argument('--desktop',action='store_true',help='Open a native desktop window; no HTTP server')
    args=parser.parse_args()
    web=Path(__file__).resolve().parents[1]/'web'
    if not web.is_dir():
        from ament_index_python.packages import get_package_share_directory
        web=Path(get_package_share_directory('isv_shape_navigation'))/'web'
    if args.desktop:
        from .desktop_dashboard import DesktopDashboard
        dashboard=DesktopDashboard()
    else:
        dashboard=Dashboard(web,args.port)
    route=RouteConfig();points=list(zip(route.wp_latitudes,route.wp_longitudes))
    label='red triangle';running=True;elapsed=0.;last=time.monotonic()
    display=SlamDisplay()
    demo_map=np.zeros((120,100),dtype=np.int8)
    demo_map[[0,-1],:]=100;demo_map[:,[0,-1]]=100;demo_map[35:45,30:45]=100
    demo_map[65:75,55:70]=100
    display.occupancy(demo_map.ravel().tolist(),100,120,.2,(-3.,-3.,0.))
    print('SIMULATED DESKTOP WINDOW ONLY' if args.desktop else 'SIMULATED DASHBOARD ONLY: http://127.0.0.1:'+str(dashboard.port),flush=True)
    if not args.desktop and not args.no_browser:webbrowser.open('http://127.0.0.1:'+str(dashboard.port))
    try:
        while True:
            if args.desktop and not dashboard.is_open():break
            now=time.monotonic();delta=now-last;last=now
            if running:elapsed=min(38.,elapsed+delta)
            position=points[0]
            if elapsed<2:stage,state,index='TO_WP1','WAYPOINT_ARRIVAL',0
            elif elapsed<12:
                stage,index='WP1_TO_WP2',1
                state='AVOID_OBSTACLE' if 4<elapsed<9 else 'WP1_TO_WP2'
                f=(elapsed-2)/10;position=tuple(a+(b-a)*f for a,b in zip(points[0],points[1]))
            elif elapsed<22:stage,state,index='WP2_HOLD','WP2_HOLD',1;position=points[1]
            elif elapsed<25:stage,state,index='DOCK_TO_WP3','SEARCH_SHAPE',2;position=points[1]
            elif elapsed<36:
                stage,index='DOCK_TO_WP3',2;state='APPROACH_SHAPE' if elapsed<32 else 'ENTER_REGION'
                f=(elapsed-25)/11;position=tuple(a+(b-a)*f for a,b in zip(points[1],points[2]))
            else:stage,state,index='COMPLETE','MISSION_COMPLETE',2;position=points[2]
            hold=22-elapsed if stage=='WP2_HOLD' else 0. if elapsed>=22 else 10.
            neutral=not running or state in ('MISSION_COMPLETE','WAYPOINT_ARRIVAL','WP2_HOLD')
            pwm=[1500,1500] if neutral else [1650+round(40*math.sin(elapsed)),1650-round(40*math.sin(elapsed))]
            distance=math.hypot((points[index][0]-position[0])*111195,
                                (points[index][1]-position[1])*91000)
            report=dict(demo=True,stamp=time.time(),state=state,stage=stage,wp_index=index,
                waypoints=points,position=dict(latitude=position[0],longitude=position[1],sigma=.45),
                yaw=1.,distance_to_wp=distance,heading_error_deg=math.sin(elapsed)*12,
                hold_remaining=max(0.,hold),intended_pwm=pwm,feedback_pwm=pwm,
                output=dict(enabled=True,active=running and stage!='COMPLETE',armed=running,stop_ack=not running,error=''),
                target_label=label,stable_frames=4 if elapsed>25 else 0,
                target=dict(distance=max(1.5,8.-(elapsed-25)*.6)) if elapsed>25 else None,
                avoidance=dict(obstacles=[[1.8+i*.01,.15+math.sin(i*.2)*.1] for i in range(45)],
                               selected_angle_deg=28.,gaps=[dict(left_deg=65,right_deg=18)]),
                gps_age=.08,heading_age=.02,scan_age=.04,telemetry_age=.03)
            timestamp=report['stamp']
            display.session('DEMO-MAP',True,timestamp)
            display.pose(Pose(2.+elapsed*.13,2.+elapsed*.12,1.,timestamp))
            ranges=np.full(181,np.inf);ranges[82:98]=2.+.1*np.sin(np.arange(16)*.3)
            fake_scan=Scan(ranges,-math.pi/2,math.pi/180,.2,20.,timestamp)
            report['slam_display']=display.snapshot(timestamp,fake_scan)
            report['command_kind']='NEUTRAL' if neutral else 'AVOIDANCE' if state=='AVOID_OBSTACLE' else 'HEADING'
            if stage=='DOCK_TO_WP3' and not neutral:report['command_kind']='DOCKING'
            report['visual_bearing_deg']=math.sin(elapsed)*7.
            report['search_turn_direction']=1.
            report['decision']=display.angles(report)
            dashboard.publish(report)
            for _ in range(8):
                try:command=dashboard.commands.get_nowait()
                except Empty:break
                success=True;message='DEMO: 가상 센서 화면만 변경합니다.'
                action=command['action']
                try:
                    if action=='start':running=True
                    elif action=='stop':running=False
                    elif action=='reset':elapsed=0.;running=False
                    elif action=='set_target':
                        if command['target_label'] not in MODEL_CLASSES:raise ValueError('Invalid model class')
                        label=command['target_label']
                    elif action=='set_route':
                        values=command['waypoints']
                        route=RouteConfig(wp_latitudes=[p[0] for p in values],wp_longitudes=[p[1] for p in values])
                        points=list(zip(route.wp_latitudes,route.wp_longitudes));elapsed=0.;running=False
                    elif action=='capture_wp':
                        if command['index'] not in (0,1,2):raise ValueError('Invalid WP')
                        points[command['index']]=position;elapsed=0.;running=False
                except (KeyError,ValueError,TypeError) as error:success=False;message=str(error)
                with dashboard.lock:dashboard.last_command=dict(action=action,success=success,message=message)
            time.sleep(.1)
    except KeyboardInterrupt:pass
    finally:dashboard.close()


if __name__=='__main__':main()
