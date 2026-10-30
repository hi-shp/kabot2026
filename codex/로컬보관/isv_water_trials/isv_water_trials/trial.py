"""Operator-started, low-PWM, finite repeated maneuvers. Default is dry run."""
import argparse
from dataclasses import dataclass
import json
import math
import time

from .common import load_config
from .shutdown import ExitSignals, neutralize


@dataclass(frozen=True)
class Step:
    label: str
    duration_s: float
    left_pwm_us: int
    right_pwm_us: int


def plan(kind, repeats, neutral=1500, delta=60):
    if not 1 <= repeats <= 10 or not 10 <= delta <= 100:
        raise ValueError('repeats 1..10; delta 10..100 us')
    steps=[]
    for _ in range(repeats):
        steps.append(Step('idle',5,neutral,neutral))
        if kind=='surge':
            for level in (delta//2,delta):
                steps.extend([Step('straight',8,neutral+level,neutral+level),Step('coast',12,neutral,neutral)])
        elif kind=='yaw':
            steps.extend([Step('turn_right',6,neutral+delta,neutral),Step('idle',8,neutral,neutral),
                          Step('turn_left',6,neutral,neutral+delta),Step('idle',8,neutral,neutral)])
        elif kind=='zigzag':
            for i in range(6):
                steps.append(Step('zigzag',3,neutral+delta if i%2==0 else neutral+delta//2,
                                  neutral+delta//2 if i%2==0 else neutral+delta))
        else:
            raise ValueError('Unknown trial')
    steps.append(Step('end',3,neutral,neutral))
    if any(not 1400 <= p <= 1600 for step in steps for p in (step.left_pwm_us,step.right_pwm_us)):
        raise ValueError('Trial PWM outside conservative 1400..1600 range')
    return steps


def main(argv=None):
    parser=argparse.ArgumentParser()
    parser.add_argument('kind',choices=['surge','yaw','zigzag'])
    parser.add_argument('--config')
    parser.add_argument('--repeats',type=int,default=3)
    parser.add_argument('--delta',type=int,default=60)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--operator-confirmed',action='store_true',help='Operator at water with tested hardware E-stop and clear recovery area')
    parser.add_argument('--exclusive-control-confirmed',action='store_true',help='All other controllers stopped')
    args=parser.parse_args(argv)
    cfg=load_config(args.config)
    steps=plan(args.kind,args.repeats,cfg['neutral_pwm_us'],args.delta)
    for step in steps:
        print(step)
    print('TOTAL_SECONDS',sum(s.duration_s for s in steps))
    if not args.execute:
        print('DRY_RUN: no ROS initialized and no actuator requests sent')
        return
    if not (args.operator_confirmed and args.exclusive_control_confirmed and cfg['frame_confirmed']):
        parser.error('Execution requires operator/exclusive-control flags and frame_confirmed=true')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import NavSatFix
    from vectornav_msgs.msg import CommonGroup
    from std_msgs.msg import Bool,String
    from mechaship_interfaces.srv import ThrottlePulseWidth
    from rclpy.signals import SignalHandlerOptions
    # Retain a working ROS context for the best-effort stop in finally on Ctrl+C.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    exits = ExitSignals()
    node=Node('water_trial_runner')
    command=node.create_publisher(String,'/water_trial/command',100)
    marker=node.create_publisher(String,'/water_trial/mark',100)
    left=node.create_client(ThrottlePulseWidth,cfg['left_service'])
    right=node.create_client(ThrottlePulseWidth,cfg['right_service'])
    seen={}
    def callback(key):
        def receive(msg): seen[key]=(time.monotonic(),msg)
        return receive
    subscriptions=[node.create_subscription(typ,topic,callback(key),qos_profile_sensor_data) for typ,topic,key in
                   [(NavSatFix,cfg['gps_topic'],'gps'),(CommonGroup,cfg['vectornav_topic'],'vn'),(Bool,cfg['estop_topic'],'estop')]]

    def healthy():
        if exits.requested:
            raise KeyboardInterrupt
        now=time.monotonic()
        if any(key not in seen or now-seen[key][0]>cfg['max_sensor_gap_s'] for key in ('gps','vn','estop')):
            return False
        gps=seen['gps'][1]
        if gps.status.status<0 or not math.isfinite(gps.latitude) or not math.isfinite(gps.longitude): return False
        if gps.position_covariance_type==0 and not cfg['allow_unknown_gps_covariance']: return False
        if gps.position_covariance_type and any(not math.isfinite(gps.position_covariance[i]) or not 0<=gps.position_covariance[i]<=cfg['max_gps_variance_m2'] for i in (0,4)): return False
        vn=seen['vn'][1]
        if not vn.group_fields&0x8 or not math.isfinite(vn.yawpitchroll.x): return False
        infos=node.get_publishers_info_by_topic(cfg['gps_topic'])
        return (not seen['estop'][1].data and len(infos)==1 and not any('spoof' in i.node_name.lower() for i in infos)
                and command.get_subscription_count()>0 and marker.get_subscription_count()>0)

    def send(l,r,label,stop=False):
        payload=dict(left_pwm_us=l,right_pwm_us=r,label=label,status='requested')
        command.publish(String(data=json.dumps(payload)))
        futures=[client.call_async(ThrottlePulseWidth.Request(pulse_width=int(p))) for client,p in ((left,l),(right,r))]
        until=time.monotonic()+.7
        while not all(f.done() for f in futures) and time.monotonic()<until:
            rclpy.spin_once(node,timeout_sec=.02)
            if not stop and not healthy(): raise RuntimeError('Sensor/E-stop/recorder guard failed while waiting for command acknowledgement')
        ok=all(f.done() and not f.exception() and f.result() and f.result().status for f in futures)
        payload['status']='acknowledged' if ok else 'failed'
        command.publish(String(data=json.dumps(payload)))
        if not ok: raise RuntimeError('Paired actuator acknowledgement failed')

    try:
        until=time.monotonic()+10
        while time.monotonic()<until and not (healthy() and left.service_is_ready() and right.service_is_ready()):
            rclpy.spin_once(node,timeout_sec=.1)
        if not (healthy() and left.service_is_ready() and right.service_is_ready()):
            raise RuntimeError('Preflight failed: no trial started')
        for step in steps:
            marker.publish(String(data=json.dumps({'label':step.label})))
            until=time.monotonic()+step.duration_s
            while time.monotonic()<until:
                if not healthy(): raise RuntimeError('Sensor/E-stop/recorder guard failed')
                send(step.left_pwm_us,step.right_pwm_us,step.label)
                tick=time.monotonic()+.15
                while time.monotonic()<tick:
                    rclpy.spin_once(node,timeout_sec=.02)
    except KeyboardInterrupt:
        print('Operator interrupted trial')
    finally:
        try:
            if not neutralize(node, (left, right)):
                print('STOP_UNCONFIRMED: use physical EMO', flush=True)
            marker.publish(String(data=json.dumps({'label':'end'})))
        finally:
            node.destroy_node()
            if rclpy.ok(): rclpy.shutdown()
            exits.restore()
