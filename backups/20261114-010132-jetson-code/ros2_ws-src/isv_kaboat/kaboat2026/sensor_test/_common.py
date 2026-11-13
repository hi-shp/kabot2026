"""Shared ROS receipt measurements; imports do not initialize ROS or send commands."""
import argparse
from collections import deque
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.executors import ExternalShutdownException
    from rosidl_runtime_py.utilities import get_message
except ImportError as exc:
    raise SystemExit('ROS environment required. Run: bash sensor_test/run.sh <sensor>') from exc
ROOT=Path(__file__).resolve().parent.parent

def positive(value):
    result=float(value)
    if not math.isfinite(result) or result<=0: raise argparse.ArgumentTypeError('must be finite and positive')
    return result

def parser(description):
    p=argparse.ArgumentParser(description=description)
    p.add_argument('--duration',type=positive,default=10.,help='Observation seconds (default 10)')
    p.add_argument('--watch',action='store_true',help='Observe until Ctrl+C')
    p.add_argument('--max-age',type=positive,default=3.,help='Maximum receive/header age, seconds')
    p.add_argument('--min-hz',type=positive,default=None,help='Optional minimum measured rate for each topic')
    p.add_argument('--output',type=Path,help='JSON report path')
    return p

def finite(values): return all(math.isfinite(float(v)) for v in values)

def clean(value):
    if type(value).__module__.startswith("numpy") and hasattr(value,"item"): value=value.item()
    if isinstance(value,float) and not math.isfinite(value): return None
    if isinstance(value,dict): return {k:clean(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)): return [clean(v) for v in value]
    return value

class Probe(Node):
    def __init__(self,name,topics,args):
        super().__init__('sensor_test_'+name)
        self.args=args;self.latest={};self.count={t:0 for t in topics}
        self.times={t:deque(maxlen=2000) for t in topics};self.owned=[]
        for topic,kind in topics.items():
            self.owned.append(self.create_subscription(get_message(kind),topic,
                lambda msg,t=topic:self.receive(t,msg),qos_profile_sensor_data))
    def receive(self,topic,msg):
        self.count[topic]+=1;self.times[topic].append(time.monotonic());self.latest[topic]=msg
    def streams(self):
        result={};now=time.monotonic();ros_now=self.get_clock().now().nanoseconds/1e9
        for topic,times in self.times.items():
            age=now-times[-1] if times else None
            hz=(len(times)-1)/(times[-1]-times[0]) if len(times)>1 and times[-1]>times[0] else 0.
            ok=age is not None and age<self.args.max_age
            if self.args.min_hz is not None: ok=ok and hz>=self.args.min_hz
            publishers=self.count_publishers(topic) if rclpy.ok() else None
            item=dict(receiving=ok,messages=self.count[topic],hz=round(hz,2),age_s=round(age,3) if age is not None else None,publishers=publishers)
            msg=self.latest.get(topic)
            if msg is not None and hasattr(msg,'header'):
                stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
                stamp_ok=stamp>0 and -.5<=ros_now-stamp<self.args.max_age
                item.update(frame_id=msg.header.frame_id,header_age_s=round(ros_now-stamp,3),stamp_valid=stamp_ok)
                item['receiving']=ok and stamp_ok
            result[topic]=item
        return result

def run(name,args,topics,analyze,display=None):
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    probe=Probe(name,topics,args)
    def report():
        streams=probe.streams();details=analyze(probe.latest)
        ok=all(v['receiving'] for v in streams.values()) and details.get('valid',False)
        return clean(dict(sensor=name,result='PASS' if ok else 'FAIL',ok=ok,
            checked_at=datetime.now(timezone.utc).isoformat(),streams=streams,details=details))
    stopped=False
    try:
        deadline=time.monotonic()+args.duration;next_print=0.
        while rclpy.ok() and (args.watch or time.monotonic()<deadline):
            rclpy.spin_once(probe,timeout_sec=.05)
            if display is not None and display(probe.latest) is False: break
            if args.watch and time.monotonic()>=next_print:
                print(json.dumps(report(),ensure_ascii=False),flush=True);next_print=time.monotonic()+1
    except (KeyboardInterrupt,ExternalShutdownException): stopped=True
    finally:
        try:
            result=report();result['interrupted']=stopped
            target=args.output or ROOT/'기록/sensor_test'/(name+'_latest.json')
            target=target.expanduser();target.parent.mkdir(parents=True,exist_ok=True)
            target.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
            print(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False))
            print('Report:',target)
        finally:
            probe.destroy_node()
            if rclpy.ok(): rclpy.shutdown()
    return 0 if result['ok'] else 2

def actuator_details(latest,topic):
    msg=latest.get(topic)
    if msg is None: return dict(valid=False,issues=['No Arduino telemetry received'])
    try: state=json.loads(msg.data)
    except (ValueError,TypeError): return dict(valid=False,issues=['Invalid actuator JSON'])
    if not isinstance(state,dict): return dict(valid=False,issues=['Actuator status must be an object'])
    valid=state.get('receiving') is True and not state.get('error')
    pwm_ok=all(type(state.get(k)) is int and 1100<=state[k]<=1900 for k in ('left','right'))
    return dict(valid=bool(valid and pwm_ok),telemetry=state,
        issues=[] if valid and pwm_ok else ['MCU not receiving, bridge error, or invalid logical PWM'],
        interpretation='Logical PWM and MCU output telemetry; physical shaft rotation/thrust is not measured')
