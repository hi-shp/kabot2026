"""Opt-in logging proxy for existing PWM clients, enabled only by explicit remaps.

This is NOT a safety controller. No commands are sent until an upstream request.
The upstream controller remains responsible for stopping and E-stop handling.
"""
import argparse
import json
import threading
import time
from .common import load_config


def main(argv=None):
    parser=argparse.ArgumentParser()
    parser.add_argument('--config')
    args=parser.parse_args(argv)
    cfg=load_config(args.config)
    import rclpy
    from rclpy.node import Node
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from std_msgs.msg import String
    from mechaship_interfaces.srv import ThrottlePulseWidth
    rclpy.init()
    node=Node('water_trial_command_proxy')
    pub=node.create_publisher(String,'/water_trial/command',100)
    group=ReentrantCallbackGroup()
    clients={side:node.create_client(ThrottlePulseWidth,cfg[side+'_service'],callback_group=group) for side in ('left','right')}
    latest={}
    lock=threading.Lock()
    # Do not allow a remapped upstream endpoint to target this proxy itself.
    proxy_names={side:'/water_trial/throttle_'+side+'/set_pulse_width' for side in ('left','right')}
    for side in clients:
        if clients[side].srv_name in proxy_names.values():
            raise ValueError('Proxy target loops to proxy service')

    def callback(side):
        def handle(request,response):
            if not 1100<=request.pulse_width<=1900:
                response.status=False
                return response
            with lock:
                pub.publish(String(data=json.dumps({'status':'requested','side':side,'pwm_us':request.pulse_width})))
            if not clients[side].service_is_ready():
                response.status=False
            else:
                event=threading.Event()
                future=clients[side].call_async(request)
                future.add_done_callback(lambda _:event.set())
                response.status=bool(event.wait(.7) and not future.exception() and future.result() and future.result().status)
            with lock:
                now=time.monotonic()
                if response.status:
                    latest[side]=(now,request.pulse_width)
                else:
                    latest.pop(side,None)
                valid=len(latest)==2 and all(now-t<cfg['command_max_age_s'] for t,_ in latest.values())
                payload={'status':'acknowledged' if valid else 'failed','source':'logging_proxy'}
                if valid:payload.update(left_pwm_us=latest['left'][1],right_pwm_us=latest['right'][1])
                pub.publish(String(data=json.dumps(payload)))
            return response
        return handle
    services=[node.create_service(ThrottlePulseWidth,proxy_names[side],callback(side),callback_group=group) for side in clients]
    executor=MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    print('Logging proxy ready. Only explicitly remapped upstream requests are forwarded.')
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
