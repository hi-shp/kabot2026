"""Control the running outdoor recorder without opening another serial port."""
import argparse
import json
from pathlib import Path
import time
import uuid


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('action',choices=('status','pause','resume','apply_model','freeze','load','start','stop','clear_trail','route'))
    p.add_argument('--id');p.add_argument('--percent',type=float,default=20);p.add_argument('--file')
    args=p.parse_args(argv)
    data={'action':args.action,'request_id':uuid.uuid4().hex}
    if args.action=='load':
        if not args.id:p.error('--id MODEL_ID 필요')
        data['id']=args.id
    if args.action=='start':
        if not 0<args.percent<=100:p.error('--percent 1..100')
        data['percent']=args.percent
    if args.action=='route':
        if not args.file:p.error('--file waypoints.json 필요')
        data['points']=json.loads(Path(args.file).expanduser().read_text(encoding='utf-8'))['points']
    import rclpy
    from std_msgs.msg import String
    rclpy.init();node=rclpy.create_node('outdoor_control_request');result=[]
    publisher=node.create_publisher(String,'/outdoor/request',10)
    def receive(msg):
        value=json.loads(msg.data)
        if args.action=='status':result.append(value)
        elif (value.get('ack') or {}).get('id')==data['request_id']:result.append(value['ack'])
    node.create_subscription(String,'/outdoor/status',receive,10)
    until=time.monotonic()+6;sent=False
    try:
        while not result and time.monotonic()<until:
            rclpy.spin_once(node,timeout_sec=.05)
            if not sent and args.action!='status' and publisher.get_subscription_count():
                publisher.publish(String(data=json.dumps(data)));sent=True
        if not result:raise SystemExit('야외 노드 응답 없음. 브링업 상태를 확인하세요.')
        if args.action=='status':
            value=result[0];print(json.dumps({k:value.get(k) for k in ('learning','frozen','execute','goal','arrivals','fit_status','error','bag','storage')},ensure_ascii=False,indent=2))
        elif not result[0]['ok']:raise SystemExit(result[0]['message'])
        else:print('적용됨')
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
