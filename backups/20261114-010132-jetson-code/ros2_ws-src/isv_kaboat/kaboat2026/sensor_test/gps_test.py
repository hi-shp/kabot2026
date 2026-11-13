#!/usr/bin/env python3
"""Check GNSS receipt and valid fix separately; never accept status=-1 as a fix."""
import math
from _common import parser,run,finite

def inspect_fix(msg):
    if msg is None: return dict(valid=False,issues=['No GPS fix message received'])
    coords=finite([msg.latitude,msg.longitude]) and -90<=msg.latitude<=90 and -180<=msg.longitude<=180
    valid=msg.status.status>=0 and coords
    return dict(valid=valid,fix_status=int(msg.status.status),position_valid=valid,
        latitude=msg.latitude,longitude=msg.longitude,altitude_m=msg.altitude,
        altitude_valid=math.isfinite(msg.altitude),covariance_type=int(msg.position_covariance_type),position_covariance_m2=list(msg.position_covariance),
        issues=[] if valid else ['No valid GPS fix or invalid coordinates; message receipt alone is not navigation readiness'])

def main():
    p=parser(__doc__);p.add_argument('--topic',default='/gps/fix');args=p.parse_args()
    return run('gps',args,{args.topic:'sensor_msgs/msg/NavSatFix'},lambda latest:inspect_fix(latest.get(args.topic)))

if __name__=='__main__': raise SystemExit(main())
