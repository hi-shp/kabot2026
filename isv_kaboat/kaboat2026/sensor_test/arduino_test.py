#!/usr/bin/env python3
"""Check MCU serial-link telemetry without arming or sending PWM commands."""
from _common import parser,run,actuator_details

def main():
    p=parser(__doc__);p.add_argument('--topic',default='/actuators/status');args=p.parse_args()
    return run('arduino',args,{args.topic:'std_msgs/msg/String'},lambda latest:actuator_details(latest,args.topic))

if __name__=='__main__': raise SystemExit(main())
