#!/usr/bin/env python3
"""Check standard IMU values and active VectorNav raw fields."""
import math
from _common import parser,run,finite

def inspect_imu(msg,raw):
    if msg is None or raw is None: return dict(valid=False,issues=['Both standard IMU and VectorNav raw messages are required'])
    q=[msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w]
    gyro=[msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z]
    accel=[msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z]
    ypr=[raw.yawpitchroll.x,raw.yawpitchroll.y,raw.yawpitchroll.z]
    rate=[raw.angularrate.x,raw.angularrate.y,raw.angularrate.z]
    norm=math.sqrt(sum(v*v for v in q));fields=int(raw.group_fields)
    active=bool(fields&0x0008 and fields&0x0020)
    orientation_available=bool(msg.orientation_covariance[0]!=-1)
    valid=bool(finite(q+gyro+accel+ypr+rate) and abs(norm-1)<.05 and active and orientation_available)
    return dict(valid=valid,quaternion_xyzw=q,quaternion_norm=norm,angular_velocity_rad_s=gyro,
        acceleration_m_s2=accel,raw_yaw_pitch_roll_deg=ypr,raw_angular_rate_rad_s=rate,
        raw_group_fields=fields,required_raw_fields_active=active,orientation_available=orientation_available,
        issues=[] if valid else ['Non-finite values, non-unit quaternion, unavailable orientation or inactive raw fields'],
        note='Raw VectorNav heading and ROS orientation have different frame conventions; they are not directly compared')

def main():
    p=parser(__doc__);p.add_argument('--topic',default='/vectornav/imu');p.add_argument('--raw-topic',default='/vectornav/raw/common');args=p.parse_args()
    return run('imu',args,{args.topic:'sensor_msgs/msg/Imu',args.raw_topic:'vectornav_msgs/msg/CommonGroup'},lambda latest:inspect_imu(latest.get(args.topic),latest.get(args.raw_topic)))

if __name__=='__main__': raise SystemExit(main())
