"""Vectorized per-return yaw/motion deskew to the cloud header timestamp.

Preserves every record and all non-XYZ fields. Missing timing/IMU history leaves
the original cloud available rather than inventing a motion estimate.
"""
from collections import deque
import math
import numpy as np


class MotionDeskew:
    def __init__(self, heading_offset_deg=0., lidar_yaw_deg=0.):
        self.imus=deque(maxlen=256);self.velocities=deque(maxlen=64)
        self.heading_offset_deg=heading_offset_deg;self.lidar_yaw=math.radians(lidar_yaw_deg)

    def imu(self, stamp, heading_deg, rate_ned_z=None):
        if not all(math.isfinite(v) for v in (stamp,heading_deg)):return
        if self.imus and stamp<=self.imus[-1][0]:self.imus.clear();self.velocities.clear()
        rate=-rate_ned_z if rate_ned_z is not None and math.isfinite(rate_ned_z) else math.nan
        self.imus.append((stamp,math.radians(90-heading_deg-self.heading_offset_deg),rate))

    def velocity(self, stamp, east, north, sigma):
        if (not all(math.isfinite(v) for v in (stamp,east,north,sigma)) or
            not 0<sigma<=.5 or math.hypot(east,north)>5):return
        self.velocities.append((stamp,east,north))

    def apply(self, xyz, records, fields, stamp, bigendian=False):
        history=list(self.imus)
        info=dict(applied=False,reason='IMU 시각 이력 대기',translation=False,imu_samples=len(history))
        if not len(records):return records,dict(info,reason='유효 라이다 점 대기')
        if len(history)<2:return records,info
        time_field=next((f for f in fields if f.name=='time' and f.datatype in (7,8) and f.count==1),None)
        if time_field is None:return records,dict(info,reason='점별 측정 시각 없음')
        endian='>' if bigendian else '<'
        field_dtype=endian+('f4' if time_field.datatype==7 else 'f8')
        offsets=np.ndarray((len(records),),dtype=field_dtype,buffer=records.data,
                           offset=time_field.offset,strides=(records.shape[1],)).astype(float)
        valid=np.isfinite(xyz).all(axis=1)&np.isfinite(offsets)
        if not valid.any():return records,dict(info,reason='유효 라이다 점 대기')
        dt=offsets[valid]
        if np.max(np.abs(dt))>.25:return records,dict(info,reason='점별 시각 범위 확인 필요')
        imu=np.asarray(history,dtype=float);t=stamp+dt
        info.update(imu_newest_age_s=float(stamp-imu[-1,0]),imu_oldest_age_s=float(stamp-imu[0,0]))
        # Bridge callback ordering with measured gyro, bounded to 1.5 IMU periods.
        if min(float(t.min()),stamp)<imu[0,0]-.075 or max(float(t.max()),stamp)>imu[-1,0]+.075:
            return records,info
        unwrapped=np.unwrap(imu[:,1])
        def interpolate(query):
            query=np.asarray(query,dtype=float);result=np.interp(query,imu[:,0],unwrapped)
            for index,other in ((0,1),(-1,-2)):
                rate=imu[index,2]
                if not math.isfinite(rate):rate=(unwrapped[index]-unwrapped[other])/(imu[index,0]-imu[other,0])
                if abs(rate)>5:rate=0.
                delta=query-imu[index,0]
                result=np.where(delta<0 if index==0 else delta>0,unwrapped[index]+delta*rate,result)
            return result
        yaw=interpolate(t);reference=float(interpolate(stamp))
        angle=yaw-reference;c=np.cos(angle);s=np.sin(angle);p=xyz[valid]
        x=c*p[:,0]-s*p[:,1];y=s*p[:,0]+c*p[:,1]
        velocities=list(self.velocities)
        velocity=min(velocities,key=lambda v:abs(v[0]-stamp)) if velocities else None
        if velocity and abs(velocity[0]-stamp)<=.5:
            a=reference+self.lidar_yaw;ca,sa=math.cos(a),math.sin(a)
            x+=(ca*velocity[1]+sa*velocity[2])*dt
            y+=(-sa*velocity[1]+ca*velocity[2])*dt
            info['translation']=True
        result=records.copy()
        for key,values in [('x',x),('y',y)]:
            f=next(f for f in fields if f.name==key)
            view=np.ndarray((len(result),),dtype=endian+('f4' if f.datatype==7 else 'f8'),
                            buffer=result.data,offset=f.offset,strides=(result.shape[1],))
            view[valid]=values
        info.update(applied=True,reason='',scan_span_s=float(np.ptp(dt)),
                    yaw_span_deg=float(np.degrees(np.ptp(yaw))),points=int(valid.sum()),
                    imu_extension_s=max(0.,float(t.max())-imu[-1,0],imu[0,0]-float(t.min())))
        return result,info
