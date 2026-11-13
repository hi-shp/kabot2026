"""Keep received GPS coordinates visible independently of navigation acceptance."""
import math


MESSAGES = {
    'GPS_VALID':'정상 · 항법 사용 가능',
    'GPS_NO_FIX':'GPS 메시지 수신 · 위치 미확정(NO FIX)',
    'GPS_COVARIANCE_UNKNOWN':'좌표 수신 · 정확도 정보 없음',
    'GPS_COVARIANCE_INVALID':'좌표 수신 · 정확도 정보 오류',
    'GPS_STALE':'좌표 수신 · GPS 시각 지연/불일치',
    'GPS_OUT_OF_ORDER':'좌표 수신 · GPS 시각 중복/역순',
    'GPS_INVALID_COORDINATES':'GPS 수신 · 좌표 값 오류',
    'GPS_UNCERTAIN':'좌표 수신 · 항법 정확도 기준 초과',
}


def inspect_fix(message, stamp, now, timeout, max_sigma, previous_stamp=None):
    lat,lon=message.latitude,message.longitude
    coordinates_ok=(math.isfinite(lat) and -90<=lat<=90 and math.isfinite(lon) and -180<=lon<=180)
    view=dict(latitude=float(lat) if coordinates_ok else None,longitude=float(lon) if coordinates_ok else None,
              stamp=stamp,fix_status=int(message.status.status),covariance_type=int(message.position_covariance_type),
              sigma=None,accepted=False,nav_valid=False)
    reason='GPS_VALID'
    if previous_stamp is not None and stamp<=previous_stamp:
        reason='GPS_OUT_OF_ORDER'
    elif not coordinates_ok:
        reason='GPS_INVALID_COORDINATES'
    elif message.status.status<0:
        reason='GPS_NO_FIX'
    elif not -.1<=now-stamp<=timeout:
        reason='GPS_STALE'
    elif message.position_covariance_type==0:
        reason='GPS_COVARIANCE_UNKNOWN'
    else:
        covariance=message.position_covariance
        if len(covariance)!=9:
            reason='GPS_COVARIANCE_INVALID'
        else:
            xx,xy,yx,yy=[covariance[i] for i in (0,1,3,4)]
            if (not all(math.isfinite(v) for v in (xx,xy,yx,yy)) or xx<0 or yy<0
                    or abs(xy-yx)>1e-6 or xx*yy-xy*yx < -1e-6):
                reason='GPS_COVARIANCE_INVALID'
            else:
                largest=.5*(xx+yy+math.sqrt(max(0.,(xx-yy)**2+4*xy*yx)))
                view['sigma']=math.sqrt(max(0.,largest))
                view['accepted']=True
                if view['sigma']>max_sigma:reason='GPS_UNCERTAIN'
                else:view['nav_valid']=True
    view.update(reason=reason,message=MESSAGES[reason])
    return view


def observation_report(observation, ros_now, receive_age, timeout):
    if observation is None:return None
    result=dict(observation,message_age=ros_now-observation['stamp'],receive_age=receive_age)
    fresh=0<=receive_age<=timeout and -.1<=result['message_age']<=timeout
    if not fresh:
        result.update(nav_valid=False,reason='GPS_STALE',message='GPS 수신 지연 · 마지막 수신 좌표')
    return result
