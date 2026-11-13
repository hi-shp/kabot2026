"""Map-scoped docking pose priors, shared by all A/B route profiles."""
import hashlib
import json
import math
from pathlib import Path
import time
from .tank_storage import atomic_json


def validate_prior(value):
    result={}
    for key,default in (('x',None),('y',None),('yaw_deg',None),
                        ('position_radius_m',3.),('yaw_tolerance_deg',20.)):
        number=value.get(key,default)
        if isinstance(number,bool):raise ValueError('Expected pose values must be numbers')
        number=float(number)
        if not math.isfinite(number):raise ValueError('Expected pose values must be finite')
        result[key]=number
    if result['position_radius_m']<=0 or not 0<result['yaw_tolerance_deg']<=180:
        raise ValueError('Position radius must be positive; angle range must be 0..180 degrees')
    result['yaw_deg']%=360.
    return result


class PosePriorStore:
    def __init__(self,root=None):
        self.root=Path(root) if root is not None else Path.home()/'water_trials/tank_maps/localization_priors'

    def path(self,map_id):
        if not isinstance(map_id,str) or not map_id or len(map_id)>256:
            raise ValueError('Load the SLAM map before setting an expected pose')
        return self.root/(hashlib.sha256(map_id.encode()).hexdigest()+'.json')

    def read(self,map_id):
        try:value=json.loads(self.path(map_id).read_text(encoding='utf-8'))
        except FileNotFoundError:return None
        if value.get('map_id')!=map_id:raise ValueError('Expected pose map mismatch')
        return dict(validate_prior(value),map_id=map_id,updated_unix=value.get('updated_unix',0.))

    def save(self,map_id,value):
        result=dict(validate_prior(value),map_id=map_id,updated_unix=time.time())
        atomic_json(self.path(map_id),result)
        return result

    def clear(self,map_id):
        self.path(map_id).unlink(missing_ok=True)


def project_prior(prior,anchor,current):
    if prior is None:return None
    result=dict(prior)
    if anchor is None or current is None:return result
    angle=math.radians(prior['yaw_deg'])-anchor[2]
    dx,dy=current[0]-anchor[0],current[1]-anchor[1]
    result['x']+=math.cos(angle)*dx-math.sin(angle)*dy
    result['y']+=math.sin(angle)*dx+math.cos(angle)*dy
    result['yaw_deg']=(prior['yaw_deg']+math.degrees(current[2]-anchor[2]))%360.
    return result


def pose_bonus(candidate,prior):
    if prior is None:return 0.
    angle=(candidate['yaw_deg']-prior['yaw_deg']+180.)%360.-180.
    distance=math.hypot(candidate['x']-prior['x'],candidate['y']-prior['y'])
    return 2.*math.exp(-.5*((distance/prior['position_radius_m'])**2+
                          (angle/prior['yaw_tolerance_deg'])**2))
