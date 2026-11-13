"""Map-scoped acquisition area for boat centres, independent of pose priors."""
import json
import math
import time
from pathlib import Path
from .tank_localization_prior import PosePriorStore
from .tank_storage import atomic_json


def validate_region(value):
    vertices=value.get('vertices')
    if vertices is None:
        bounds=value['bounds']
        if len(bounds)!=4 or any(isinstance(v,bool) for v in bounds):
            raise ValueError('Region requires four numeric world coordinates')
        x0,y0,x1,y1=map(float,bounds)
        if not all(math.isfinite(v) for v in (x0,y0,x1,y1)) or x0>=x1 or y0>=y1:
            raise ValueError('Region must have positive area')
        vertices=[[x0,y0],[x1,y0],[x1,y1],[x0,y1]]
    if (not isinstance(vertices,(list,tuple)) or len(vertices)!=4 or
        any(not isinstance(p,(list,tuple)) or len(p)!=2 or
            any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in p) for p in vertices)):
        raise ValueError('Region requires four finite world-coordinate vertices')
    vertices=[[float(v) for v in p] for p in vertices]
    turns=[]
    for i,a in enumerate(vertices):
        b,c=vertices[(i+1)%4],vertices[(i+2)%4]
        turns.append((b[0]-a[0])*(c[1]-b[1])-(b[1]-a[1])*(c[0]-b[0]))
    if not (all(v>1e-8 for v in turns) or all(v< -1e-8 for v in turns)):
        raise ValueError('꼭짓점은 서로 교차하지 않는 사각형으로 배치하세요')
    bounds=[min(p[0] for p in vertices),min(p[1] for p in vertices),
            max(p[0] for p in vertices),max(p[1] for p in vertices)]
    enabled=value.get('enabled',True)
    if not isinstance(enabled,bool):raise ValueError('Region enabled must be boolean')
    return dict(bounds=bounds,vertices=vertices,enabled=enabled)


def region_vertices(region):
    if region.get('vertices') is not None:return region['vertices']
    x0,y0,x1,y1=region['bounds']
    return [[x0,y0],[x1,y0],[x1,y1],[x0,y1]]


def region_mask_xy(region,x,y):
    """Vectorized world-space quadrilateral test, including boundary points."""
    import numpy as np
    x,y=np.asarray(x),np.asarray(y)
    finite=np.isfinite(x)&np.isfinite(y)
    if not region or not region['enabled']:return finite
    vertices=region_vertices(region)
    positive=np.ones(x.shape,dtype=bool);negative=positive.copy()
    for i,(ax,ay) in enumerate(vertices):
        bx,by=vertices[(i+1)%4]
        cross=(bx-ax)*(y-ay)-(by-ay)*(x-ax)
        tolerance=1e-8*max(1.,math.hypot(bx-ax,by-ay))
        positive&=cross>=-tolerance;negative&=cross<=tolerance
    return finite&(positive|negative)


def region_allows(region,point):
    if not region or not region['enabled']:return True
    x,y=point['x'],point['y']
    if not math.isfinite(x) or not math.isfinite(y):return False
    vertices=region_vertices(region);crosses=[]
    for i,(ax,ay) in enumerate(vertices):
        bx,by=vertices[(i+1)%4]
        cross=(bx-ax)*(y-ay)-(by-ay)*(x-ax)
        tolerance=1e-8*max(1.,math.hypot(bx-ax,by-ay))
        crosses.append((cross,tolerance))
    return all(c>=-t for c,t in crosses) or all(c<=t for c,t in crosses)


class LocalizationRegionStore(PosePriorStore):
    def __init__(self,root=None):
        super().__init__(root if root is not None else Path.home()/'water_trials/tank_maps/localization_regions')

    def read(self,map_id):
        try:value=json.loads(self.path(map_id).read_text(encoding='utf-8'))
        except FileNotFoundError:return None
        if value.get('map_id')!=map_id:raise ValueError('Localization region map mismatch')
        return dict(validate_region(value),map_id=map_id,updated_unix=value.get('updated_unix',0.))

    def save(self,map_id,value):
        result=dict(validate_region(value),map_id=map_id,updated_unix=time.time())
        atomic_json(self.path(map_id),result)
        return result
