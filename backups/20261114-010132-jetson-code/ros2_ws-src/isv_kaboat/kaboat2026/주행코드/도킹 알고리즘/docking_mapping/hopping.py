"""Map-bound 3D hopping point editing and persistence; no control output."""
from dataclasses import asdict,dataclass
from pathlib import Path
import shutil
import math
import numpy as np
import yaml
from .geometry import ENU,apply
from .reference import digest


def number(value,name):
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value):
        raise ValueError(name+' must be finite numeric input')
    return float(value)


def click_on_plane(enu,latitude,longitude,height_enu_m):
    """Hold clicked lat/lon exactly while solving WGS84 height for fixed ENU Z."""
    lat=number(latitude,'latitude');lon=number(longitude,'longitude');z=number(height_enu_m,'ENU height')
    if abs(lat)>85.05112878 or abs(lon)>180:raise ValueError('Click outside Web Mercator coordinates')
    altitude=enu.origin[2]
    first=enu.to_enu([lat,lon,altitude]);second=enu.to_enu([lat,lon,altitude+10.])
    dz=(second[2]-first[2])/10.
    if abs(dz)<.5:raise ValueError('Click is outside the local ENU map')
    return enu.to_enu([lat,lon,altitude+(z-first[2])/dz])


@dataclass(frozen=True)
class HoppingPoint:
    reference_id:str
    reference_version:int
    point_dock:tuple
    name:str='hop1'
    radius_m:float=1.
    hold_seconds:float=10.
    heading_dock_deg:object=None

    def __post_init__(self):
        if not isinstance(self.name,str) or not 1<=len(self.name.strip())<=64 or any(ord(c)<32 for c in self.name):
            raise ValueError('Hopping name must be 1..64 printable characters')
        point=np.asarray(self.point_dock,dtype=float)
        if point.shape!=(3,) or not np.isfinite(point).all():raise ValueError('Invalid dock_ref hopping XYZ')
        if not .05<=number(self.radius_m,'radius_m')<=20:raise ValueError('Radius must be .05..20 m')
        if not .1<=number(self.hold_seconds,'hold_seconds')<=600:raise ValueError('Hold must be .1..600 s')
        if self.heading_dock_deg is not None:number(self.heading_dock_deg,'heading_dock_deg')
        if type(self.reference_version) is not int or self.reference_version<1:raise ValueError('Invalid reference version')
        object.__setattr__(self,'point_dock',tuple(map(float,point)))

    def check_reference(self,reference):
        if (self.reference_id,self.reference_version)!=(reference.reference_id,reference.version):
            raise ValueError('Hopping point belongs to another reference map/version')

    @classmethod
    def from_click(cls,reference,latitude,longitude,height_enu_m,settings,max_distance_m):
        enu=ENU(reference.origin_lla)
        point=click_on_plane(enu,latitude,longitude,height_enu_m)
        dock=apply([point],np.linalg.inv(reference.enu_from_dock))[0]
        if np.linalg.norm(dock[:2])>max_distance_m:raise ValueError('Hopping point too far from reference structure')
        return cls(reference.reference_id,reference.version,tuple(dock),**settings)

    def preview(self,reference,world_from_dock=None):
        self.check_reference(reference)
        point=apply([self.point_dock],reference.enu_from_dock if world_from_dock is None else world_from_dock)[0]
        lla=ENU(reference.origin_lla).to_lla(point)
        return dict(asdict(self),point_enu=point.tolist(),latlon=lla[:2].tolist(),height_enu_m=float(point[2]))


def write_hopping(folder,reference,hopping):
    if hopping is not None:hopping.check_reference(reference)
    folder=Path(folder)
    data=dict(schema=1,reference_id=reference.reference_id,reference_version=reference.version,
              reference_points_sha256=digest(reference.points),frame='dock_ref',
              deleted=hopping is None,point=None if hopping is None else asdict(hopping))
    if data['point'] is not None:data['point']['point_dock']=list(hopping.point_dock)
    destination=folder/'hopping_point.yaml';temporary=folder/'hopping_point.tmp'
    temporary.write_text(yaml.safe_dump(data,allow_unicode=True,sort_keys=False),encoding='utf-8')
    if destination.exists():shutil.copy2(destination,folder/'hopping_point.previous.yaml')
    temporary.replace(destination)
    return destination


def read_hopping(folder,reference):
    filename=Path(folder)/'hopping_point.yaml'
    if not filename.is_file():return None
    data=yaml.safe_load(filename.read_text(encoding='utf-8'))
    if (not isinstance(data,dict) or data.get('schema')!=1 or data.get('frame')!='dock_ref'
            or data.get('reference_id')!=reference.reference_id
            or data.get('reference_version')!=reference.version
            or data.get('reference_points_sha256')!=digest(reference.points)
            or type(data.get('deleted')) is not bool):
        raise ValueError('Hopping file reference/version/geometry mismatch')
    if data['deleted']:return None
    if not isinstance(data.get('point'),dict):raise ValueError('Missing hopping point data')
    result=HoppingPoint(**data['point']);result.check_reference(reference)
    return result


def roi_from_clicks(enu,corners,zmin,zmax):
    if not isinstance(corners,list) or len(corners)!=2 or any(not isinstance(p,list) or len(p)!=2 for p in corners):
        raise ValueError('Structure selection requires two map corners')
    low_z=number(zmin,'ROI min Z');high_z=number(zmax,'ROI max Z')
    if not low_z<high_z:raise ValueError('ROI min Z must be below max Z')
    points=np.asarray([click_on_plane(enu,lat,lon,0.) for lat in [corners[0][0],corners[1][0]]
                       for lon in [corners[0][1],corners[1][1]]])
    low=points.min(axis=0);high=points.max(axis=0);low[2]=low_z;high[2]=high_z
    if np.any(high-low<=0) or np.any(high[:2]-low[:2]>200):raise ValueError('Invalid or excessive structure ROI')
    return low.tolist(),high.tolist()
