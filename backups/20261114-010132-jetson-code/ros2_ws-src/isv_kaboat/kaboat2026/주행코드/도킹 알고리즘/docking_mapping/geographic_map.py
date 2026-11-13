"""Full GPS-anchored survey snapshots, independent of docking ROI and output."""
from pathlib import Path
from datetime import datetime,timezone
import json,math,uuid,hashlib
import yaml

class GeographicMapStore:
    def __init__(self,root):self.root=Path(root)
    def available(self):
        return any(p.is_file() and not p.parent.is_symlink() for p in self.root.glob('map_*/docking_map_metadata.yaml'))
    def save(self,mapper):
        # Mapper.save writes XYZ, path, WGS84 origin, configuration and display anchor.
        # It commits its metadata marker only after the point files are complete.
        return mapper.save(self.root)
    def load(self,config):
        from .mapper import Mapper
        candidate=Mapper(config)
        folder=candidate.load(self.root)
        return candidate,folder
    def _folder(self,map_name):
        if not isinstance(map_name,str) or Path(map_name).name!=map_name or not map_name.startswith('map_'):
            raise ValueError('Invalid GPS map identity')
        folder=self.root/map_name
        if folder.is_symlink() or folder.resolve().parent!=self.root.resolve() or not (folder/'docking_map_metadata.yaml').is_file():
            raise ValueError('GPS map is unavailable')
        return folder
    @staticmethod
    def _coordinates(latitude,longitude):
        if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in (latitude,longitude)) or abs(latitude)>90 or abs(longitude)>180:
            raise ValueError('Invalid target GPS coordinates')
        return float(latitude),float(longitude)
    def write_target(self,map_name,latitude,longitude):
        folder=self._folder(map_name);latitude,longitude=self._coordinates(latitude,longitude)
        value=dict(schema=1,map_name=map_name,latitude=latitude,longitude=longitude,
                   updated_utc=datetime.now(timezone.utc).isoformat())
        value=self.target_in_map(map_name,value)
        temporary=folder/('gps_target_'+uuid.uuid4().hex+'.tmp')
        with temporary.open('x',encoding='utf-8') as stream:json.dump(value,stream,allow_nan=False)
        temporary.replace(folder/'gps_target.json')
        return value
    def read_target(self,map_name):
        file=self._folder(map_name)/'gps_target.json'
        if not file.exists():return None
        if file.is_symlink() or file.stat().st_size>4096:raise ValueError('Invalid target file')
        value=json.loads(file.read_text(encoding='utf-8'))
        if not isinstance(value,dict) or value.get('schema') not in (1,2) or value.get('map_name')!=map_name:
            raise ValueError('Target belongs to another GPS map')
        self._coordinates(value.get('latitude'),value.get('longitude'))
        return self.target_in_map(map_name,value)
    def target_in_map(self,map_name,value):
        from .geometry import ENU
        folder=self._folder(map_name)
        metadata=yaml.safe_load((folder/'docking_map_metadata.yaml').read_text(encoding='utf-8'))
        geometry_hash=hashlib.sha256((folder/'docking_map.npz').read_bytes()).hexdigest()
        origin=metadata['origin_lla']
        if value.get('schema')==2:
            xy=value.get('point_map_xy_m')
            if (value.get('point_frame')!='saved_map_enu_xy' or value.get('map_origin_lla')!=origin or value.get('map_geometry_sha256')!=geometry_hash
                    or not isinstance(xy,list) or len(xy)!=2
                    or any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in xy)):
                raise ValueError('Target map frame or geometry differs from saved map')
            return dict(value)
        # Planar navigation target: XY only; no unmeasured water height is invented.
        xy=ENU(origin).to_enu([value['latitude'],value['longitude'],origin[2]])[:2]
        return dict(value,schema=2,point_frame='saved_map_enu_xy',point_map_xy_m=xy.tolist(),
                    map_geometry_sha256=geometry_hash,map_origin_lla=origin)
    def upgrade_target(self,map_name):
        value=self.read_target(map_name)
        if value is None:return None
        folder=self._folder(map_name);file=folder/'gps_target.json'
        previous=json.loads(file.read_text(encoding='utf-8'))
        if previous==value:return value
        temporary=folder/('gps_target_'+uuid.uuid4().hex+'.tmp')
        with temporary.open('x',encoding='utf-8') as stream:json.dump(value,stream,allow_nan=False)
        temporary.replace(file)
        return value
