"""Small, persistent A/B profiles referencing an existing SLAM map."""
import hashlib
import json
import math
from pathlib import Path
import time
import uuid

VERSIONS = ('left', 'middle', 'right')


def atomic_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.' + uuid.uuid4().hex + '.tmp')
    try:
        temporary.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False), encoding='utf-8')
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


class DockingRoutes:
    def __init__(self, home=None):
        home = Path.home() if home is None else Path(home)
        self.maps = (home / 'water_trials/tank_maps').resolve()
        self.root = (home / 'water_trials/docking_routes').resolve()

    def map_path(self, value):
        path = Path(value).expanduser().resolve()
        if path.parent != self.maps or path.suffix != '.tankmap' or not path.is_file():
            raise ValueError('Save or load the SLAM map first.')
        return path

    def read(self, value):
        path = Path(value).expanduser().resolve()
        if path.parent != self.root or not path.name.endswith('.dockroute.json'):
            raise ValueError('Select a saved docking route.')
        profile = json.loads(path.read_text(encoding='utf-8'))
        if profile.get('format') != 'isv-docking-route-v1' or profile.get('version') not in VERSIONS:
            raise ValueError('Unsupported docking route file.')
        self.map_path(profile['map_file'])
        for key in ('a', 'b'):
            if not all(math.isfinite(float(profile['route'][key][axis])) for axis in ('x', 'y')):
                raise ValueError('Invalid docking route coordinates.')
        return profile


    def files(self):
        result = []
        for path in self.root.glob('*.dockroute.json'):
            try:
                profile = self.read(path)
                result.append({'path': str(path), 'map_file': profile['map_file'],
                               'map_name': profile['map_name'], 'map_id': profile['map_id'],
                               'version': profile['version'], 'saved_unix': profile['saved_unix']})
            except (OSError, ValueError, KeyError, TypeError):
                continue
        return sorted(result, key=lambda value: value['saved_unix'], reverse=True)

    def applied(self):
        try:
            value=json.loads((self.root/'applied.json').read_text(encoding='utf-8'))
            if value.get('format')!='isv-applied-docking-route-v1':raise ValueError('Invalid applied route.')
            return value
        except FileNotFoundError:
            return None

    def apply(self, profile):
        route=dict(profile['route'],session_id=profile['map_id'],frame_id='tank_map',
                   revision=uuid.uuid4().hex,updated_unix=time.time(),selection_source='manual_apply')
        value=dict(format='isv-applied-docking-route-v1',map_id=profile['map_id'],map_file=profile['map_file'],
                   profile_file=route['profile_file'],version=profile['version'],route=route,applied_unix=time.time())
        atomic_json(self.root/'applied.json',value)
        return value

    def save(self, map_file, map_id, version, route):
        if version not in VERSIONS:
            raise ValueError('Choose left, middle or right.')
        path = self.map_path(map_file)
        if not map_id:
            raise ValueError('Map identity is unavailable.')
        key = hashlib.sha256(str(map_id).encode()).hexdigest()[:24]
        target = self.root / (key + '.' + version + '.dockroute.json')
        active = dict(route, session_id=map_id, frame_id='tank_map',
                      profile_version=version, profile_file=str(target),
                      revision=uuid.uuid4().hex, updated_unix=time.time())
        profile = {'format': 'isv-docking-route-v1', 'version': version,
                   'map_file': str(path), 'map_name': path.stem, 'map_id': map_id,
                   'saved_unix': active['updated_unix'], 'route': active}
        atomic_json(target, profile)
        return profile

def erase_occupancy(info, raw, strokes):
    """Make every brushed cell free, matching the cyan preview exactly."""
    import numpy as np
    cells=np.frombuffer(raw,dtype=np.int8).reshape(info['height'],info['width']).copy()
    qx,qy,qz,qw=info['orientation']
    angle=math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
    co,si=math.cos(angle),math.sin(angle)
    ox,oy=info['position'][:2];r=float(info['resolution']);changed=0
    for x0,y0,x1,y1,radius in strokes:
        ax,ay=(co*(x0-ox)+si*(y0-oy))/r,(-si*(x0-ox)+co*(y0-oy))/r
        bx,by=(co*(x1-ox)+si*(y1-oy))/r,(-si*(x1-ox)+co*(y1-oy))/r
        reach=radius/r
        left=max(0,math.floor(min(ax,bx)-reach));right=min(info['width'],math.ceil(max(ax,bx)+reach))
        bottom=max(0,math.floor(min(ay,by)-reach));top=min(info['height'],math.ceil(max(ay,by)+reach))
        if left>=right or bottom>=top:continue
        yy,xx=np.ogrid[bottom:top,left:right];xx=xx+.5;yy=yy+.5
        dx,dy=bx-ax,by-ay;length=dx*dx+dy*dy
        fraction=np.clip(((xx-ax)*dx+(yy-ay)*dy)/length,0,1) if length else 0.
        inside=(xx-ax-fraction*dx)**2+(yy-ay-fraction*dy)**2<=reach**2
        area=cells[bottom:top,left:right];mask=inside&(area!=0)
        changed+=int(np.count_nonzero(mask));area[mask]=0
    return cells.tobytes(),changed


def erase_saved_map(path, map_id, strokes):
    """Replace one saved map and all its profile references, keeping world poses."""
    import fcntl
    import shutil
    import tempfile
    import numpy as np
    import yaml
    from isv_water_trials.tank_map_file import read_bundle, bundle, ACTIVE
    from isv_water_trials.tank_localization_prior import PosePriorStore
    library=DockingRoutes();path=library.map_path(path)
    values=[]
    for stroke in strokes:
        if len(stroke)!=5:raise ValueError('지우개 좌표를 확인하세요.')
        value=[float(v) for v in stroke]
        if not all(math.isfinite(v) for v in value) or value[4]<=0:raise ValueError('지우개 크기와 좌표를 확인하세요.')
        values.append(value)
    if not values:raise ValueError('지도에서 지울 부분을 먼저 드래그하세요.')
    with (library.maps/'.edit.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        old_id,manifest,files=read_bundle(path)
        if map_id!=old_id:raise ValueError('지도가 변경됐습니다. 다시 불러온 뒤 지워 주세요.')
        active=json.loads(ACTIVE.read_text())
        if active.get('map_id')!=old_id or Path(active['file'])!=path:
            raise ValueError('현재 불러온 저장 지도에서 지워 주세요.')
        info=json.loads(files['grid.json'])
        new_files=dict(files)
        new_files['grid.bin'],changed=erase_occupancy(info,files['grid.bin'],values)
        navigation_changed=0
        if 'navigation_grid.json' in files:
            nav=json.loads(files['navigation_grid.json'])
            new_files['navigation_grid.bin'],navigation_changed=erase_occupancy(nav,files['navigation_grid.bin'],values)
        if not changed and not navigation_changed:return dict(map_id=old_id,erased_cells=0,navigation_erased_cells=0)
        cells=np.frombuffer(new_files['grid.bin'],dtype=np.int8).reshape(info['height'],info['width'])
        pixels=np.where(cells<0,205,np.where(cells>=65,0,254)).astype(np.uint8)
        new_files['tank.pgm']=f"P5\n{info['width']} {info['height']}\n255\n".encode()+np.flipud(pixels).tobytes()
        new_files.pop('tank.png',None)
        config=yaml.safe_load(files['tank.yaml'])
        config.update(image='tank.pgm',mode='trinary',negate=0,occupied_thresh=.65,free_thresh=.196)
        new_files['tank.yaml']=yaml.safe_dump(config).encode()
        home=Path.home();root=home/'water_trials';priors=PosePriorStore()
        prior=priors.read(old_id)
        profiles=[]
        for profile_path in library.root.glob('*.dockroute.json'):
            value=json.loads(profile_path.read_text())
            if value.get('map_id')==old_id:profiles.append((profile_path,value))
        with tempfile.TemporaryDirectory(prefix='.erase-',dir=library.maps) as directory:
            prepared=bundle(Path(directory)/path.name,new_files,seed=manifest.get('seed'))
            new_id,_,_=read_bundle(prepared)
            key=hashlib.sha256(new_id.encode()).hexdigest()[:24]
            renames={str(p):str(p.with_name(key+'.'+v['version']+'.dockroute.json')) for p,v in profiles}
            revisions={};edited_at=time.time()
            def migrate(value):
                if isinstance(value,dict):
                    result={k:(new_id if k in ('map_id','session_id') and v==old_id else migrate(v)) for k,v in value.items()}
                    if value.get('frame_id')=='tank_map' and value.get('session_id')==old_id:
                        revision=revisions.setdefault(value.get('revision') or uuid.uuid4().hex,uuid.uuid4().hex)
                        result.update(revision=revision,updated_unix=edited_at)
                    return result
                if isinstance(value,list):return [migrate(v) for v in value]
                return renames.get(value,value) if isinstance(value,str) else value
            pending=[(Path(renames[str(p)]),migrate(v)) for p,v in profiles]
            related=[path,ACTIVE,library.root/'applied.json',root/'tank/route.json',
                     root/'tank/last_pose.json',path.with_suffix('.route.json'),priors.path(old_id)]
            related.extend(p for p,_ in profiles)
            snapshots={p:p.read_bytes() for p in related if p.exists()}
            for p in related[1:]:
                if p not in snapshots or p==priors.path(old_id) or p in [v[0] for v in profiles]:continue
                original=json.loads(snapshots[p]);updated=migrate(original)
                if updated!=original:pending.append((p,updated))
            backup=root/'map_backups'/('erase-'+time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:8])
            for p,raw in snapshots.items():
                saved=backup/p.relative_to(root);saved.parent.mkdir(parents=True,exist_ok=True);saved.write_bytes(raw)
            created=[p for p,_ in pending if not p.exists()]
            if prior is not None and not priors.path(new_id).exists():created.append(priors.path(new_id))
            if read_bundle(path)[0]!=old_id:raise ValueError('저장 중 지도가 변경됐습니다.')
            try:
                prepared.replace(path)
                for p,value in pending:atomic_json(p,value)
                if prior is not None:priors.save(new_id,prior)
                for p,_ in profiles:
                    if str(p)!=renames[str(p)]:p.unlink()
            except BaseException:
                for p,raw in snapshots.items():
                    temporary=p.with_name(p.name+'.restore');temporary.write_bytes(raw);temporary.replace(p)
                for p in created:p.unlink(missing_ok=True)
                raise
            route_path=root/'tank/route.json';applied_path=library.root/'applied.json'
            result=dict(map_id=new_id,erased_cells=changed,navigation_erased_cells=navigation_changed,
                        backup=str(backup),profiles_updated=len(profiles))
            if route_path.exists():result['route']=json.loads(route_path.read_text())
            if applied_path.exists():result['applied']=json.loads(applied_path.read_text())
            atomic_json(backup/'strokes.json',dict(map_file=str(path),previous_map_id=old_id,strokes=values))
            atomic_json(backup/'result.json',result)
            return result
