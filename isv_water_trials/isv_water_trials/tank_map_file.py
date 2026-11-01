"""One immutable .tankmap bundle, verified content and a separate active pointer."""
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
import zipfile
from .tank_storage import atomic_json

ACTIVE = Path.home()/'water_trials/tank_maps/active.json'
REQUIRED = {'tank.posegraph','tank.data','grid.json','grid.bin'}


def bundle(path, files, seed=None):
    path = Path(path).expanduser().resolve()
    if path.suffix!='.tankmap': path = Path(str(path)+'.tankmap')
    if path.exists(): raise FileExistsError('기존 고정 지도입니다. 새 이름을 사용하세요: '+str(path))
    if not REQUIRED.issubset(files): raise ValueError('Incomplete map bundle')
    if seed is not None and (len(seed)!=3 or not all(math.isfinite(v) for v in seed)):raise ValueError('Invalid seed pose')
    manifest = {'version':1,'seed':seed,'sha256':{k:hashlib.sha256(v).hexdigest() for k,v in files.items()}}
    path.parent.mkdir(parents=True,exist_ok=True)
    fd,temporary = tempfile.mkstemp(prefix='.tankmap-',dir=path.parent)
    try:
        with os.fdopen(fd,'w+b') as stream:
            with zipfile.ZipFile(stream,'w',zipfile.ZIP_DEFLATED) as archive:
                for name,data in files.items(): archive.writestr(name,data)
                archive.writestr('manifest.json',json.dumps(manifest))
            stream.flush(); os.fsync(stream.fileno())
        # Exclusive publication: a concurrent save cannot overwrite an existing map.
        os.link(temporary,path)
    finally: Path(temporary).unlink(missing_ok=True)
    if os.name=='posix':
        descriptor=os.open(str(path.parent),os.O_RDONLY)
        try: os.fsync(descriptor)
        finally: os.close(descriptor)
    return path


def import_snapshot(prefix,target):
    """Import a previously completed SLAM SaveMap/SerializePoseGraph snapshot."""
    import yaml
    import numpy as np
    prefix=Path(prefix).expanduser().resolve()
    config=yaml.safe_load(Path(str(prefix)+'.yaml').read_text(encoding='utf-8'))
    image=Path(config['image']);image=image if image.is_absolute() else prefix.parent/image
    raw=image.read_bytes();index=0;words=[]
    while len(words)<4:
        while raw[index:index+1].isspace():index+=1
        if raw[index:index+1]==b'#':index=raw.index(b'\n',index)+1;continue
        end=index
        while end<len(raw) and not raw[end:end+1].isspace():end+=1
        words.append(raw[index:end]);index=end
    magic,width,height,maximum=words;width=int(width);height=int(height)
    if magic!=b'P5' or int(maximum)!=255:raise ValueError('Expected 8-bit P5 occupancy image')
    if not 0<width*height<=8_000_000:raise ValueError('Map exceeds 8 million cells')
    index+=2 if raw[index:index+2]==b'\r\n' else 1
    if len(raw)-index!=width*height:raise ValueError('Truncated occupancy image')
    pixels=np.frombuffer(raw[index:],dtype=np.uint8).reshape(height,width)
    occupancy=(pixels if config.get('negate') else 255.-pixels)/255.
    cells=np.where(occupancy>config['occupied_thresh'],100,np.where(occupancy<config['free_thresh'],0,-1)).astype(np.int8)
    # Nav2 SaveMap writes unknown cells as gray 205 even when free_thresh=.25.
    # Restore that sentinel explicitly instead of turning unobserved space free.
    if config.get('mode','trinary')!='trinary' or config.get('negate'):
        raise ValueError('Snapshot import expects a Nav2 trinary, non-negated map')
    cells[pixels==205]=-1
    x,y,angle=config['origin']
    grid=dict(width=width,height=height,resolution=float(config['resolution']),position=[x,y,0.],
              orientation=[0.,0.,math.sin(angle/2),math.cos(angle/2)])
    files={'tank.posegraph':Path(str(prefix)+'.posegraph').read_bytes(),'tank.data':Path(str(prefix)+'.data').read_bytes(),
           'tank.pgm':raw,'grid.bin':np.flipud(cells).tobytes(),'grid.json':json.dumps(grid).encode()}
    config['image']='tank.pgm';config['free_thresh']=min(config['free_thresh'],.196)
    files['tank.yaml']=yaml.safe_dump(config).encode()
    return bundle(target,files)


def read_bundle(path):
    path = Path(path).expanduser().resolve()
    with zipfile.ZipFile(path) as archive:
        entries = archive.infolist()
        if len(entries)>16 or sum(e.file_size for e in entries)>512*1024**2:
            raise ValueError('Oversized map bundle')
        if len({e.filename for e in entries})!=len(entries): raise ValueError('Duplicate bundle entries')
        manifest = json.loads(archive.read('manifest.json'))
        if manifest.get('version')!=1 or not REQUIRED.issubset(manifest.get('sha256',{})):
            raise ValueError('Invalid map manifest')
        data = {}
        for name,digest in manifest['sha256'].items():
            if Path(name).name!=name or '\\' in name or '/' in name: raise ValueError('Invalid bundle path')
            content=archive.read(name)
            if hashlib.sha256(content).hexdigest()!=digest: raise ValueError('Map checksum failed: '+name)
            data[name]=content
    identity=hashlib.sha256(json.dumps(manifest['sha256'],sort_keys=True).encode()).hexdigest()
    grid=json.loads(data['grid.json'])
    if not 0<grid['width']*grid['height']<=8_000_000 or len(data['grid.bin'])!=grid['width']*grid['height']:
        raise ValueError('Invalid fixed grid')
    if not .005<=grid['resolution']<=1 or not all(math.isfinite(v) for v in grid['position']+grid['orientation']):
        raise ValueError('Invalid map geometry')
    return identity,manifest,data


def activate(path):
    path=Path(path).expanduser().resolve(); identity,_,_=read_bundle(path)
    atomic_json(ACTIVE,{'file':str(path),'map_id':identity})
    return identity


def materialize(path):
    identity,manifest,data=read_bundle(path)
    root=Path.home()/'.cache/kabot_maps'/identity
    root.mkdir(parents=True,exist_ok=True)
    for name,content in data.items():
        target=root/name
        if not target.exists() or target.read_bytes()!=content:
            temporary=target.with_suffix(target.suffix+'.incoming');temporary.write_bytes(content);os.replace(temporary,target)
    return str(root/'tank'),identity,manifest
