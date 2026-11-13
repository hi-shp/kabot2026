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


def crop_files(files, bounds):
    """Crop occupancy only; retain world coordinates and the SLAM restore graph."""
    import numpy as np
    import yaml
    if len(bounds)!=4 or not all(math.isfinite(float(v)) for v in bounds):
        raise ValueError('직사각형 구역을 다시 지정하세요.')
    x0,y0,x1,y1=map(float,bounds)
    x0,x1=sorted((x0,x1));y0,y1=sorted((y0,y1))
    if x1<=x0 or y1<=y0:raise ValueError('직사각형 구역을 드래그해 지정하세요.')
    info=json.loads(files['grid.json']);r=float(info['resolution'])
    qx,qy,qz,qw=info['orientation'];angle=math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
    co,si=math.cos(angle),math.sin(angle);ox,oy=info['position'][:2]
    corners=np.array([(x0,y0),(x1,y0),(x0,y1),(x1,y1)])-np.array([ox,oy])
    local=np.column_stack((co*corners[:,0]+si*corners[:,1],-si*corners[:,0]+co*corners[:,1]))/r
    left,bottom=map(int,np.floor(local.min(axis=0)));right,top=map(int,np.ceil(local.max(axis=0)))
    left=max(0,left);bottom=max(0,bottom);right=min(info['width'],right);top=min(info['height'],top)
    if left>=right or bottom>=top:raise ValueError('선택 구역이 지도와 겹치지 않습니다.')
    grid=np.frombuffer(files['grid.bin'],dtype=np.int8).reshape(info['height'],info['width'])
    cropped=grid[bottom:top,left:right].copy()
    rows,cols=np.indices(cropped.shape);gx=(cols+left+.5)*r;gy=(rows+bottom+.5)*r
    wx=ox+co*gx-si*gy;wy=oy+si*gx+co*gy
    cropped[(wx<x0)|(wx>x1)|(wy<y0)|(wy>y1)]=-1
    if not np.any(cropped>=0):raise ValueError('선택 구역에 관측된 지도가 없습니다.')
    info=dict(info,width=int(right-left),height=int(top-bottom),
              position=[ox+co*left*r-si*bottom*r,oy+si*left*r+co*bottom*r,info['position'][2]],
              crop_bounds_m=[x0,y0,x1,y1])
    result=dict(files);result['grid.json']=json.dumps(info).encode();result['grid.bin']=cropped.tobytes()
    # The selected crop is recognition evidence, not an artificial sailing fence.
    result.setdefault('navigation_grid.json',files['grid.json'])
    result.setdefault('navigation_grid.bin',files['grid.bin'])
    pixels=np.where(cropped<0,205,np.where(cropped>=65,0,254)).astype(np.uint8)
    result['tank.pgm']=f"P5\n{info['width']} {info['height']}\n255\n".encode()+np.flipud(pixels).tobytes()
    result.pop('tank.png',None)
    result['tank.yaml']=yaml.safe_dump(dict(image='tank.pgm',mode='trinary',resolution=r,
        origin=[*info['position'][:2],angle],negate=0,occupied_thresh=.65,free_thresh=.196)).encode()
    return result


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


def import_snapshot(prefix,target,crop=None):
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
    return bundle(target,crop_files(files,crop) if crop is not None else files)


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
