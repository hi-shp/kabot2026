"""Read only complete checkpoints belonging to the current new-map boundary."""
import json
import math
from pathlib import Path


def latest_snapshot(home=None,reset_after_ns=None):
    home=Path(home) if home is not None else Path.home()
    root=home/'water_trials/tank_maps/autosave'
    try:
        if reset_after_ns is None:
            reset=json.loads((home/'water_trials/runtime/tank_map_reset.json').read_text())
            reset_after_ns=int(reset['stamp_ns'])
    except (OSError,ValueError,KeyError):reset_after_ns=0
    try:
        latest=json.loads((root/'latest.json').read_text());prefix=Path(latest['map_file']).resolve()
        complete=json.loads(prefix.parent.joinpath('complete.json').read_text())
        stamp=float(complete['saved_unix'])
        if (not prefix.is_relative_to(root.resolve()) or not math.isfinite(stamp) or
            stamp*1e9<(reset_after_ns or 0) or
            any(not Path(str(prefix)+suffix).is_file() for suffix in ('.yaml','.posegraph','.data'))):return None
        return {'prefix':str(prefix),'saved_unix':stamp}
    except (OSError,ValueError,KeyError,TypeError):return None


def snapshot_grid(prefix):
    import yaml
    import numpy as np
    prefix=Path(prefix)
    config=yaml.safe_load(Path(str(prefix)+'.yaml').read_text())
    image=Path(config['image']);image=image if image.is_absolute() else prefix.parent/image
    raw=image.read_bytes();index=0;words=[]
    while len(words)<4:
        while raw[index:index+1].isspace():index+=1
        if raw[index:index+1]==b'#':index=raw.index(b'\n',index)+1;continue
        end=index
        while end<len(raw) and not raw[end:end+1].isspace():end+=1
        words.append(raw[index:end]);index=end
    magic,width,height,maximum=words;width=int(width);height=int(height)
    if magic!=b'P5' or maximum!=b'255' or not 0<width*height<=8_000_000:raise ValueError('Invalid checkpoint grid')
    index+=2 if raw[index:index+2]==b'\r\n' else 1
    if len(raw)-index!=width*height:raise ValueError('Truncated checkpoint image')
    pixels=np.frombuffer(raw[index:],np.uint8).reshape(height,width)
    if config.get('negate') or config.get('mode','trinary')!='trinary':raise ValueError('Unsupported checkpoint image mode')
    occupancy=(255.-pixels)/255.
    cells=np.where(occupancy>config['occupied_thresh'],100,np.where(occupancy<config['free_thresh'],0,-1)).astype(np.int8)
    cells[pixels==205]=-1
    x,y,angle=config['origin']
    return {'width':width,'height':height,'resolution':float(config['resolution']),
            'x':x,'y':y,'yaw':angle},np.flipud(cells).tobytes()
