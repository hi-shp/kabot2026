"""Optional, explicit trained-class -> saved A/B profile selection."""
import json
import math
from pathlib import Path
import time
import uuid
from .vision_settings import atomic_json


def select_profile(map_id,version,target):
    if version not in ('left','middle','right'):return False
    profiles=[]
    for path in (Path.home()/'water_trials/docking_routes').glob('*.dockroute.json'):
        try:
            data=json.loads(path.read_text(encoding='utf-8'))
            if data.get('format')=='isv-docking-route-v1' and data['map_id']==map_id and data['version']==version:
                profiles.append(data)
        except (OSError,ValueError,KeyError):continue
    if not profiles:return False
    profile=max(profiles,key=lambda p:p.get('saved_unix',0))
    route=dict(profile['route'],revision=uuid.uuid4().hex,updated_unix=time.time(),selection_source='YOLO',session_id=map_id,frame_id='tank_map')
    try:
        if not all(math.isfinite(float(route[k][a])) for k in ('a','b') for a in ('x','y')):return False
    except (KeyError,TypeError,ValueError):return False
    atomic_json(target,route)
    return True
