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
