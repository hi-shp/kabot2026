"""Read-only start prerequisites, independent of localization acquisition."""
import json
from pathlib import Path

from .tank_mission import docking_points


def docking_start_state(home, mapping_running, session, session_fresh):
    home = Path(home)
    result = dict(ready=False, reason='', slam_enabled=bool(mapping_running),
                  route_applied=False, map_id=None, version=None)
    if not mapping_running:
        return dict(result, reason='SLAM을 켠 뒤 주행을 시작하세요')
    try:
        active = json.loads((home/'water_trials/tank_maps/active.json').read_text(encoding='utf-8'))
        map_id = active.get('map_id')
        if not map_id:
            return dict(result, reason='도킹에 사용할 SLAM 지도를 불러오세요')
        result['map_id'] = map_id
        selected = json.loads((home/'water_trials/docking_routes/applied.json').read_text(encoding='utf-8'))
        if selected.get('format') != 'isv-applied-docking-route-v1':
            raise ValueError('Invalid applied docking route')
        if selected.get('map_id') != map_id:
            return dict(result, reason='현재 SLAM 지도의 도킹 경로를 적용하세요')
        docking_points(selected['route'], map_id)
        result.update(route_applied=True, version=selected.get('version'), route=selected['route'])
    except (OSError, ValueError, KeyError, TypeError, AttributeError):
        return dict(result, reason='도킹 경로를 불러온 뒤 적용을 누르세요')
    # A pose match is intentionally not required before leaving the start area.
    if not session_fresh or not session.get('fixed_map') or session.get('session_id') != map_id:
        return dict(result, reason='불러온 SLAM 지도 기동을 기다리세요')
    return dict(result, ready=True)
