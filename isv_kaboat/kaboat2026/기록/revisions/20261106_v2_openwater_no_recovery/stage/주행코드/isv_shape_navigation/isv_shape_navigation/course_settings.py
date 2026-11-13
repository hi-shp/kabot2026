"""Shared stopped-only settings path for the native and SSH Windows dashboards.

Applies runtime mission settings only. Never arms, starts, or commands actuators.
"""
from dataclasses import asdict
import math

from .follow import FollowConfig
from .route_mission import RouteConfig

SETTINGS_ACTIONS = ('set_route', 'capture_wp', 'set_target')
GUI_ACTIONS = SETTINGS_ACTIONS + ('start', 'stop', 'reset')


def apply_course_settings(node, command):
    if not isinstance(command, dict) or command.get('action') not in SETTINGS_ACTIONS:
        raise ValueError('좌표·도킹 목표 설정만 허용됩니다.')
    output = node.output
    if output and (output.active or output.pending or output.stopping or output.armed):
        raise ValueError('먼저 정지하고 중립 ACK를 확인하세요.')
    action = command['action']
    route = node.controller.route_config
    if action == 'set_route':
        values = command.get('waypoints')
        if (not isinstance(values, list) or len(values) != 3
                or any(not isinstance(row, list) or len(row) != 2 for row in values)
                or any(type(value) not in (float, int) or not math.isfinite(value)
                       for row in values for value in row)):
            raise ValueError('WP는 유효한 위도·경도 3쌍이어야 합니다.')
        changed = asdict(route)
        changed.update(wp_latitudes=tuple(row[0] for row in values),
                       wp_longitudes=tuple(row[1] for row in values))
        updated = RouteConfig(**changed)  # Fully validate before mutating anything.
        node.controller.route_config = updated
        node.controller.reset()
    elif action == 'capture_wp':
        index = command.get('index')
        if type(index) is not int or index not in (0, 1, 2):
            raise ValueError('WP 번호가 올바르지 않습니다.')
        fix = node.controller.fix
        if (fix is None or not -.1 <= node.now()-fix.stamp <= node.p['gps_timeout']
                or not math.isfinite(fix.sigma) or fix.sigma > node.p['max_gps_sigma']):
            raise ValueError('최신 정상 GPS가 필요합니다.')
        lat, lon = list(route.wp_latitudes), list(route.wp_longitudes)
        lat[index], lon[index] = fix.latitude, fix.longitude
        changed = asdict(route)
        changed.update(wp_latitudes=lat, wp_longitudes=lon)
        updated = RouteConfig(**changed)
        node.controller.route_config = updated
        node.controller.reset()
        node.controller.fix = fix
    else:
        label = command.get('target_label')
        if not isinstance(label, str):
            raise ValueError('학습된 도킹 목표를 선택하세요.')
        changed = asdict(node.cfg)
        changed['target_label'] = label
        cfg = FollowConfig(**changed)
        node.cfg = node.controller.config = cfg
        node.controller.reset()
        node.yolo, node.yolo_stamp = [], -math.inf
    return '설정 적용됨. 임무는 WP1부터 초기화됩니다. 재실행 시 YAML 설정을 사용합니다.'


def handle_settings_rpc(node, request, response):
    """SetParametersAtomically-shaped private API; does not alter ROS parameters."""
    import json
    command = {}
    try:
        params = request.parameters
        if (len(params) != 1 or params[0].name != 'ui_request'
                or params[0].value.type != 4 or len(params[0].value.string_value) > 8192):
            raise ValueError('Invalid settings request')
        command = json.loads(params[0].value.string_value)
        if not isinstance(command, dict) or command.get('action') not in GUI_ACTIONS:
            raise ValueError('지원하지 않는 GUI 명령입니다.')
        if command['action'] in ('start', 'stop', 'reset'):
            from types import SimpleNamespace
            result=getattr(node,command['action'])(None,SimpleNamespace(success=False,message=''))
            response.result.reason=result.message
            response.result.successful=result.success
        else:
            response.result.reason = apply_course_settings(node, command)
            response.result.successful = True
    except (ValueError, KeyError, TypeError, OverflowError) as error:
        response.result.successful = False
        response.result.reason = str(error)
    if node.dashboard:
        with node.dashboard.lock:
            node.dashboard.last_command = dict(action=command.get('action') if isinstance(command, dict) else None,
                success=response.result.successful, message=response.result.reason)
    return response
