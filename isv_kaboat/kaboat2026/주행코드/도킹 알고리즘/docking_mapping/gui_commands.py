"""Mapping/hopping-only GUI commands; no actuator or autonomous mission action."""
from .hopping import HoppingPoint,write_hopping,roi_from_clicks,number

GUI_ACTIONS=('map_output_prepare','map_drive_start','map_dry_run_start','map_dry_run_stop','set_gps_target','save_gps_map','load_gps_map','start','pause','resume','stop','save','load','reset','select_roi',
             'set_hopping','delete_hopping','save_hopping','auto_start','auto_stop','auto_emergency')
RC_ACTIONS=('start','resume','select_roi','set_hopping','delete_hopping','save_hopping')


def execute(node,payload):
    action=payload['action']
    if action not in GUI_ACTIONS:raise ValueError('Unsupported mapping GUI action')
    if action=='map_output_prepare':
        node.map_navigation.prepare_output();return 'Map output prepared; not armed; no departure'
    if action=='map_drive_start':
        node.map_navigation.start_drive(payload.get('dry_generation'));return 'GPS approach / LiDAR docking drive started'
    if action=='map_dry_run_start':
        node.map_navigation.start(payload.get('dry_generation'));return 'GPS approach / LiDAR docking preview started; hardware output disabled'
    if action=='map_dry_run_stop':
        node.map_navigation.stop();return 'Map Dry-run stopped'
    if action=='auto_start':
        if node.map_navigation.active:raise ValueError('Stop map Dry-run before AUTO')
        node.autonomy.start(payload.get('auto_generation'))
        return 'AUTO docking started (see dry-run/output status)'
    if action in ('auto_stop','auto_emergency'):
        node.map_navigation.stop('operator '+action)
        node.autonomy.stop('operator stop',action=='auto_emergency');return 'AUTO stopped'
    if action in RC_ACTIONS and not node.rc_ready():raise ValueError('RC_NOT_READY')
    if action in ('save_gps_map','load_gps_map','start','pause','resume','stop','save','load','reset'):
        node.execute_command(action)
        return node.command_result
    with node.map_lock:
        if getattr(node,'map_navigation',None) is not None and node.map_navigation.active:raise ValueError('Stop map Dry-run before editing')
        if hasattr(node,'autonomy') and node.autonomy.active:raise ValueError('Stop AUTO before editing')
        if node.session.active:raise ValueError('Stop mapping before editing map/hopping data')
        if action=='set_gps_target':
            if not node.gps_map_loaded or payload.get('map_name')!=node.gps_map_name:
                raise ValueError('Load the current GPS map before selecting its target')
            target=node.geographic_maps.write_target(node.gps_map_name,payload.get('latitude'),payload.get('longitude'))
            node.gps_target=target
            return '목표 지점을 이 맵에 저장했습니다.'
        if action=='select_roi':
            if node.reference is not None:raise ValueError('Reference is frozen; reset to create another map')
            if node.mapper.enu is None:raise ValueError('No valid map origin')
            low,high=roi_from_clicks(node.mapper.enu,payload.get('corners'),payload.get('z_min'),payload.get('z_max'))
            node.config['reference'].update(roi_min_enu_m=low,roi_max_enu_m=high,
                yaw_enu_deg=number(payload.get('yaw_enu_deg',0.),'dock_ref yaw'))
            node.mapper.config['reference']=dict(node.config['reference'])
            return '구조물 영역을 적용했습니다. 맵 저장으로 기준 맵을 고정하세요.'
        reference=node.reference
        if reference is None:raise ValueError('Save/load a dock_ref reference before editing hopping point')
        if (payload.get('reference_id')!=reference.reference_id
                or type(payload.get('reference_version')) is not int
                or payload['reference_version']!=reference.version):
            raise ValueError('GUI reference map/version is stale; reload current map')
        if action=='set_hopping':
            settings=dict(name=payload.get('name','hop1'),radius_m=payload.get('radius_m'),
                          hold_seconds=payload.get('hold_seconds'),heading_dock_deg=payload.get('heading_dock_deg'))
            node.hopping=HoppingPoint.from_click(reference,payload.get('latitude'),payload.get('longitude'),
                payload.get('height_enu_m'),settings,node.config['hopping']['max_reference_distance_m'])
            node.hopping_dirty=True
            return '호핑 지점을 적용했습니다. 호핑 지점 저장을 누르세요.'
        if action=='delete_hopping':
            node.hopping=None;node.hopping_dirty=True
            return '호핑 지점을 삭제했습니다. 저장하면 삭제 상태가 보존됩니다.'
        if node.reference_folder is None:raise ValueError('Reference map has no storage folder')
        filename=write_hopping(node.reference_folder,reference,node.hopping)
        node.hopping_dirty=False
        return '호핑 지점을 저장했습니다: '+str(filename)
