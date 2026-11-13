"""선택한 객체의 등록 GPS를 공통 GPS 호핑에 전달한다."""
from pathlib import Path
import math
import yaml

from kaboat_gps_hopping.gps_hopping import GPSHoppingMission


def default_goals_path():
    source = Path(__file__).resolve().parents[2]/'설정/object_gps.yaml'
    if source.is_file(): return source
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('isv_shape_navigation'))/'config/object_gps.yaml'


def load_object_goals(path=None):
    path = Path(path).expanduser() if path else default_goals_path()
    try:
        data = yaml.safe_load(path.read_text(encoding='utf-8'))
    except yaml.YAMLError as error:
        raise ValueError('Invalid object GPS YAML: '+str(error)) from error
    if not isinstance(data, dict) or not isinstance(data.get('objects'), dict):
        raise ValueError('object_gps.yaml requires an objects mapping')
    goals = {}
    for name, item in data['objects'].items():
        if not isinstance(name, str) or not name.strip() or not isinstance(item, dict):
            raise ValueError('Invalid object entry')
        key = ' '.join(name.lower().replace('_', ' ').split())
        if key in goals: raise ValueError('Duplicate object: '+key)
        values = {}
        for field in ('latitude', 'longitude', 'radius_m'):
            value = item.get(field)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError('Invalid '+key+'.'+field)
            values[field] = float(value)
        if not -90 <= values['latitude'] <= 90 or not -180 <= values['longitude'] <= 180 or values['radius_m'] <= 0:
            raise ValueError('Invalid coordinates/radius for '+key)
        goals[key] = values
    return goals


class ObjectGPSMission(GPSHoppingMission):
    def __init__(self, config, guidance, gps_config, goals):
        self.goals, self.selected_object, self.selection_error = goals, '', ''
        super().__init__(config, guidance, gps_config)
        self.select_object(config.target_label)

    def select_object(self, name):
        name = ' '.join(str(name).lower().replace('_', ' ').split())
        goal = self.goals.get(name)
        if goal is None:
            self.clear_goal()
            self.selected_object = ''
            self.selection_error = 'GPS not registered for object: '+name
            return False
        self.set_goal(goal['latitude'], goal['longitude'], goal['radius_m'])
        self.selected_object, self.selection_error = name, ''
        return True
