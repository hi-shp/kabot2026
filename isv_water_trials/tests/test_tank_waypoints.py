import json
from pathlib import Path
import tempfile
import unittest

try:
    from isv_water_trials.tank_waypoints import mission_points, store_waypoint
except ModuleNotFoundError:
    from tank_waypoints import mission_points, store_waypoint


class MissionWaypointsTest(unittest.TestCase):
    def route(self):
        return {'frame_id': 'tank_map', 'session_id': 'map-1',
                'a': {'x': 0., 'y': 0.}, 'b': {'x': 2., 'y': 0.}}

    def test_legacy_ab_route_is_preserved(self):
        self.assertEqual(mission_points(self.route()), [('A', (0., 0.)), ('B', (2., 0.))])

    def test_waypoints_override_ab_in_order_and_match_map(self):
        route = self.route()
        route['mission_waypoints'] = {'map_id': 'map-1', 'points': [
            {'x': 4., 'y': 5.}, {'x': 8., 'y': 5.}]}
        self.assertEqual(mission_points(route), [('WP1', (4., 5.)), ('WP2', (8., 5.))])
        route['mission_waypoints']['map_id'] = 'another-map'
        self.assertEqual(len(mission_points(route)), 2)

    def test_single_registered_waypoint_is_a_valid_mission(self):
        route = self.route()
        route['mission_waypoints'] = {'map_id': 'map-1', 'points': [{'x': 4., 'y': 5.}, None]}
        self.assertEqual(mission_points(route), [('WP1', (4., 5.))])

    def test_close_waypoints_are_accepted(self):
        route = self.route()
        route['mission_waypoints'] = {'map_id': 'map-1', 'points': [
            {'x': 4., 'y': 5.}, {'x': 4.1, 'y': 5.}]}
        self.assertEqual(len(mission_points(route)), 2)

    def test_capture_preserves_route_and_replaces_only_selected_waypoint(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / 'route.json'
            original = self.route()
            path.write_text(json.dumps(original), encoding='utf-8')
            store_waypoint(path, 'map-2', 1, (4.5, 6.5))
            updated = json.loads(path.read_text(encoding='utf-8'))
            self.assertEqual(updated['a'], original['a'])
            self.assertEqual(updated['b'], original['b'])
            self.assertEqual(updated['session_id'], 'map-2')
            self.assertEqual(updated['mission_waypoints']['points'], [{'x': 4.5, 'y': 6.5}, None])


if __name__ == '__main__':
    unittest.main()
