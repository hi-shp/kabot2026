"""Desktop display data formatting; GUI process/window tested separately."""
import unittest
from isv_shape_navigation.desktop_dashboard import metrics,formatted


class DesktopMetricsTests(unittest.TestCase):
    def test_precise_coordinates_and_remaining_waypoint_distance(self):
        report={'position':{'latitude':35.186192906,'longitude':128.56547113},'wp_index':1,
                'waypoints':[(35.,128.),(35.186355548,128.565695804),(35.2,128.6)],
                'distance_to_wp':23.456,'hold_remaining':7.25}
        result=metrics(report)
        self.assertEqual(result['latitude'],'35.18619291')
        self.assertEqual(result['longitude'],'128.56547113')
        self.assertEqual(result['wp'],'WP2')
        self.assertEqual(result['distance'],'23.5 m')
        self.assertEqual(result['goal_latitude'],'35.18635555')

    def test_missing_sensor_values_are_not_shown_as_zero(self):
        result=metrics({})
        self.assertEqual(result['distance'],'—')
        self.assertEqual(result['latitude'],'—')
        self.assertEqual(formatted(float('nan')),'—')


if __name__=='__main__':unittest.main()
