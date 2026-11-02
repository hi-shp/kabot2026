import math
import unittest
import numpy as np
from test_tank import message
from test_tank_experiment import room_scan
from isv_water_trials.tank_core import Grid
from isv_water_trials.tank_safety import select_command


class EndpointRecoveryTests(unittest.TestCase):
    def setUp(self):self.grid=Grid(message(np.zeros((120,120))),.75)

    def test_near_wall_goal_uses_arrival_circle_without_moving_saved_goal(self):
        goal=(13.5,5.)
        grid,path,target,recovery=self.grid.trial_path((6.,5.),goal)
        self.assertEqual(goal,(13.5,5.))
        self.assertLessEqual(math.dist(target,goal),.45)
        self.assertTrue(grid.safe(grid.index(target)))
        self.assertEqual(path[-1],target)
        self.assertFalse(recovery)

    def test_start_inside_extra_margin_can_escape_at_ten_percent(self):
        start=(13.35,5.)
        grid,path,target,recovery=self.grid.trial_path(start,(6.,5.))
        self.assertTrue(recovery)
        self.assertFalse(self.grid.safe(self.grid.index(start)))
        self.assertTrue(all(grid.line_clear(a,b) for a,b in zip(path[:-1],path[1:])))
        output,info=select_command(grid,room_scan(start),start,math.pi,(0.,0.,0.),path[1],target,0.,0.,radius=grid.clearance,margin_recovery=True)
        self.assertNotEqual(output,(0.,0.))
        self.assertLessEqual(max(abs(v) for v in output),10.)
        self.assertEqual(info['phase'],'clearance_recovery')
        self.assertAlmostEqual(self.grid.clearance,.75)

    def test_unknown_start_and_hull_overlap_remain_blocked(self):
        for start in ((1.,5.),(13.8,5.)):
            with self.assertRaises(ValueError):self.grid.trial_path(start,(6.,5.))
        cells=np.zeros((120,120));cells[60,60]=-1
        with self.assertRaises(ValueError):Grid(message(cells),.75).trial_path((8.05,5.05),(6.,5.))

    def test_goal_outside_arrival_reach_is_not_silently_moved(self):
        with self.assertRaises(ValueError):self.grid.trial_path((6.,5.),(14.5,5.))

    def test_recovery_does_not_cross_wall(self):
        cells=np.zeros((120,120));cells[:,60]=100
        with self.assertRaises(ValueError):Grid(message(cells),.75).trial_path((6.,5.),(13.5,5.))


if __name__=='__main__':unittest.main()
