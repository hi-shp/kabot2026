import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'isv_water_trials'))
from isv_water_trials.tank_pose import known_position


class PoseBoundsTests(unittest.TestCase):
    def test_free_unknown_wall_outside_and_nonfinite(self):
        info=dict(width=2,height=2,resolution=1.,position=[10.,20.,0.],orientation=[0.,0.,0.,1.])
        cells=bytes([0,100,255,0])
        for x,y,expected in [(10.5,20.5,True),(11.5,20.5,False),(10.5,21.5,False),
                             (11.5,21.5,True),(12.,21.,False),(9.99,20.,False),(float('nan'),20.,False)]:
            self.assertEqual(known_position(info,cells,x,y),expected)

    def test_rotated_grid(self):
        info=dict(width=2,height=1,resolution=1.,position=[0.,0.,0.],orientation=[0.,0.,math.sqrt(.5),math.sqrt(.5)])
        self.assertTrue(known_position(info,bytes([0,100]),-.5,.5))
        self.assertFalse(known_position(info,bytes([0,100]),-.5,1.5))
