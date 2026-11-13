import math
import ast
from types import SimpleNamespace as N
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'isv_water_trials'))
from isv_water_trials.tank_pose import known_position


class PoseBoundsTests(unittest.TestCase):
    def test_delay_recovery_requires_three_fresh_consistent_matches(self):
        path=Path(__file__).resolve().parents[2]/'isv_water_trials/isv_water_trials/tank_pose.py'
        tree=ast.parse(path.read_text(encoding='utf-8'))
        cls=next(n for n in ast.walk(tree) if isinstance(n,ast.ClassDef) and n.name=='TankPose')
        methods=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in ('matched','wait_for_recovery')]
        ns=dict(__name__='isv_water_trials.tank_pose',__package__='isv_water_trials',math=math,known_position=known_position)
        exec(compile(ast.Module(body=methods,type_ignores=[]),str(path),'exec'),ns)
        now=[10_000_000_000]
        node=N(get_clock=lambda:N(now=lambda:N(nanoseconds=now[0])),last_slam=9_000_000_000,ready=False,manual_locked=False,
               map_info=dict(width=10,height=10,resolution=1.,position=[0.,0.,0.],orientation=[0.,0.,0.,1.]),map_cells=bytes([0]*100))
        def msg(stamp,x=3.,y=4.):
            return N(header=N(frame_id='tank_map',stamp=N(sec=stamp//10**9,nanosec=stamp%10**9)),pose=N(pose=N(position=N(x=x,y=y),orientation=N(x=0.,y=0.,z=0.,w=1.))))
        ns['wait_for_recovery'](node,msg(now[0]))
        ns['matched'](node,msg(8_000_000_000));self.assertEqual(node.matches,0)
        for i in range(3):
            now[0]=10_300_000_000+i*100_000_000
            ns['matched'](node,msg(now[0]));self.assertEqual(node.ready,i==2)
        self.assertIsNone(node.requested)
        node.ready=False;ns['wait_for_recovery'](node,msg(now[0]))
        for i in range(4):
            now[0]+=300_000_000;ns['matched'](node,msg(now[0],x=8.))
        self.assertFalse(node.ready);self.assertEqual(node.matches,0)

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
