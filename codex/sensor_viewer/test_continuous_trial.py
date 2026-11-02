import argparse
import ast
import math
from pathlib import Path
from types import SimpleNamespace as N
import unittest
import numpy as np
from test_tank_experiment import message,room_scan
from isv_water_trials.tank_core import Grid,pwm
from isv_water_trials.tank_safety import DepartureRest,select_command


SOURCE=Path(__file__).resolve().parents[2]/'isv_water_trials/isv_water_trials/indoor_trial.py'
MAIN=next(n for n in ast.parse(SOURCE.read_text(encoding='utf-8')).body if isinstance(n,ast.FunctionDef) and n.name=='main')


class ContinuousTrialTests(unittest.TestCase):
    def arguments(self,argv):
        statements=[]
        for statement in MAIN.body:
            statements.append(statement)
            if isinstance(statement,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='args' for t in statement.targets):break
        ns=dict(argparse=argparse,argv=argv)
        exec(compile(ast.Module(body=statements,type_ignores=[]),str(SOURCE),'exec'),ns)
        return ns['args']

    def step(self,position=(6.,5.),goal=0,dwell=0.):
        branch=next(n for n in ast.walk(MAIN) if isinstance(n,ast.Try) and
                    any(isinstance(s,ast.Call) and isinstance(s.func,ast.Attribute) and s.func.attr=='trial_path' for s in ast.walk(n))
                    and any(isinstance(h.type,ast.Name) and h.type.id=='ValueError' for h in n.handlers))
        ns=dict(math=math,position=position,points=[(6.,5.),(11.,5.)],goal=goal,legs=0,
            path=[],reason='',state='WAITING',phase='hold',output=(1500,1500),now=10.,last_plan=0.,
            departure=DepartureRest(minimum=dwell,enabled=dwell>0),grid=Grid(message(np.zeros((120,120))),.75),
            args=N(arrival=.5,percent=20.,no_excitation=True,execute=True),
            latest={'scan':(10.,room_scan(position))},heading=0.,velocity=(.1,0.,0.),forward=0.,experiment_time=19.,
            select_command=select_command,pwm=pwm)
        exec(compile(ast.Module(body=[branch],type_ignores=[]),str(SOURCE),'exec'),ns)
        return ns

    def test_default_continuous_and_explicit_experiments(self):
        args=self.arguments([])
        self.assertEqual(args.dwell,0.)
        self.assertTrue(args.no_excitation)
        self.assertFalse(self.arguments(['--excitation']).no_excitation)
        self.assertEqual(self.arguments(['--dwell','3']).dwell,3.)

    def test_arrival_switches_goal_and_commands_in_same_tick(self):
        ns=self.step()
        self.assertEqual(ns['goal'],1);self.assertEqual(ns['legs'],1)
        self.assertEqual(ns['state'],'RUNNING');self.assertNotEqual(ns['output'],(1500,1500))
        self.assertFalse(ns['departure'].active)

    def test_default_has_no_scheduled_coast(self):
        ns=self.step(position=(7.,5.),goal=1)
        self.assertEqual(ns['state'],'RUNNING')
        self.assertEqual(ns['phase'],'transit')
        self.assertEqual(ns['output'],(1580,1580))

    def test_explicit_dwell_still_stops_at_arrival(self):
        ns=self.step(dwell=3.)
        self.assertEqual(ns['state'],'DWELL');self.assertEqual(ns['output'],(1500,1500))


if __name__=='__main__':unittest.main()
