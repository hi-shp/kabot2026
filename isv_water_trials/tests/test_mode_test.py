from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from isv_water_trials.mode_test import ModeGate


def payload(mode=2, **kw):
    return dict(receiving=True, error=None, age_s=0., rc_valid=True, estop_active=False,
                mode=mode, armed=False, rc_left_us=1500, rc_right_us=1500, **kw)


class ModeGateTests(unittest.TestCase):
    def test_rc_auto_rc_auto_needs_new_neutral_hold(self):
        gate = ModeGate()
        gate.observe(payload(1), 0.)
        self.assertEqual(gate.action(.1), 'rc')
        for now in (.2, .4, .6):
            gate.observe(payload(), now)
            self.assertEqual(gate.action(now), 'neutral')
        gate.observe(payload(), .71)
        self.assertEqual(gate.action(.71), 'arm')
        gate.armed(.71)
        m = payload(); m['armed'] = True
        gate.observe(m, .8)
        self.assertEqual(gate.action(.8), 'drive')
        gate.observe(payload(1), 1.)
        self.assertIsNone(gate.active_since)
        self.assertEqual(gate.action(1.), 'rc')
        gate.observe(payload(), 1.1)
        self.assertEqual(gate.action(1.1), 'neutral')
        for now in (1.3, 1.5, 1.7):
            gate.observe(payload(), now)
        self.assertEqual(gate.action(1.7), 'arm')
        self.assertEqual(gate.epoch, 4)

    def test_neutral_needs_both_raw_channels_and_continuity(self):
        for bad in (None, 1460., 1459, 1541, True):
            gate = ModeGate()
            gate.observe(payload(), 0.)
            m = payload(); m['rc_right_us'] = bad
            gate.observe(m, .1)
            self.assertIsNone(gate.neutral_since)
            self.assertEqual(gate.action(.1), 'neutral')
        gate.observe(payload(), .2)
        self.assertEqual(gate.action(.2), 'neutral')

    def test_stale_receipt_includes_bridge_reported_age(self):
        gate = ModeGate()
        m = payload(); m['age_s'] = .2
        gate.observe(m, 1.)
        with self.assertRaises(RuntimeError):
            gate.action(1.11)

    def test_all_faults_abort(self):
        for key, value in [('receiving', False), ('error', 'USB'), ('rc_valid', False),
                           ('estop_active', True), ('mode', 0), ('mode', True),
                           ('armed', None), ('age_s', .3), ('age_s', float('nan'))]:
            m = payload(); m[key] = value
            with self.subTest(key=key, value=value), self.assertRaises(RuntimeError):
                ModeGate().observe(m, 1.)

    def test_unexpected_disarm_does_not_rearm(self):
        gate = ModeGate()
        gate.observe(payload(), 0.)
        gate.armed(.1)
        with self.assertRaises(RuntimeError):
            gate.observe(payload(), .5)

    def test_already_armed_controller_is_rejected(self):
        m = payload(); m['armed'] = True
        with self.assertRaises(RuntimeError):
            ModeGate().observe(m, 1.)


if __name__ == '__main__':
    unittest.main()
