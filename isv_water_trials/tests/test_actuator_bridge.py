import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import threading
import time
import unittest
from isv_water_trials.actuator_protocol import command, crc16, packet, status, raw_status
from isv_water_trials.arduino_bridge import Pairer


class ProtocolTests(unittest.TestCase):
    def test_crc_reference(self):
        self.assertEqual(crc16(b'123456789'), 0x29B1)

    def test_status(self):
        m = status(packet('S1,65535,2,1,1560,1530,1,0,0'))
        self.assertEqual((m.sequence, m.left, m.right, m.armed, m.estop_known), (65535, 1560, 1530, True, False))

    def test_raw_pulse_observation(self):
        raw = raw_status(packet('R1,1485,1490,2016,1508,1492'))
        self.assertEqual(raw['rc_mode_us'], 2016)
        self.assertEqual((raw['observed_left_us'], raw['observed_right_us']), (1508, 1492))
        with self.assertRaises(ValueError):
            raw_status(packet('R1,1485,1490,2016,2300,1492'))

    def test_corruption_and_range(self):
        valid = packet('S1,1,2,1,1560,1530,1,0,0')
        for i in range(len(valid) - 1):
            bad = bytearray(valid); bad[i] ^= 1
            with self.assertRaises(ValueError):
                status(bytes(bad))
        for payload in ('S1,65536,2,1,1560,1530,1,0,0', 'S1,1,2,1,2000,1500,1,0,0',
                        'S1,1,3,1,1500,1500,1,0,0', 'S1,1,2,1,1500,1500,1,2,0'):
            with self.assertRaises(ValueError):
                status(packet(payload))

    def test_commands(self):
        self.assertTrue(command(0, 'C', 1500, 1560).startswith(b'K1,0,C,1500,1560*'))
        for values in [(0, 'Z', 1500, 1500), (0, 'A', 1560, 1500), (65536, 'D', 1500, 1500),
                       (0, 'C', 0, 1500), (True, 'C', 1500, 1500), (0, 'C', 1500., 1500)]:
            with self.assertRaises(ValueError):
                command(*values)


class PairTests(unittest.TestCase):
    def setUp(self):
        self.sent, self.logged = [], []
        def send(*args):
            self.sent.append(args)
            return True
        self.pair = Pairer(send, lambda *args: self.logged.append(args))

    def both(self, left=1560, right=1530):
        result = []
        t = threading.Thread(target=lambda: result.append(self.pair.submit('left', left)))
        t.start()
        time.sleep(.02)
        result.append(self.pair.submit('right', right))
        t.join(1)
        return result

    def test_atomic_concurrent_services(self):
        self.assertEqual(self.both(), [True, True])
        self.assertEqual(self.sent, [('C', 1560, 1530)])
        self.assertEqual([row[0] for row in self.logged], ['requested', 'acknowledged'])

    def test_one_side_never_drives(self):
        self.assertFalse(self.pair.submit('left', 1560))
        self.assertEqual(self.sent, [])

    def test_old_half_not_reused(self):
        t = threading.Thread(target=lambda: self.pair.submit('left', 1560)); t.start()
        time.sleep(.13)
        self.assertFalse(self.pair.submit('right', 1560)); t.join(1)
        self.assertEqual(self.sent, [])

    def test_rejection_never_acknowledged(self):
        self.pair.send = lambda *args: False
        self.assertEqual(self.both(), [False, False])
        self.assertEqual(self.logged[-1][0], 'failed')

    def test_unsupported_values(self):
        for side, value in [('left', 1900), ('right', 1300), ('left', True), ('x', 1500)]:
            self.assertFalse(self.pair.submit(side, value))
        self.assertEqual(self.sent, [])


if __name__ == '__main__':
    unittest.main()
