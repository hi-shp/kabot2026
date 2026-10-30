"""Process-only shutdown regression; never opens a sensor or serial port."""
import os
from pathlib import Path
import runpy
import signal
import subprocess
import sys
import tempfile
import unittest


@unittest.skipUnless(sys.platform == 'linux', 'Linux process ownership required')
class LifecycleTests(unittest.TestCase):
    def setUp(self):
        source = os.environ.get('LIFECYCLE_SOURCE') or str(
            Path(__file__).resolve().parents[1] / 'deploy/jetson/mechaship_bringup/launch/process_lifecycle.py')
        self.module = runpy.run_path(source)

    def test_reused_pid_identity_is_never_signalled(self):
        child = subprocess.Popen([sys.executable, '-c', 'import time;time.sleep(60)'])
        try:
            start = self.module['processes']()[child.pid][1]
            self.assertFalse(self.module['terminate']({child.pid: str(int(start) + 1)}))
            self.assertIsNone(child.poll())
            self.assertFalse(self.module['terminate']({child.pid: start}))
            child.wait(timeout=3)
        finally:
            if child.poll() is None:
                child.kill()
            child.wait()

    def test_shutdown_reaps_stubborn_driver_and_grandchild(self):
        ignore = 'import signal,time;signal.signal(2,signal.SIG_IGN);signal.signal(15,signal.SIG_IGN);time.sleep(60)'
        code = ('import signal,subprocess,sys,time;'
                'signal.signal(2,signal.SIG_IGN);signal.signal(15,signal.SIG_IGN);'
                'p=subprocess.Popen([sys.executable,"-c",%r]);print(p.pid,flush=True);time.sleep(60)') % ignore
        child = subprocess.Popen([sys.executable, '-u', '-c', code], stdout=subprocess.PIPE, text=True)
        grandchild = int(child.stdout.readline())
        try:
            with tempfile.TemporaryDirectory() as cache:
                session = self.module['BringupSession'](cache)
                table = self.module['processes']()
                session.owned = {child.pid: table[child.pid][1]}
                session.shutdown()
                child.wait(timeout=3)
                table = self.module['processes']()
                self.assertFalse(session.owned)
                self.assertFalse(grandchild in table and table[grandchild][2] != 'Z')
        finally:
            for pid in (child.pid, grandchild):
                try: os.kill(pid, signal.SIGKILL)
                except ProcessLookupError: pass
            child.wait()
            child.stdout.close()


if __name__ == '__main__':
    unittest.main()
