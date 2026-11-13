"""Mapping owns the recorder; fake children only, no ROS or device access."""
import contextlib
import io
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'isv_water_trials'))
from isv_water_trials import tank_session as session


class SessionRecordingTests(unittest.TestCase):
    def test_repeated_sensor_start_requests_focus_without_restarting(self):
        import json
        with tempfile.TemporaryDirectory() as folder,patch.object(session,'ROOT',Path(folder)),\
                patch.object(session,'running',return_value={'pid':999}),patch.object(session.subprocess,'Popen') as child,\
                contextlib.redirect_stdout(io.StringIO()):
            session.main(['start','sensors'])
            first=json.loads((Path(folder)/'dashboard_wake.json').read_text())['id']
            session.main(['start','sensors'])
            second=json.loads((Path(folder)/'dashboard_wake.json').read_text())['id']
            self.assertNotEqual(first,second);child.assert_not_called()

    def run_mapping_worker(self,fail=False):
        with tempfile.TemporaryDirectory() as folder:
            child=Mock();child.poll.return_value=0;child.returncode=0
            fcntl=SimpleNamespace(LOCK_EX=1,LOCK_NB=2,flock=lambda *_:None)
            signals=SimpleNamespace(SIGHUP=1,SIG_IGN=0,SIGINT=2,SIGTERM=15,signal=Mock())
            with patch.dict(sys.modules,fcntl=fcntl),patch.object(session,'ROOT',Path(folder)),\
                    patch.object(session,'identity',return_value='fake'),patch.object(session,'running',return_value=None),\
                    patch.object(session,'signal',signals),patch.object(session,'main') as calls,\
                    patch.object(session.subprocess,'Popen',side_effect=OSError('fake launch failure') if fail else None,return_value=child):
                if fail:
                    with self.assertRaises(OSError):session.worker('mapping',[])
                else:session.worker('mapping',[])
                self.assertEqual(calls.call_args_list[0].args,(['start','record'],))
                self.assertEqual(calls.call_args_list[-1].args,(['stop','record'],))
                self.assertTrue(all(c.kwargs=={'_internal':True} for c in calls.call_args_list))

    def test_mapping_starts_and_stops_recorder(self):self.run_mapping_worker()
    def test_launch_failure_also_closes_recorder(self):self.run_mapping_worker(fail=True)

    def test_stop_mapping_closes_recording_left_by_old_worker(self):
        original=session.main
        with tempfile.TemporaryDirectory() as folder,patch.object(session,'ROOT',Path(folder)),\
                patch.object(session,'running',side_effect=lambda n: {'pid':999} if n=='record' else None),patch.object(session,'main') as calls,\
                contextlib.redirect_stdout(io.StringIO()):
            original(['stop','mapping'])
            calls.assert_called_once_with(['stop','record'],_internal=True)

    def test_stop_all_closes_recorder_once_without_duplicate_output(self):
        active={'record':True};output=io.StringIO()
        def running(name):return {'pid':999} if active.get(name) else None
        def stop(*_):active['record']=False
        with tempfile.TemporaryDirectory() as folder,patch.object(session,'ROOT',Path(folder)),\
                patch.object(session,'running',side_effect=running),patch.object(session.os,'kill',side_effect=stop) as kill,\
                contextlib.redirect_stdout(output):
            session.main(['stop','all'])
        kill.assert_called_once_with(999,session.signal.SIGINT)
        self.assertEqual(output.getvalue().splitlines(),['trial: stopped','mapping: stopped','sensors: stopped'])

    def test_separate_record_command_removed_from_public_cli(self):
        with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit) as error:
            session.main(['stop','record'])
        self.assertEqual(error.exception.code,2)


if __name__=='__main__':unittest.main()
