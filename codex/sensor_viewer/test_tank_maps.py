import threading
from pathlib import Path
import tempfile
import unittest
import json
from unittest.mock import patch

from tank_feed import TankFeed


def feed():
    instance = TankFeed.__new__(TankFeed)
    instance.map_jobs = {}
    instance.map_jobs_lock = threading.RLock()
    return instance


class TankMapsTests(unittest.TestCase):
 def test_map_library_lists_saved_maps_and_active_file(self):
    with tempfile.TemporaryDirectory() as directory:
        tmp_path=Path(directory);maps=tmp_path/'water_trials'/'tank_maps';maps.mkdir(parents=True)
        saved=maps/'pool.tankmap';saved.write_bytes(b'map')
        (maps/'active.json').write_text(json.dumps({'file':str(saved.resolve())}),encoding='utf-8')
        with patch.object(Path,'home',return_value=tmp_path):status,result=feed().maps()
        self.assertEqual(status,200)
        self.assertEqual(result['maps'],[{'name':'pool','path':str(saved.resolve()),'active':True}])


 def test_map_load_rejects_paths_outside_map_library(self):
    with tempfile.TemporaryDirectory() as directory:
        tmp_path=Path(directory);outside=tmp_path/'private.tankmap';outside.write_bytes(b'map')
        with patch.object(Path,'home',return_value=tmp_path):status,result=feed().maps({'action':'load','path':str(outside)})
        self.assertEqual(status,400)
        self.assertIn('확인',result['error'])


 def test_map_save_rejects_path_like_names(self):
    with tempfile.TemporaryDirectory() as directory:
        tmp_path=Path(directory)
        with patch.object(Path,'home',return_value=tmp_path):status,result=feed().maps({'action':'save','name':'../outside'})
        self.assertEqual(status,400)
        self.assertIn('확인',result['error'])


if __name__=='__main__':unittest.main()
