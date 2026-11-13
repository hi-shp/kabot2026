import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'isv_water_trials'))
from isv_water_trials.tank_map_file import bundle,read_bundle,materialize,import_snapshot


class FixedMapTests(unittest.TestCase):
    def files(self):
        return {'tank.posegraph':b'graph','tank.data':b'data','grid.bin':bytes([0,100,255,0]),
                'grid.json':json.dumps(dict(width=2,height=2,resolution=.1,position=[1,2,0],orientation=[0,0,0,1])).encode()}

    def test_existing_map_is_never_overwritten(self):
        with tempfile.TemporaryDirectory() as folder:
            target=bundle(Path(folder)/'pool',self.files());before=target.read_bytes()
            with self.assertRaises(FileExistsError):bundle(target,self.files())
            self.assertEqual(target.read_bytes(),before)

    def test_reloading_cache_preserves_identity_and_source_bytes(self):
        with tempfile.TemporaryDirectory() as folder,patch('pathlib.Path.home',return_value=Path(folder)):
            target=bundle(Path(folder)/'pool',self.files(),[1.,2.,0.]);before=target.read_bytes()
            prefix,identity,_=materialize(target)
            Path(prefix+'.data').write_bytes(b'bad cache')
            self.assertEqual(materialize(target)[1],identity)
            self.assertEqual(Path(prefix+'.data').read_bytes(),b'data')
            self.assertEqual(target.read_bytes(),before)

    def test_pgm_snapshot_row_flip_and_binary_whitespace_pixel(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);prefix=root/'old'
            (root/'old.yaml').write_text('image: old.pgm\nresolution: 0.05\norigin: [1, 2, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
            (root/'old.pgm').write_bytes(b'P5\n# map\n2 2\n255\n'+bytes([10,255,205,0]))
            (root/'old.posegraph').write_bytes(b'graph');(root/'old.data').write_bytes(b'data')
            target=import_snapshot(prefix,root/'pool.tankmap');_,_,files=read_bundle(target)
            self.assertEqual(files['grid.bin'],bytes([255,100,100,0]))

    def test_corrupt_bundle_is_rejected(self):
        import zipfile
        with tempfile.TemporaryDirectory() as folder:
            target=bundle(Path(folder)/'pool',self.files())
            with zipfile.ZipFile(target) as source:files={n:source.read(n) for n in source.namelist()}
            files['tank.data']=b'corrupt'
            with zipfile.ZipFile(target,'w') as archive:
                for name,value in files.items():archive.writestr(name,value)
            with self.assertRaisesRegex(ValueError,'checksum'):read_bundle(target)

    def test_large_real_map_dimensions_are_preserved(self):
        with tempfile.TemporaryDirectory() as folder:
            files=self.files();grid=json.loads(files['grid.json'])
            grid.update(width=2088,height=1640,resolution=.05)
            files['grid.json']=json.dumps(grid).encode();files['grid.bin']=bytes([255])*(2088*1640)
            target=bundle(Path(folder)/'pool',files)
            _,_,restored=read_bundle(target)
            self.assertEqual(restored['grid.bin'],files['grid.bin'])
            self.assertEqual(json.loads(restored['grid.json']),grid)


if __name__=='__main__':unittest.main()
