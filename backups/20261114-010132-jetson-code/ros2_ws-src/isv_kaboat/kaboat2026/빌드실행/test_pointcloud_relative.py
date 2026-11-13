import sys,unittest
import numpy as np
from types import SimpleNamespace as NS
from viewer_pointcloud import xyz_preview,PointCloudBuffer

class DisplayRangeTests(unittest.TestCase):
    def message(self):
        points=np.array([[.1,0,-20],[.5,0,20],[.6,0,0],[.09,0,0]],dtype='<f4')
        return NS(width=4,height=1,point_step=12,row_step=48,data=points.tobytes(),is_bigendian=False,
            fields=[NS(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate('xyz')],
            header=NS(frame_id='velodyne',stamp=NS(sec=1,nanosec=0)))
    def test_mapping_display_keeps_height_and_limits_horizontal_range(self):
        d=xyz_preview(self.message(),max_range=.5,range_mode='horizontal')
        self.assertEqual(d['range_count'],2)
        self.assertEqual([p[2] for p in d['points']],[-20.,20.])
        buffer=PointCloudBuffer(max_range=.5,range_mode='horizontal');buffer.snapshot()
        buffer.update(self.message(),'/velodyne_points_raw')
        self.assertEqual(buffer.snapshot()['source_topic'],'/velodyne_points_raw')
        self.assertFalse(buffer.snapshot()['stale'])
    def test_mapping_front_sector_includes_side_boundaries_and_preserves_z(self):
        msg=self.message()
        points=np.array([[1,0,20],[-1,0,-20],[0,1,30],[0,-1,-30],[2.01,0,1]],dtype='<f4')
        msg.width=5;msg.row_step=60;msg.data=points.tobytes()
        d=xyz_preview(msg,max_range=2.,range_mode='horizontal',front_only=True)
        self.assertEqual(d['display_count'],3)
        self.assertTrue(all(p[0]>=0 for p in d['points']))
        self.assertEqual(set(p[2] for p in d['points']),{20,30,-30})
        full=xyz_preview(msg,max_range=2.,range_mode='horizontal')
        self.assertEqual(full['display_count'],4)

    def test_navigation_display_default_remains_radial_twelve_meters(self):
        self.assertEqual(PointCloudBuffer().snapshot()['max_range_m'],12.)
        self.assertEqual(xyz_preview(self.message())['range_mode'],'radial')

if __name__=='__main__':unittest.main()
