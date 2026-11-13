import math,unittest
from types import SimpleNamespace as NS
import numpy as np
from isv_shape_navigation.lidar_preprocess import extract_xyz,filter_points,voxel_centroids,statistical_outliers,adaptive_dbscan,project_ranges,navigation_slice

class PreprocessingTests(unittest.TestCase):
 def test_point_fields_padding_and_endianness(self):
  for endian in ('<','>'):
   dtype=np.dtype(dict(names=['x','y','z'],formats=[endian+'f4']*3,offsets=[4,8,12],itemsize=20))
   data=bytearray(52);arr=np.ndarray((2,1),dtype=dtype,buffer=data,strides=(28,20))
   for i,k in enumerate(('x','y','z')):arr[k]=np.array([[i+1],[i+4]])
   msg=NS(fields=[NS(name=k,datatype=7,count=1,offset=4+4*i) for i,k in enumerate(('x','y','z'))],
     width=1,height=2,point_step=20,row_step=28,is_bigendian=endian=='>',data=data)
   np.testing.assert_allclose(extract_xyz(msg),[[1,2,3],[4,5,6]])
 def test_fov_and_range(self):
  xyz=np.array([[1,0,0],[3,0,0],[1,0,1],[2.5,0,0]],dtype=float)
  fov,raw=filter_points(xyz,-15,15,2.5)
  self.assertEqual(len(fov),3);np.testing.assert_allclose(raw,[[1,0,0],[2.5,0,0]])
 def test_voxel_centroid(self):
  xyz=np.array([[.001,.002,.003],[.011,.012,.013],[1,1,1]])
  ds=voxel_centroids(xyz,.03)
  np.testing.assert_allclose(ds,[[.006,.007,.008],[1,1,1]],atol=1e-7)
 def test_sor_removes_isolated_outlier(self):
  rng=np.random.default_rng(1);xyz=np.vstack([rng.normal(0,.01,(100,3)),[2,2,2]])
  ds=statistical_outliers(xyz,30,1.5)
  self.assertLess(len(ds),len(xyz));self.assertLess(np.linalg.norm(ds,axis=1).max(),.1)
 def test_adaptive_clusters_and_exact_range_boundary(self):
  rng=np.random.default_rng(2);xyz=np.vstack([rng.normal([.5,0,0],.005,(20,3)),rng.normal([1.8,0,0],.005,(20,3)),[2.5,0,0]])
  labels=adaptive_dbscan(xyz,2.5,.13,.1,8)
  self.assertTrue(np.all(labels[:20]>=0));self.assertTrue(np.all(labels[20:40]>=0))
  self.assertNotEqual(labels[0],labels[20]);self.assertEqual(labels[-1],-1)
 def test_scan_uses_nearest_range_and_inf_for_no_hit(self):
  ranges,inc=project_ranges(np.array([[1,0,0],[2,0,0],[0,2,0]]),.007,.1,2.5)
  self.assertEqual(len(ranges),math.ceil(2*math.pi/.007))
  finite=ranges[np.isfinite(ranges)];np.testing.assert_allclose(sorted(finite),[1,2])
  self.assertTrue(np.isposinf(ranges[0]));self.assertAlmostEqual(len(ranges)*inc,2*math.pi)
 def test_navigation_slice_excludes_ground_and_keeps_horizontal_ring(self):
  xyz=np.array([[2.,0.,-.44],[1.,0.,math.tan(math.radians(1.))],[1.,0.,math.tan(math.radians(3.))]])
  original=xyz.copy();selected=navigation_slice(xyz,0.,2.)
  np.testing.assert_allclose(selected,xyz[1:2])
  np.testing.assert_allclose(xyz,original)
 def test_ground_projection_no_longer_blocks_clear_forward_path(self):
  from isv_shape_navigation.core import Scan
  from isv_shape_navigation.route_mission import RouteConfig,choose_corridor
  xyz=np.array([[1.,0.,-.25],[2.,1.,math.tan(math.radians(1.))*math.sqrt(5.)]])
  ranges,inc=project_ranges(xyz,.007,.1,2.5)
  old=Scan(ranges,-math.pi,inc,.1,2.5,100.)
  self.assertNotEqual(choose_corridor(old,0.,RouteConfig(),.25)[0],0.)
  ranges,inc=project_ranges(navigation_slice(xyz),.007,.1,2.5)
  filtered=Scan(ranges,-math.pi,inc,.1,2.5,100.)
  self.assertAlmostEqual(choose_corridor(filtered,0.,RouteConfig(),.25)[0],0.)
 def test_empty_frames_are_supported(self):
  empty=np.empty((0,3))
  self.assertEqual(len(voxel_centroids(empty,.03)),0)
  self.assertEqual(len(statistical_outliers(empty,30,1.5)),0)
  self.assertEqual(len(adaptive_dbscan(empty,2.5,.13,.1,8)),0)
  self.assertTrue(np.isposinf(project_ranges(empty,.007,.1,2.5)[0]).all())

if __name__=='__main__':unittest.main()
