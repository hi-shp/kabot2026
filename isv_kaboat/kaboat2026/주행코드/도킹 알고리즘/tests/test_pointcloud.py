import unittest
import numpy as np
from sensor_msgs.msg import PointCloud2,PointField
from sensor_msgs_py import point_cloud2
from docking_mapping.pointcloud import read_scalar_fields

class MixedCloudTests(unittest.TestCase):
 def cloud(self,empty=False):
  dtype=np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('ring','<u2'),('time','<f8')])
  rows=np.zeros(0 if empty else 3,dtype=dtype)
  if not empty:
   rows['x']=[1,np.nan,7];rows['y']=[2,5,8];rows['z']=[3,6,9];rows['ring']=[4,5,6];rows['time']=[.01,.02,.03]
  fields=[PointField(name=n,offset=dtype.fields[n][1],datatype=PointField.UINT16 if n=='ring' else PointField.FLOAT64 if n=='time' else PointField.FLOAT32,count=1) for n in dtype.names]
  return PointCloud2(height=1,width=len(rows),fields=fields,is_bigendian=False,point_step=dtype.itemsize,row_step=len(rows)*dtype.itemsize,data=rows.tobytes(),is_dense=False)
 def test_reproduces_original_ros_failure(self):
  with self.assertRaisesRegex(AssertionError,'same datatype'):point_cloud2.read_points_numpy(self.cloud(),field_names=('x','y','z'),skip_nans=True)
 def test_mixed_xyz_and_invalid_point(self):
  result=read_scalar_fields(self.cloud(),('x','y','z'))
  np.testing.assert_array_equal(result,[[1,2,3],[7,8,9]]);self.assertEqual(result.dtype,np.float64)
 def test_time_field_and_requested_order(self):
  np.testing.assert_allclose(read_scalar_fields(self.cloud(),('time',)).ravel(),[.01,.02,.03])
  np.testing.assert_array_equal(read_scalar_fields(self.cloud(),('z','x')),[[3,1],[9,7]])
 def test_empty_and_missing_field(self):
  self.assertEqual(read_scalar_fields(self.cloud(True),('x','y','z')).shape,(0,3))
  with self.assertRaises(ValueError):read_scalar_fields(self.cloud(),('missing',))

if __name__=='__main__':unittest.main()
