"""CUDA nearest-surface batches, with an explicit CPU fallback and no ROS IO."""
import os
import threading
import numpy as np

_lock=threading.RLock()
_engine=None
_attempted=False
_status={'backend':'CPU','device':None,'gpu_calls':0,'reason':'not initialized'}
_kernels={}
_source=r'''
extern "C" __global__ void nearest_surface(
 const double* q,const double* o,double* out,int n,int m,double hull){
 int i=blockDim.x*blockIdx.x+threadIdx.x;if(i>=n)return;
 double best=1.0/0.0;
 for(int j=0;j<m;j++){
  double dx=q[2*i]-o[3*j],dy=q[2*i+1]-o[3*j+1];
  double d=sqrt(dx*dx+dy*dy)-o[3*j+2]-hull;
  if(d<best)best=d;
 }
 out[i]=best;
}
'''


def gpu_status():
    with _lock:return {**_status,'stages':dict(_status.get('stages',{}))}


def cuda_kernel(stage,source,name,arrays,output_shape,scalars,rows):
    """One transfer/launch/readback per batch; shared device, stream and kernels.

    No fast-math: signed distances and double precision remain safety inputs.
    Returning None always means the caller must evaluate the CPU implementation.
    """
    global _engine,_attempted
    if os.environ.get('ISV_GPU_PLANNER','1')=='0':return None
    with _lock:
        try:
            if not _attempted:
                _attempted=True
                import cupy as cp
                if not cp.cuda.runtime.getDeviceCount():raise RuntimeError('CUDA device unavailable')
                device=cp.cuda.Device(0)
                with device:stream=cp.cuda.Stream(non_blocking=True)
                device_name=cp.cuda.runtime.getDeviceProperties(0)['name']
                _engine=(cp,device,stream)
                _status.update(backend='CUDA',device=device_name.decode() if isinstance(device_name,bytes) else str(device_name),reason=None)
            if _engine is None:return None
            cp,device,stream=_engine
            with device,stream:
                key=(source,name)
                if key not in _kernels:_kernels[key]=cp.RawKernel(source,name)
                inputs=tuple(cp.asarray(a) for a in arrays)
                result=cp.empty(output_shape,dtype=cp.float64)
                _kernels[key](((rows+127)//128,),(128,),inputs+(result,)+tuple(scalars))
                answer=result.get(stream=stream)
            _status['gpu_calls']+=1
            stages=_status.setdefault('stages',{})
            stages[stage]=stages.get(stage,0)+1
            return answer
        except Exception as exc:
            _engine=None
            _status.update(backend='CPU',reason=type(exc).__name__+': '+str(exc)[:160])
            return None


def cuda_clearance(points,obstacles,hull_radius=0.,force=False):
    """Return a NumPy result, or None when CPU is appropriate/unavailable."""
    points=np.ascontiguousarray(points,dtype=np.float64).reshape(-1,2)
    obstacles=np.ascontiguousarray(obstacles,dtype=np.float64).reshape(-1,3)
    if not len(points) or not len(obstacles):return None
    if not force and len(points)*len(obstacles)<65536:return None
    return cuda_kernel('grid_distance',_source,'nearest_surface',(points,obstacles),
                       (len(points),),(np.int32(len(points)),np.int32(len(obstacles)),np.float64(hull_radius)),len(points))
