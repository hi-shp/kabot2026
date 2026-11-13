"""Batched oriented hull / moving tracks / polygon distances, CUDA or compiled CPU.

No dense pose-by-obstacle temporary arrays, no change to observed geometry.
"""
import math
import numpy as np
from .gpu_clearance import cuda_kernel
try:
    from numba import njit
except ImportError:
    njit=None

_hull_source=r'''
__device__ double rectangle(double dx,double dy,double c,double s,double length,double width,double radius){
 double qx=fabs(dx*c+dy*s)-length/2.,qy=fabs(-dx*s+dy*c)-width/2.;
 double x=fmax(qx,0.),y=fmax(qy,0.);
 double d=sqrt(x*x+y*y)+fmin(fmax(qx,qy),0.)-radius;
 return fabs(d)<1e-9?0.:d;
}
extern "C" __global__ void trajectory_hulls(const double* z,const double* times,
 const double* obs,const double* moving,double* out,int n,int m,int tracks,double length,double width,double dt){
 int i=blockDim.x*blockIdx.x+threadIdx.x;if(i>=n)return;
 double c=cos(z[3*i+2]),s=sin(z[3*i+2]),best=1./0.,dynamic=1./0.;
 for(int j=0;j<m;j++){
  double d=rectangle(obs[3*j]-z[3*i],obs[3*j+1]-z[3*i+1],c,s,length,width,obs[3*j+2]);
  best=fmin(best,d);
 }
 for(int j=0;j<tracks;j++){
  const double* q=moving+6*j;
  double radius=q[2]+q[5]*times[i]+sqrt(q[3]*q[3]+q[4]*q[4])*dt/2.;
  double d=rectangle(q[0]+q[3]*times[i]-z[3*i],q[1]+q[4]*times[i]-z[3*i+1],c,s,length,width,radius);
  dynamic=fmin(dynamic,d);
 }
 out[2*i]=best;out[2*i+1]=dynamic;
}
'''

_polygon_source=r'''
extern "C" __global__ void region_distance(const double* points,const double* a,const double* b,
 double* out,int n,int m){
 int i=blockDim.x*blockIdx.x+threadIdx.x;if(i>=n)return;
 double x=points[2*i],y=points[2*i+1],best=1./0.;int crossings=0;
 for(int j=0;j<m;j++){
  double ex=b[2*j]-a[2*j],ey=b[2*j+1]-a[2*j+1];
  double dx=x-a[2*j],dy=y-a[2*j+1];
  double t=fmin(1.,fmax(0.,(dx*ex+dy*ey)/(ex*ex+ey*ey)));
  double px=dx-t*ex,py=dy-t*ey;best=fmin(best,sqrt(px*px+py*py));
  if((a[2*j+1]>y)!=(b[2*j+1]>y)){
   if(x<a[2*j]+(y-a[2*j+1])*ex/(ey==0.?1.:ey))crossings++;
  }
 }
 out[i]=(crossings%2)?best:-best;
}
'''


def _hulls(poses,times,obstacles,moving,length,width,dt):
    result=np.full((len(poses),2),np.inf)
    for i in range(len(poses)):
        c=math.cos(poses[i,2]);s=math.sin(poses[i,2])
        for group in range(2):
            count=len(obstacles) if group==0 else len(moving)
            for j in range(count):
                if group==0:
                    dx=obstacles[j,0]-poses[i,0];dy=obstacles[j,1]-poses[i,1];radius=obstacles[j,2]
                else:
                    dx=moving[j,0]+moving[j,3]*times[i]-poses[i,0]
                    dy=moving[j,1]+moving[j,4]*times[i]-poses[i,1]
                    radius=moving[j,2]+moving[j,5]*times[i]+math.hypot(moving[j,3],moving[j,4])*dt/2
                qx=abs(dx*c+dy*s)-length/2;qy=abs(-dx*s+dy*c)-width/2
                distance=math.hypot(max(qx,0.),max(qy,0.))+min(max(qx,qy),0.)-radius
                if abs(distance)<1e-9:distance=0.
                result[i,group]=min(result[i,group],distance)
    return result


def _polygon(points,a,b):
    result=np.empty(len(points))
    for i in range(len(points)):
        best=math.inf;crossings=0;x=points[i,0];y=points[i,1]
        for j in range(len(a)):
            ex=b[j,0]-a[j,0];ey=b[j,1]-a[j,1];dx=x-a[j,0];dy=y-a[j,1]
            t=min(1.,max(0.,(dx*ex+dy*ey)/(ex*ex+ey*ey)))
            best=min(best,math.hypot(dx-t*ex,dy-t*ey))
            if (a[j,1]>y)!=(b[j,1]>y) and x<a[j,0]+(y-a[j,1])*ex/(1. if ey==0. else ey):crossings+=1
        result[i]=best if crossings%2 else -best
    return result


compiled_hulls=njit(cache=True,nogil=True)(_hulls) if njit else _hulls
compiled_polygon=njit(cache=True,nogil=True)(_polygon) if njit else _polygon


def trajectory_hulls(poses,times,obstacles,moving,length,width,dt,force_gpu=False):
    poses=np.ascontiguousarray(np.asarray(poses)[:,:3],dtype=np.float64)
    times=np.ascontiguousarray(times,dtype=np.float64).reshape(len(poses))
    obstacles=np.ascontiguousarray(obstacles,dtype=np.float64).reshape(-1,3)
    moving=np.ascontiguousarray(moving,dtype=np.float64).reshape(-1,6)
    count=len(poses)
    if count and (force_gpu or count*(len(obstacles)+len(moving))>=65536):
        result=cuda_kernel('trajectory_hulls',_hull_source,'trajectory_hulls',
            (poses,times,obstacles,moving),(count,2),
            (np.int32(count),np.int32(len(obstacles)),np.int32(len(moving)),np.float64(length),np.float64(width),np.float64(dt)),count)
        if result is not None:return result
    return compiled_hulls(poses,times,obstacles,moving,float(length),float(width),float(dt))


def polygon_clearance(points,a,b,force_gpu=False):
    points=np.ascontiguousarray(points,dtype=np.float64).reshape(-1,2)
    a=np.ascontiguousarray(a,dtype=np.float64).reshape(-1,2)
    b=np.ascontiguousarray(b,dtype=np.float64).reshape(-1,2)
    if len(points) and (force_gpu or len(points)*len(a)>=65536):
        result=cuda_kernel('region_distance',_polygon_source,'region_distance',(points,a,b),
            (len(points),),(np.int32(len(points)),np.int32(len(a))),len(points))
        if result is not None:return result
    return compiled_polygon(points,a,b)


def warmup_geometry():
    # Compile CPU fallbacks even when CUDA is present, then initialize CUDA
    # kernels here rather than on the first propulsion planning tick.
    trajectory_hulls(np.zeros((1,3)),np.zeros(1),np.zeros((1,3)),np.empty((0,6)),1.,.5,.2)
    polygon_clearance(np.zeros((1,2)),np.array([[0.,0.],[1.,0.],[0.,1.]]),np.array([[1.,0.],[0.,1.],[0.,0.]]))
    trajectory_hulls(np.zeros((1,3)),np.zeros(1),np.zeros((1,3)),np.empty((0,6)),1.,.5,.2,force_gpu=True)
    polygon_clearance(np.zeros((1,2)),np.array([[0.,0.],[1.,0.],[0.,1.]]),np.array([[1.,0.],[0.,1.],[0.,0.]]),force_gpu=True)
