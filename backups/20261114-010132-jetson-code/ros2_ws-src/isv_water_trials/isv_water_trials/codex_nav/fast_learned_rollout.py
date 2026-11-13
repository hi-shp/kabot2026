"""Fused measured-model dynamics/allocation. No IO; NumPy remains the oracle."""
import math
import numpy as np
try:
    from numba import njit
except ImportError:
    njit=None


def _motion(initial,sequences,history,coeff,matrix,inverse,lo,hi,dt,slew,iterations):
    n,horizon,_=sequences.shape
    result=np.empty((n,horizon,8),dtype=np.float64)
    for k in range(n):
        z=initial.copy()
        for t in range(horizon):
            base=np.empty(3)
            for j in range(3):
                base[j]=z[3]*coeff[0,j]+z[4]*coeff[1,j]+z[5]*coeff[2,j]+coeff[7,j]
            target0=z[3]+dt*(sequences[k,t,0]-z[3])
            target1=z[5]+dt*(sequences[k,t,1]-z[5])
            q0=q1=0.
            for _ in range(iterations):
                r0=2.*(target0-base[0]-q0*abs(q0)*coeff[5,0]-q1*abs(q1)*coeff[6,0])
                r1=target1-base[2]-q0*abs(q0)*coeff[5,2]-q1*abs(q1)*coeff[6,2]
                best=math.inf
                # Interior, then the same four box edges and tie order as inverse().
                for choice in range(5):
                    if choice==0:
                        a=min(hi[0],max(lo[0],r0*inverse[0,0]+r1*inverse[1,0]))
                        b=min(hi[1],max(lo[1],r0*inverse[0,1]+r1*inverse[1,1]))
                    elif choice<=2:
                        a=lo[0] if choice==1 else hi[0]
                        norm=max(matrix[1,0]**2+matrix[1,1]**2,1e-12)
                        b=min(hi[1],max(lo[1],((r0-a*matrix[0,0])*matrix[1,0]+(r1-a*matrix[0,1])*matrix[1,1])/norm))
                    else:
                        b=lo[1] if choice==3 else hi[1]
                        norm=max(matrix[0,0]**2+matrix[0,1]**2,1e-12)
                        a=min(hi[0],max(lo[0],((r0-b*matrix[1,0])*matrix[0,0]+(r1-b*matrix[1,1])*matrix[0,1])/norm))
                    cost=(a*matrix[0,0]+b*matrix[1,0]-r0)**2+(a*matrix[0,1]+b*matrix[1,1]-r1)**2
                    if cost<best:best=cost;q0=a;q1=b
            command0=min(hi[0],max(lo[0],z[6]+min(slew,max(-slew,q0-z[6]))))
            command1=min(hi[1],max(lo[1],z[7]+min(slew,max(-slew,q1-z[7]))))
            delay=len(history)
            if t<delay:d0=history[t,0];d1=history[t,1]
            elif delay:d0=result[k,t-delay,6];d1=result[k,t-delay,7]
            else:d0=command0;d1=command1
            velocity=np.empty(3)
            for j in range(3):
                velocity[j]=(z[3]*coeff[0,j]+z[4]*coeff[1,j]+z[5]*coeff[2,j]+
                             d0*coeff[3,j]+d1*coeff[4,j]+d0*abs(d0)*coeff[5,j]+d1*abs(d1)*coeff[6,j]+coeff[7,j])
                if not math.isfinite(velocity[j]) or abs(velocity[j])>20:
                    raise ValueError('Measured model prediction diverged')
            u=.5*(velocity[0]+z[3]);v=.5*(velocity[1]+z[4]);r=.5*(velocity[2]+z[5])
            yaw=z[2]+r*dt/2
            z[0]+=dt*(math.cos(yaw)*u-math.sin(yaw)*v)
            z[1]+=dt*(math.sin(yaw)*u+math.cos(yaw)*v)
            z[2]+=r*dt
            z[3:6]=velocity;z[6]=command0;z[7]=command1
            result[k,t]=z
    return result


compiled_motion=njit(cache=True,nogil=True)(_motion) if njit else None


def _command_motion(initial,targets,history,coeff,lo,hi,dt,slew,first_dt,committed):
    """Direct PWM prediction including queued commands and bounded slew.

    The committed prefix holds the current output for extra reaction latency;
    it never discards the model's separate measured input-delay queue.
    """
    n,horizon,_=targets.shape
    result=np.empty((n,horizon,8),dtype=np.float64)
    for k in range(n):
        z=initial.copy()
        for t in range(horizon):
            step=first_dt if t==committed else dt
            q=np.empty(2)
            for side in range(2):
                target=z[6+side] if t<committed else targets[k,t,side]
                q[side]=min(hi[side],max(lo[side],z[6+side]+min(slew*step,max(-slew*step,target-z[6+side]))))
            delay=len(history)
            if t<delay:d0=history[t,0];d1=history[t,1]
            elif delay:d0=result[k,t-delay,6];d1=result[k,t-delay,7]
            else:d0=q[0];d1=q[1]
            velocity=np.empty(3)
            for j in range(3):
                velocity[j]=(z[3]*coeff[0,j]+z[4]*coeff[1,j]+z[5]*coeff[2,j]+
                    d0*coeff[3,j]+d1*coeff[4,j]+d0*abs(d0)*coeff[5,j]+d1*abs(d1)*coeff[6,j]+coeff[7,j])
                if not math.isfinite(velocity[j]) or abs(velocity[j])>20:raise ValueError('Measured model prediction diverged')
            u=.5*(velocity[0]+z[3]);v=.5*(velocity[1]+z[4]);r=.5*(velocity[2]+z[5]);yaw=z[2]+r*dt/2
            z[0]+=dt*(math.cos(yaw)*u-math.sin(yaw)*v);z[1]+=dt*(math.sin(yaw)*u+math.cos(yaw)*v);z[2]+=r*dt
            z[3:6]=velocity;z[6:8]=q;result[k,t]=z
    return result


command_motion=njit(cache=True,nogil=True)(_command_motion) if njit else _command_motion


def warmup_command_motion():
    command_motion(np.zeros(8),np.zeros((1,1,2)),np.empty((0,2)),np.zeros((8,3)),
                   np.full(2,-1.),np.ones(2),.2,.35,.1,0)
