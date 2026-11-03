"""Validated local PWM response models; extrapolation is rejected."""
import math


def predict(model, position, heading, velocity, left, right, seconds=3., dt=.1):
    channels = [model.get(k,{}) for k in ('surge','yaw')]
    if not all(c.get('qualified') for c in channels): return None
    u,v,r = velocity; q = ((left+right)/200.,(right-left)/200.)
    for c,value,command in zip(channels,(u,r),q):
        bounds = c.get('observed_command_range',[]); speeds = c.get('observed_velocity_range',[])
        if len(bounds)!=2 or len(speeds)!=2 or not bounds[0]-.01<=command<=bounds[1]+.01: return None
        if not speeds[0]-.03<=value<=speeds[1]+.03: return None
    x,y = position; points = [[x,y]]
    for _ in range(round(seconds/dt)):
        u += dt*(-channels[0]['decay_per_s']*u+channels[0]['pwm_gain']*q[0]+channels[0]['bias'])
        r += dt*(-channels[1]['decay_per_s']*r+channels[1]['pwm_gain']*q[1]+channels[1]['bias'])
        heading += r*dt
        x += dt*(u*math.cos(heading)-v*math.sin(heading))
        y += dt*(u*math.sin(heading)+v*math.cos(heading))
        if not all(math.isfinite(n) for n in (x,y,u,r)) or abs(u)>2 or abs(r)>3: return None
        points.append([x,y])
    return {'kind':'validated_local_response','seconds':seconds,'points_m':points,
            'limits':'Local 20% tank data; lateral drift held constant; no extrapolation'}
