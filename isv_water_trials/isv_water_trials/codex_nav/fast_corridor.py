"""Optional compiled form of the existing exact raw-segment corridor test."""
import math
try:
    from numba import njit
except ImportError:
    njit = None


if njit is not None:
    @njit(cache=True)
    def compiled_within_corridor(points,starts,delta,length_sq):
        for i in range(len(points)):
            px,py = points[i,0],points[i,1]
            nearest = math.inf
            for j in range(len(starts)):
                ox,oy = px-starts[j,0],py-starts[j,1]
                dx,dy = delta[j,0],delta[j,1]
                fraction = (ox*dx+oy*dy)/length_sq[j]
                fraction = max(0.,min(1.,fraction))
                distance = math.hypot(ox-fraction*dx,oy-fraction*dy)
                nearest = min(nearest,distance)
                if nearest <= .40:
                    break
            if nearest > .40:
                return False
        return True
else:
    compiled_within_corridor = None
