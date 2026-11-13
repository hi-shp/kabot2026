"""Optional compiled form of the existing exact raw-segment corridor test."""
import math
import numpy as np
try:
    from numba import njit
except ImportError:
    njit = None


if njit is not None:
    @njit(cache=True, nogil=True)
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

    @njit(cache=True, nogil=True)
    def compiled_corridor_candidate(raw, starts, delta, length_sq, i, maximum):
        """Batch the existing eight-point rejection probes, without Python sets.

        This only selects a candidate. Dense original corridor and obstacle
        checks are still mandatory before the caller accepts a chord.
        """
        for j in range(maximum, i, -1):
            dx, dy = raw[j,0]-raw[i,0], raw[j,1]-raw[i,1]
            count = max(2, int(math.ceil(math.hypot(dx,dy)/.04))+1)
            n = min(8, count)
            probes = np.empty((n,2), dtype=np.float64)
            for k in range(n):
                index = k*(count-1)//(n-1)
                fraction = index/(count-1)
                probes[k,0] = raw[i,0]+dx*fraction
                probes[k,1] = raw[i,1]+dy*fraction
            # Passing the probe never bypasses the exact final .40m check.
            if compiled_within_corridor(probes, starts, delta, length_sq):
                return j
        return i+1
else:
    compiled_within_corridor = None
    compiled_corridor_candidate = None
