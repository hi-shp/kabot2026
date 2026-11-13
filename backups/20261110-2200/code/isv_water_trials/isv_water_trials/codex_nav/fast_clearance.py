"""Compiled equivalent of scan-derived circle-clearance evaluation."""
import math
import numpy as np
try:
    from numba import njit
except ImportError:
    njit = None


if njit is not None:
    @njit(cache=True)
    def compiled_clearance(points, obstacles, hull_radius):
        result = np.empty(len(points))
        for i in range(len(points)):
            nearest = math.inf
            for j in range(len(obstacles)):
                dx = points[i,0]-obstacles[j,0]
                dy = points[i,1]-obstacles[j,1]
                nearest = min(nearest,math.hypot(dx,dy)-obstacles[j,2]-hull_radius)
            result[i] = nearest
        return result
else:
    compiled_clearance = None


def warmup_clearance():
    if compiled_clearance is not None:
        compiled_clearance(np.zeros((1,2)),np.zeros((1,3)),.54)
        compiled_clearance(np.zeros((1,2),dtype=np.float32),np.zeros((1,3)),.54)
