"""Compiled implementation of the existing eight-neighbor weighted A*.

The heap orders entries by (f, y, x), exactly like the Python tuple heap in
route_planner.a_star_reference. Unknown-space costs and blocked-cell decisions
are supplied by the existing perception code without alteration.
"""
import math
import numpy as np

try:
    from numba import njit
except ImportError:
    njit = None


if njit is not None:
    @njit(cache=True)
    def compiled_astar(free, weights, xs, ys, sy, sx, gy, gx, resolution):
        ny, nx = free.shape
        # A cell can be relaxed by at most its eight neighbors before closing.
        capacity = ny*nx*8+2
        heap_f = np.empty(capacity, dtype=np.float64)
        heap_y = np.empty(capacity, dtype=np.int32)
        heap_x = np.empty(capacity, dtype=np.int32)
        length = 1
        heap_f[1], heap_y[1], heap_x[1] = 0., sy, sx
        costs = np.full((ny,nx), np.inf)
        costs[sy,sx] = 0.
        parent_y = np.full((ny,nx), -1, dtype=np.int32)
        parent_x = np.full((ny,nx), -1, dtype=np.int32)
        visited = np.zeros((ny,nx), dtype=np.bool_)
        diagonal = math.hypot(1.,1.)*resolution
        dys = (-1,1,0,0,-1,-1,1,1)
        dxs = (0,0,-1,1,-1,1,-1,1)
        while length:
            # Pop minimum (f, y, x), retaining the Python heap tie break.
            y, x = heap_y[1], heap_x[1]
            last_f, last_y, last_x = heap_f[length], heap_y[length], heap_x[length]
            length -= 1
            if length:
                k = 1
                while 2*k <= length:
                    child = 2*k
                    if child+1 <= length:
                        left = (heap_f[child], heap_y[child], heap_x[child])
                        right = (heap_f[child+1], heap_y[child+1], heap_x[child+1])
                        if right < left:
                            child += 1
                    if (last_f,last_y,last_x) <= (heap_f[child],heap_y[child],heap_x[child]):
                        break
                    heap_f[k],heap_y[k],heap_x[k] = heap_f[child],heap_y[child],heap_x[child]
                    k = child
                heap_f[k],heap_y[k],heap_x[k] = last_f,last_y,last_x
            if visited[y,x]:
                continue
            visited[y,x] = True
            if y == gy and x == gx:
                node_count = 1
                yy, xx = y, x
                while parent_y[yy,xx] >= 0:
                    py, px = parent_y[yy,xx], parent_x[yy,xx]
                    yy, xx = py, px
                    node_count += 1
                path = np.empty((node_count,2), dtype=np.float64)
                yy, xx = y, x
                for j in range(node_count-1,-1,-1):
                    path[j,0],path[j,1] = xs[xx],ys[yy]
                    py, px = parent_y[yy,xx],parent_x[yy,xx]
                    yy, xx = py, px
                return path
            current = costs[y,x]
            for j in range(8):
                dy,dx = dys[j],dxs[j]
                yy,xx = y+dy,x+dx
                if yy < 0 or yy >= ny or xx < 0 or xx >= nx or not free[yy,xx]:
                    continue
                if dx and dy and not (free[y,xx] and free[yy,x]):
                    continue
                step = diagonal if dx and dy else resolution
                candidate = current+step*weights[yy,xx]
                if candidate < costs[yy,xx]:
                    costs[yy,xx] = candidate
                    parent_y[yy,xx],parent_x[yy,xx] = y,x
                    f = candidate+math.hypot(gy-yy,gx-xx)*resolution
                    length += 1
                    k = length
                    while k > 1:
                        parent = k//2
                        if (heap_f[parent],heap_y[parent],heap_x[parent]) <= (f,yy,xx):
                            break
                        heap_f[k],heap_y[k],heap_x[k] = heap_f[parent],heap_y[parent],heap_x[parent]
                        k = parent
                    heap_f[k],heap_y[k],heap_x[k] = f,yy,xx
        return np.empty((0,2), dtype=np.float64)
else:
    compiled_astar = None


def warmup_astar():
    """Compile before the timed GUI loop so JIT startup is not a frame hitch."""
    if compiled_astar is not None:
        small = np.ones((2,2),dtype=np.bool_)
        weights = np.ones((2,2),dtype=np.float64)
        axis = np.array([0.,1.])
        compiled_astar(small,weights,axis,axis,0,0,1,1,1.)
