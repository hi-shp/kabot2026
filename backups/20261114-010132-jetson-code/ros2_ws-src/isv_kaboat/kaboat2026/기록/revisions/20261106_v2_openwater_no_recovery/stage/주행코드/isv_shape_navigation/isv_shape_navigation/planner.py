"""Occupancy-grid A*: unknown and map edges are blocked; inflate full hull."""
import heapq
import math
import numpy as np
from scipy.ndimage import distance_transform_edt


class Grid:
    def __init__(self, cells, resolution, origin=(0., 0., 0.), radius=.77,
                 occupied_threshold=35, max_expansions=150000):
        self.cells = np.asarray(cells)
        if (self.cells.ndim != 2 or not self.cells.size or not math.isfinite(resolution)
                or resolution <= 0 or not math.isfinite(radius) or radius <= 0
                or len(origin) != 3 or not np.isfinite(origin).all()):
            raise ValueError('Invalid map geometry')
        self.resolution = resolution
        self.origin = origin
        self.height, self.width = self.cells.shape
        self.max_expansions = max_expansions
        free = (self.cells >= 0) & (self.cells < occupied_threshold)
        padded = np.pad(free, 1, constant_values=False)
        # Account for the obstacle cell's area, not only its centre.
        clearance = distance_transform_edt(padded)[1:-1, 1:-1]*resolution
        self.free = clearance > radius+resolution*math.sqrt(2)/2

    def cell(self, point):
        x, y = np.asarray(point)-self.origin[:2]
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        return math.floor((c*x+s*y)/self.resolution), math.floor((-s*x+c*y)/self.resolution)

    def world(self, cell):
        x, y = (np.asarray(cell)+.5)*self.resolution
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        return self.origin[0]+c*x-s*y, self.origin[1]+s*x+c*y

    def valid(self, cell):
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height and bool(self.free[y, x])

    def segment_free(self, start, end):
        start, end = np.asarray(start), np.asarray(end)
        count = max(1, math.ceil(np.linalg.norm(end-start)/(self.resolution*.25)))
        previous = None
        for ratio in np.linspace(0, 1, count+1):
            current = self.cell(start+(end-start)*ratio)
            if not self.valid(current):
                return False
            if previous and current[0] != previous[0] and current[1] != previous[1]:
                if not self.valid((current[0], previous[1])) or not self.valid((previous[0], current[1])):
                    return False
            previous = current
        return True

    def plan(self, start, goal):
        start_cell, end_cell = self.cell(start), self.cell(goal)
        if not self.valid(start_cell) or not self.valid(end_cell):
            return []
        if self.segment_free(start, goal):
            return [tuple(start), tuple(goal)]
        queue = [(0., start_cell)]
        costs, came, closed = {start_cell: 0.}, {}, set()
        while queue and len(closed) < self.max_expansions:
            _, current = heapq.heappop(queue)
            if current in closed:
                continue
            if current == end_cell:
                cells = [current]
                while cells[-1] != start_cell:
                    cells.append(came[cells[-1]])
                raw = [tuple(start), *[self.world(c) for c in reversed(cells[:-1])], tuple(goal)]
                # Greedy LOS pruning only across collision-checked segments.
                path, index = [raw[0]], 0
                while index < len(raw)-1:
                    following = index+1
                    for candidate in range(index+2, len(raw)):
                        if not self.segment_free(raw[index], raw[candidate]):
                            break
                        following = candidate
                    path.append(raw[following]); index = following
                return path
            closed.add(current)
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1),
                           (1, 1), (1, -1), (-1, 1), (-1, -1)):
                nxt = current[0]+dx, current[1]+dy
                if not self.valid(nxt):
                    continue
                if dx and dy and (not self.valid((current[0]+dx, current[1]))
                                  or not self.valid((current[0], current[1]+dy))):
                    continue
                cost = costs[current]+math.hypot(dx, dy)
                if cost < costs.get(nxt, math.inf):
                    costs[nxt], came[nxt] = cost, current
                    estimate = math.hypot(nxt[0]-end_cell[0], nxt[1]-end_cell[1])
                    heapq.heappush(queue, (cost+estimate, nxt))
        return []
