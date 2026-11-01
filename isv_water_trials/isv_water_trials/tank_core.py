"""Pure geometry, conservative grid planning and low-speed tank guidance (SI)."""
import heapq
import math
import numpy as np
from scipy.ndimage import distance_transform_edt


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class Grid:
    def __init__(self, message, clearance=.75):
        info = message.info
        self.width, self.height = int(info.width), int(info.height)
        self.resolution = float(info.resolution)
        if not 0 < self.width*self.height <= 2_000_000 or not .005 <= self.resolution <= 1:
            raise ValueError('Invalid occupancy grid dimensions')
        self.frame = message.header.frame_id
        self.ox, self.oy = info.origin.position.x, info.origin.position.y
        self.angle = yaw(info.origin.orientation)
        self.co, self.si = math.cos(self.angle), math.sin(self.angle)
        self.cells = np.asarray(message.data, dtype=np.int8).reshape(self.height, self.width)
        free = (self.cells >= 0) & (self.cells < 35)
        # Padding treats map boundaries and unknown cells as obstacles.
        distance = distance_transform_edt(np.pad(free, 1, constant_values=False))[1:-1, 1:-1]*self.resolution
        self.free = free & (distance >= clearance)

    def index(self, point):
        x, y = point[0]-self.ox, point[1]-self.oy
        if not math.isfinite(x+y): raise ValueError('Non-finite coordinate')
        return (math.floor((self.co*x+self.si*y)/self.resolution),
                math.floor((-self.si*x+self.co*y)/self.resolution))

    def point(self, index):
        x, y = (index[0]+.5)*self.resolution, (index[1]+.5)*self.resolution
        return (self.ox+self.co*x-self.si*y, self.oy+self.si*x+self.co*y)

    def safe(self, index):
        x, y = index
        return 0 <= x < self.width and 0 <= y < self.height and bool(self.free[y, x])

    def line_clear(self, start, end):
        n = max(1, math.ceil(math.dist(start, end)/(self.resolution*.4)))
        return all(self.safe(self.index((start[0]+(end[0]-start[0])*i/n,
                                         start[1]+(end[1]-start[1])*i/n))) for i in range(n+1))

    def path(self, start, goal):
        a, b = self.index(start), self.index(goal)
        if not self.safe(a) or not self.safe(b):
            raise ValueError('위치 또는 목적지가 벽·미관측 영역에 너무 가깝습니다.')
        if self.line_clear(start, goal): return [tuple(start), tuple(goal)]
        queue = [(0., a)]; costs = {a: 0.}; parent = {}
        moves = [(1,0), (-1,0), (0,1), (0,-1), (1,1), (1,-1), (-1,1), (-1,-1)]
        while queue:
            _, current = heapq.heappop(queue)
            if current == b: break
            for dx, dy in moves:
                nxt = (current[0]+dx, current[1]+dy)
                if not self.safe(nxt): continue
                if dx and dy and (not self.safe((current[0]+dx,current[1])) or not self.safe((current[0],current[1]+dy))): continue
                cost = costs[current] + math.hypot(dx,dy)
                if cost < costs.get(nxt, float('inf')):
                    costs[nxt] = cost; parent[nxt] = current
                    heapq.heappush(queue, (cost+math.dist(nxt,b), nxt))
            if len(costs) > 200_000: raise ValueError('경로 탐색 범위가 너무 큽니다.')
        if b not in costs: raise ValueError('A와 B 사이에 통과할 수 있는 경로가 없습니다.')
        indices = [b]
        while indices[-1] != a: indices.append(parent[indices[-1]])
        result = [tuple(start)]
        points = [self.point(i) for i in reversed(indices)] + [tuple(goal)]
        i = 0
        while i < len(points)-1:
            j = i+1
            while j+1 < len(points) and self.line_clear(result[-1], points[j+1]): j += 1
            result.append(points[j]); i = j
        return result


def guidance(position, heading, target, percent=20.):
    error = wrap(math.atan2(target[1]-position[1], target[0]-position[0])-heading)
    base = percent*max(0., math.cos(error))
    if abs(error) > math.radians(55): base = 0.
    turn = max(-percent, min(percent, percent*error/.8))
    # Positive yaw is CCW: starboard forward and port reverse create positive yaw.
    left, right = base-turn, base+turn
    scale = max(1., abs(left)/percent, abs(right)/percent)
    return left/scale, right/scale


def pwm(percent):
    if not math.isfinite(percent) or abs(percent)>100: raise ValueError('Invalid percent')
    return 1500+round(4*percent)


def projected_clearance(scan, forward_angle, turning=False):
    points = []
    for i, distance in enumerate(scan.ranges):
        if not math.isfinite(distance) or not max(scan.range_min,.05) < distance <= scan.range_max: continue
        angle = wrap(scan.angle_min+i*scan.angle_increment-forward_angle)
        if turning or abs(angle) < math.radians(50): points.append(distance)
    return min(points) if len(points)>=4 else None
