"""Persistent GPS operating areas: union of allowed polygons minus exclusions."""
import numpy as np
from .codex_nav.gpu_geometry import polygon_clearance


class OperatingRegion:
    def __init__(self, regions):
        # Lazy import keeps telemetry available before optional deployment dependencies install.
        from shapely.geometry import Polygon
        from shapely.ops import unary_union
        if not isinstance(regions, list) or not 1 <= len(regions) <= 16:
            raise ValueError('구역은 1~16개로 지정하세요')
        included, excluded = [], []
        self.regions = []
        total = 0
        for item in regions:
            if not isinstance(item, dict):
                raise ValueError('구역 형식을 확인하세요')
            kind = item.get('kind')
            p = np.asarray(item.get('points', []), dtype=float)
            if kind not in ('include', 'exclude') or p.ndim != 2 or p.shape[1] != 2 or not 3 <= len(p) <= 32 or not np.isfinite(p).all():
                raise ValueError('각 구역의 종류와 꼭짓점 3~32개를 확인하세요')
            total += len(p)
            if total > 256:
                raise ValueError('전체 꼭짓점은 256개까지 지정하세요')
            if np.min(np.linalg.norm(np.roll(p,-1,axis=0)-p,axis=1)) < .05:
                raise ValueError('같은 위치의 꼭짓점을 중복해서 지정하지 마세요')
            shape = Polygon(p)
            if not shape.is_valid or shape.area < .25:
                raise ValueError('구역 선이 교차하거나 면적이 너무 작습니다')
            (included if kind == 'include' else excluded).append(shape)
            self.regions.append({'kind': kind, 'points': p.tolist()})
        if not included:
            raise ValueError('이동 구역을 하나 이상 지정하세요')
        shape = unary_union(included).difference(unary_union(excluded))
        if shape.is_empty or shape.area < 1:
            raise ValueError('제외 구역을 뺀 이동 공간이 부족합니다')
        self.shape = shape
        components = [shape] if shape.geom_type == 'Polygon' else list(shape.geoms)
        self.rings = [np.asarray(r.coords)[:-1] for s in components for r in [s.exterior, *s.interiors]]
        self.a = np.concatenate(self.rings)
        self.b = np.concatenate([np.roll(p, -1, axis=0) for p in self.rings])
        self.bounds = np.array(shape.bounds).reshape(2, 2)

    def clearance(self, points):
        """Signed distance to the actual union boundary, excluding internal seams."""
        return polygon_clearance(points,self.a,self.b)

    def clearance_reference(self, points):
        """Original bounded NumPy calculation for equivalence checks."""
        points = np.atleast_2d(points).astype(float)
        result = []
        edge = self.b-self.a
        norm = np.sum(edge*edge, axis=1)
        for p in np.array_split(points, max(1, (len(points)+127)//128)):
            delta = p[:, None, :]-self.a
            t = np.clip(np.sum(delta*edge, axis=2)/norm, 0, 1)
            distance = np.min(np.linalg.norm(delta-t[:, :, None]*edge, axis=2), axis=1)
            x, y = p[:, 0, None], p[:, 1, None]
            dy = edge[:, 1]
            cross = ((self.a[:, 1] > y) != (self.b[:, 1] > y)) & (x < self.a[:, 0]+(y-self.a[:, 1])*edge[:, 0]/np.where(dy == 0, 1, dy))
            inside = np.count_nonzero(cross, axis=1) % 2 == 1
            result.extend(np.where(inside, distance, -distance))
        return np.asarray(result)

    def hull_clearance(self, states, length, width):
        # Circumscribed hull radius covers corners, enclosed holes and edges alike.
        return self.clearance(np.atleast_2d(states)[:, :2])-np.hypot(length, width)/2

    def fence(self, step=.35):
        chunks = [np.linspace(a, b, max(2, int(np.ceil(np.linalg.norm(b-a)/step))+1)) for a, b in zip(self.a, self.b)]
        p = np.concatenate(chunks)
        return np.c_[p, np.full(len(p), step/2+.02)]

    def reachable(self, points, position, margin):
        from shapely.geometry import Point
        from shapely.prepared import prep
        space = self.shape.buffer(-margin)
        parts = [space] if space.geom_type == 'Polygon' else getattr(space, 'geoms', [])
        component = next((p for p in parts if p.covers(Point(position))), None)
        if component is None:
            return np.zeros(len(points), dtype=bool)
        prepared = prep(component)
        return np.array([prepared.covers(Point(p)) for p in points])


def gps_regions(regions, origin=None):
    from .outdoor_model import local_xy
    if not isinstance(regions, list) or not regions:
        raise ValueError('이동 구역을 지정하고 저장하세요')
    converted = []
    anchor = origin
    for item in regions:
        if not isinstance(item, dict):
            raise ValueError('구역 형식을 확인하세요')
        p = np.asarray(item.get('points', []), dtype=float)
        if p.ndim != 2 or p.shape[1] != 2 or not len(p) or not np.isfinite(p).all() or np.any(abs(p[:, 0]) >= 85) or np.any(abs(p[:, 1]) > 180):
            raise ValueError('구역 GPS 좌표를 확인하세요')
        if anchor is None:
            anchor = p[0]
        converted.append({'kind': item.get('kind'), 'points': [local_xy(*v, anchor).tolist() for v in p]})
    region = OperatingRegion(converted)
    if np.max(region.bounds[1]-region.bounds[0]) > 2000:
        raise ValueError('구역 전체 범위는 2km 이내로 지정하세요')
    return region
