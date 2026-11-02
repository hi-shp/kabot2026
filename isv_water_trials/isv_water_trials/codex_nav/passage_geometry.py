"""Physical twin-hull/deck surface geometry for the exact-passage experiment.

These are the polygons used by BoatEnv.collide, converted to SI units.
No obstacle inflation or controller safety margin is baked into this geometry.
"""
import numpy as np
import math

try:
    from numba import njit
except ImportError:
    njit = None


def physical_hull_polygons(pixels_per_m=50.):
    length, beam_half, spacing = 84., 16., 11.
    hull = np.array([(length*.50, 0.), (length*.12, beam_half),
                     (-length*.28, beam_half*.85), (-length*.48, beam_half*.6),
                     (-length*.50, 0.), (-length*.48, -beam_half*.6),
                     (-length*.28, -beam_half*.85), (length*.12, -beam_half)])
    deck = np.array([(length*.25, -spacing*.85), (length*.25, spacing*.85),
                     (-length*.35, spacing*.85), (-length*.35, -spacing*.85)])
    # Repeated deck vertices preserve the polygon while keeping one dense array.
    return np.asarray([hull+[0., spacing], hull-[0., spacing],
                       np.vstack((deck, np.repeat(deck[-1:], 4, axis=0)))])/pixels_per_m


def projected_width(heading_error, polygons):
    angle = np.asarray(heading_error)
    normal = np.stack((np.sin(angle), np.cos(angle)), axis=-1)
    projection = normal@polygons.reshape(-1, 2).T
    return np.ptp(projection, axis=-1)


def surface_clearances(states, obstacles, width, height, polygons, exact_walls=False):
    """Per-obstacle signed surface distances, then left/right/bottom/top walls.

    An obstacle row is already the observed representation. Its uncertainty
    padding is retained; this function adds neither hull inflation nor margin.
    """
    z = np.atleast_2d(states)
    c, s = np.cos(z[:, 2]), np.sin(z[:, 2])
    # Preserve the existing independent wall gate. This ticket changes only
    # obstacle/hull surface representation, not boundary collision semantics.
    walls = np.column_stack((z[:, 0]-.93, width-.93-z[:, 0],
                             z[:, 1]-.93, height-.93-z[:, 1]))
    if exact_walls:
        vertices=polygons.reshape(-1,2)
        x=z[:,0,None]+c[:,None]*vertices[:,0]-s[:,None]*vertices[:,1]
        y=z[:,1,None]+s[:,None]*vertices[:,0]+c[:,None]*vertices[:,1]
        walls=np.column_stack((x.min(1),width-x.max(1),y.min(1),height-y.max(1)))
        # Preserve BoatEnv.collide's independent axis-aligned center gates.
        walls=np.minimum(walls,np.column_stack((z[:,0]-.84,width-.84-z[:,0],
                                                z[:,1]-.54,height-.54-z[:,1])))
    if not len(obstacles):
        return walls
    delta = obstacles[None, :, :2]-z[:, None, :2]
    q = np.stack((delta[:, :, 0]*c[:, None]+delta[:, :, 1]*s[:, None],
                  -delta[:, :, 0]*s[:, None]+delta[:, :, 1]*c[:, None]), axis=-1)
    clearance = np.full(q.shape[:2], np.inf)
    for polygon in polygons:
        end = np.roll(polygon, -1, axis=0)
        edge = end-polygon
        offset = q[:, :, None, :]-polygon
        fraction = np.clip(np.sum(offset*edge, axis=-1)/
                           np.maximum(np.sum(edge*edge, axis=-1), 1e-20), 0., 1.)
        distance = np.linalg.norm(offset-fraction[..., None]*edge, axis=-1).min(-1)
        py, px = q[:, :, None, 1], q[:, :, None, 0]
        straddles = (polygon[:, 1] > py) != (end[:, 1] > py)
        denominator = np.where(edge[:, 1] != 0., edge[:, 1], 1.)
        intercept = polygon[:, 0]+(py-polygon[:, 1])*edge[:, 0]/denominator
        inside = np.count_nonzero(straddles & (px < intercept), axis=-1)%2 == 1
        clearance = np.minimum(clearance, np.where(inside, -distance, distance)-obstacles[None, :, 2])
    return np.column_stack((clearance, walls))


def _surface_clearance_at_pose(x, y, heading, obstacles, width, height, polygons,
                               stats=None):
    """Scalar equivalent for the fused physics kernel; same surfaces and walls."""
    if stats is not None:
        stats[0] += 1  # physical poses
    c, s = math.cos(heading), math.sin(heading)
    best = min(x-.93, width-.93-x, y-.93, height-.93-y)
    radius2 = 0.
    for polygon in polygons:
        for vertex in polygon:
            radius2 = max(radius2, vertex[0]**2+vertex[1]**2)
    bound = math.sqrt(radius2)
    for obstacle in obstacles:
        if stats is not None:
            stats[1] += 1  # obstacle broad-phase probes
        dx, dy = obstacle[0]-x, obstacle[1]-y
        reach = best+obstacle[2]+bound
        if reach > 0. and dx*dx+dy*dy > reach*reach:
            continue
        if stats is not None:
            stats[2] += 1  # polygon-circle narrow-phase pairs
        px, py = dx*c+dy*s, -dx*s+dy*c
        for polygon in polygons:
            distance2 = math.inf
            inside = False
            for i in range(len(polygon)):
                if stats is not None:
                    stats[3] += 1  # segment distance and containment edges
                a, b = polygon[i], polygon[(i+1)%len(polygon)]
                ex, ey = b[0]-a[0], b[1]-a[1]
                length2 = ex*ex+ey*ey
                fraction = (max(0., min(1., ((px-a[0])*ex+(py-a[1])*ey)/length2))
                            if length2 > 0. else 0.)
                distance2 = min(distance2, (px-a[0]-fraction*ex)**2+
                                          (py-a[1]-fraction*ey)**2)
                if (a[1] > py) != (b[1] > py):
                    if px < a[0]+(py-a[1])*ex/ey:
                        inside = not inside
            distance = math.sqrt(distance2)
            best = min(best, (-distance if inside else distance)-obstacle[2])
    return best


surface_clearance_at_pose = (njit(cache=True)(_surface_clearance_at_pose)
                             if njit is not None else _surface_clearance_at_pose)


def prepare_hull_edges(polygons):
    """Cache the unchanged local polygon edges once per controller instance."""
    edges = []
    for polygon, count in zip(polygons, (8, 8, 4)):
        for index in range(count):
            start, end = polygon[index], polygon[(index+1) % count]
            dx, dy = end-start
            length2 = dx*dx+dy*dy
            edges.append((start[0], start[1], dx, dy,
                          1./length2 if length2 else 0.))
    bound = float(np.sqrt(np.max(np.sum(polygons.reshape(-1, 2)**2, axis=1))))
    vertices = polygons.reshape(-1, 2)
    box = np.array([vertices[:, 0].min(), vertices[:, 0].max(),
                    vertices[:, 1].min(), vertices[:, 1].max()])
    return np.ascontiguousarray(edges, dtype=np.float64), bound, box


def _fast_surface_clearance_at_pose(x, y, heading, obstacles, width, height,
                                    edges, hull_bound, hull_box, previous_min=math.inf,
                                    stats=None, exact_walls=False):
    """The actual hull's enclosing oriented box safely prunes exact pairs."""
    if stats is not None:
        stats[0] += 1
    c, s = math.cos(heading), math.sin(heading)
    best = min(previous_min, x-.93, width-.93-x, y-.93, height-.93-y)
    if exact_walls:
        best=min(previous_min,x-.84,width-.84-x,y-.54,height-.54-y)
        for k in range(len(edges)):
            vx=x+c*edges[k,0]-s*edges[k,1]
            vy=y+s*edges[k,0]+c*edges[k,1]
            best=min(best,vx,width-vx,vy,height-vy)
    for j in range(len(obstacles)):
        if stats is not None:
            stats[1] += 1
        dx, dy = obstacles[j, 0]-x, obstacles[j, 1]-y
        radius = obstacles[j, 2]
        reach = best+radius+hull_bound
        if reach > 0. and dx*dx+dy*dy > reach*reach:
            continue
        px, py = dx*c+dy*s, -dx*s+dy*c
        outside_x = max(hull_box[0]-px, px-hull_box[1], 0.)
        outside_y = max(hull_box[2]-py, py-hull_box[3], 0.)
        if outside_x > 0. or outside_y > 0.:
            box_distance = math.hypot(outside_x, outside_y)
        else:
            box_distance = -min(px-hull_box[0], hull_box[1]-px,
                                py-hull_box[2], hull_box[3]-py)
        if box_distance-radius >= best:
            continue
        if stats is not None:
            stats[2] += 1
        for polygon in range(3):
            distance2 = math.inf
            inside = True  # all three physical polygons are convex, CCW
            first, last = ((0, 8) if polygon == 0 else
                           (8, 16) if polygon == 1 else (16, 20))
            for k in range(first, last):
                if stats is not None:
                    stats[3] += 1
                ax, ay = edges[k, 0], edges[k, 1]
                ex, ey, inverse_length2 = edges[k, 2], edges[k, 3], edges[k, 4]
                rx, ry = px-ax, py-ay
                fraction = max(0., min(1., (rx*ex+ry*ey)*inverse_length2))
                vx, vy = rx-fraction*ex, ry-fraction*ey
                distance2 = min(distance2, vx*vx+vy*vy)
                if ex*ry-ey*rx < 0.:
                    inside = False
            distance = math.sqrt(distance2)
            best = min(best, (-distance if inside else distance)-radius)
    return best


fast_surface_clearance_at_pose = (njit(cache=True)(_fast_surface_clearance_at_pose)
                                  if njit is not None else _fast_surface_clearance_at_pose)


def _fast_surface_clearances(states, obstacles, width, height, edges, hull_bound,
                             hull_box, exact_walls=False):
    result = np.empty(len(states), dtype=np.float64)
    for i in range(len(states)):
        result[i] = fast_surface_clearance_at_pose(
            states[i, 0], states[i, 1], states[i, 2], obstacles,
            width, height, edges, hull_bound, hull_box, math.inf, None, exact_walls)
    return result


fast_surface_clearances = (njit(cache=True)(_fast_surface_clearances)
                           if njit is not None else _fast_surface_clearances)
