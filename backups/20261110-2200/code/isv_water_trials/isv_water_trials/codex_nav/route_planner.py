"""Scan-map A* with gap corridor preference and validated smooth control path.

Unknown cells carry explicit additional cost; map boundaries are a surveyed prior.
All buoy geometry comes from perception, never the simulation obstacle list.
"""
import heapq
import math
import numpy as np
from isv_water_trials.codex_nav.control_path import smooth_path, lookahead, path_geometry
from isv_water_trials.codex_nav.fast_astar import compiled_astar


def route_target(env):
    scale = env.dynamics.pixels_per_m
    start = env.boat_pos/scale
    cfg = env.control
    perception = env.navigation_map
    width, height = env.map_w/scale, env.sim_h/scale
    def safe(points):
        points=np.asarray(points)
        wall=np.minimum.reduce([points[:,0]-.54,width-.54-points[:,0],points[:,1]-.54,height-.54-points[:,1]])
        return bool(np.all(wall>=cfg.safety_margin_m+.025) and np.all(perception.clearance(points)>=cfg.safety_margin_m+.025))
    wp = getattr(env, 'current_wp', None)
    selected = None
    if wp is not None:
        gap=np.asarray(wp['pos'])/scale
        rel=gap-start
        # Gap is an optional safe preferred corridor, never an unconditional gate.
        if .8 < np.linalg.norm(rel) < env.lidar_range/scale and np.dot(rel,env.target/scale-start)>0 and safe(gap[None,:]):
            selected=gap
    env.selected_gap = selected
    gap_key = None if selected is None else tuple(wp['pair'])
    old=getattr(env,'control_path',None)
    geometry=getattr(env,'path_geometry',None)
    if old is not None and geometry is None:
        geometry=path_geometry(old)
        env.path_geometry=geometry
    due=env.frame-getattr(env,'route_plan_frame',-1000)>=24
    unsafe=old is not None and not safe(old)
    changed=gap_key!=getattr(env,'route_gap_key',None)
    if old is None or due or unsafe or changed:
        # A safe remaining route is retained for up to 2.88 s when its corridor
        # choice is unchanged. New occupied evidence invalidates it immediately.
        retain=False
        if old is not None and not unsafe and not changed and env.frame-getattr(env,'route_created_frame',0)<72:
            _, progress=lookahead(old,start,cfg.lookahead_m,getattr(env,'path_progress',0.),geometry)
            remain=np.linalg.norm(np.diff(old,axis=0),axis=1).sum()-progress
            retain=remain>cfg.lookahead_m
        if retain:
            env.route_plan_frame=env.frame
        else:
            return build_route(env, start, selected, gap_key, safe)
    if old is None:return None
    target,env.path_progress=lookahead(old,start,cfg.lookahead_m,getattr(env,'path_progress',0.),geometry)
    env.pursuit_target=target*scale
    return target


def build_route(env,start,selected,gap_key,safe):
        cfg=env.control;scale=env.dynamics.pixels_per_m
        perception=env.navigation_map
        xs,ys=perception.xs,perception.ys
        gx,gy=perception.gx,perception.gy
        resolution=perception.resolution
        known_free=perception.known_free
        width,height=env.map_w/scale,env.sim_h/scale
        clearance=np.minimum.reduce([gx-.54,width-.54-gx,gy-.54,height-.54-gy])
        for x,y,radius in perception.obstacles:
            clearance=np.minimum(clearance,np.hypot(gx-x,gy-y)-radius-.54)
        free = clearance >= cfg.safety_margin_m+.04
        # The destination-cell penalty is invariant throughout this A* search.
        # Compute it once instead of constructing NumPy vectors for every edge.
        corridor_cost = np.zeros_like(clearance)
        if selected is not None:
            direction = selected-start
            distance = np.linalg.norm(direction)
            offset_x = gx-start[0]
            offset_y = gy-start[1]
            along = (offset_x*direction[0]+offset_y*direction[1])/distance
            lateral = np.abs(offset_x*direction[1]-offset_y*direction[0])/distance
            corridor_cost = np.where((along>0)&(along<distance), .35*np.minimum(lateral,2.), 0.)
        cell_weight = 1.+np.where(known_free,0.,1.5)+corridor_cost+.15/np.maximum(clearance,.1)
        start_idx = (int(np.argmin(abs(ys-start[1]))),int(np.argmin(abs(xs-start[0]))))
        goal_idx = (int(np.argmin(abs(ys-env.target[1]/scale))),int(np.argmin(abs(xs-env.target[0]/scale))))
        path_nodes = a_star(free,cell_weight,xs,ys,start_idx,goal_idx,resolution)
        env.route_plan_frame=env.frame
        env.route_created_frame=env.frame
        env.route_gap_key=gap_key
        env.raw_route=path_nodes
        if env.raw_route is not None:
            # Do not invent a clearance violation by snapping the vessel to a cell center.
            env.raw_route[0]=start
        env.control_path=None if env.raw_route is None else smooth_path(env.raw_route,safe)
        env.path_geometry=None if env.control_path is None else path_geometry(env.control_path)
        env.path_progress=0.
        if env.control_path is None:
            env.pursuit_target=None
            return None
        target,env.path_progress=lookahead(env.control_path,start,cfg.lookahead_m,geometry=env.path_geometry)
        env.pursuit_target=target*scale
        return target


def a_star(free,cell_weight,xs,ys,start_idx,goal_idx,resolution):
    """Same eight-neighbor weighted search, with per-search invariant costs."""
    if compiled_astar is not None:
        free[start_idx] = True
        path = compiled_astar(free,cell_weight,xs,ys,start_idx[0],start_idx[1],
                              goal_idx[0],goal_idx[1],resolution)
        return path if len(path) else None
    return a_star_reference(free,cell_weight,xs,ys,start_idx,goal_idx,resolution)


def a_star_reference(free,cell_weight,xs,ys,start_idx,goal_idx,resolution):
    """Unchanged Python search retained for path-by-path equivalence checks."""
    # Permit leaving the occupied start cell after a new detection, while
    # the dynamics rollout independently checks every motion.
    free[start_idx] = True
    heap = [(0.,start_idx)]
    ny,nx = free.shape
    costs = [[float('inf')]*nx for _ in range(ny)]
    costs[start_idx[0]][start_idx[1]]=0.
    parent = [[None]*nx for _ in range(ny)]
    visited = [[False]*nx for _ in range(ny)]
    diagonal_step = math.hypot(1,1)*resolution
    neighbors = ((-1,0,resolution),(1,0,resolution),(0,-1,resolution),(0,1,resolution),
                 (-1,-1,diagonal_step),(-1,1,diagonal_step),(1,-1,diagonal_step),(1,1,diagonal_step))
    heuristic = [[math.hypot(goal_idx[0]-y,goal_idx[1]-x)*resolution for x in range(nx)] for y in range(ny)]
    while heap:
        _, node = heapq.heappop(heap)
        y,x = node
        if visited[y][x]:
            continue
        visited[y][x]=True
        if node == goal_idx:
            nodes = [node]
            while parent[node[0]][node[1]] is not None:
                node = parent[node[0]][node[1]]
                nodes.append(node)
            return np.array([[xs[x],ys[y]] for y,x in nodes[::-1]])
        current_cost = costs[y][x]
        for dy,dx,step in neighbors:
            yy,xx = y+dy,x+dx
            if not (0 <= yy < ny and 0 <= xx < nx and free[yy,xx]):
                continue
            if dx and dy and not (free[y,xx] and free[yy,x]):
                continue
            cost = current_cost+step*cell_weight[yy,xx]
            nxt = (yy,xx)
            if cost < costs[yy][xx]:
                costs[yy][xx] = cost
                parent[yy][xx] = node
                heapq.heappush(heap,(cost+heuristic[yy][xx],nxt))
    return None
