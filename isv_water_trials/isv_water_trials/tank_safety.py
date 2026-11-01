"""Bounded tank manoeuvres and conservative map/scan rollout screening (SI)."""
import math
import numpy as np
from .tank_core import guidance, wrap


def excitation(t, cruise=20.):
    """Repeatable input schedule; enabled only in open water by select_command."""
    phase = int(t // 6) % 6
    turn = 5. * math.sin(2 * math.pi * (t % 6) / 6)
    return [(cruise, 0., 'cruise'), (12., 0., 'step_low'),
            (cruise-abs(turn), turn, 'weave'), (0., 0., 'coast'),
            (cruise-abs(turn), -turn, 'weave_reverse'), (cruise, 0., 'step_high')][phase]


def scan_points(scan, position, heading, forward):
    """Require returns around the hull; unknown sectors inhibit autonomous motion."""
    sectors = set(); points = []
    for i, distance in enumerate(scan.ranges):
        if not math.isfinite(distance) or not max(.05, scan.range_min) < distance <= scan.range_max:
            continue
        angle = wrap(scan.angle_min+i*scan.angle_increment-forward)
        sectors.add(int((angle+math.pi)/(2*math.pi)*12) % 12)
        a = heading+angle
        points.append((position[0]+distance*math.cos(a), position[1]+distance*math.sin(a)))
    # Finite wall returns, rather than inf rays, establish coverage in an indoor tank.
    if len(sectors) < 12 or len(points) < 24:
        raise ValueError('라이다 전방·측면·후방 관측 범위 부족')
    return np.asarray(points)


def rollout(position, heading, velocity, command, cap, horizon=3., dt=.15):
    """Screening envelope, NOT a calibrated hydrodynamic model.

    Retains measured lateral drift. Tests both commanded motion and no-response
    coast in select_command; an unvalidated model can never shrink clearance.
    """
    x,y = position; u,v,r = velocity
    desired_u = .45*(command[0]+command[1])/(2*cap)
    desired_r = .65*(command[1]-command[0])/(2*cap)
    points = [(x,y)]
    for _ in range(round(horizon/dt)):
        u += dt*(desired_u-u)/1.5
        r += dt*(desired_r-r)/1.
        heading += r*dt
        x += dt*(u*math.cos(heading)-v*math.sin(heading))
        y += dt*(u*math.sin(heading)+v*math.cos(heading))
        points.append((x,y))
    return points


def trajectory_clear(grid, points, obstacles, radius):
    # Subcell spacing catches thin obstacles between rollout samples.
    dense = []
    for a,b in zip(points[:-1],points[1:]):
        if not grid.line_clear(a,b): return False
        count = max(1,math.ceil(math.dist(a,b)/.04))
        dense.extend((a[0]+(b[0]-a[0])*i/count,a[1]+(b[1]-a[1])*i/count) for i in range(count+1))
    if not dense: dense = points
    for p in dense:
        if np.min(np.sum((obstacles-np.asarray(p))**2,axis=1)) < radius**2: return False
    return True


def select_command(grid, scan, position, heading, velocity, target, goal, forward,
                   elapsed, cap=20., radius=.75, excite=True):
    if not 0 < cap <= 20: raise ValueError('Tank output cap must be <=20%')
    if velocity is None or not all(math.isfinite(v) for v in velocity):
        return (0.,0.), {'phase':'HOLD','reason':'연속 속도 추정 대기','points_m':[]}
    u,v,r = velocity
    if math.hypot(u,v)>.6 or abs(r)>1.2:
        return (0.,0.), {'phase':'HOLD','reason':'시험 속도 범위 초과','points_m':[]}
    try: obstacles = scan_points(scan,position,heading,forward)
    except ValueError as exc:
        return (0.,0.), {'phase':'HOLD','reason':str(exc),'points_m':[]}
    # Allow reaction/communication latency plus 3 s of unbraked motion. This is
    # a provisional envelope, not proof of stopping distance in arbitrary water.
    vx,vy = u*math.cos(heading)-v*math.sin(heading), u*math.sin(heading)+v*math.cos(heading)
    coast = [(position[0]+s*vx,position[1]+s*vy) for s in np.linspace(0,3.5,25)]
    if not trajectory_clear(grid,coast,obstacles,radius+.10):
        return (0.,0.), {'phase':'HOLD','reason':'타력 이동 여유 부족','points_m':coast}
    desired = guidance(position,heading,target,cap)
    if math.dist(position,goal)<1.5 and abs(wrap(math.atan2(target[1]-position[1],target[0]-position[0])-heading))<.5:
        scale = max(.2,math.dist(position,goal)/1.5)
        desired = tuple(v*scale for v in desired)
    error = wrap(math.atan2(target[1]-position[1],target[0]-position[0])-heading)
    nearest = float(np.min(np.linalg.norm(obstacles-np.asarray(position),axis=1)))
    phase = 'transit'
    if excite and nearest > radius+1.2 and math.dist(position,goal)>2. and abs(error)<.18:
        base,turn,phase = excitation(elapsed,cap)
        base = min(cap,max(0.,base)); turn += cap*error/.8
        desired = (max(-cap,min(cap,base-turn)),max(-cap,min(cap,base+turn)))
    candidates = [(desired,phase)]
    # Tracking/avoidance takes priority over excitation. Keep alternatives bounded.
    for turn in (0., -cap*.5, cap*.5, -cap, cap):
        for base in (cap*.5, 0.):
            scale = max(1.,abs(base-turn)/cap,abs(base+turn)/cap)
            candidates.append((((base-turn)/scale,(base+turn)/scale),'avoidance'))
    best = None
    for order,(command,label) in enumerate(candidates):
        points = rollout(position,heading,velocity,command,cap)
        if not trajectory_clear(grid,points,obstacles,radius+.10): continue
        score = math.dist(points[-1],target) + .01*(abs(command[0]-desired[0])+abs(command[1]-desired[1]))
        if order==0: score -= .25
        if best is None or score<best[0]: best = (score,command,label,points)
    if best is None:
        return (0.,0.), {'phase':'HOLD','reason':'충돌 여유를 만족하는 조향 없음','points_m':coast}
    return best[1], {'phase':best[2], 'reason':'', 'points_m':best[3],
                     'nearest_m':nearest,'kind':'provisional_safety_rollout','seconds':3.}
