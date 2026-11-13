"""Bounded tank manoeuvres and conservative map/scan rollout screening (SI)."""
import math
from collections import deque
import numpy as np
from .tank_core import guidance, wrap


class MotionVelocity:
    """Estimate body velocity from received odometry, independent of map corrections."""
    def __init__(self):
        self.samples=deque(maxlen=32);self.received=None

    def add(self,stamp,x,y,heading,received):
        if not all(math.isfinite(v) for v in (stamp,x,y,heading,received)):return
        sample=(stamp,x,y,heading)
        if self.samples:
            old=self.samples[-1];dt=stamp-old[0]
            if dt==0:return  # Repeated data cannot refresh a stale estimate.
            if dt<0 or dt>.8 or math.hypot(x-old[1],y-old[2])>max(.3,2.*dt) or abs(wrap(heading-old[3]))>max(.35,2.*dt):
                self.samples.clear()
        self.samples.append(sample);self.received=received
        while len(self.samples)>2 and stamp-self.samples[0][0]>.8:self.samples.popleft()

    def estimate(self,now,ros_now):
        if len(self.samples)<2 or self.received is None or now-self.received>.6:return None
        a,b=self.samples[0],self.samples[-1];dt=b[0]-a[0]
        if dt<.08 or not -.1<=ros_now-b[0]<.6:return None
        vx,vy=(b[1]-a[1])/dt,(b[2]-a[2])/dt;co,si=math.cos(b[3]),math.sin(b[3])
        return (vx*co+vy*si,-vx*si+vy*co,wrap(b[3]-a[3])/dt)


class DepartureRest:
    """Require confirmed neutral and fresh, continuously stationary observations."""
    def __init__(self, minimum=3., stable=1., speed=.08, rate=math.radians(5.), enabled=True):
        self.minimum=minimum; self.stable=stable; self.speed=speed; self.rate=rate;self.enabled=enabled
        self.begin()

    def begin(self):
        self.active=self.enabled; self.interrupt()

    def interrupt(self):
        self.neutral_since=None; self.stable_since=None; self.last_stamp=None; self.last_seen=None; self.samples=[]

    def update(self, now, stamps, velocity, gyro, quaternion, neutral):
        if not neutral:
            self.interrupt(); return None
        if self.last_seen is not None and now-self.last_seen>.5: self.interrupt()
        if self.neutral_since is None: self.neutral_since=now
        if velocity is None or not all(math.isfinite(x) for x in (*stamps,*velocity,*gyro,*quaternion)):
            self.interrupt(); return None
        if self.last_stamp is not None:
            if any(b<a for a,b in zip(self.last_stamp,stamps)):
                self.interrupt(); return None
            if any(b==a for a,b in zip(self.last_stamp,stamps)): return None
        self.last_stamp=stamps; self.last_seen=now
        stationary=math.hypot(*velocity[:2])<=self.speed and abs(velocity[2])<=self.rate and math.sqrt(sum(g*g for g in gyro))<=self.rate
        norm=math.sqrt(sum(q*q for q in quaternion))
        if not stationary or not .9<norm<1.1:
            self.stable_since=None; self.samples=[]; return None
        if self.stable_since is None: self.stable_since=now
        self.samples.append((now,tuple(q/norm for q in quaternion),tuple(gyro),tuple(velocity)))
        self.samples=[s for s in self.samples if now-s[0]<=self.stable+.5]
        if now-self.neutral_since<self.minimum or now-self.stable_since<self.stable or len(self.samples)<max(5,math.ceil(self.stable*5)): return None
        reference=self.samples[0][1]; quats=[q if sum(a*b for a,b in zip(q,reference))>=0 else tuple(-v for v in q) for _,q,_,_ in self.samples]
        mean=[sum(q[i] for q in quats)/len(quats) for i in range(4)]
        norm=math.sqrt(sum(v*v for v in mean)); mean=[v/norm for v in mean]
        self.active=False
        return {'quaternion_xyzw':mean,'gyro_mean_rad_s':[sum(s[2][i] for s in self.samples)/len(self.samples) for i in range(3)],
                'velocity_body_mean':[sum(s[3][i] for s in self.samples)/len(self.samples) for i in range(3)],
                'limits':{'speed_m_s':self.speed,'rate_rad_s':self.rate,'minimum_neutral_s':self.minimum,'stable_s':self.stable},
                'samples':len(self.samples),'neutral_s':now-self.neutral_since,'stable_s':now-self.stable_since}


def excitation(t, cruise=20.):
    """Repeatable input schedule; enabled only in open water by select_command."""
    phase = int(t // 6) % 6
    turn = 5. * math.sin(2 * math.pi * (t % 6) / 6)
    return [(cruise, 0., 'cruise'), (12., 0., 'step_low'),
            (cruise-abs(turn), turn, 'weave'), (0., 0., 'coast'),
            (cruise-abs(turn), -turn, 'weave_reverse'), (cruise, 0., 'step_high')][phase]


def scan_points(scan, position, heading, forward,open_water=False):
    """Require returns around the hull; unknown sectors inhibit autonomous motion."""
    sectors = set(); points = []
    for i, distance in enumerate(scan.ranges):
        if open_water and math.isinf(distance) and distance>0:distance=min(10.,scan.range_max)
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


def _checked_command(grid, scan, position, heading, velocity, target, goal, forward,
                   elapsed, cap=20., radius=.75, excite=True, margin_recovery=False, reverse_only=False,open_water=False):
    if not 0 < cap <= (100 if open_water else 20): raise ValueError('Invalid docking output cap')
    if velocity is None or not all(math.isfinite(v) for v in velocity):
        return (0.,0.), {'phase':'HOLD','reason':'연속 속도 추정 대기','points_m':[]}
    u,v,r = velocity
    if math.hypot(u,v)>.6 or abs(r)>1.2:
        return (0.,0.), {'phase':'HOLD','reason':'시험 속도 범위 초과','points_m':[]}
    try: obstacles = scan_points(scan,position,heading,forward,open_water=open_water)
    except ValueError as exc:
        return (0.,0.), {'phase':'HOLD','reason':str(exc),'points_m':[]}
    # Allow reaction/communication latency plus 3 s of unbraked motion. This is
    # a provisional envelope, not proof of stopping distance in arbitrary water.
    vx,vy = u*math.cos(heading)-v*math.sin(heading), u*math.sin(heading)+v*math.cos(heading)
    coast = [(position[0]+s*vx,position[1]+s*vy) for s in np.linspace(0,3.5,25)]
    coast_safe = trajectory_clear(grid,coast,obstacles,radius)
    recovery = not coast_safe or margin_recovery
    if recovery and (math.hypot(u,v)>.15 or abs(r)>.4):
        return (0.,0.), {'phase':'HOLD','reason':'타력 이동 여유 부족','points_m':coast}
    desired = guidance(position,heading,target,cap)
    if reverse_only:
        error=wrap(math.atan2(target[1]-position[1],target[0]-position[0])-heading-math.pi)
        turn=cap*max(-1.,min(1.,error/.8))
        surge=-cap*max(0.,math.cos(error))
        desired=(max(-cap,min(0.,surge-turn)),max(-cap,min(0.,surge+turn)))
    if math.dist(position,goal)<1.5 and abs(wrap(math.atan2(target[1]-position[1],target[0]-position[0])-heading))<.5:
        scale = max(.2,math.dist(position,goal)/1.5)
        desired = tuple(v*scale for v in desired)
    error = wrap(math.atan2(target[1]-position[1],target[0]-position[0])-heading)
    nearest = float(np.min(np.linalg.norm(obstacles-np.asarray(position),axis=1)))
    phase = 'transit'
    if excite and not reverse_only and not recovery and nearest > radius+1.2 and math.dist(position,goal)>2. and abs(error)<.18:
        base,turn,phase = excitation(elapsed,cap)
        base = min(cap,max(0.,base)); turn += cap*error/.8
        desired = (max(-cap,min(cap,base-turn)),max(-cap,min(cap,base+turn)))
    candidates = [(desired,phase)]
    # Tracking/avoidance takes priority over excitation. Keep alternatives bounded.
    for turn in (0., -cap*.5, cap*.5, -cap, cap):
        for base in ((-cap*.5,0.) if reverse_only else (cap*.5,0.)):
            scale = max(1.,abs(base-turn)/cap,abs(base+turn)/cap)
            candidates.append((((base-turn)/scale,(base+turn)/scale),'avoidance'))
    if recovery:
        # At low speed, actively leave a threatened coast trajectory instead of
        # deadlocking at neutral. Every candidate still clears map and live scan.
        limit=min(cap,10.)
        candidates=[((left,right),'clearance_recovery') for left,right in (
            (-limit,-limit),(limit,limit),(-limit,limit),(limit,-limit),
            (-limit,0.),(0.,-limit),(limit,0.),(0.,limit))]
    best = None
    for order,(command,label) in enumerate(candidates):
        if reverse_only and max(command)>1e-9:continue
        points = rollout(position,heading,velocity,command,cap)
        if not trajectory_clear(grid,points,obstacles,radius): continue
        score = math.dist(points[-1],target) + .01*(abs(command[0]-desired[0])+abs(command[1]-desired[1]))
        if order==0: score -= .25
        if best is None or score<best[0]: best = (score,command,label,points)
    if best is None:
        return (0.,0.), {'phase':'HOLD','reason':'충돌 여유를 만족하는 조향 없음','points_m':coast}
    return best[1], {'phase':best[2], 'reason':'', 'points_m':best[3],
                     'nearest_m':nearest,'coast_clear':coast_safe,'kind':'provisional_safety_rollout','seconds':3.}


def select_command(grid, scan, position, heading, velocity, target, goal, forward,
                   elapsed, cap=20., radius=.75, excite=True, margin_recovery=False,
                   reverse_only=False, open_water=False, buoy_course=False):
    if buoy_course and not open_water:
        raise ValueError('Buoy contact mode requires the open-water course')
    command,result=_checked_command(grid,scan,position,heading,velocity,target,goal,forward,
        elapsed,cap,radius,excite,margin_recovery,reverse_only,open_water)
    if not buoy_course or (not result.get('reason') and result.get('phase')!='clearance_recovery'
                           and max(abs(v) for v in command)>1e-9):
        return command,result
    from .outdoor_follow import buoy_goal_command
    state={'position':position,'yaw':heading,'velocity':velocity}
    command,_,error=buoy_goal_command(state,goal,cap,reverse_only=reverse_only)
    return tuple(value*100 for value in command),dict(phase='buoy_goal',reason='',points_m=[],
        kind='buoy_goal_fallback',heading_error_deg=error,
        obstacle_advisory=result.get('reason') or result.get('phase','zero_candidate'))
