"""Real-boat adapter for the upstream CODEX SamplingNavigator.

Reuse its sampled sequences, warm start, route scoring and proposal recheck.
Replace force dynamics/allocation with the frozen measured response model.
No simulated observations, world objects, pixel coordinates or true state.
"""
from collections import deque
from dataclasses import replace
import math
import time
import numpy as np
from .outdoor_model import ResponseModel
from .codex_nav.experiments.sampling_navigation import SamplingConfig, SamplingNavigator
from .codex_nav.vessel_dynamics import VesselParameters
from .codex_nav.navigation_map import NavigationMap
from .codex_nav.control_path import smooth_path
from .outdoor_obstacles import ObstacleTracker, compact_obstacles
from .outdoor_test_course import polygon, hull_clearance, fence


class LearnedNavigator(SamplingNavigator):
    def __init__(self, model, config, cap):
        self.model, self.boat, self.cap = model, config, cap
        self.history = np.zeros((model.delay, 2))
        p = VesselParameters()
        # Upstream proposal units remain desired m/s and rad/s. Its force model
        # is never called: every candidate and the final output use inverse().
        from dataclasses import replace
        p = replace(p, mass_kg=config['mass_kg'], thruster_arm_m=config['thruster_spacing_m']/2,
                    cruise_speed_m_s=min(.5, max(.1, model.data['velocity_limit'][0])),
                    max_yaw_rate_rad_s=min(.5, max(.1, model.data['velocity_limit'][2])))
        uncertainty = model.data.get('metrics', {}).get('horizons', {}).get('2', {}).get('integrated_position_rmse_m', 0.)
        cfg = SamplingConfig(samples=64, horizon_steps=max(8, round(3/model.dt)),
            noise_speed=.2, noise_yaw=.15, margin=config['obstacle_margin_m']+max(0., float(uncertainty)),
            symmetric_yaw_bias=.12)
        super().__init__(p, dt=model.dt/3, mode='trajectory_control', config=cfg)
        self.inverse_matrix = np.linalg.pinv(model.coefficients[3:5][:, [0, 2]])
        self.dynamic = np.empty((0, 6))
        self.origin = np.zeros(2)
        self.route_world = None
        self.raw_route_world = None
        self.route_stamp = -np.inf
        self.route_goal = None
        self.now = 0.
        self.recovery = False
        self.normal_margin = cfg.margin; self.base_margin=cfg.margin; self.test_boundary=None

    def guidance(self, state, observation, goal, frame):
        # Cache in ENU, never in the sliding local-map coordinate system.
        local = None if self.route_world is None else self.route_world-self.origin
        goal_world = goal+self.origin
        margin = self.cfg.margin+.04
        if (local is not None and self.now-self.route_stamp < .6 and
            np.linalg.norm(goal_world-self.route_goal)<.5 and
            np.all(local>.9) and np.all(local<39.1) and
            np.all(observation.clearance(local)>=margin)):
            local = local.copy(); local[0] = state[:2]
            # Revalidate the connector after ownship moved.
            connector = np.linspace(local[0],local[1],max(2,int(np.linalg.norm(local[1]-local[0])/.05)+1))
            if np.all(observation.clearance(connector)>=margin):
                return local
        self.path = None
        raw = super().guidance(state, observation, goal, frame)
        if raw is None:
            self.raw_route_world=None; self.route_world=None
            return None
        self.raw_route_world=raw+self.origin
        def safe(points):
            # Bound the large point-by-obstacle temporary arrays.
            return all(np.all(observation.clearance(chunk)>=margin)
                       for chunk in np.array_split(points,max(1,math.ceil(len(points)/128))))
        rounded = smooth_path(raw, safe)
        path = raw if rounded is None else rounded
        # Dense curve samples were checked above; score at ~0.15 m spacing.
        if len(path)>2:
            arc = np.r_[0.,np.cumsum(np.linalg.norm(np.diff(path,axis=0),axis=1))]
            indices = np.unique(np.r_[np.searchsorted(arc,np.arange(0,arc[-1],.15)),len(path)-1])
            path = path[indices]
        self.route_world = path+self.origin; self.route_stamp = self.now
        self.route_goal = goal_world
        return path

    def inverse(self, state, desired):
        state, desired = np.atleast_2d(state), np.atleast_2d(desired)
        coefficients = self.model.coefficients
        base = state@coefficients[:3]+coefficients[7]
        # One-second approach to requested speed/rate rather than an impulsive
        # one-step inverse. Empirical command bounds prohibit extrapolation.
        target = state[:, [0, 2]]+self.model.dt*(desired-state[:, [0, 2]])
        q = np.zeros((len(state), 2))
        cap = min(self.cap,.2) if self.recovery else self.cap
        lo = np.maximum(-cap, self.model.data['command_min'])
        hi = np.minimum(cap, self.model.data['command_max'])
        for _ in range(3):
            nonlinear = (q*np.abs(q))@coefficients[5:7]
            q = (target-base[:, [0, 2]]-nonlinear[:, [0, 2]])@self.inverse_matrix
            q = np.clip(q, lo, hi)
        return q

    def safety_clearance(self, states, obstacles, width, height):
        z = np.atleast_2d(states)
        boundary=(hull_clearance(z,self.test_boundary,self.boat['length_m'],self.boat['width_m'])-max(0.,self.normal_margin-self.cfg.margin)
                  if self.test_boundary is not None else np.full(len(z),np.inf))
        if not len(obstacles): return boundary
        # Oriented full-hull rectangle; obstacles are surface discs. Substeps
        # inflate by distance travelled so samples cover the swept motion.
        delta = obstacles[None, :, :2]-z[:, None, :2]
        c, s = np.cos(z[:, 2, None]), np.sin(z[:, 2, None])
        x = delta[:, :, 0]*c+delta[:, :, 1]*s
        y = -delta[:, :, 0]*s+delta[:, :, 1]*c
        dx = np.maximum(abs(x)-self.boat['length_m']/2, 0)
        dy = np.maximum(abs(y)-self.boat['width_m']/2, 0)
        return np.minimum(boundary,np.min(np.hypot(dx, dy)-obstacles[None, :, 2], axis=1))

    def rollout(self, initial, sequences, obstacles, width, height, goal=None):
        n, horizon, _ = sequences.shape
        z = np.repeat(np.asarray(initial)[None, :], n, axis=0)
        result = np.empty((n, horizon, 8)); clearance = self.safety_clearance(z, obstacles, width, height)
        clearance = np.minimum(clearance,self.moving_clearance(z,0.))
        queue = deque([np.tile(q, (n, 1)) for q in self.history])
        for i in range(horizon):
            command = self.inverse(z[:, 3:6], sequences[:, i]); queue.append(command)
            delayed = queue.popleft()
            previous = z.copy(); z[:, 3:6] = self.model.step(z[:, 3:6], delayed)
            mean = .5*(z[:, 3:6]+previous[:, 3:6]); yaw = z[:, 2]+mean[:, 2]*self.model.dt/2
            z[:, 0] += self.model.dt*(np.cos(yaw)*mean[:, 0]-np.sin(yaw)*mean[:, 1])
            z[:, 1] += self.model.dt*(np.sin(yaw)*mean[:, 0]+np.cos(yaw)*mean[:, 1])
            z[:, 2] += mean[:, 2]*self.model.dt; z[:, 6:] = command
            swept = np.linalg.norm(z[:, :2]-previous[:, :2], axis=1)/2
            swept += abs(z[:, 2]-previous[:, 2])*math.hypot(self.boat['length_m'], self.boat['width_m'])/4
            for probe, elapsed in ((z,(i+1)*self.model.dt), (.5*(z+previous),(i+.5)*self.model.dt)):
                clearance = np.minimum(clearance, self.safety_clearance(probe, obstacles, width, height)-swept)
                clearance = np.minimum(clearance,self.moving_clearance(probe,elapsed)-swept)
            result[:, i] = z
        return result, clearance

    def moving_clearance(self, states, elapsed):
        if not len(self.dynamic): return np.full(len(states), np.inf)
        future = self.dynamic[:,:3].copy()
        future[:,:2] += self.dynamic[:,3:5]*elapsed
        future[:,2] += self.dynamic[:,5]*elapsed
        # Covers relative obstacle motion between the midpoint probes too.
        future[:,2] += np.linalg.norm(self.dynamic[:,3:5],axis=1)*self.model.dt/2
        clearance = self.safety_clearance(states,future,40,40)
        # Recovery can reduce static-wall clearance, never moving-track clearance.
        return clearance-max(0.,self.normal_margin-self.cfg.margin)


class OutdoorPlanner:
    def __init__(self, model_data, config, percent):
        if model_data.get('synthetic'):
            raise ValueError('가상 시험 모델은 실선 주행에 사용할 수 없습니다')
        self.model = ResponseModel(model_data); self.config = config; self.cap = percent/100
        if not model_data.get('qualified'): raise ValueError('분리 검증을 통과한 모델을 선택하세요')
        if not 0 < self.cap <= 1: raise ValueError('출력 범위 오류')
        if any(x > -.01 for x in model_data['command_min']) or any(x < self.cap for x in model_data['command_max']):
            raise ValueError('선택한 출력·좌우 역추진을 포함한 RC 기록이 필요합니다')
        self.navigator = LearnedNavigator(self.model, config, self.cap)
        self.frame = 0
        self.tracker = ObstacleTracker()

    def plan(self, state, scan, command_history, now_ros):
        started = time.perf_counter()
        origin = np.array(state['position'])-20.
        initial = np.r_[20., 20., state['yaw'], state['velocity'], 0., 0.]
        goal = np.clip(np.array(state['goal'])-origin, 3., 37.)
        angles = np.asarray(scan['angles']); distances = np.asarray(scan['ranges'])
        if len(angles) < 180 or len(distances) != len(angles): raise ValueError('360도 라이다 수신 대기')
        # Preserve invalid/no-return distinction; invalid beams mark no free area.
        source_range = float(scan.get('range_max',20.))
        if not np.isfinite(source_range) or source_range<=0: source_range=20.
        configured = float(self.config['scan_range_m'])
        max_range = min(source_range,configured) if configured>0 else source_range
        distances = np.where(np.isposinf(distances), max_range, distances)
        distances = np.where(np.isfinite(distances) & (distances > scan['range_min']), distances, 0.)
        distances = np.minimum(distances, max_range)
        if np.count_nonzero(distances > 0) < .8*len(distances): raise ValueError('라이다 유효 관측 범위 부족')
        # Latest scan is the only obstacle source; no guessed pool perimeter.
        observation = NavigationMap(40, 40, resolution=.4)
        scan_pose = scan.get('pose', state)
        yaw = scan_pose['yaw']; lx, ly = self.config['lidar_lever_arm_m']
        lidar_position = np.array(scan_pose['position'])-origin+np.array([math.cos(yaw)*lx-math.sin(yaw)*ly, math.sin(yaw)*lx+math.cos(yaw)*ly])
        observation.observe(lidar_position, yaw+math.radians(self.config['lidar_yaw_deg']),
                            angles, np.where(distances>0,distances,max_range), max_range, now_ros)
        # Missing beams are unknown, not fictitious hits at the laser origin,
        # nor a free corridor out to the maximum range.
        relative = (np.arctan2(observation.gy-lidar_position[1],observation.gx-lidar_position[0])
                    -yaw-math.radians(self.config['lidar_yaw_deg'])-angles[0])%(2*math.pi)
        beam = np.rint(relative/(2*math.pi/len(angles))).astype(int)%len(angles)
        observation.free_seen[distances[beam]<=0] = -np.inf
        # Retain every measured surface. The upstream sequential track merger
        # can slide along a densely sampled wall and erase its earlier points.
        hit = (distances>0)&(distances<max_range-1e-4)
        bearing = angles[hit]+yaw+math.radians(self.config['lidar_yaw_deg'])
        surfaces = lidar_position+distances[hit,None]*np.c_[np.cos(bearing),np.sin(bearing)]
        surfaces = surfaces[np.all((surfaces>=0)&(surfaces<=40),axis=1)]
        raw_discs = np.c_[surfaces,np.full(len(surfaces),.06)]
        observation.obstacles = compact_obstacles(np.concatenate([observation.obstacles,raw_discs]))
        self.tracker.update(angles,distances,lidar_position+origin,
                            yaw+math.radians(self.config['lidar_yaw_deg']),
                            float(scan.get('stamp',now_ros)),max_range)
        self.navigator.dynamic, tracks = self.tracker.prediction(now_ros,origin)
        self.navigator.test_boundary=None;self.navigator.normal_margin=self.navigator.base_margin
        if state.get('test_area') is not None:
            boundary=polygon(state['test_area'])-origin
            virtual=np.asarray(state.get('virtual_obstacles',[]),dtype=float).reshape(-1,3)
            if len(virtual)>8 or not np.isfinite(virtual).all() or np.any(virtual[:,2]<=0):raise ValueError('가상 장애물 설정 오류')
            virtual=virtual.copy();virtual[:,:2]-=origin
            self.navigator.test_boundary=boundary
            self.navigator.normal_margin+=2*float(state['position_sigma_m'])
            observation.obstacles=np.concatenate([observation.obstacles,virtual,fence(boundary)])
            # Never replace sensor returns with the virtual course.
        self.navigator.origin = origin; self.navigator.now = now_ros
        self.navigator.recovery = False
        self.navigator.cfg = replace(self.navigator.cfg,margin=self.navigator.normal_margin)
        # Local map is rebuilt around current position; invalidate old route coordinates.
        self.navigator.path = None
        if self.model.delay:
            h = np.asarray(command_history); times = now_ros+np.arange(-self.model.delay, 0)*self.model.dt
            ids = np.searchsorted(h[:, 0], times, side='right')-1
            if np.any(ids < 0) or np.any(times-h[np.maximum(ids, 0), 0] > .35): raise ValueError('최신 추진 명령 이력 대기')
            self.navigator.history = h[ids, 1:]
        command, prediction, _, clearance = self.navigator.plan(initial, observation, goal, self.frame)
        self.frame += 1
        recovery = False
        # Try the same CODEX candidates at low output with reduced *static*
        # clearance before stopping. Never execute its unsafe maximum-clearance
        # fallback; actual hull overlap and moving tracks remain prohibited.
        if clearance < self.navigator.cfg.margin and time.perf_counter()-started < self.config['planner_deadline_s']*.5:
            recovery = True; self.navigator.recovery = True
            self.navigator.cfg = replace(self.navigator.cfg,margin=max(.1,float(self.config.get('recovery_margin_m',.1))))
            self.navigator.route_world = None
            command, prediction, _, clearance = self.navigator.plan(initial,observation,goal,self.frame)
        for values in self.navigator.timings.values(): del values[:-50]
        if not np.isfinite(prediction).all() or clearance < self.navigator.cfg.margin:
            raise ValueError('현재 관측에서 선체가 통과할 경로 없음')
        # Route scoring may explore unknown cells. Before execution require all
        # predicted hull corners to lie inside current measured free rays.
        for sx in (-.5, .5):
            for sy in (-.5, .5):
                a = prediction[:, 2]
                xx = prediction[:, 0]+np.cos(a)*sx*self.config['length_m']-np.sin(a)*sy*self.config['width_m']-lidar_position[0]
                yy = prediction[:, 1]+np.sin(a)*sx*self.config['length_m']+np.cos(a)*sy*self.config['width_m']-lidar_position[1]
                bearing = (np.arctan2(yy, xx)-yaw-math.radians(self.config['lidar_yaw_deg'])+math.pi)%(2*math.pi)
                bins = np.rint(bearing/(2*math.pi)*len(distances)).astype(int)%len(distances)
                if np.any(np.hypot(xx, yy)+self.navigator.cfg.margin > distances[bins]):
                    raise ValueError('예측 경로의 라이다 관측 범위 부족')
        q = self.navigator.inverse(np.array(state['velocity']), command)[0]
        path = prediction[:, :2]+origin
        return {'command': q.tolist(), 'path': path.tolist(), 'clearance_m': float(clearance),
                'route':(self.navigator.route_world.tolist() if self.navigator.route_world is not None else []),
                'raw_route':(self.navigator.raw_route_world.tolist() if self.navigator.raw_route_world is not None else []),
                'obstacles':tracks, 'recovery':recovery,
                'planning_ms':(time.perf_counter()-started)*1000,
                'scan_range_m':max_range, 'obstacle_discs':len(observation.obstacles),
                'plan_stamp':now_ros,'dt_s':self.model.dt,
                'planner': 'CODEX SamplingNavigator + learned PWM response'}
