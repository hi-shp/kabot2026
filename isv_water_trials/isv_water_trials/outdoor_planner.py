"""Real-boat adapter for the upstream CODEX SamplingNavigator.

Reuse its sampled sequences, warm start, route scoring and proposal recheck.
Replace force dynamics/allocation with the frozen measured response model.
No simulated observations, world objects, pixel coordinates or true state.
"""
from collections import deque
from dataclasses import replace
import math
import os
import time
import numpy as np
from .outdoor_model import ResponseModel
from .codex_nav.experiments.sampling_navigation import SamplingConfig, SamplingNavigator
from .codex_nav.vessel_dynamics import VesselParameters
from .codex_nav.navigation_map import NavigationMap
from .codex_nav.control_path import smooth_path
from .outdoor_obstacles import ObstacleTracker, compact_obstacles
from .outdoor_test_course import polygon, hull_clearance, fence
from .outdoor_regions import OperatingRegion
from .outdoor_follow import forward_approach_metrics
from .codex_nav.route_planner import a_star
from .codex_nav.fast_clearance import warmup_clearance
from .codex_nav.fast_learned_rollout import compiled_motion
from .codex_nav.gpu_geometry import trajectory_hulls,warmup_geometry
from .codex_nav.gpu_clearance import gpu_status
from .outdoor_anticipation import reaction_profile


class LearnedNavigator(SamplingNavigator):
    def __init__(self, model, config, cap):
        self.model, self.boat, self.cap = model, config, cap
        self.exact_passage=bool(config.get('exact_passage',False))
        self.history = np.zeros((model.delay, 2))
        p = VesselParameters()
        # Upstream proposal units remain desired m/s and rad/s. Its force model
        # is never called: every candidate and the final output use inverse().
        from dataclasses import replace
        p = replace(p, mass_kg=config['mass_kg']*float(config.get('prediction_mass_scale',1.)),
                    yaw_inertia_kg_m2=p.yaw_inertia_kg_m2*float(config.get('prediction_yaw_inertia_scale',1.)),
                    thruster_arm_m=config['thruster_spacing_m']/2,
                    cruise_speed_m_s=min(.5, max(.1, model.data['velocity_limit'][0])),
                    max_yaw_rate_rad_s=min(.5, max(.1, model.data['velocity_limit'][2])))
        uncertainty = config.get('prediction_margin_m',model.data.get('metrics', {}).get('horizons', {}).get('2', {}).get('integrated_position_rmse_m', 0.))
        self.reaction=reaction_profile(model,config)
        cfg = SamplingConfig(samples=64, horizon_steps=math.ceil(self.reaction['horizon_s']/model.dt-1e-9),
            noise_speed=.2, noise_yaw=.15, margin=0. if self.exact_passage else config['obstacle_margin_m']+max(0., float(uncertainty)),
            symmetric_yaw_bias=.12)
        super().__init__(p, dt=model.dt/3, mode='trajectory_control', config=cfg)
        self.allocation_weights=np.array([2.,1.])
        self.allocation_matrix=model.coefficients[3:5][:,[0,2]]*self.allocation_weights
        self.allocation_inverse=np.linalg.pinv(self.allocation_matrix)
        self.nonlinear_allocation=bool(np.any(model.coefficients[5:7]))
        self.dynamic = np.empty((0, 6))
        self.origin = np.zeros(2)
        self.route_world = None
        self.raw_route_world = None
        self.route_stamp = -np.inf
        self.route_goal = None
        self.now = 0.
        self.recovery = False
        self.normal_margin = cfg.margin; self.base_margin=cfg.margin; self.test_boundary=None
        self.operating_region=None
        self.forward_goal=None

    def guidance(self, state, observation, goal, frame):
        # Cache in ENU, never in the sliding local-map coordinate system.
        local = None if self.route_world is None else self.route_world-self.origin
        goal_world = goal+self.origin
        margin = self.cfg.margin+(0. if self.exact_passage else .04)
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
        raw = self.region_guidance(state,observation,goal)
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

    def region_guidance(self,state,m,goal):
        """A* uses the same region distances as smoothing and MPC validation."""
        started=time.perf_counter();margin=self.cfg.margin+(0. if self.exact_passage else .04)
        if self.exact_passage:
            connector=np.linspace(state[:2],goal,max(2,math.ceil(np.linalg.norm(goal-state[:2])/.04)+1))
            if np.all(m.clearance(connector)>=margin-1e-9):
                self.timings['route'].append(time.perf_counter()-started)
                return np.asarray([state[:2],goal])
        half_width=self.boat['width_m']/2 if self.exact_passage else .54
        clearance=np.minimum.reduce([m.gx-half_width,m.width-half_width-m.gx,m.gy-half_width,m.height-half_width-m.gy])
        # Same distances and hull margin, evaluated in one compiled batch.
        # Avoid hundreds of full-grid NumPy passes for each new A* route.
        grid=np.c_[m.gx.ravel(),m.gy.ravel()]
        clearance=np.minimum(clearance,m.clearance(grid).reshape(m.gx.shape))
        if self.operating_region is not None:
            clearance=np.minimum(clearance,m.region_clearance)
        free=clearance>=margin
        weights=1.+np.where(m.known_free,0.,1.5)+.15/np.maximum(clearance,.1)
        start=(int(np.argmin(abs(m.ys-state[1]))),int(np.argmin(abs(m.xs-state[0]))))
        end=(int(np.argmin(abs(m.ys-goal[1]))),int(np.argmin(abs(m.xs-goal[0]))))
        path=a_star(free,weights,m.xs,m.ys,start,end,m.resolution)
        if path is not None:path[0]=state[:2]
        self.progress=0.;self.timings['route'].append(time.perf_counter()-started)
        return path

    def inverse(self, state, desired):
        state, desired = np.atleast_2d(state), np.atleast_2d(desired)
        coefficients = self.model.coefficients
        base = state@coefficients[:3]+coefficients[7]
        # One-second approach to requested speed/rate rather than an impulsive
        # one-step inverse. Empirical command bounds prohibit extrapolation.
        target = state[:, [0, 2]]+self.model.dt*(desired-state[:, [0, 2]])
        q = np.zeros((len(state), 2))
        cap = min(self.cap,.22) if self.recovery else self.cap
        lo = np.maximum(-cap, self.model.data['command_min'])
        hi = np.minimum(cap, self.model.data['command_max'])
        for _ in range(3 if self.nonlinear_allocation else 1):
            nonlinear = (q*np.abs(q))@coefficients[5:7]
            residual=(target-base[:,[0,2]]-nonlinear[:,[0,2]])*self.allocation_weights
            matrix=self.allocation_matrix
            # Interior plus four optimized edges covers the box optimum.
            # This preserves forward authority when a turn request saturates.
            choices=[np.clip(residual@self.allocation_inverse,lo,hi)]
            for fixed in (0,1):
                free=1-fixed;norm=max(float(matrix[free]@matrix[free]),1e-12)
                for bound in (lo[fixed],hi[fixed]):
                    candidate=np.empty_like(q);candidate[:,fixed]=bound
                    candidate[:,free]=np.clip((residual-bound*matrix[fixed])@matrix[free]/norm,lo[free],hi[free])
                    choices.append(candidate)
            candidates=np.stack(choices,axis=1)
            cost=np.sum((candidates@matrix-residual[:,None,:])**2,axis=2)
            q=candidates[np.arange(len(state)),cost.argmin(axis=1)]
        return q

    def reference_cost_adjustment(self, initial, sequences, states):
        cost=np.zeros(len(states))
        if self.forward_goal is not None:
            _,reverse_closing,distance,alignment=forward_approach_metrics(initial,states,self.forward_goal,self.boat.get('arrival_radius_m',1.))
            cost+=20.*reverse_closing+6.*(1.-alignment[:,-1])
        if self.boat.get('allow_reverse_maneuvers',False):return cost
        # Penalize backwards travel, not one reversing thruster used to turn.
        # Reverse thrust while moving forward can still be needed for braking.
        reverse_distance=np.maximum(-states[:,:,3]-.03,0.).sum(axis=1)*self.model.dt
        reverse_drive=np.maximum(-states[:,:,6:8].mean(axis=2)-.01,0.)
        reverse_drive*=states[:,:,3]<.03
        return cost+8.*reverse_distance+2.*reverse_drive.sum(axis=1)*self.model.dt

    def boundary_clearance(self, states):
        z = np.atleast_2d(states)
        boundary=(hull_clearance(z,self.test_boundary,self.boat['length_m'],self.boat['width_m'])-max(0.,self.normal_margin-self.cfg.margin)
                  if self.test_boundary is not None else np.full(len(z),np.inf))
        if self.operating_region is not None:
            world=z.astype(float,copy=True);world[:,:2]+=self.origin
            boundary=np.minimum(boundary,self.operating_region.hull_clearance(world,self.boat['length_m'],self.boat['width_m'])-max(0.,self.normal_margin-self.cfg.margin))
        return boundary

    def safety_clearance(self, states, obstacles, width, height):
        z=np.atleast_2d(states)
        boundary=self.boundary_clearance(z)
        if not len(obstacles): return boundary
        # Oriented full-hull rectangle; obstacles are surface discs. Substeps
        # inflate by distance travelled so samples cover the swept motion.
        delta = obstacles[None, :, :2]-z[:, None, :2]
        c, s = np.cos(z[:, 2, None]), np.sin(z[:, 2, None])
        x = delta[:, :, 0]*c+delta[:, :, 1]*s
        y = -delta[:, :, 0]*s+delta[:, :, 1]*c
        qx=abs(x)-self.boat['length_m']/2;qy=abs(y)-self.boat['width_m']/2
        signed=np.hypot(np.maximum(qx,0),np.maximum(qy,0))+np.minimum(np.maximum(qx,qy),0)-obstacles[None,:,2]
        signed=np.where(abs(signed)<1e-9,0.,signed)
        return np.minimum(boundary,np.min(signed,axis=1))

    def rollout(self, initial, sequences, obstacles, width, height, goal=None):
        """Fuse allocation/dynamics, then check every original probe in one batch.

        Small dependent model operations stay on compiled CPU. The much larger
        oriented-hull/static/dynamic cross product is eligible for one CUDA job.
        """
        if compiled_motion is None or os.environ.get('ISV_FAST_ROLLOUT','1')=='0':
            return self.rollout_reference(initial,sequences,obstacles,width,height,goal)
        initial=np.ascontiguousarray(initial,dtype=np.float64)
        sequences=np.ascontiguousarray(sequences,dtype=np.float64)
        cap=min(self.cap,.22) if self.recovery else self.cap
        result=compiled_motion(initial,sequences,np.ascontiguousarray(self.history,dtype=np.float64),
            self.model.coefficients,self.allocation_matrix,self.allocation_inverse,
            np.maximum(-cap,self.model.data['command_min']),np.minimum(cap,self.model.data['command_max']),
            self.model.dt,max(.01,float(self.boat.get('command_slew_per_s',.35)))*self.model.dt,
            3 if self.nonlinear_allocation else 1)
        n,horizon,_=result.shape
        if horizon==0:return result,np.minimum(self.safety_clearance(np.tile(initial,(n,1)),obstacles,width,height),self.moving_clearance(np.tile(initial,(n,1)),0.))
        previous=np.concatenate((np.broadcast_to(initial,(n,1,8)),result[:,:-1]),axis=1)
        swept=np.linalg.norm(result[:,:,:2]-previous[:,:,:2],axis=2)/2
        swept+=abs(result[:,:,2]-previous[:,:,2])*math.hypot(self.boat['length_m'],self.boat['width_m'])/4
        probes=np.concatenate((np.broadcast_to(initial,(n,1,8)),result,.5*(result+previous)),axis=1)
        times=np.broadcast_to(np.r_[0.,(np.arange(horizon)+1)*self.model.dt,(np.arange(horizon)+.5)*self.model.dt],(n,1+2*horizon)).ravel().copy()
        sweep=np.concatenate((np.zeros((n,1)),swept,swept),axis=1)
        flat=probes.reshape(-1,8)
        distances=trajectory_hulls(flat,times,obstacles,self.dynamic,self.boat['length_m'],self.boat['width_m'],self.model.dt)
        boundary=self.boundary_clearance(flat)
        clearance=np.minimum(boundary,distances[:,0])
        if len(self.dynamic):
            # Moving-track recovery margin also applies to region boundaries,
            # exactly as in moving_clearance(), including its initial probe.
            clearance=np.minimum(clearance,np.minimum(boundary,distances[:,1])-max(0.,self.normal_margin-self.cfg.margin))
        clearance=(clearance.reshape(n,1+2*horizon)-sweep).min(axis=1)
        if self.forward_goal is not None:
            invalid,_,_,_=forward_approach_metrics(initial,result,self.forward_goal,self.boat.get('arrival_radius_m',1.))
            clearance=np.where(invalid,-np.inf,clearance)
        return result,clearance

    def rollout_reference(self, initial, sequences, obstacles, width, height, goal=None):
        """Unchanged NumPy oracle and fallback when compilation is unavailable."""
        n, horizon, _ = sequences.shape
        z = np.repeat(np.asarray(initial)[None, :], n, axis=0)
        result = np.empty((n, horizon, 8)); clearance = self.safety_clearance(z, obstacles, width, height)
        clearance = np.minimum(clearance,self.moving_clearance(z,0.))
        queue = deque([np.tile(q, (n, 1)) for q in self.history])
        for i in range(horizon):
            command = self.inverse(z[:, 3:6], sequences[:, i])
            delta=max(.01,float(self.boat.get('command_slew_per_s',.35)))*self.model.dt
            command=z[:,6:]+np.clip(command-z[:,6:],-delta,delta)
            cap=min(self.cap,.22) if self.recovery else self.cap
            command=np.clip(command,np.maximum(-cap,self.model.data['command_min']),np.minimum(cap,self.model.data['command_max']))
            queue.append(command)
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
        if self.forward_goal is not None:
            invalid,_,_,_=forward_approach_metrics(initial,result,self.forward_goal,self.boat.get('arrival_radius_m',1.))
            clearance=np.where(invalid,-np.inf,clearance)
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
        self.model = ResponseModel(model_data).for_navigation(config); self.config = config; self.cap = percent/100
        # The operator explicitly selects the validation model. Qualification
        # remains a reported metric, separate from coefficient/coverage checks.
        if not 0 < self.cap <= 1: raise ValueError('출력 범위 오류')
        if any(x > -.01 for x in model_data['command_min']) or any(x < self.cap for x in model_data['command_max']):
            raise ValueError('선택한 출력·좌우 역추진을 포함한 RC 기록이 필요합니다')
        self.navigator = LearnedNavigator(self.model, config, self.cap)
        # Load compiled kernels before accepting the first propulsion plan.
        warmup_clearance()
        warmup_geometry()
        if compiled_motion is not None:
            nav=self.navigator;cap=nav.cap
            compiled_motion(np.zeros(8),np.zeros((1,1,2)),np.ascontiguousarray(nav.history),
                self.model.coefficients,nav.allocation_matrix,nav.allocation_inverse,
                np.maximum(-cap,self.model.data['command_min']),np.minimum(cap,self.model.data['command_max']),
                self.model.dt,max(.01,float(config.get('command_slew_per_s',.35)))*self.model.dt,
                3 if nav.nonlinear_allocation else 1)
        a_star(np.ones((3,3),dtype=bool),np.ones((3,3)),np.arange(3,dtype=float),np.arange(3,dtype=float),(0,0),(2,2),1.)
        smooth_path(np.array([[0.,0.],[.4,0.],[.8,.4]]),lambda points:True)
        self.frame = 0
        self.tracker = ObstacleTracker()
        self.region_key=None;self.region=None;self.region_fence=None

    def plan(self, state, scan, command_history, now_ros):
        started = time.perf_counter()
        origin = np.array(state['position'])-20.
        previous=np.asarray(state.get('previous_command',[0.,0.]),dtype=float)
        if previous.shape!=(2,) or not np.isfinite(previous).all():raise ValueError('이전 추진 명령 오류')
        initial = np.r_[20., 20., state['yaw'], state['velocity'], np.clip(previous,-self.cap,self.cap)]
        goal = np.clip(np.array(state['goal'])-origin, 3., 37.)
        self.navigator.forward_goal=(np.asarray(state.get('arrival_goal',state['goal']))-origin
            if self.config.get('forward_arrival_required',False) else None)
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
        exact=bool(self.config.get('exact_passage',False))
        observation = NavigationMap(40, 40, resolution=.25 if exact else .4)
        if exact:
            observation.xs+=20.-observation.xs[np.argmin(abs(observation.xs-20.))]
            observation.ys+=20.-observation.ys[np.argmin(abs(observation.ys-20.))]
            observation.gx,observation.gy=np.meshgrid(observation.xs,observation.ys)
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
        raw_discs = np.c_[surfaces,np.full(len(surfaces),0. if exact else .06)]
        if exact:
            # Retain measured surfaces without fitted/merged-disc inflation.
            observation.obstacles=raw_discs
            base_clearance=observation.clearance
            observation.clearance=lambda points,hull_radius=self.config['width_m']/2:base_clearance(points,hull_radius)
        else:observation.obstacles = compact_obstacles(np.concatenate([observation.obstacles,raw_discs]))
        self.tracker.update(angles,distances,lidar_position+origin,
                            yaw+math.radians(self.config['lidar_yaw_deg']),
                            float(scan.get('stamp',now_ros)),max_range)
        self.navigator.dynamic, tracks = self.tracker.prediction(now_ros,origin)
        self.navigator.test_boundary=None;self.navigator.normal_margin=self.navigator.base_margin
        self.navigator.operating_region=None
        if state.get('test_regions'):
            key=repr(state['test_regions'])
            if key!=self.region_key:
                self.region=OperatingRegion(state['test_regions']);self.region_fence=self.region.fence();self.region_key=key
                self.navigator.route_world=None
            region=self.region;self.navigator.operating_region=region
            if not exact:self.navigator.normal_margin+=2*float(state['position_sigma_m'])
            boundary=self.region_fence.copy();boundary[:,:2]-=origin
            boundary=boundary[np.all((boundary[:,:2]>-2)&(boundary[:,:2]<42),axis=1)]
            if not exact:observation.obstacles=np.concatenate([observation.obstacles,boundary])
            measured_clearance=observation.clearance
            radius=self.config['width_m']/2 if exact else math.hypot(self.config['length_m'],self.config['width_m'])/2
            observation.clearance=lambda points,hull_radius=radius:np.minimum(measured_clearance(points,hull_radius),region.clearance(np.asarray(points).reshape(-1,2)+origin)-hull_radius)
            observation.region_clearance=(region.clearance(np.c_[observation.gx.ravel(),observation.gy.ravel()]+origin)-radius).reshape(observation.gx.shape)
            observation.free_seen[observation.region_clearance<0]=-np.inf
        elif state.get('test_area') is not None:
            boundary=polygon(state['test_area'])-origin
            self.navigator.test_boundary=boundary
            if not exact:self.navigator.normal_margin+=2*float(state['position_sigma_m'])
            if not exact:observation.obstacles=np.concatenate([observation.obstacles,fence(boundary)])
        if state.get('virtual_obstacles'):
            virtual=np.asarray(state.get('virtual_obstacles',[]),dtype=float).reshape(-1,3)
            if len(virtual)>8 or not np.isfinite(virtual).all() or np.any(virtual[:,2]<=0):raise ValueError('가상 장애물 설정 오류')
            virtual=virtual.copy();virtual[:,:2]-=origin
            observation.obstacles=np.concatenate([observation.obstacles,virtual])
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
            self.navigator.cfg = replace(self.navigator.cfg,margin=0. if exact else max(.1,float(self.config.get('recovery_margin_m',.1))))
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
        # Execute the very PWM checked in the chosen rollout, including slew.
        q = prediction[0,6:8]
        path = prediction[:, :2]+origin
        reaction=self.navigator.reaction
        safety_steps=min(len(path),max(1,math.ceil((reaction['model_delay_s']+min(2.,reaction['inertia_tau_s'])+.6)/self.model.dt)))
        return {'command': q.tolist(), 'path': path.tolist(), 'clearance_m': float(clearance),
                'safety_path':np.c_[path[:safety_steps],prediction[:safety_steps,2]].tolist(),
                'route':(self.navigator.route_world.tolist() if self.navigator.route_world is not None else []),
                'raw_route':(self.navigator.raw_route_world.tolist() if self.navigator.raw_route_world is not None else []),
                'obstacles':tracks, 'recovery':recovery,
                'planning_ms':(time.perf_counter()-started)*1000,
                'anticipation_profile':self.navigator.reaction,
                'compute':dict(gpu_status(),motion_backend='compiled CPU' if compiled_motion is not None and os.environ.get('ISV_FAST_ROLLOUT','1')!='0' else 'NumPy CPU'),
                'scan_range_m':max_range, 'obstacle_discs':len(observation.obstacles),
                'plan_stamp':now_ros,'dt_s':self.model.dt,
                'planner': 'CODEX SamplingNavigator + learned PWM response'}
