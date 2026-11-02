"""Experimental vessel-model sampling MPC, with optional coarse A* guidance.

This is an MPPI-inspired bounded shooting optimizer, not a claim of exact
path-integral optimal control: deterministic exploration primitives augment
correlated Gaussian perturbations, and the weighted proposal is safety checked.
Inputs deliberately exclude the simulation environment and ground-truth objects.
Physical integration and thrust allocation are imported unchanged.
"""
from dataclasses import dataclass
import time
import numpy as np
from isv_water_trials.codex_nav.fast_command_arrays import slew_yaw
from isv_water_trials.codex_nav.vessel_dynamics import allocate, integrate
from isv_water_trials.codex_nav.route_planner import a_star
from isv_water_trials.codex_nav.control_path import lookahead, path_geometry
from isv_water_trials.codex_nav.goal_guidance import approach_heading, terminal_cost
from isv_water_trials.codex_nav.experiments.fast_rollout import compiled_rollout, parameter_vector
from isv_water_trials.codex_nav.trajectory_objective import arrival_terms, select_eta, entry_heading_quality, GOAL_RADIUS_M
from isv_water_trials.codex_nav.passage_guidance import observed_passages, observed_wall_passages, passage_sequences
from isv_water_trials.codex_nav.passage_geometry import (physical_hull_polygons, surface_clearances,
                              prepare_hull_edges, fast_surface_clearances)


@dataclass(frozen=True)
class SamplingConfig:
    samples: int = 128
    horizon_steps: int = 40
    temperature: float = .3
    noise_speed: float = .35
    noise_yaw: float = .18
    smooth_weight: float = 3.
    clearance_weight: float = .25
    margin: float = .20
    yaw_command_step: float | None = None  # rad/s command change per .12 s knot
    symmetric_yaw_bias: float = 0.  # opt-in no-target exploration only
    objective: str = 'reference'  # 'eta' is fly-through, trajectory_control only
    terminal_value: bool = False
    continuity: bool = False
    multimodal: bool = False
    viability: bool = False
    recovery_policy: bool = False
    strategy_guidance: bool = False
    goal_entry_heading: bool = False
    goal_radius_m: float = GOAL_RADIUS_M
    center_entry: bool = False
    passage_guidance: bool = False
    exact_passage_safety: bool = False
    forward_policy: bool = False  # opt-in feasibility before reverse ranking


def hull_clearance(states, obstacles, width, height):
    """Same conservative twin-capsule envelope as the existing controller."""
    z = np.atleast_2d(states)
    margin = np.minimum.reduce([z[:, 0]-.93, width-.93-z[:, 0],
                                z[:, 1]-.93, height-.93-z[:, 1]])
    if len(obstacles):
        dx = obstacles[None, :, 0]-z[:, None, 0]
        dy = obstacles[None, :, 1]-z[:, None, 1]
        c, s = np.cos(z[:, None, 2]), np.sin(z[:, None, 2])
        along, lateral = dx*c+dy*s, -dx*s+dy*c
        gap = np.maximum(np.maximum(-.56-along, along-.52), 0.)
        side = np.minimum(abs(lateral-.22), abs(lateral+.22))
        margin = np.minimum(margin, (np.hypot(gap, side)-obstacles[None, :, 2]-.32).min(axis=1))
    return margin


def route_coordinates(points, path, arc=None):
    """Continuous segment projection and forward tangent, without a target point."""
    points = np.atleast_2d(points)
    segment = np.diff(path, axis=0)
    length2 = np.maximum(np.sum(segment*segment, axis=1), 1e-12)
    length = np.sqrt(length2)
    if arc is None:
        arc = np.r_[0., np.cumsum(length)]
    offset = points[:, None, :]-path[None, :-1, :]
    fraction = np.clip(np.sum(offset*segment[None, :, :], axis=2)/length2, 0., 1.)
    foot = path[None, :-1, :]+fraction[:, :, None]*segment[None, :, :]
    distance2 = np.sum((points[:, None, :]-foot)**2, axis=2)
    index = np.argmin(distance2, axis=1)
    row = np.arange(len(points))
    progress = arc[index]+fraction[row, index]*length[index]
    heading = np.arctan2(segment[index, 1], segment[index, 0])
    return progress, distance2[row, index], heading


def direct_corridor_open(state, goal, observation, margin):
    """Only observed local free space may license direct-goal ETA guidance."""
    delta = goal-state[:2]
    distance = np.linalg.norm(delta)
    if distance < 1e-6:
        return True
    direction = delta/distance
    length = min(distance, 5.)
    points = state[:2]+np.linspace(0., length, max(3, int(length/.25)+1))[:, None]*direction
    probe = np.zeros((len(points), 8))
    probe[:, :2] = points
    probe[:, 2] = np.arctan2(direction[1], direction[0])
    if np.min(hull_clearance(probe, observation.obstacles,
                             observation.width, observation.height)) < margin+.12:
        return False
    # An unknown cell is never evidence that the direct passage is open.
    xi = np.abs(observation.xs[:, None]-points[:, 0]).argmin(axis=0)
    yi = np.abs(observation.ys[:, None]-points[:, 1]).argmin(axis=0)
    return bool(np.all(observation.known_free[yi, xi]))


def corridor_families(state, goal, states, sequences, direct_open, previous=None):
    """Stable world-space side labels for strategy continuity."""
    delta = goal-state[:2]
    bearing = np.arctan2(delta[1], delta[0])
    motion = states[:, -1, :2]-state[:2]
    lateral = -np.sin(bearing)*motion[:, 0]+np.cos(bearing)*motion[:, 1]
    heading_error = np.arctan2(np.sin(states[:, -1, 2]-bearing),
                                np.cos(states[:, -1, 2]-bearing))
    family = np.where(lateral > .45, 'LEFT', np.where(lateral < -.45, 'RIGHT', 'CURRENT'))
    if direct_open:
        direct = ((np.abs(lateral) <= .75) & (np.abs(heading_error) <= .45) &
                  (states[:, -1, 3] > .3))
        family = np.where(direct, 'DIRECT', family)
    family = np.where(np.mean(sequences[:, :, 0], axis=1) < 0., 'RECOVERY', family)
    if previous is not None and family[0] == 'CURRENT':
        family[0] = previous
    return family


class SamplingNavigator:
    def __init__(self, physics, dt=.04, mode='corridor', config=None):
        if mode not in ('corridor', 'direct', 'route_heading', 'trajectory_control'):
            raise ValueError(mode)
        self.p, self.dt, self.mode = physics, dt, mode
        self.cfg = config or SamplingConfig()
        self.hull_polygons = (physical_hull_polygons(physics.pixels_per_m)
                              if self.cfg.exact_passage_safety else None)
        self.hull_edges, self.hull_bound, self.hull_box = (
            prepare_hull_edges(self.hull_polygons)
            if self.hull_polygons is not None else (None, 0., None))
        self.rng = np.random.default_rng(731)  # same stream for every map; not a map seed
        self.sequence = None
        self.path = None
        self.path_frame = -1000
        self.progress = 0.
        self.previous = np.zeros(2)
        self.goal_heading = None
        self.strategy = None
        self.passage_commitment = None
        self.timings = {'route': [], 'rollout': [], 'plan': []}
        self.fast_params = parameter_vector(physics)

    def guidance(self, state, observation, goal, frame):
        if self.mode == 'direct':
            return np.array([state[:2], goal])
        m = observation
        planner_margin = self.cfg.margin if self.cfg.passage_guidance else self.cfg.margin+.04
        unsafe = self.path is not None and np.any(m.clearance(self.path) < planner_margin)
        if self.path is None or unsafe or frame-self.path_frame >= 24:
            started = time.perf_counter()
            clearance = np.minimum.reduce([m.gx-.54, m.width-.54-m.gx,
                                            m.gy-.54, m.height-.54-m.gy])
            for x, y, r in m.obstacles:
                clearance = np.minimum(clearance, np.hypot(m.gx-x, m.gy-y)-r-.54)
            free = clearance >= planner_margin
            weights = 1.+np.where(m.known_free, 0., 1.5)+.15/np.maximum(clearance, .1)
            start = (int(np.argmin(abs(m.ys-state[1]))), int(np.argmin(abs(m.xs-state[0]))))
            end = (int(np.argmin(abs(m.ys-goal[1]))), int(np.argmin(abs(m.xs-goal[0]))))
            self.path = a_star(free, weights, m.xs, m.ys, start, end, m.resolution)
            if self.path is not None:
                self.path[0] = state[:2]
            self.path_frame, self.progress = frame, 0.
            self.timings['route'].append(time.perf_counter()-started)
        return self.path

    def rollout(self, initial, sequences, obstacles, width, height, goal=None):
        """Batch candidates, fixed .04 s steps, allocator refreshed every step."""
        if compiled_rollout is not None:
            finish = (0., 0., 0.) if goal is None else (*goal, self.cfg.goal_radius_m)
            if self.hull_polygons is not None:
                if not hasattr(self, 'hull_box'):  # archived diagnostic snapshots
                    self.hull_edges, self.hull_bound, self.hull_box = prepare_hull_edges(
                        self.hull_polygons)
                return compiled_rollout(initial, sequences, obstacles, width, height,
                                        self.dt, self.fast_params, *finish,
                                        self.hull_polygons, None,
                                        self.hull_edges, self.hull_bound, self.hull_box,
                                        None, self.cfg.forward_policy)
            return compiled_rollout(initial, sequences, obstacles, width, height,
                                    self.dt, self.fast_params, *finish)
        return self.rollout_reference(initial, sequences, obstacles, width, height, goal)

    def safety_clearance(self, states, obstacles, width, height):
        if self.hull_polygons is not None:
            return fast_surface_clearances(np.atleast_2d(states), obstacles,
                                           width, height, self.hull_edges,
                                           self.hull_bound, self.hull_box,self.cfg.forward_policy)
        return hull_clearance(states, obstacles, width, height)

    def rollout_reference(self, initial, sequences, obstacles, width, height, goal=None):
        """NumPy reference retained as an exact-physics equivalence oracle."""
        n, horizon, _ = sequences.shape
        z = np.repeat(initial[None, :], n, axis=0)
        states = np.empty((n, horizon, 8))
        closest = np.full(n, np.inf)
        finished = np.zeros(n, dtype=bool)
        # Check every physical integration step between command knots.
        for t in range(horizon):
            for substep in range(3):
                forces = allocate(z, sequences[:, t, 0], sequences[:, t, 1], self.p)
                z = integrate(z, *forces, self.dt, self.p)
                clearance = (surface_clearances(z, obstacles, width, height,
                                                self.hull_polygons,self.cfg.forward_policy).min(axis=1)
                             if self.hull_polygons is not None else
                             hull_clearance(z, obstacles, width, height))
                closest = np.where(finished, closest, np.minimum(closest, clearance))
                if goal is not None:
                    inside = np.sum((z[:, :2]-goal)**2, axis=1) < self.cfg.goal_radius_m**2
                    finished |= inside
            states[:, t] = z
        return states, closest

    def plan(self, state, observation, goal, frame):
        started = time.perf_counter()
        cfg, p = self.cfg, self.p
        prior_passage_commitment = (None if self.passage_commitment is None else
                                    self.passage_commitment.copy())
        path = self.guidance(state, observation, goal, frame)
        if path is None or len(path) < 2:
            # Topology unavailable: local goal still supplies direction, while
            # every rollout retains the same perceived hull collision check.
            path = np.array([state[:2], goal])
        if cfg.center_entry:
            path = path.copy()
            path[-1] = goal
            if self.path is not None:
                self.path = path
        self.goal_heading = approach_heading(path, state[:2], goal, self.goal_heading)
        geometry = path_geometry(path)
        if self.mode == 'trajectory_control':
            vessel_s, _, _ = route_coordinates(state[:2], path, geometry[2])
            self.progress = max(self.progress, float(vessel_s[0]))
            desired = 0.
            target = None  # A selected physical state supplies display only.
        else:
            target, self.progress = lookahead(path, state[:2], 3., self.progress, geometry)
            # Existing opt-in modes retain their original initial seed.
            delta = target-state[:2]
            error = (np.arctan2(delta[1], delta[0])-state[2]+np.pi)%(2*np.pi)-np.pi
            desired = np.clip(.65*error, -.5, .5)
        horizon = cfg.horizon_steps
        if self.sequence is None:
            self.sequence = np.tile([p.cruise_speed_m_s, desired], (horizon, 1))
        else:
            self.sequence = np.vstack([self.sequence[1:], self.sequence[-1:]])
        # Correlated perturbations avoid using hundreds of independent switches.
        knots = self.rng.normal(size=(cfg.samples, (horizon+4)//5+1, 2))
        # The knot mapping is immutable for a given horizon. Broadcasting keeps
        # the exact multiply/add order and RNG draw, avoiding per-knot dispatch.
        if getattr(self, '_noise_horizon', None) != horizon:
            steps = np.arange(horizon)
            self._noise_index = steps//5
            self._noise_fraction = ((steps%5)/5.)[None, :, None]
            self._noise_horizon = horizon
        f = self._noise_fraction
        noise = ((1-f)*knots[:, self._noise_index]+
                 f*knots[:, self._noise_index+1])
        sequences = self.sequence[None, :, :]+noise*np.array([cfg.noise_speed, cfg.noise_yaw])
        sequences[0] = self.sequence
        # Broad turn-then-straight primitives provide multi-modal exploration,
        # including braking/reverse, without map-specific recovery rules.
        index = 1
        for speed in [p.cruise_speed_m_s, .65*p.cruise_speed_m_s, .3*p.cruise_speed_m_s, 0., -.25*p.cruise_speed_m_s]:
            for rate in np.linspace(-.5, .5, 9):
                if index >= cfg.samples:
                    break
                sequences[index, :, 0] = speed
                sequences[index, :, 1] = rate
                if index % 2:
                    sequences[index, horizon//2:, 1] = 0.
                index += 1
        passage = None
        passage_slots = np.empty(0, dtype=int)
        if cfg.passage_guidance:
            visible = observed_passages(observation.obstacles, state[:2], state[2], goal,
                                        cfg.margin, hull_polygons=self.hull_polygons,
                                        sort=not cfg.forward_policy)
            if cfg.forward_policy:
                visible += observed_wall_passages(observation.obstacles,state[:2],state[2],
                    goal,cfg.margin,observation.width,observation.height,self.hull_polygons,
                    sort=False)
                # One stable sort replaces both preliminary family sorts and
                # this combined sort, with identical keys and equal-key order.
                visible.sort(key=lambda p:np.linalg.norm(p.center-state[:2])+
                             .5*np.linalg.norm(goal-p.center))
            if self.passage_commitment is not None:
                matching = [candidate for candidate in visible if
                            np.linalg.norm(candidate.center-self.passage_commitment) < .8]
                if matching:
                    passage = min(matching, key=lambda candidate:
                                  np.linalg.norm(candidate.center-self.passage_commitment))
                else:
                    self.passage_commitment = None
                    if self.strategy == 'PASSAGE':
                        self.strategy = None
            if passage is None and visible:
                passage = visible[0]
            if passage is not None:
                behind = np.dot(state[:2]-passage.center, passage.tangent)
                if behind > 1.3:
                    self.passage_commitment = None
                    if self.strategy == 'PASSAGE':
                        self.strategy = None
                    passage = None
            if passage is not None and index < cfg.samples:
                proposals = passage_sequences(state, passage, p, horizon, 3*self.dt)
                count = min(len(proposals), cfg.samples-index)
                passage_slots = np.arange(index, index+count)
                sequences[passage_slots] = proposals[:count]
                index += count
        if cfg.multimodal:
            # Explicit alternatives occupy existing sample slots. The rest
            # retain the correlated exploration and shifted warm start.
            rates = [-.48, -.32, -.18, 0., .18, .32, .48]
            slot = 1
            for speed in (p.cruise_speed_m_s, .7*p.cruise_speed_m_s):
                for rate in rates:
                    for release in (False, True):
                        sequences[slot, :, 0] = speed
                        sequences[slot, :, 1] = rate
                        if release:
                            sequences[slot, horizon//2:, 1] = 0.
                        slot += 1
            sequences[slot, :, 0] = .35*p.cruise_speed_m_s
            sequences[slot, :, 1] = 0.
        if self.mode == 'trajectory_control' and cfg.samples > index+2:
            # Symmetric, time-varying steering alternatives around the shifted
            # previous sequence. Neither side is chosen from a pursuit point;
            # the physical rollout and coarse-route progress decide.
            available = cfg.samples-index
            width = available//3
            sequences[index:index+width, :, 1] += cfg.symmetric_yaw_bias
            sequences[index+width:index+2*width, :, 1] -= cfg.symmetric_yaw_bias
        sequences[:, :, 0] = np.clip(sequences[:, :, 0], -.25*p.cruise_speed_m_s, p.cruise_speed_m_s)
        sequences[:, :, 1] = np.clip(sequences[:, :, 1], -p.max_yaw_rate_rad_s, p.max_yaw_rate_rad_s)
        if cfg.yaw_command_step is not None:
            slew_yaw(sequences, self.previous[1], cfg.yaw_command_step)
        # Same perception map, bounded local subset; no simulator object access.
        obs = observation.obstacles
        obstacle_ids = np.flatnonzero(np.linalg.norm(obs[:, :2]-state[:2], axis=1) < 6.8)
        obs = obs[obstacle_ids]
        tick = time.perf_counter()
        finish = goal if cfg.objective == 'eta' else None
        states, closest = self.rollout(state, sequences, obs, observation.width, observation.height, finish)
        self.timings['rollout'].append(time.perf_counter()-tick)
        # Arc-length route progress and distance, evaluated against route samples.
        arc = geometry[2]
        local = (arc >= max(0., self.progress-.5)) & (arc <= self.progress+9.)
        guide, guide_arc = path[local], arc[local]
        if len(guide) < 2:
            guide, guide_arc = path, arc
        if self.mode == 'direct':
            # Resample the straight goal reference to avoid endpoint-only costs.
            length = np.linalg.norm(goal-state[:2])
            guide_arc = np.linspace(0., length, max(2, int(length/.25)+1))
            guide = state[:2]+guide_arc[:, None]*(goal-state[:2])/max(length, .001)
        def objective(u, z, clearance):
            distance2 = np.sum((z[:, -1, None, :2]-guide[None, :, :])**2, axis=-1)
            nearest = distance2.argmin(axis=1)
            progress = guide_arc[nearest]-self.progress
            change = np.diff(np.concatenate([np.tile(self.previous, (len(u), 1, 1)), u], axis=1), axis=1)
            cost = -2.*progress + .8*distance2[np.arange(len(u)), nearest]
            cost += .2*np.linalg.norm(z[:, -1, :2]-goal, axis=1)
            if self.mode in ('route_heading', 'trajectory_control'):
                # A short rollout can reach a point near a U-shaped route while
                # still facing into the closed end. The coarse route supplies
                # topology; its forward tangent is a terminal *orientation*
                # hint for the physically simulated candidate, not a target
                # point the vessel must pass through.
                seg = np.minimum(nearest, len(guide)-2)
                tangent = guide[seg+1]-guide[seg]
                direction = np.arctan2(tangent[:, 1], tangent[:, 0])
                cost += 3.*(1.-np.cos(z[:, -1, 2]-direction))
            cost += cfg.smooth_weight*np.sum(change[:, :, 1]**2, axis=1)
            cost += .2*np.sum(change[:, :, 0]**2, axis=1)
            cost += .05*np.mean(z[:, :, 5]**2, axis=1)
            cost += cfg.clearance_weight/np.maximum(clearance+.15, .03)
            cost += .05*np.mean(u[:, :, 0]**2, axis=1)
            cost += terminal_cost(z[:, -1], state[:2], goal, self.goal_heading)
            return cost
        # The ETA branch never reads this legacy weighted objective. It is a
        # pure calculation, not a safety gate or a source of random draws.
        costs = (objective(sequences, states, closest)
                 if cfg.objective != 'eta' else None)
        feasible = closest >= cfg.margin
        if cfg.objective == 'eta':
            terms = arrival_terms(states, sequences, path, geometry[2], goal, p, 3*self.dt,
                                  self.previous, self.sequence, terminal_value=cfg.terminal_value,
                                  obstacles=obs if cfg.viability else None,
                                  width=observation.width, height=observation.height,
                                  margin=cfg.margin, goal_radius=cfg.goal_radius_m)
            families = None
            if cfg.strategy_guidance:
                open_direct = direct_corridor_open(state, goal, observation, cfg.margin)
                previous_family = (None if cfg.passage_guidance and self.strategy == 'PASSAGE'
                                   else self.strategy)
                families = corridor_families(state, goal, states, sequences,
                                             open_direct, previous_family)
                if open_direct:
                    direct = families == 'DIRECT'
                    goal_delta = goal-states[:, -1, :2]
                    direct_distance = np.maximum(0., np.linalg.norm(goal_delta, axis=1)-cfg.goal_radius_m)
                    bearing = np.arctan2(goal_delta[:, 1], goal_delta[:, 0])
                    heading_error = np.arctan2(np.sin(states[:, -1, 2]-bearing),
                                               np.cos(states[:, -1, 2]-bearing))
                    turn_loss = (np.abs(heading_error)-np.sin(np.abs(heading_error)))/p.max_yaw_rate_rad_s
                    direct_eta = (horizon*3*self.dt+direct_distance/p.cruise_speed_m_s+
                                  terms['speed_recovery_s']+turn_loss)
                    terms['eta_s'] = np.where(direct, np.minimum(terms['eta_s'], direct_eta),
                                               terms['eta_s'])
            passage_viable = np.zeros(cfg.samples, dtype=bool)
            if passage is not None and len(passage_slots):
                # A sampled or shifted sequence may also traverse this gap.
                # Assign identity from the safe physical trajectory, not only
                # from the index of the explicit seed family.
                passage_tested = np.arange(cfg.samples)
                terminal = states[passage_tested, -1]
                travel = terminal[:, :2]-state[:2]
                forward = travel@passage.tangent
                offset = terminal[:, :2]-passage.center
                signed = offset@passage.tangent
                lateral = np.abs(offset[:, 0]*passage.tangent[1]-
                                 offset[:, 1]*passage.tangent[0])
                passage_heading = np.arctan2(passage.tangent[1], passage.tangent[0])
                heading_error = np.abs(np.arctan2(np.sin(terminal[:, 2]-passage_heading),
                                                  np.cos(terminal[:, 2]-passage_heading)))
                # Identify an actual crossing of the observed passage plane.
                # The admissible lateral span comes from this observation's
                # measured free width, never a fixed map-specific gap width.
                points = np.concatenate((np.broadcast_to(state[:2],
                                         (len(passage_tested), 1, 2)),
                                         states[passage_tested, :, :2]), axis=1)
                relative = points-passage.center
                along = relative@passage.tangent
                across = (relative[:, :, 0]*passage.tangent[1]-
                          relative[:, :, 1]*passage.tangent[0])
                crossing = (along[:, :-1] <= 0.) & (along[:, 1:] >= 0.)
                fraction = np.clip(-along[:, :-1]/np.maximum(
                    along[:, 1:]-along[:, :-1], 1e-9), 0., 1.)
                lateral_at_crossing = (across[:, :-1]+fraction*
                                       (across[:, 1:]-across[:, :-1]))
                through_gap = np.any(crossing & (np.abs(lateral_at_crossing) <
                                                 passage.free_width/2), axis=1)
                # Verify room beyond the rollout endpoint with the same
                # observed obstacles and twin-hull geometry. This cheap probe
                # only ranks a hypothesis; the full rollout remains the gate.
                extension = np.repeat(terminal, 2, axis=0)
                extension[:, :2] += passage.tangent[None, :]*np.tile([.6, 1.2], len(terminal))[:, None]
                extension[:, 2] = passage_heading
                extension_clearance = self.safety_clearance(
                    extension, obs, observation.width, observation.height).reshape(-1, 2).min(axis=1)
                viable = ((closest[passage_tested] >= cfg.margin) &
                          (extension_clearance >= cfg.margin) &
                          through_gap & (forward > .4) & (lateral < .8) &
                          (heading_error < .45))
                passage_viable[passage_tested] = viable
                via_center = (np.maximum(0., -signed)+np.linalg.norm(goal-passage.center))/p.cruise_speed_m_s
                beyond = np.linalg.norm(goal-terminal[:, :2])/p.cruise_speed_m_s
                passage_eta = horizon*3*self.dt+np.where(signed > 0., beyond, via_center)
                passage_eta += terms['speed_recovery_s'][passage_tested]
                terms['eta_s'][passage_tested] = np.where(
                    viable, np.minimum(terms['eta_s'][passage_tested], passage_eta),
                    terms['eta_s'][passage_tested])
                if families is not None:
                    families[passage_tested[viable]] = 'PASSAGE'
            if cfg.goal_entry_heading:
                heading_quality = entry_heading_quality(
                    states, goal, self.goal_heading, float(np.linalg.norm(goal-state[:2])),
                    goal_radius=cfg.goal_radius_m, center_entry=cfg.center_entry)
                terms['quality'] += heading_quality
                if cfg.center_entry:
                    terms['eta_s'] += .6*heading_quality
            ranking_clearance = closest
            if cfg.recovery_policy:
                terminal_s, _, _ = route_coordinates(states[:, -1, :2], path, geometry[2])
                safe_forward = ((closest >= cfg.margin+.05) &
                                (sequences[:, 0, 0] > .2*p.cruise_speed_m_s) &
                                (states[:, -1, 3] > .2*p.cruise_speed_m_s) &
                                ((terminal_s-vessel_s[0] > .5) | terms['captured']))
                if np.count_nonzero(safe_forward) >= 3:
                    reverse_family = ((sequences[:, 0, 0] < 0.) |
                                      (np.mean(sequences[:, :, 0], axis=1) < 0.))
                    ranking_clearance = np.where(reverse_family, -np.inf, closest)
            best, feasible = select_eta(terms, ranking_clearance, cfg.margin)
            if (cfg.continuity and feasible[0] and
                    terms['eta_s'][0] <= terms['eta_s'][best]+.08):
                best = 0
            if cfg.strategy_guidance and self.strategy is not None:
                same = feasible & (families == self.strategy)
                if np.any(same):
                    same_best, _ = select_eta(terms, np.where(same, closest, -np.inf),
                                              cfg.margin)
                    safety_escape = (closest[same_best] < cfg.margin+.08 and
                                     closest[best] > closest[same_best]+.15)
                    if (not safety_escape and
                            terms['eta_s'][same_best] <= terms['eta_s'][best]+.18):
                        best = same_best
            self.last_eta_s = float(terms['eta_s'][best])
            if cfg.forward_policy and not terms['captured'][best]:
                from forward_policy import choose_forward
                best, sequences, states, closest, families, policy = choose_forward(
                    self, state, observation, goal, path, geometry[2], passage,
                    sequences, states, closest, families, terms['quality'], best,
                    open_direct, prior_passage_commitment, float(vessel_s[0]))
                self.last_forward_diagnostics = policy
                if best >= cfg.samples:
                    probe_terms = arrival_terms(
                        states[best:best+1], sequences[best:best+1], path,
                        geometry[2], goal, p, 3*self.dt, self.previous,
                        self.sequence, goal_radius=cfg.goal_radius_m)
                    self.last_eta_s = float(probe_terms['eta_s'][0])
            elif cfg.forward_policy:
                self.last_forward_diagnostics = dict(
                    activated=False, probes=0, selected_source='goal_entry',
                    selected_forward=True)
            if getattr(self, 'record_diagnostics', False):
                self.last_diagnostics = {**terms, 'clearance': closest.copy(),
                                         'commands': sequences[:, 0].copy(), 'selected': best,
                                         'passage_slots': passage_slots.copy(),
                                         'passage_viable': passage_viable.copy()}
        else:
            best = int(np.argmin(np.where(feasible, costs, np.inf))) if np.any(feasible) else int(np.argmax(closest))
        selected, prediction = sequences[best].copy(), states[best].copy()
        selected_clearance = closest[best]
        if np.any(feasible) and cfg.objective == 'eta' and not (cfg.multimodal or cfg.continuity or cfg.strategy_guidance):
            near = feasible & (terms['eta_s'] <= np.min(terms['eta_s'][feasible])+.12)
            weights = np.where(near, np.exp(np.clip(
                -(terms['quality']-terms['quality'][best])/cfg.temperature, -80., 0.)), 0.)
            proposal = np.sum(weights[:,None,None]*sequences, axis=0)/weights.sum()
            pstate, pcl = self.rollout(state, proposal[None], obs, observation.width, observation.height, goal)
            pt = arrival_terms(pstate, proposal[None], path, geometry[2], goal, p, 3*self.dt,
                               self.previous, self.sequence, terminal_value=cfg.terminal_value,
                               obstacles=obs if cfg.viability else None,
                               width=observation.width, height=observation.height,
                               margin=cfg.margin, goal_radius=cfg.goal_radius_m)
            if (pcl[0] >= cfg.margin and pt['eta_s'][0] <= np.min(terms['eta_s'][feasible])+.12
                    and pt['quality'][0] <= terms['quality'][best]):
                selected, prediction, selected_clearance = proposal, pstate[0], pcl[0]
                self.last_eta_s = float(pt['eta_s'][0])
        if np.any(feasible) and cfg.objective != 'eta':
            weights = np.where(feasible, np.exp(np.clip(-(costs-costs[best])/cfg.temperature, -80., 0.)), 0.)
            proposal = np.sum(weights[:, None, None]*sequences, axis=0)/weights.sum()
            proposal_states, proposal_clearance = self.rollout(state, proposal[None, :], obs, observation.width, observation.height)
            proposal_cost = objective(proposal[None, :], proposal_states, proposal_clearance)[0]
            # Averaging safe left/right paths is not necessarily safe. Recheck it.
            if proposal_clearance[0] >= cfg.margin and proposal_cost <= costs[best]:
                selected, prediction = proposal, proposal_states[0]
                selected_clearance = proposal_clearance[0]
        self.sequence, self.previous = selected, selected[0].copy()
        if cfg.strategy_guidance and families is not None:
            self.strategy = str(families[best])
            if cfg.passage_guidance:
                self.passage_commitment = (passage.center.copy() if
                                           passage is not None and self.strategy == 'PASSAGE'
                                           else None)
        if cfg.objective == 'eta' and not cfg.center_entry:
            inside = np.sum((prediction[:, :2]-goal)**2, axis=1) < cfg.goal_radius_m**2
            if np.any(inside):
                prediction = prediction[:int(np.argmax(inside))+1]
        self.timings['plan'].append(time.perf_counter()-started)
        if self.mode == 'trajectory_control':
            target = prediction[min(8, len(prediction)-1), :2].copy()
        return selected[0], prediction, target, float(selected_clearance)
