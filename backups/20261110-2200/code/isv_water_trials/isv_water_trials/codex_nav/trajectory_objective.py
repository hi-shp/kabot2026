"""Fly-through arrival value in seconds; no pursuit point or docking cost."""
import numpy as np

GOAL_RADIUS_M = 1.4  # Existing 70 px / 50 px-per-m mission radius.


def flythrough_goal_reached(distance_m):
    """Original mission boundary, independent of heading and controller ETA."""
    return distance_m < GOAL_RADIUS_M


def _terminal_extension(end, path, arc, nearest, distance2, physics, goal_radius):
    """Cheap moving-turn ETA from a terminal state; no extra rollout or target."""
    segment = np.minimum(nearest, len(path)-2)
    direction = np.arctan2(path[segment+1, 1]-path[segment, 1],
                           path[segment+1, 0]-path[segment, 0])
    next_segment = np.minimum(segment+1, len(path)-2)
    next_direction = np.arctan2(path[next_segment+1, 1]-path[next_segment, 1],
                                path[next_segment+1, 0]-path[next_segment, 0])
    angle = np.abs(np.arctan2(np.sin(end[:, 2]-direction), np.cos(end[:, 2]-direction)))
    bend = np.abs(np.arctan2(np.sin(next_direction-direction), np.cos(next_direction-direction)))
    vertex_distance = np.linalg.norm(end[:, :2]-path[segment+1], axis=1)
    required = angle+np.clip(1.-vertex_distance/3., 0., 1.)*bend
    remaining = np.maximum(0., arc[-1]-arc[nearest]-goal_radius)
    distance = remaining+np.sqrt(distance2[np.arange(len(end)), nearest])
    cruise = physics.cruise_speed_m_s
    speed = np.maximum(0., end[:, 3])
    # Body sway changes immediate world travel direction, even at fixed bow.
    drift = np.arctan2(end[:, 4], np.maximum(speed, .1))
    required += .35*np.abs(drift)
    turn_sign = np.sign(np.sin(direction-end[:, 2]))
    aligned_yaw = turn_sign*end[:, 5]
    aligned_thrust = turn_sign*(end[:, 7]-end[:, 6])*physics.thruster_arm_m
    response = physics.yaw_response_s+physics.actuator_tau_s

    def moving_turn(speed_ratio, rate_ratio):
        rate = rate_ratio*physics.max_yaw_rate_rad_s
        turning_speed = np.minimum(np.maximum(speed, .2*cruise), speed_ratio*cruise)
        yaw_delay = response*np.clip(1.-aligned_yaw/rate, 0., 2.)
        yaw_delay *= np.clip(1.-aligned_thrust/(2*physics.max_thrust_N*physics.thruster_arm_m), .5, 1.5)
        turn_time = required/rate+np.minimum(yaw_delay, required/rate)
        # Forward travel during turning is subtracted before straight travel.
        forward_gain = turning_speed*np.maximum(0., np.sin(np.minimum(required, np.pi)))/rate
        straight = np.maximum(0., distance-forward_gain)/cruise
        braking = response*np.maximum(0., speed-speed_ratio*cruise)/cruise
        acceleration = physics.speed_response_s*np.maximum(0., cruise-turning_speed)/cruise
        return turn_time+straight+braking+acceleration

    wide = moving_turn(1., .7)
    tight = moving_turn(.65, 1.)
    return np.minimum(wide, tight), wide, tight


def viability_penalty(end, obstacles, width, height, margin):
    """Approximate one/two-second continuation room using perceived circles."""
    speed = np.maximum(end[:, 3], 0.)
    # Three possible forward continuations: hold heading or steer either way.
    offsets = np.array([0., -.35, .35])
    times = np.array([1., 2.])
    heading = end[:, None, None, 2]+end[:, None, None, 5]*times[None, :, None]+offsets[None, None, :]*times[None, :, None]
    travel = speed[:, None, None]*times[None, :, None]
    x = end[:, None, None, 0]+travel*np.cos(heading)
    y = end[:, None, None, 1]+travel*np.sin(heading)
    clearance = np.minimum.reduce(np.broadcast_arrays(x-.93, width-.93-x, y-.93, height-.93-y))
    if len(obstacles):
        separation = np.hypot(x[..., None]-obstacles[None, None, None, :, 0],
                              y[..., None]-obstacles[None, None, None, :, 1])
        clearance = np.minimum(clearance, np.min(separation-obstacles[None, None, None, :, 2]-.7, axis=-1))
    best = np.max(np.min(clearance, axis=1), axis=1)
    return 2.*np.clip((margin+.2-best)/.5, 0., 1.)


def entry_heading_quality(states, goal, heading, distance_now,
                          goal_radius=GOAL_RADIUS_M, center_entry=False):
    """Tie-break heading at first goal entry, with a gradual near-goal ramp."""
    if heading is None:
        return np.zeros(len(states))
    inside = np.sum((states[:, :, :2]-goal)**2, axis=2) < goal_radius**2
    first = inside.argmax(axis=1)
    sample = states[np.arange(len(states)), np.where(inside.any(axis=1), first, states.shape[1]-1)]
    desired = heading
    if center_entry:
        toward_center = goal-sample[:, :2]
        center_bearing = np.arctan2(toward_center[:, 1], toward_center[:, 0])
        # At the exact center the bearing is ill-defined; use the frozen
        # incoming route tangent there.
        desired = np.where(np.linalg.norm(toward_center, axis=1) > .12,
                           center_bearing, heading)
    error = np.arctan2(np.sin(sample[:, 2]-desired), np.cos(sample[:, 2]-desired))
    weight = np.clip((12.-distance_now)/8., 0., 1.)
    return weight*(1.2*(1.-np.cos(error))+.2*sample[:, 5]**2)


def arrival_terms(states, sequences, path, arc, goal, physics, knot_dt, previous=None, warm_start=None,
                  terminal_value=False, obstacles=None, width=None, height=None, margin=.2,
                  goal_radius=GOAL_RADIUS_M):
    """Finite rollout time plus a route-based terminal remaining-time estimate.

    Speed deficit represents the distance lost while recovering cruise speed,
    not a permanent low speed for the entire remaining route. Orientation adds
    a finite turn-time estimate. This is a heuristic, not optimal travel time.
    """
    end = states[:, -1]
    distance2 = np.sum((end[:, None, :2]-path[None, :, :])**2, axis=2)
    nearest = distance2.argmin(axis=1)
    segment = np.minimum(nearest, len(path)-2)
    tangent = path[segment+1]-path[segment]
    direction = np.arctan2(tangent[:, 1], tangent[:, 0])
    angle = np.arctan2(np.sin(end[:, 2]-direction), np.cos(end[:, 2]-direction))
    cruise = physics.cruise_speed_m_s
    remaining = np.maximum(0., arc[-1]-arc[nearest]-goal_radius)
    corridor_distance = np.sqrt(distance2[np.arange(len(end)), nearest])
    useful_speed = np.maximum(0., end[:, 3])
    speed_recovery = physics.speed_response_s*np.maximum(0., cruise-useful_speed)/cruise
    # A forward arc advances while turning. Integrating its tangent progress
    # yields the lost-distance time (theta-sin(theta))/yaw_rate, rather than
    # treating every heading correction as a stop-and-turn maneuver.
    turn_time = (np.abs(angle)-np.sin(np.abs(angle)))/physics.max_yaw_rate_rad_s
    eta = states.shape[1]*knot_dt + (remaining+corridor_distance)/cruise + speed_recovery + turn_time
    if terminal_value:
        extension, wide, tight = _terminal_extension(end, path, arc, nearest, distance2, physics, goal_radius)
        eta = states.shape[1]*knot_dt+extension
    if obstacles is not None:
        eta += viability_penalty(end, obstacles, width, height, margin)
    inside = np.sum((states[:, :, :2]-goal)**2, axis=2) < goal_radius**2
    captured = inside.any(axis=1)
    capture_time = (inside.argmax(axis=1)+1)*knot_dt
    eta = np.where(captured, capture_time, eta)
    anchored = sequences if previous is None else np.concatenate(
        [np.broadcast_to(previous, (len(sequences),1,2)), sequences], axis=1)
    delta = np.diff(anchored, axis=1)
    quality = np.sum(delta[:, :, 1]**2, axis=1)
    quality += .2*np.sum(delta[:, :, 0]**2, axis=1)
    quality += .02*np.sum(np.abs(sequences[:, :, 1]), axis=1)*knot_dt
    quality += np.sum(np.maximum(-states[:, :, 3], 0.), axis=1)*knot_dt
    if warm_start is not None:
        quality += np.mean(np.sum((sequences-warm_start[None, :, :])**2, axis=2), axis=1)
    result = dict(eta_s=eta, remaining_m=remaining, speed_recovery_s=speed_recovery,
                turn_s=turn_time, captured=captured, quality=quality,
                terminal_speed=end[:, 3], corridor_distance_m=corridor_distance)
    if terminal_value:
        result.update(wide_fast_s=wide, tight_slow_s=tight)
    return result


def select_eta(terms, clearance, margin, tie_s=.12):
    safe = np.isfinite(terms['eta_s']) & (clearance >= margin)
    if not np.any(safe):
        return int(np.argmax(clearance)), safe
    earliest = np.min(terms['eta_s'][safe])
    near = safe & (terms['eta_s'] <= earliest+tie_s)
    return int(np.argmin(np.where(near, terms['quality'], np.inf))), safe
