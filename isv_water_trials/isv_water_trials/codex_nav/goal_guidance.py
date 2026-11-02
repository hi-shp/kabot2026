"""Mission-terminal state, separate from the fixed vessel dynamics.

The approach tangent is chosen once on entering the terminal zone. Replanning
may change the coarse route but must not flip the goal heading every cycle.
All distances and velocities are SI units.
"""
import math
import numpy as np

GOAL_ZONE_M = 12.0  # 1.5 m/s * (pi / 0.5 rad/s + 0.25 s lag) ≈ 9.8 m, plus margin
GOAL_CAPTURE_M = 1.0  # within the original 1.4 m outer goal region
HEADING_TOLERANCE_RAD = math.radians(35.)
YAW_RATE_TOLERANCE_RAD_S = .12
CROSS_TRACK_TOLERANCE_M = .8
LATERAL_SPEED_TOLERANCE_M_S = .35
MIN_FORWARD_SPEED_M_S = .05


def approach_heading(path, position, goal, existing=None):
    """Stable final route tangent, with a bearing fallback if no route exists."""
    if existing is not None or np.linalg.norm(goal-position) > GOAL_ZONE_M:
        return existing
    direction = None
    if path is not None and len(path) >= 2:
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
        start = int(np.searchsorted(arc, max(0., arc[-1]-2.0)))
        direction = goal-path[min(start, len(path)-2)]
    if direction is None or np.linalg.norm(direction) < .15:
        direction = goal-position
    return float(math.atan2(direction[1], direction[0]))


def terminal_cost(end_states, position, goal, heading):
    """Dimensionless position, orientation, yaw momentum and lateral errors.

    Evaluates the physical rollout endpoint. The gradual terminal-zone factor
    lets the command sequence begin turning before the vessel reaches the goal.
    """
    z = np.atleast_2d(end_states)
    distance_now = float(np.linalg.norm(position-goal))
    if heading is None:
        return np.zeros(len(z))
    factor = float(np.clip((GOAL_ZONE_M-distance_now)/4., 0., 1.))
    if factor == 0.:
        return np.zeros(len(z))
    d = np.linalg.norm(z[:, :2]-goal, axis=1)
    error = np.arctan2(np.sin(z[:, 2]-heading), np.cos(z[:, 2]-heading))
    c, s = math.cos(heading), math.sin(heading)
    cross = (z[:, 0]-goal[0])*(-s)+(z[:, 1]-goal[1])*c
    reverse = np.minimum(z[:, 3], 0.)
    return factor*(.35*(d/1.4)**2 + (error/.6)**2 +
                   .4*(z[:, 5]/.15)**2 + .4*(cross/.8)**2 +
                   .5*(reverse/.5)**2)


def goal_reached(state, goal, heading):
    """A centered, stable forward arrival; no scripted steering or pose changes."""
    if heading is None:
        return False
    z = np.asarray(state)
    delta = z[:2]-goal
    error = math.atan2(math.sin(z[2]-heading), math.cos(z[2]-heading))
    cross = -math.sin(heading)*delta[0]+math.cos(heading)*delta[1]
    return bool(np.linalg.norm(delta) <= GOAL_CAPTURE_M and
                abs(error) <= HEADING_TOLERANCE_RAD and
                abs(z[5]) <= YAW_RATE_TOLERANCE_RAD_S and
                abs(cross) <= CROSS_TRACK_TOLERANCE_M and
                abs(z[4]) <= LATERAL_SPEED_TOLERANCE_M_S and
                z[3] >= MIN_FORWARD_SPEED_M_S)
