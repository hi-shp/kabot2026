"""Fused CPU implementation of the existing fixed-step vessel rollout.

The reference NumPy implementation in SamplingNavigator.rollout remains the
fallback and the equivalence oracle. This kernel keeps every physics and hull
sample; it only removes thousands of Python/NumPy dispatches per plan.
"""
import math
import numpy as np
from isv_water_trials.codex_nav.passage_geometry import surface_clearance_at_pose, fast_surface_clearance_at_pose

try:
    from numba import njit
except ImportError:  # The experiment still runs without the optional accelerator.
    njit = None


def parameter_vector(p):
    gain = (p.yaw_inertia_kg_m2 / p.yaw_response_s if p.yaw_rate_gain_Nm_s is None
            else p.yaw_rate_gain_Nm_s)
    return np.asarray((p.mass_kg, p.yaw_inertia_kg_m2,
                       p.surge_linear_drag, p.surge_quadratic_drag,
                       p.sway_linear_drag, p.sway_quadratic_drag,
                       p.yaw_linear_drag, p.yaw_quadratic_drag,
                       p.thruster_arm_m, p.max_thrust_N,
                       p.actuator_tau_s, p.speed_response_s, gain), dtype=np.float64)


if njit is not None:
    @njit(cache=True)
    def compiled_rollout(initial, sequences, obstacles, width, height, dt, params,
                         goal_x=0., goal_y=0., goal_radius=0., hull_polygons=None,
                         geometry_stats=None, hull_edges=None, hull_bound=0.,
                         hull_box=None, initial_states=None, exact_walls=False):
        count, horizon, _ = sequences.shape
        states = np.empty((count, horizon, 8), dtype=np.float64)
        closest = np.full(count, np.inf)
        finished = np.zeros(count, dtype=np.bool_)
        z = np.empty((count, 8), dtype=np.float64)
        for i in range(count):
            for j in range(8):
                z[i, j] = initial[j] if initial_states is None else initial_states[i, j]
        mass, inertia = params[0], params[1]
        surge_lin, surge_quad = params[2], params[3]
        sway_lin, sway_quad = params[4], params[5]
        yaw_lin, yaw_quad = params[6], params[7]
        arm, max_thrust = params[8], params[9]
        alpha = -math.expm1(-dt / params[10])
        speed_response, yaw_gain = params[11], params[12]
        for i in range(count):
            for t in range(horizon):
                x, y, h, u, v, r, fl, fr = (z[i, 0], z[i, 1], z[i, 2], z[i, 3],
                                           z[i, 4], z[i, 5], z[i, 6], z[i, 7])
                speed, yaw_command = sequences[i, t, 0], sequences[i, t, 1]
                for substep in range(3):
                    force = surge_lin*u + surge_quad*u*abs(u) + mass*(speed-u)/speed_response
                    moment = yaw_lin*r + yaw_quad*r*abs(r) + yaw_gain*(yaw_command-r)
                    diff = max(-max_thrust, min(max_thrust, moment/(2*arm)))
                    common = max(-max_thrust+abs(diff),
                                 min(max_thrust-abs(diff), force/2))
                    left = max(-max_thrust, min(max_thrust, common-diff))
                    right = max(-max_thrust, min(max_thrust, common+diff))
                    next_fl = fl + alpha*(left-fl)
                    next_fr = fr + alpha*(right-fr)

                    c, s = math.cos(h), math.sin(h)
                    dx1, dy1, dh1 = u*c-v*s, u*s+v*c, r
                    du1 = ((next_fl+next_fr-surge_lin*u-surge_quad*u*abs(u))/mass+v*r)
                    dv1 = ((-sway_lin*v-sway_quad*v*abs(v))/mass-u*r)
                    dr1 = (((next_fr-next_fl)*arm-yaw_lin*r-yaw_quad*r*abs(r))/inertia)
                    hm, um, vm, rm = (h+.5*dt*dh1, u+.5*dt*du1,
                                      v+.5*dt*dv1, r+.5*dt*dr1)
                    cm, sm = math.cos(hm), math.sin(hm)
                    x += dt*(um*cm-vm*sm)
                    y += dt*(um*sm+vm*cm)
                    h += dt*rm
                    u += dt*((next_fl+next_fr-surge_lin*um-surge_quad*um*abs(um))/mass+vm*rm)
                    v += dt*((-sway_lin*vm-sway_quad*vm*abs(vm))/mass-um*rm)
                    r += dt*(((next_fr-next_fl)*arm-yaw_lin*rm-yaw_quad*rm*abs(rm))/inertia)
                    fl, fr = next_fl, next_fr

                    if hull_polygons is not None:
                        if hull_edges is not None:
                            margin = fast_surface_clearance_at_pose(
                                x, y, h, obstacles, width, height, hull_edges,
                                hull_bound, hull_box,
                                closest[i],
                                None if geometry_stats is None else geometry_stats[i],exact_walls)
                        else:
                            margin = surface_clearance_at_pose(
                                x, y, h, obstacles, width, height, hull_polygons,
                                None if geometry_stats is None else geometry_stats[i])
                    else:
                        margin = min(x-.93, width-.93-x, y-.93, height-.93-y)
                        c, s = math.cos(h), math.sin(h)
                        for j in range(len(obstacles)):
                            ox, oy, radius = obstacles[j, 0], obstacles[j, 1], obstacles[j, 2]
                            dx, dy = ox-x, oy-y
                            # Entire twin-capsule hull lies within 0.93 m of the
                            # vessel center. A farther circle cannot lower the
                            # best swept clearance already found, so avoid its
                            # trig/projection/hypot work without dropping checks.
                            reach = closest[i]+radius+.93
                            if reach > 0. and dx*dx+dy*dy > reach*reach:
                                continue
                            along, lateral = dx*c+dy*s, -dx*s+dy*c
                            gap = max(-.56-along, along-.52, 0.)
                            side = min(abs(lateral-.22), abs(lateral+.22))
                            margin = min(margin, math.hypot(gap, side)-radius-.32)
                    if not finished[i]:
                        # Include the goal-entry step in safety. The mission
                        # ends there, so post-finish boundary crossings do not
                        # invalidate an otherwise safe fly-through arrival.
                        closest[i] = min(closest[i], margin)
                        if goal_radius > 0. and (x-goal_x)**2+(y-goal_y)**2 < goal_radius**2:
                            finished[i] = True
                states[i, t, 0], states[i, t, 1] = x, y
                states[i, t, 2], states[i, t, 3] = h, u
                states[i, t, 4], states[i, t, 5] = v, r
                states[i, t, 6], states[i, t, 7] = fl, fr
                for j in range(8):
                    z[i, j] = states[i, t, j]
        return states, closest

else:
    compiled_rollout = None
