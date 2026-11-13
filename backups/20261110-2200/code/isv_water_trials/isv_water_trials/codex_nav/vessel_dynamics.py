"""Planar surge/sway/yaw model, SI units internally and 50 px/m at the UI.

Diagonal mass/inertia and dissipative drag, with body-frame Coriolis terms.
The parameters are engineering assumptions, not an identified vessel model.
See https://www.fossen.biz/html/marineCraftModel.html for the model structure.
"""
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class VesselParameters:
    pixels_per_m: float = 50.0
    mass_kg: float = 10.0  # twice the original nominal mass (10)
    yaw_inertia_kg_m2: float = 6.0  # twice the original nominal inertia (4.5)
    surge_linear_drag: float = 1.0  # N/(m/s)
    surge_quadratic_drag: float = 18.0  # N/(m/s)^2
    sway_linear_drag: float = 55.0
    sway_quadratic_drag: float = 45.0
    yaw_linear_drag: float = 3.0  # Nm/(rad/s)
    yaw_quadratic_drag: float = 5.0
    thruster_arm_m: float = 0.22
    max_thrust_N: float = 25.0  # per thruster, forward or reverse
    actuator_tau_s: float = 0.25
    cruise_speed_m_s: float = 1.0
    max_yaw_rate_rad_s: float = 0.50
    yaw_response_s: float = 0.85
    speed_response_s: float = 1.5
    yaw_rate_gain_Nm_s: float | None = None  # explicit torque gain; None keeps legacy I/response

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            value = getattr(self,name)
            if name == "yaw_rate_gain_Nm_s" and value is None:
                continue
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')


def integrate(state, left, right, dt, p):
    """Vectorized midpoint integration, for both the vessel and predictions.

    state[..., :] = [x_m, y_m, heading, u_m_s, v_m_s, r_rad_s, left_N, right_N].
    Forces are bounded and lagged. No direct heading/position corrections.
    """
    state = np.asarray(state, dtype=float)
    alpha = -math.expm1(-dt / p.actuator_tau_s)
    left = np.clip(left, -p.max_thrust_N, p.max_thrust_N)
    right = np.clip(right, -p.max_thrust_N, p.max_thrust_N)
    fl = state[..., 6] + alpha * (left-state[..., 6])
    fr = state[..., 7] + alpha * (right-state[..., 7])

    def deriv(z):
        h, u, v, r = (z[..., i] for i in [2, 3, 4, 5])
        c, s = np.cos(h), np.sin(h)
        return np.stack((u*c-v*s, u*s+v*c, r,
                         (fl+fr-p.surge_linear_drag*u-p.surge_quadratic_drag*u*np.abs(u))/p.mass_kg+v*r,
                         (-p.sway_linear_drag*v-p.sway_quadratic_drag*v*np.abs(v))/p.mass_kg-u*r,
                         ((fr-fl)*p.thruster_arm_m-p.yaw_linear_drag*r-p.yaw_quadratic_drag*r*np.abs(r))/p.yaw_inertia_kg_m2), axis=-1)

    z = state[..., :6]
    next_z = z + dt*deriv(z + .5*dt*deriv(z))
    return np.concatenate((next_z, np.asarray(fl)[..., None], np.asarray(fr)[..., None]), axis=-1)


def allocate(state, speed, yaw_rate, p):
    """Speed and yaw-rate feedback with drag feedforward and thrust limits."""
    u, r = state[..., 3], state[..., 5]
    force = p.surge_linear_drag*u + p.surge_quadratic_drag*u*np.abs(u) + p.mass_kg*(speed-u)/p.speed_response_s
    # Preserve the legacy response design when no independent gain is supplied.
    # An explicit gain keeps torque authority independent of physical inertia.
    feedback_moment = (p.yaw_inertia_kg_m2*(yaw_rate-r)/p.yaw_response_s
                       if p.yaw_rate_gain_Nm_s is None
                       else p.yaw_rate_gain_Nm_s*(yaw_rate-r))
    moment = p.yaw_linear_drag*r + p.yaw_quadratic_drag*r*np.abs(r) + feedback_moment
    # Prioritize yaw authority; reduce surge if the pair would saturate.
    diff = np.clip(moment/(2*p.thruster_arm_m), -p.max_thrust_N, p.max_thrust_N)
    common = np.clip(force/2, -p.max_thrust_N+np.abs(diff), p.max_thrust_N-np.abs(diff))
    return common-diff, common+diff
