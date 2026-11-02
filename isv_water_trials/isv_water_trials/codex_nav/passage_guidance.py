"""Observed, orientation-aware gap hypotheses for the opt-in passage mode.

Geometry only proposes a route. The existing per-step physical swept-hull
rollout makes the final safety decision.
"""
from dataclasses import dataclass
import numpy as np
from isv_water_trials.codex_nav.passage_geometry import projected_width

# Existing twin capsules: centerlines at lateral ±0.22 m, axial endpoints
# -0.56/+0.52 m and radius 0.32 m. Their projected full width is the capsule
# diameter plus the centerline's lateral and longitudinal projections.
CAPSULE_RADIUS_M = .32
HALF_HULL_SPACING_M = .22
HALF_AXIAL_SPAN_M = .54


def required_width(heading_error, safety_margin):
    """Projected twin-hull width across a passage, including hard clearance."""
    angle = np.asarray(heading_error)
    return 2*(CAPSULE_RADIUS_M+HALF_HULL_SPACING_M*np.abs(np.cos(angle))+
              HALF_AXIAL_SPAN_M*np.abs(np.sin(angle))+safety_margin)


@dataclass(frozen=True)
class Passage:
    center: np.ndarray
    tangent: np.ndarray
    free_width: float
    required_width: float
    current_required_width: float
    obstacle_pair: tuple = ()  # indices in this planning update's observed array
    wall: str | None = None

    @property
    def requires_alignment(self):
        return self.free_width <= self.current_required_width


def observed_passages(obstacles, position, heading, goal, safety_margin, max_range=6.8,
                      hull_polygons=None, sort=True):
    """Recompute each observed circle-pair gap and the current hull projection."""
    obstacles = np.asarray(obstacles, dtype=float).reshape(-1, 3)
    if len(obstacles) < 2:
        return []
    position = np.asarray(position, dtype=float)
    goal_direction = np.asarray(goal, dtype=float)-position
    goal_direction /= max(np.linalg.norm(goal_direction), 1e-9)
    first_indices, second_indices = np.triu_indices(len(obstacles), 1)
    first, second = obstacles[first_indices], obstacles[second_indices]
    between = second[:, :2]-first[:, :2]
    center_distance = np.linalg.norm(between, axis=1)
    across = between/np.maximum(center_distance[:, None], 1e-9)
    free_width = center_distance-first[:, 2]-second[:, 2]
    needed = float(required_width(0., safety_margin) if hull_polygons is None else
                   projected_width(0., hull_polygons)+2*safety_margin)
    tangent = np.column_stack((-across[:, 1], across[:, 0]))
    tangent[np.sum(tangent*goal_direction, axis=1) < 0.] *= -1.
    center = first[:, :2]+across*(first[:, 2]+free_width/2)[:, None]
    offset = center-position
    forward = offset@goal_direction
    lateral = np.abs(offset[:, 0]*goal_direction[1]-offset[:, 1]*goal_direction[0])
    valid = ((center_distance >= 1e-9) & (free_width > needed) &
             (tangent@goal_direction >= .65) & (.3 < forward) &
             (forward < max_range) & (lateral < 2.2))
    indices = np.flatnonzero(valid)
    if not len(indices):
        return []
    passage_heading = np.arctan2(tangent[indices, 1], tangent[indices, 0])
    current_needed = (required_width(heading-passage_heading, safety_margin)
                      if hull_polygons is None else
                      projected_width(heading-passage_heading, hull_polygons)+
                      2*safety_margin)
    passages = [Passage(center[index], tangent[index], float(free_width[index]), needed,
                        float(current_needed[k]),
                        (int(first_indices[index]), int(second_indices[index])))
                for k, index in enumerate(indices)]
    if sort:
        passages.sort(key=lambda passage: np.linalg.norm(passage.center-position)+
                      .5*np.linalg.norm(np.asarray(goal)-passage.center))
    return passages


def observed_wall_passages(obstacles, position, heading, goal, safety_margin,
                           width, height, hull_polygons, max_range=6.8, sort=True):
    """Surveyed walls paired with currently observed circle surfaces only."""
    result=[];position=np.asarray(position);goal=np.asarray(goal)
    goal_direction=(goal-position)/max(np.linalg.norm(goal-position),1e-9)
    needed=float(projected_width(0.,hull_polygons)+2*safety_margin)
    # These four directions and hull projections are identical for every
    # observed circle in this update. Reuse them without changing arithmetic.
    wall_geometry=[]
    for name,normal in (('left',np.array([1.,0.])),
                        ('right',np.array([-1.,0.])),
                        ('bottom',np.array([0.,1.])),
                        ('top',np.array([0.,-1.]))):
        tangent=np.array([-normal[1],normal[0]])
        if tangent@goal_direction<0:tangent=-tangent
        alignment=tangent@goal_direction
        angle=heading-np.arctan2(tangent[1],tangent[0])
        current=float(projected_width(angle,hull_polygons)+2*safety_margin)
        wall_geometry.append((name,normal,tangent,alignment,current))
    for i,(x,y,r) in enumerate(np.asarray(obstacles).reshape(-1,3)):
        boundaries=((x-r,np.array([0.,y])),
                    (width-x-r,np.array([width,y])),
                    (y-r,np.array([x,0.])),
                    (height-y-r,np.array([x,height])))
        for (gap,wall_point),(name,normal,tangent,alignment,current) in zip(boundaries,wall_geometry):
            if gap<=needed or alignment<.65:continue
            center=wall_point+normal*gap/2
            offset=center-position
            if (np.linalg.norm(offset)>max_range or offset@tangent<=.3):continue
            result.append(Passage(center,tangent,float(gap),needed,current,(i,),name))
    if sort:
        result.sort(key=lambda p:np.linalg.norm(p.center-position)+
                    .5*np.linalg.norm(goal-p.center))
    return result


def passage_sequences(state, passage, physics, horizon, knot_dt):
    """Fixed-size align/enter/release hypotheses; no point-tracking feedback."""
    heading = float(np.arctan2(passage.tangent[1], passage.tangent[0]))
    error = float(np.arctan2(np.sin(heading-state[2]), np.cos(heading-state[2])))
    approach = passage.center-state[:2]
    entry_bearing = float(np.arctan2(approach[1], approach[0]))
    entry_error = float(np.arctan2(np.sin(entry_bearing-state[2]),
                                   np.cos(entry_bearing-state[2])))
    align_knots = min(horizon//2, max(1, int(np.ceil(
        (abs(error)/physics.max_yaw_rate_rad_s+physics.yaw_response_s)/knot_dt))))
    commands = np.empty((6, horizon, 2), dtype=float)
    for index, (speed, gain, entry_weight) in enumerate((
            (1., .7, 0.), (1., 1., .35), (.8, .7, 0.),
            (.8, 1., .35), (1., 1.2, 0.), (.8, 1.2, .35))):
        commands[index, :, 0] = speed*physics.cruise_speed_m_s
        correction = (1-entry_weight)*error+entry_weight*entry_error
        rate = np.clip(gain*correction/physics.yaw_response_s-.4*state[5],
                       -physics.max_yaw_rate_rad_s, physics.max_yaw_rate_rad_s)
        commands[index, :align_knots, 1] = rate
        commands[index, align_knots:, 1] = 0.
    return commands
