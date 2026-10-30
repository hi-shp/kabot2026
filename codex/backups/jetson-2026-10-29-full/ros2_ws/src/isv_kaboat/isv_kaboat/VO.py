# import numpy as np
# from typing import Dict, List, Tuple, Optional
# import yaml
# import math

# # ------------------------------------------------------------
# # [유틸] 각도 차이 계산 (Target - Current) -> -180 ~ 180 정규화
# # ------------------------------------------------------------
# def get_angle_diff(target_deg: float, current_deg: float) -> float:
#     """
#     ENU 절대각 두 개를 받아, current 기준 target이 어느 방향인지 계산합니다.
#     Return:
#         (+) : 우측(CW) 회전 필요
#         (-) : 좌측(CCW) 회전 필요
#     """
#     diff = target_deg - current_deg
#     # -180 ~ 180 범위로 정규화
#     diff = (diff + 180.0) % 360.0 - 180.0
#     return diff


# # ------------------------------------------------------------
# # 0) LiDAR XY 기반 블라인드 각도 계산 유틸
# # ------------------------------------------------------------
# def compute_blocked_angle_intervals_from_xy(
#     points_xy: np.ndarray,
#     *,
#     cam_fov_deg: float = 90.0,
#     r_min: float = 0.0,
#     r_max: float = 1.0,
#     cluster_gap_deg: float = 3.0
# ) -> List[Tuple[float, float]]:
#     if points_xy is None or points_xy.size == 0:
#         return []

#     xs = points_xy[:, 0]
#     ys = points_xy[:, 1]
#     rs = np.hypot(xs, ys)
#     # x=전방, y=좌측 가정 (LiDAR Frame, CCW+)
#     thetas = np.degrees(np.arctan2(ys, xs))  # [-180, 180]

#     half_fov = cam_fov_deg * 0.5
#     mask = (rs >= r_min) & (rs <= r_max) & (np.abs(thetas) > half_fov)
#     th_f = thetas[mask]
#     if th_f.size == 0:
#         return []

#     th_sorted = np.sort(th_f)
#     blocked_intervals: List[Tuple[float, float]] = []
#     cluster_start = float(th_sorted[0])
#     prev_theta = float(th_sorted[0])

#     for theta in th_sorted[1:]:
#         theta = float(theta)
#         if (theta - prev_theta) > cluster_gap_deg:
#             blocked_intervals.append((cluster_start, prev_theta))
#             cluster_start = theta
#         prev_theta = theta

#     blocked_intervals.append((cluster_start, prev_theta))
#     return blocked_intervals


# def heading_blocked(
#     heading_deg: float,
#     blocked_intervals: Optional[List[Tuple[float, float]]],
#     margin_deg: float = 0.0,
# ) -> bool:
#     if not blocked_intervals:
#         return False
#     for th_min, th_max in blocked_intervals:
#         if heading_deg >= (th_min - margin_deg) and heading_deg <= (th_max + margin_deg):
#             return True
#     return False


# # ------------------------------------------------------------
# # 1) 가능한 속도장(VO 비위반 후보들) 생성
# # ------------------------------------------------------------
# def get_feasible_velocity_field(
#     objs: List[Dict],
#     v_ship_obs_xy: np.ndarray,
#     *,
#     config_path: str = "vo_config.yaml",
#     ship_heading_deg: float = 0.0,
# ) -> dict:
#     """
#     LiDAR 프레임에서 VO 비위반(안전) 후보 속도장과 디버깅 맵을 생성한다.
#     """
#     try:
#         with open(config_path, "r", encoding="utf-8") as f:
#             cfg = yaml.safe_load(f) or {}
#     except:
#         cfg = {}
#     vo = cfg.get("vo", {})

#     # 파라미터
#     class_radius_m = vo.get("class_radius_m", {})
#     class_radius_m = {int(k): float(v) for k, v in class_radius_m.items()}
#     default_radius_m = float(vo.get("default_radius_m", 0.5))

#     tau = float(vo.get("tau", 5.0))
#     v_max = float(vo.get("v_max", 4.0))
#     v_allow_max = float(vo.get("v_allow_max", v_max))
#     min_speed_mps = float(vo.get("min_speed_mps", 0.2))
#     heading_samples = int(vo.get("heading_samples", 360))
#     speed_samples = int(vo.get("speed_samples", 6))

#     lidar_to_enu_deg_static = float(vo.get("lidar_to_enu_deg", 0.0))
#     lidar_heading_offset_deg = float(vo.get("lidar_heading_offset_deg", 0.0))

#     ship_radius_m = float(vo.get("ship_radius_m", 1.0))
#     convert_from_ship_wrt_obs = bool(vo.get("convert_from_ship_wrt_obs", True))
#     vo_angle_margin_deg = float(vo.get("vo_angle_margin_deg", 5.0))
#     EPS = 1e-9
#     range_max_for_vo_m = float(vo.get("range_max_for_vo_m", 2.5))

#     def rot2d(rad: float) -> np.ndarray:
#         c, s = np.cos(rad), np.sin(rad)
#         return np.array([[c, -s], [s, c]], dtype=float)

#     # VO Cone Check
#     def in_cone_and_ttc(
#         v_ship: np.ndarray,
#         p: np.ndarray,
#         R_combined: float,
#         v_obs_ship: np.ndarray,
#         tau: float,
#     ) -> Tuple[bool, Optional[float]]:
#         v_rel = v_ship - v_obs_ship
#         d = np.linalg.norm(p)

#         if d <= R_combined:
#             return True, 0.0

#         sin_alpha = np.clip(R_combined / (d + EPS), 0.0, 1.0)
#         alpha = np.arcsin(sin_alpha)

#         p_hat = p / (d + EPS)
#         along = float(np.dot(v_rel, p_hat))

#         if along <= 0.0:
#             return False, None

#         nvr = np.linalg.norm(v_rel)
#         if nvr < EPS:
#             return False, None

#         cos_th = np.clip(np.dot(v_rel / (nvr + EPS), p_hat), -1.0, 1.0)
#         theta = np.arccos(cos_th)

#         margin_rad = np.deg2rad(vo_angle_margin_deg)
#         if theta > (alpha + margin_rad):
#             return False, None

#         ttc = d / (along + EPS)
#         if 0.0 < ttc <= tau:
#             return True, ttc
#         return False, None

#     # LiDAR → ENU Rotation
#     heading_nav = (float(ship_heading_deg) + lidar_heading_offset_deg) % 360.0
#     heading_math_deg = (90.0 - heading_nav) % 360.0
#     heading_math_rad = np.deg2rad(heading_math_deg)
#     R_L2E = rot2d(heading_math_rad)

#     # Data Parsing
#     p_surface_xyz: List[List[float]] = []
#     v_obs_ship_xyz_l: List[List[float]] = []
#     obs_info_list: List[Dict] = []

#     for o, v_ship_obs in zip(objs, v_ship_obs_xy):
#         x_l = float(o.get("x", 0.0))
#         y_l = float(o.get("y", 0.0))
#         z_l = float(o.get("z", 0.0))
#         class_id = int(o.get("class_id", -1))
#         obs_r_val = float(class_radius_m.get(class_id, default_radius_m))

#         r_lidar = (x_l ** 2 + y_l ** 2 + z_l ** 2) ** 0.5
#         if r_lidar >= range_max_for_vo_m:
#             continue

#         p_surface_xyz.append([x_l, y_l, z_l])
        
#         v_ship_obs = np.asarray(v_ship_obs, dtype=float)
#         if v_ship_obs.shape[0] == 2:
#             v_ship_obs = np.array([v_ship_obs[0], v_ship_obs[1], 0.0], dtype=float)

#         v_obs_ship = -v_ship_obs if convert_from_ship_wrt_obs else v_ship_obs
#         v_obs_ship_xyz_l.append(v_obs_ship.tolist())

#         # 각 장애물의 거리(r)와 방향(rel_angle_deg) 저장 (Cost 계산용)
#         obs_info_list.append({
#             "r": r_lidar,
#             "rel_angle_deg": np.degrees(np.arctan2(y_l, x_l)), 
#             "obs_radius": obs_r_val,
#         })

#     p_surface_xyz = np.asarray(p_surface_xyz, dtype=float)
#     v_obs_ship_xyz_l = np.asarray(v_obs_ship_xyz_l, dtype=float)

#     obs_list = []
#     for i in range(len(p_surface_xyz)):
#         total_radius = ship_radius_m + obs_info_list[i]["obs_radius"]
#         obs_list.append((p_surface_xyz[i], total_radius, v_obs_ship_xyz_l[i]))

#     # Grid Generation
#     vmax_use = float(min(v_allow_max, v_max))
#     headings_deg = np.linspace(-180.0, 180.0, heading_samples, endpoint=False)
#     speeds = np.linspace(0.0, vmax_use, speed_samples)

#     H, S = len(headings_deg), len(speeds)
#     vel_field_lidar = np.zeros((H, S, 2), dtype=float)
#     safe_mask = np.zeros((H, S), dtype=bool)
#     ttc_min_map = np.full((H, S), np.inf, dtype=float)
#     viol_idx_map = -np.ones((H, S), dtype=int)

#     # VO Check Loop
#     for hi, th_deg in enumerate(headings_deg):
#         th = np.deg2rad(th_deg)
#         ct, st = np.cos(th), np.sin(th)

#         for si, sp in enumerate(speeds):
#             vc = np.array([sp * ct, sp * st], dtype=float)
#             vel_field_lidar[hi, si, :] = vc

#             if sp <= 0.01:
#                 safe_mask[hi, si] = True
#                 continue

#             violated = False
#             min_ttc = np.inf
#             worst_k = -1

#             for k, (p, R_combined, v_obs_ship) in enumerate(obs_list):
#                 vc3 = np.array([vc[0], vc[1], 0.0], dtype=float)
#                 inside, ttc = in_cone_and_ttc(vc3, p, R_combined, v_obs_ship, tau)

#                 if inside:
#                     violated = True
#                     if ttc is not None and ttc < min_ttc:
#                         min_ttc = ttc
#                         worst_k = k

#             if not violated:
#                 safe_mask[hi, si] = True
#             else:
#                 ttc_min_map[hi, si] = min_ttc
#                 viol_idx_map[hi, si] = worst_k

#     return {
#         "headings_deg": headings_deg,
#         "speeds": speeds,
#         "vel_field_lidar": vel_field_lidar,
#         "safe_mask": safe_mask,
#         "ttc_min_map": ttc_min_map,
#         "viol_idx_map": viol_idx_map,
#         "R_L2E": R_L2E,
#         "obs_info_list": obs_info_list,
#         "meta": {
#             "tau": tau,
#             "v_max": v_max,
#             "v_allow_max": v_allow_max,
#             "vmax_use": vmax_use,
#             "ship_radius_m": ship_radius_m,
#             "min_speed_mps": min_speed_mps,
#             "w_angle": float(vo.get("w_angle", 1.0)),
#             "w_speed": float(vo.get("w_speed", 0.5)),
#             "w_obs": float(vo.get("w_obs", 20.0)),         # 🔹 [NEW] 장애물 회피 가중치
#             "obs_sigma_deg": float(vo.get("obs_sigma_deg", 25.0)) # 🔹 [NEW] 장애물 영향 범위(각도)
#         },
#     }


# # ------------------------------------------------------------
# # 2) 속도장 위에서 최적 속도 선택 (Cost Function 수정됨)
# # ------------------------------------------------------------
# def select_optimal_velocity_from_field_target(
#     field: dict,
#     *,
#     heading_target_deg: float,
#     min_speed_mps: float = 0.2,
#     blocked_intervals: Optional[List[Tuple[float, float]]] = None,
#     block_margin_deg: float = 0.0,
# ) -> Tuple[float, float]:
#     """
#     Cost Function = w_angle * (angle_err)^2 
#                   + w_speed * (speed_err)^2 
#                   + w_obs   * sum( (1/dist) * Gaussian(angle_diff) )
#     """
#     def enu_nav_deg_to_math_rad(hdeg: float) -> float:
#         return np.deg2rad((90.0 - hdeg) % 360.0)

#     def enu_math_rad_to_nav_deg(hrad: float) -> float:
#         return (90.0 - np.degrees(hrad)) % 360.0

#     def ang_wrap_rad(a: float) -> float:
#         return (a + np.pi) % (2 * np.pi) - np.pi

#     def ang_wrap_deg(a: float) -> float:
#         return (a + 180.0) % 360.0 - 180.0

#     headings_deg = field["headings_deg"]
#     speeds = field["speeds"]
#     vel_field_lidar = field["vel_field_lidar"]
#     safe_mask = field["safe_mask"]
#     ttc_min_map = field.get("ttc_min_map", None)
#     R_L2E = field["R_L2E"]
#     meta = field.get("meta", {})
#     obs_info_list = field.get("obs_info_list", [])

#     if "min_speed_mps" in meta:
#         try: min_speed_mps = float(meta["min_speed_mps"])
#         except: pass

#     target_rad_math = enu_nav_deg_to_math_rad(float(heading_target_deg))

#     H, S = len(headings_deg), len(speeds)

#     # Cost Weights
#     w_angle = float(meta.get("w_angle", 1.0))
#     w_speed = float(meta.get("w_speed", 0.5))
#     vmax_use = float(meta.get("vmax_use", np.max(speeds) if len(speeds) else 1.0))

#     # 🔹 [NEW] 장애물 회피 전용 가중치 설정 (기본값 강화)
#     w_obs = float(meta.get("w_obs", 20.0))       # 장애물 코스트 스케일
#     obs_sigma = float(meta.get("obs_sigma_deg", 25.0)) # 장애물 영향 각도 범위 (표준편차)
    
#     # Sigma 값 보정 (0이 되지 않도록)
#     if obs_sigma < 1.0: obs_sigma = 1.0

#     safe_candidates = []
#     risky_candidates = []

#     for hi in range(H):
#         heading_deg_lidar = float(headings_deg[hi])

#         # 물리적 블라인드 처리 (센서 가림 등)
#         if heading_blocked(heading_deg_lidar, blocked_intervals, block_margin_deg):
#             continue
            
#         # 현재 후보 방향 (LiDAR Frame, -180~180)
#         vel_angle_deg = ang_wrap_deg(heading_deg_lidar)

#         for si in range(S):
#             vc_lidar = vel_field_lidar[hi, si, :]
#             vc_enu = R_L2E @ vc_lidar
#             spd = float(np.linalg.norm(vc_enu))
            
#             if spd <= 1e-9:
#                 continue

#             if safe_mask[hi, si]:
#                 # 1. Target Heading Cost
#                 vel_rad_math = np.arctan2(vc_enu[1], vc_enu[0])
#                 ang_err_rad = abs(ang_wrap_rad(vel_rad_math - target_rad_math))
                
#                 # 2. Speed Cost
#                 speed_err = max(0.0, vmax_use - spd)
#                 if spd < min_speed_mps:
#                     speed_err += (min_speed_mps - spd) * 5.0
                
#                 # 3. 🔹 [NEW] Obstacle Proximity & Direction Cost
#                 obs_cost_sum = 0.0
                
#                 for obs in obs_info_list:
#                     r_o = obs["r"]
#                     if r_o > 4.0: continue  # 너무 먼 장애물은 코스트 계산 생략 (최적화)
                    
#                     ang_o = obs["rel_angle_deg"]
                    
#                     # 각도 차이 (절댓값)
#                     diff_deg = abs(ang_wrap_deg(vel_angle_deg - ang_o))
                    
#                     # 영향 범위(2 * sigma) 밖이면 무시 (최적화)
#                     if diff_deg > (2.5 * obs_sigma):
#                         continue
                        
#                     # 거리 팩터 (가까울수록 매우 커짐, 최소거리 0.5m로 clamp하여 발산 방지)
#                     dist_factor = 1.0 / max(r_o, 0.5)
                    
#                     # 각도 팩터 (Gaussian 분포: 정면일 때 1.0, 멀어지면 0.0)
#                     angle_factor = math.exp(-0.5 * (diff_deg / obs_sigma) ** 2)
                    
#                     # 최종 합산
#                     obs_cost_sum += (dist_factor * angle_factor)

#                 # Total Cost
#                 cost = (w_angle * (ang_err_rad ** 2)) + \
#                        (w_speed * (speed_err ** 2)) + \
#                        (w_obs * obs_cost_sum)

#                 safe_candidates.append((cost, spd, vc_lidar))
#             else:
#                 if ttc_min_map is not None:
#                     ttc = ttc_min_map[hi, si]
#                     if np.isfinite(ttc):
#                         risky_candidates.append((float(ttc), spd, vc_lidar))

#     chosen = None
#     if safe_candidates:
#         # Cost 오름차순 정렬 (가장 작은 Cost 선택)
#         safe_candidates.sort(key=lambda x: x[0])
#         chosen = safe_candidates[0][2]

#     if chosen is None and risky_candidates:
#         # 안전한 후보가 없으면 TTC가 가장 긴 것(가장 늦게 부딪히는 것) 선택
#         risky_candidates.sort(key=lambda x: -x[0])
#         chosen = risky_candidates[0][2]

#     if chosen is None:
#         chosen = np.array([0.0, 0.0], dtype=float)

#     # 결과 변환
#     v_cmd_enu = R_L2E @ chosen
#     speed_cmd = float(np.linalg.norm(v_cmd_enu))

#     if speed_cmd > 1e-6:
#         heading_cmd_deg_out = float(
#             enu_math_rad_to_nav_deg(np.arctan2(v_cmd_enu[1], v_cmd_enu[0]))
#         )
#     else:
#         heading_cmd_deg_out = float(heading_target_deg)

#     return speed_cmd, heading_cmd_deg_out


# # ------------------------------------------------------------
# # 3) 외부 호출 래퍼 (최종 인터페이스)
# # ------------------------------------------------------------
# def compute_vo_velocity_for_target_proj(
#     objs: List[Dict],
#     v_ship_obs_xy: np.ndarray,
#     *,
#     heading_target_deg: float,
#     config_path: str = "vo_config.yaml",
#     blocked_intervals: Optional[List[Tuple[float, float]]] = None,
#     block_margin_deg: float = 0.0,
#     lidar_points_xy: Optional[np.ndarray] = None,
#     cam_fov_deg: float = 90.0,
#     lidar_r_min: float = 0.0,
#     lidar_r_max: float = 1.0,
#     lidar_cluster_gap_deg: float = 3.0,
#     ship_heading_deg: float = 0.0,
#     max_steer_deg: float = 60.0
# ) -> Tuple[float, float]:
#     """
#     Return: (optimal_speed, relative_steering_angle)
#     """

#     # 1. 장애물이 없는 경우
#     if not objs and (lidar_points_xy is None or lidar_points_xy.size == 0):
#         try:
#             with open(config_path, "r", encoding="utf-8") as f:
#                 cfg = yaml.safe_load(f) or {}
#             v_max = float(cfg.get("vo", {}).get("v_allow_max", 4.0))
#         except:
#             v_max = 4.0
            
#         relative_cmd = get_angle_diff(heading_target_deg, ship_heading_deg)
#         relative_cmd = max(min(relative_cmd, max_steer_deg), -max_steer_deg)
#         return v_max, float(relative_cmd)

#     # 2. LiDAR 포인트로 블라인드 구간 계산
#     if blocked_intervals is None and lidar_points_xy is not None:
#         blocked_intervals = compute_blocked_angle_intervals_from_xy(
#             lidar_points_xy,
#             cam_fov_deg=cam_fov_deg,
#             r_min=lidar_r_min,
#             r_max=lidar_r_max,
#             cluster_gap_deg=lidar_cluster_gap_deg,
#         )

#     # 3. VO 필드 생성
#     field = get_feasible_velocity_field(
#         objs,
#         v_ship_obs_xy,
#         config_path=config_path,
#         ship_heading_deg=ship_heading_deg, 
#     )

#     # 4. 최적 '절대' 속도/각도 계산 (장애물 거리/방향 코스트 반영됨)
#     opt_speed, opt_abs_heading_deg = select_optimal_velocity_from_field_target(
#         field,
#         heading_target_deg=heading_target_deg,
#         blocked_intervals=blocked_intervals,
#         block_margin_deg=block_margin_deg,
#     )

#     # 5. 상대 조향각 계산 및 제한
#     relative_steering_cmd = get_angle_diff(opt_abs_heading_deg, ship_heading_deg)
#     relative_steering_cmd = max(min(relative_steering_cmd, max_steer_deg), -max_steer_deg)

#     return opt_speed, float(relative_steering_cmd)

import os
import numpy as np
try:
    _use_cupy = bool(int(os.environ.get("VO_USE_CUPY", "0")))
except Exception:
    _use_cupy = False

if _use_cupy:
    try:
        import cupy as cp  # type: ignore
        xp = cp
    except Exception:
        xp = np
else:
    xp = np
from typing import Dict, List, Tuple, Optional
import yaml
import math

# ------------------------------------------------------------
# [유틸] 각도 차이 계산 (Target - Current) -> -180 ~ 180 정규화
# ------------------------------------------------------------
def get_angle_diff(target_deg: float, current_deg: float) -> float:
    """
    ENU 절대각 두 개를 받아, current 기준 target이 어느 방향인지 계산합니다.
    Return:
        (+) : 우측(CW) 회전 필요
        (-) : 좌측(CCW) 회전 필요
    """
    diff = target_deg - current_deg
    # -180 ~ 180 범위로 정규화
    diff = (diff + 180.0) % 360.0 - 180.0
    return diff


# ------------------------------------------------------------
# LiDAR XY 포인트로부터 블라인드(가려진) 각도 구간 계산
# ------------------------------------------------------------
def compute_blocked_angle_intervals_from_xy(
    points_xy: xp.ndarray,
    cam_fov_deg: float = 90.0,
    r_min: float = 0.0,
    r_max: float = 1.0,
    cluster_gap_deg: float = 3.0,
    min_cluster_width_deg: float = 1.0,
    min_cluster_points: int = 2,
) -> List[Tuple[float, float]]:
    """
    LiDAR 기준 XY 포인트들을 받아, '시야가 가려지는(블라인드) 각도 구간'을 계산합니다.

    입력:
      - points_xy: (N,2) [x,y] (LiDAR Frame)
      - cam_fov_deg: 카메라 FOV (±cam_fov_deg/2 범위만 고려)
      - r_min, r_max: 거리 필터
      - cluster_gap_deg: 각도 클러스터 간격 (deg)

    출력:
      - blocked_intervals: [ (start_deg, end_deg), ... ] (LiDAR 기준, -180~180)
    """

    points_xy = xp.asarray(points_xy) if points_xy is not None else None
    if points_xy is None or points_xy.size == 0:
        return []

    xs = points_xy[:, 0]
    ys = points_xy[:, 1]
    rs = xp.hypot(xs, ys)
    # x=전방, y=좌측 가정 (LiDAR Frame, CCW+)
    thetas = xp.degrees(xp.arctan2(ys, xs))  # [-180, 180]

    # 거리 필터
    mask_r = (rs >= r_min) & (rs <= r_max)
    xs = xs[mask_r]
    ys = ys[mask_r]
    rs = rs[mask_r]
    thetas = thetas[mask_r]

    if thetas.size == 0:
        return []

    # 카메라 FOV 범위 내 각도만 사용
    half_fov = cam_fov_deg * 0.5
    mask_fov = (thetas >= -half_fov) & (thetas <= half_fov)
    thetas = thetas[mask_fov]
    rs      = rs[mask_fov]

    if thetas.size == 0:
        return []

    # 각도 기준 정렬
    order = xp.argsort(thetas)
    thetas = thetas[order]
    rs     = rs[order]

    # 각도 클러스터링
    clusters: List[List[float]] = []
    current_cluster: List[float] = []

    prev_theta = float(thetas[0])
    current_cluster.append(prev_theta)

    for th in thetas[1:]:
        th_val = float(th)
        if abs(th_val - prev_theta) <= cluster_gap_deg:
            current_cluster.append(th_val)
        else:
            clusters.append(current_cluster)
            current_cluster = [th_val]
        prev_theta = th_val

    if current_cluster:
        clusters.append(current_cluster)

    # 각 클러스터를 "가려진 각도 구간"으로 변환
    blocked_intervals: List[Tuple[float, float]] = []
    for c in clusters:
        if not c:
            continue
        th_min = float(xp.min(xp.asarray(c)))
        th_max = float(xp.max(xp.asarray(c)))
        ang_width = th_max - th_min

        # 너무 좁거나 포인트 개수가 적으면 블라인드로 보지 않음
        if ang_width < min_cluster_width_deg or len(c) < min_cluster_points:
            continue

        blocked_intervals.append((th_min, th_max))

    return blocked_intervals


# ------------------------------------------------------------
# VO 필드(속도 샘플 그리드) 생성
# ------------------------------------------------------------
def get_feasible_velocity_field(
    objs: List[Dict],
    v_ship_obs_xy: xp.ndarray,
    *,
    config_path: str = "vo_config.yaml",
    ship_heading_deg: float = 0.0,
) -> Dict:
    """
    장애물들에 대한 VO(Field of Feasible Velocities)를 계산합니다.
    반환:
      {
        "headings_deg": (H,),
        "speeds": (S,),
        "vel_field_lidar": (H,S,2),
        "safe_mask": (H,S),
        "ttc_min_map": (H,S),
        "viol_idx_map": (H,S),
        "obs_info_list": list[ dict(...) ],
      }
    """
    # Config 로드
    with open(config_path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}

    vo_cfg = cfg.get("vo", {})
    meta = cfg.get("meta", {})

    tau = float(vo_cfg.get("tau", 5.0))
    v_max = float(vo_cfg.get("v_max", 4.0))
    v_allow_max = float(vo_cfg.get("v_allow_max", v_max))
    min_speed_mps = float(vo_cfg.get("min_speed_mps", 0.2))
    heading_samples = int(vo_cfg.get("heading_samples", 360))
    speed_samples = int(vo_cfg.get("speed_samples", 6))

    lidar_to_enu_deg_static = float(vo_cfg.get("lidar_to_enu_deg", 0.0))
    lidar_heading_offset_deg = float(vo_cfg.get("lidar_heading_offset_deg", 0.0))

    ship_radius_m = float(vo_cfg.get("ship_radius_m", 1.0))
    convert_from_ship_wrt_obs = bool(vo_cfg.get("convert_from_ship_wrt_obs", True))
    vo_angle_margin_deg = float(vo_cfg.get("vo_angle_margin_deg", 5.0))
    range_max_for_vo_m = float(vo_cfg.get("range_max_for_vo_m", 2.5))

    # 장애물 클래스별 반경
    class_radius_m = {
        int(k): float(v)
        for k, v in vo_cfg.get("class_radius_m", {}).items()
    }
    default_radius_m = float(vo_cfg.get("default_radius_m", 0.1))

    EPS = 1e-9

    def rot2d(rad: float) -> xp.ndarray:
        c, s = xp.cos(rad), xp.sin(rad)
        R = xp.zeros((2, 2), dtype=float)
        R[0, 0] = c
        R[0, 1] = -s
        R[1, 0] = s
        R[1, 1] = c
        return R

    # VO 판정 함수들 --------------------------------------------------

    def in_cone_and_ttc(
        v_ship: xp.ndarray,
        p: xp.ndarray,
        R_combined: float,
        v_obs_ship: xp.ndarray,
        tau: float,
    ) -> Tuple[bool, Optional[float]]:
        # p: [x,y,z], v_ship, v_obs_ship: [x,y,z]
        v_rel = v_ship - v_obs_ship
        v_rel_xy = v_rel[:2]
        p_xy = p[:2]

        v_rel_norm = float(xp.linalg.norm(v_rel_xy) + EPS)
        R_over_v = R_combined / v_rel_norm

        cross = v_rel_xy[0] * p_xy[1] - v_rel_xy[1] * p_xy[0]
        dist = abs(cross) / v_rel_norm

        # VO 콘 내에 있는지 간단 체크
        if dist > R_combined:
            return False, None

        # Time to Collision
        a = float(xp.dot(v_rel_xy, v_rel_xy))
        b = 2.0 * float(xp.dot(p_xy, v_rel_xy))
        c = float(xp.dot(p_xy, p_xy) - R_combined * R_combined)

        disc = b * b - 4.0 * a * c
        if disc < 0.0 or abs(a) < EPS:
            return False, None

        sqrt_disc = math.sqrt(disc)
        t1 = (-b - sqrt_disc) / (2.0 * a)
        t2 = (-b + sqrt_disc) / (2.0 * a)

        t_candidates = [t for t in (t1, t2) if t >= 0.0]
        if not t_candidates:
            return False, None

        t_min = min(t_candidates)
        if t_min > tau:
            return False, None

        return True, t_min

    # 장애물 정보 전처리 -----------------------------------------------
    obs_info_list = []

    for obj in objs:
        x_b = float(obj.get("x", 0.0))
        y_b = float(obj.get("y", 0.0))
        z_b = float(obj.get("z", 0.0))
        cid = int(obj.get("class_id", -1))

        r_obs = float(class_radius_m.get(cid, default_radius_m))

        # 범위 제한
        dist_xy = math.hypot(x_b, y_b)
        if dist_xy > range_max_for_vo_m:
            continue

        obs_info_list.append(
            {
                "p_b": xp.array([x_b, y_b, z_b], dtype=float),
                "class_id": cid,
                "obs_radius": r_obs,
            }
        )

    if not obs_info_list:
        # 장애물 없으면 필드 비어있는 형태로 리턴
        headings_deg = xp.linspace(-180.0, 180.0, heading_samples, endpoint=False)
        speeds = xp.linspace(0.0, v_allow_max, speed_samples)

        H, S = len(headings_deg), len(speeds)
        vel_field_lidar = xp.zeros((H, S, 2), dtype=float)
        safe_mask = xp.ones((H, S), dtype=bool)
        ttc_min_map = xp.full((H, S), xp.inf, dtype=float)
        viol_idx_map = -xp.ones((H, S), dtype=int)

        return {
            "headings_deg": headings_deg,
            "speeds": speeds,
            "vel_field_lidar": vel_field_lidar,
            "safe_mask": safe_mask,
            "ttc_min_map": ttc_min_map,
            "viol_idx_map": viol_idx_map,
            "obs_info_list": obs_info_list,
        }

    # ENU 변환 (선박 heading 사용)
    ship_heading_rad = xp.deg2rad(ship_heading_deg)
    R_enu = rot2d(ship_heading_rad)

    p_surface_xyz = []
    v_obs_ship_xyz_l = []

    v_ship_obs_xy = xp.asarray(v_ship_obs_xy, dtype=float)
    if v_ship_obs_xy.ndim == 1:
        v_ship_obs_xy = v_ship_obs_xy.reshape(1, -1)
    if v_ship_obs_xy.size == 0:
        v_ship_obs_xy = xp.zeros((0, 2), dtype=float)

    for idx, info in enumerate(obs_info_list):
        p_b = info["p_b"]  # [x_b, y_b, z_b]
        r_obs = info["obs_radius"]

        # 선박 주행 방향 기준 표면점 계산
        theta_b = xp.arctan2(p_b[1], p_b[0])
        x_surf = p_b[0] - r_obs * xp.cos(theta_b)
        y_surf = p_b[1] - r_obs * xp.sin(theta_b)
        z_surf = p_b[2]

        p_surf_b = xp.asarray([x_surf, y_surf], dtype=float)
        p_surf_enu = R_enu @ p_surf_b
        p_surf_xyz = xp.zeros(3, dtype=float)
        p_surf_xyz[0:2] = p_surf_enu
        p_surf_xyz[2] = z_surf
        p_surface_xyz.append(p_surf_xyz)

        # 속도 (관측자 기준 -> 선박 기준)
        if idx < v_ship_obs_xy.shape[0]:
            v_row = v_ship_obs_xy[idx]
        else:
            v_row = xp.zeros((v_ship_obs_xy.shape[1] if v_ship_obs_xy.ndim > 1 else 2,), dtype=float)

        v_row_xy = xp.asarray(v_row[:2], dtype=float).reshape(-1)
        v_obs_ship_xy = -v_row_xy if convert_from_ship_wrt_obs else v_row_xy

        v_obs_ship_xyz = xp.zeros(3, dtype=float)
        v_obs_ship_xyz[:2] = v_obs_ship_xy[:2]
        v_obs_ship_xyz_l.append(v_obs_ship_xyz)

    # VO 계산용 리스트
    obs_list = []
    for i in range(len(p_surface_xyz)):
        total_radius = ship_radius_m + float(obs_info_list[i]["obs_radius"])
        obs_list.append((p_surface_xyz[i], total_radius, v_obs_ship_xyz_l[i]))

    # 격자 생성 (GPU 브로드캐스트용)
    vmax_use = float(min(v_allow_max, v_max))
    headings_deg = xp.linspace(-180.0, 180.0, heading_samples, endpoint=False)
    speeds = xp.linspace(0.0, vmax_use, speed_samples)

    H, S = len(headings_deg), len(speeds)

    headings_rad = xp.deg2rad(headings_deg)
    ct = xp.cos(headings_rad)[:, None]  # (H,1)
    st = xp.sin(headings_rad)[:, None]  # (H,1)
    sp = speeds[None, :]                # (1,S)

    vel_field_lidar = xp.zeros((H, S, 2), dtype=float)
    vel_field_lidar[..., 0] = sp * ct
    vel_field_lidar[..., 1] = sp * st

    # 초기 마스크/TT C맵
    safe_mask = xp.ones((H, S), dtype=bool)
    ttc_min_map = xp.full((H, S), xp.inf, dtype=float)
    viol_idx_map = -xp.ones((H, S), dtype=int)

    # 최소 속력 이하 그리드는 무조건 안전 처리
    safe_mask &= (sp >= min_speed_mps)

    # GPU 브로드캐스트로 장애물별 VO 위반 계산
    for obs_idx, (p_surf, R_combined, v_obs_ship) in enumerate(obs_list):
        p_xy = p_surf[:2]  # shape (2,)
        v_obs_xy = v_obs_ship[:2]  # shape (2,)

        p_norm_sq = xp.sum(p_xy * p_xy)
        p_norm = xp.sqrt(p_norm_sq) + EPS
        p_hat = p_xy / p_norm

        # 상대속도: (H,S,2)
        v_rel = vel_field_lidar - v_obs_xy
        v_rel_norm_sq = xp.sum(v_rel * v_rel, axis=-1)
        v_rel_norm = xp.sqrt(v_rel_norm_sq) + EPS

        # 코너 거리 기준 (cross product 기반 거리)
        cross = v_rel[..., 0] * p_hat[1] - v_rel[..., 1] * p_hat[0]
        dist = xp.abs(cross) / v_rel_norm
        inside_cone = dist <= R_combined

        # TTC 계산
        dot_p_v = v_rel[..., 0] * p_xy[0] + v_rel[..., 1] * p_xy[1]
        a = v_rel_norm_sq
        b = 2.0 * dot_p_v
        c = p_norm_sq - R_combined * R_combined

        disc = b * b - 4.0 * a * c
        disc = xp.maximum(disc, 0.0)
        sqrt_disc = xp.sqrt(disc)

        t1 = (-b - sqrt_disc) / (2.0 * a + EPS)
        t2 = (-b + sqrt_disc) / (2.0 * a + EPS)

        t1 = xp.where(t1 >= 0.0, t1, xp.inf)
        t2 = xp.where(t2 >= 0.0, t2, xp.inf)
        ttc = xp.minimum(t1, t2)

        inside_ttc = (ttc <= tau)
        violated = inside_cone & inside_ttc

        # ttc 업데이트
        better = violated & (ttc < ttc_min_map)
        ttc_min_map = xp.where(better, ttc, ttc_min_map)
        viol_idx_map = xp.where(better, obs_idx, viol_idx_map)
        safe_mask = safe_mask & (~violated)

    return {
        "headings_deg": headings_deg,
        "speeds": speeds,
        "vel_field_lidar": vel_field_lidar,
        "safe_mask": safe_mask,
        "ttc_min_map": ttc_min_map,
        "viol_idx_map": viol_idx_map,
        "obs_info_list": obs_info_list,
    }


# ------------------------------------------------------------
# VO 필드 + 목표각 + 블라인드 구간 -> 최적 속도/각도 선택
# ------------------------------------------------------------
def select_optimal_velocity_from_field_target(
    field: Dict,
    *,
    heading_target_deg: float,
    blocked_intervals: Optional[List[Tuple[float, float]]] = None,
    block_margin_deg: float = 0.0,
) -> Tuple[float, float]:
    """
    VO 필드와 목표각, 블라인드 각도 정보를 바탕으로
    "안전하고 장애물 회피에 유리한" 속도/각도를 선택합니다.
    """
    headings_deg = field["headings_deg"]
    speeds = field["speeds"]
    vel_field_lidar = field["vel_field_lidar"]
    safe_mask = field["safe_mask"]
    ttc_min_map = field["ttc_min_map"]
    viol_idx_map = field["viol_idx_map"]
    obs_info_list = field["obs_info_list"]

    # 장애물 관련 메타 파라미터
    w_obs_angle = 20.0          # 장애물 각도 기반 코스트 가중치
    w_obs_dist = 5.0            # 장애물 거리 기반 코스트 가중치 (1/dist)
    obs_sigma_deg = 25.0
    if obs_sigma_deg < 1.0:
        obs_sigma_deg = 1.0
    obs_sigma = math.radians(obs_sigma_deg)

    # 속도 관련 메타 파라미터
    # 최고 속도와의 차이를 제곱해서 코스트로 사용
    if len(speeds) > 0:
        v_max = float(max(speeds))   # 최고 속도
    else:
        v_max = 0.0
    w_v = 0.01   # 속도 코스트 가중치 (단위/스케일에 맞게 튜닝 필요)

    # 블라인드 구간 처리: 시야가 가려진 각도는 페널티
    def is_blocked_angle(angle_deg: float) -> bool:
        if not blocked_intervals:
            return False
        a = angle_deg
        for (b_start, b_end) in blocked_intervals:
            if b_start <= a <= b_end:
                return True
            if block_margin_deg > 0.0:
                if (b_start - block_margin_deg) <= a <= (b_end + block_margin_deg):
                    return True
        return False

    safe_candidates = []
    risky_candidates = []

    H = len(headings_deg)
    S = len(speeds)

    for hi in range(H):
        heading_deg_lidar = float(headings_deg[hi])

        # 블라인드 각도인지 체크
        if is_blocked_angle(heading_deg_lidar):
            continue

        for si in range(S):
            sp = float(speeds[si])
            if sp <= 0.0:
                continue

            if safe_mask[hi, si]:
                # -----------------------------
                # 1) 목표각과의 차이 코스트
                # -----------------------------
                rel = get_angle_diff(heading_target_deg, heading_deg_lidar)
                cost_heading = abs(rel)

                # -----------------------------
                # 2) 장애물 각도/거리 코스트
                # -----------------------------
                obs_cost_angle = 0.0
                obs_cost_dist = 0.0

                worst_idx = int(viol_idx_map[hi, si])
                if 0 <= worst_idx < len(obs_info_list):
                    obs = obs_info_list[worst_idx]
                    p_b = obs["p_b"]
                    # r_obs = obs["obs_radius"]  # 필요시 사용
                    dist = float(xp.linalg.norm(p_b[:2]))  # 장애물까지의 거리

                    # 각도 기반 코스트 (장애물 방향과 진행 방향이 비슷하면 penalty ↑)
                    theta_obs = math.degrees(math.atan2(p_b[1], p_b[0]))
                    dtheta_deg = get_angle_diff(heading_deg_lidar, theta_obs)
                    dtheta = math.radians(dtheta_deg)

                    obs_cost_angle = (
                        w_obs_angle
                        * math.exp(-0.5 * (dtheta / obs_sigma) ** 2)
                        / (dist + 1e-6)
                    )

                    # 거리만 보는 코스트: 가까울수록 1/dist가 커짐
                    obs_cost_dist = w_obs_dist / (dist + 1e-6)

                # -----------------------------
                # 3) 속도 코스트: (v - v_max)^2
                # -----------------------------
                cost_speed = w_v * (sp - v_max) ** 2

                # -----------------------------
                # 총 코스트
                # -----------------------------
                total_cost = cost_heading + obs_cost_angle + obs_cost_dist + cost_speed
                safe_candidates.append((total_cost, sp, heading_deg_lidar))

            else:
                # 위험 후보: TTC + 속도까지 포함해서 랭킹
                ttc_val = float(ttc_min_map[hi, si])
                if not math.isfinite(ttc_val):
                    continue

                rel = get_angle_diff(heading_target_deg, heading_deg_lidar)
                cost_heading = abs(rel)
                cost_ttc = 1.0 / (ttc_val + 1e-6)

                # 속도 코스트 동일하게 적용
                cost_speed = w_v * (sp - v_max) ** 2

                total_cost = cost_heading + 10.0 * cost_ttc + cost_speed
                risky_candidates.append((total_cost, sp, heading_deg_lidar))

    # 우선 안전 후보 중 최저 코스트 선택
    if safe_candidates:
        safe_candidates.sort(key=lambda x: x[0])
        _, best_sp, best_heading = safe_candidates[0]
        return float(best_sp), float(best_heading)

    # 안전 후보가 없다면 위험 후보 중에서 고름
    if risky_candidates:
        risky_candidates.sort(key=lambda x: x[0])
        _, best_sp, best_heading = risky_candidates[0]
        return float(best_sp), float(best_heading)

    # 그래도 없다면 최소 전진 속도로 목표 각도 유지 (완전 정지 방지)
    try:
        positive_speeds = [float(s) for s in speeds if float(s) > 0.0]
        fallback_speed = min(positive_speeds) if positive_speeds else 0.0
    except Exception:
        fallback_speed = 0.0
    return float(fallback_speed), float(heading_target_deg)



# ------------------------------------------------------------
# 최종 인터페이스 함수:
#   장애물 + LiDAR 블라인드 + 목표각 -> 속도/상대조향
# ------------------------------------------------------------
def compute_vo_velocity_for_target_proj(
    objs: List[Dict],
    v_ship_obs_xy: xp.ndarray,
    *,
    heading_target_deg: float,
    config_path: str = "vo_config.yaml",
    blocked_intervals: Optional[List[Tuple[float, float]]] = None,
    block_margin_deg: float = 0.0,
    lidar_points_xy: Optional[xp.ndarray] = None,
    cam_fov_deg: float = 90.0,
    lidar_r_min: float = 0.0,
    lidar_r_max: float = 1.0,
    lidar_cluster_gap_deg: float = 3.0,
    ship_heading_deg: float = 0.0,
    max_steer_deg: float = 30.0,
) -> Tuple[float, float]:
    """
    상위 레벨에서 사용하기 위한 최종 함수입니다.

    - objs: LiDAR 기준 (x,y,z,class_id)
    - v_ship_obs_xy: 선박 속도 (관측자 기준, XY)
    - heading_target_deg: 목표 절대각(ENU 기준)
    - config_path: vo_config.yaml 경로
    - blocked_intervals: 이미 계산된 블라인드 각도 구간 (옵션)
    - lidar_points_xy: LiDAR XY 포인트 (옵션, 블라인드 구간 자동계산용)
    - ship_heading_deg: 현재 선박 heading(ENU)
    - max_steer_deg: 조타 제한 (상대각도 절대값 최대치)
    """

    # 장애물도 없고 LiDAR 포인트도 없으면 단순히 목표각만 향하도록
    if not objs and (lidar_points_xy is None or lidar_points_xy.size == 0):
        try:
            with open(config_path, "r", encoding="utf-8") as f:
                cfg = yaml.safe_load(f) or {}
            v_max = float(cfg.get("vo", {}).get("v_allow_max", 4.0))
        except Exception:
            v_max = 4.0

        relative_cmd = get_angle_diff(heading_target_deg, ship_heading_deg)
        return float(v_max), float(relative_cmd)

    # LiDAR 포인트로 블라인드 구간 자동 계산
    if blocked_intervals is None and lidar_points_xy is not None:
        blocked_intervals = compute_blocked_angle_intervals_from_xy(
            lidar_points_xy,
            cam_fov_deg=cam_fov_deg,
            r_min=lidar_r_min,
            r_max=lidar_r_max,
            cluster_gap_deg=lidar_cluster_gap_deg,
        )

    # VO 필드 생성
    field = get_feasible_velocity_field(
        objs,
        v_ship_obs_xy,
        config_path=config_path,
        ship_heading_deg=ship_heading_deg,
    )

    # VO 필드 + 블라인드 구간 + 목표각을 이용해 최적 속도/각도 선택
    opt_speed, opt_abs_heading_deg = select_optimal_velocity_from_field_target(
        field,
        heading_target_deg=heading_target_deg,
        blocked_intervals=blocked_intervals,
        block_margin_deg=block_margin_deg,
    )

    # 상대 조향각으로 변환 및 제한
    relative_steering_cmd = get_angle_diff(opt_abs_heading_deg, ship_heading_deg)
    relative_steering_cmd = max(min(relative_steering_cmd, max_steer_deg), -max_steer_deg)

    return float(opt_speed), float(relative_steering_cmd)
