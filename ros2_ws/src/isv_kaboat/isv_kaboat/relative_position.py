import math
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
from typing import Dict, List, Optional

def objs_to_rel_matrix_single(
    objs: List[Dict],
    CLASS_RADIUS_M: Optional[Dict[int, float]] = None,
    DEFAULT_RADIUS_M: float = 1.0,
) -> xp.ndarray:
    """
    라이다 기준 장애물 중심 및 표면점 행렬 생성.

    입력:
      - objs: [{"x": float, "y": float, "z": float, "class_id": int}, ...]

    처리(라이다 기준):
      1) 중심 좌표 p_center = (x_b, y_b, z_b)
      2) 각 장애물마다 반경 r_obs 결정
         - CLASS_RADIUS_M에 있으면 해당 값
         - 없으면 DEFAULT_RADIUS_M 사용
      3) LiDAR에서 선박(자선) 진행방향 x축, 좌측 y축 기준:
         - 각도 theta = atan2(y_b, x_b)
         - 표면점 p_surface = p_center - r_obs * [cos(theta), sin(theta), 0]

    출력:
      - (N x 7) xp.ndarray:
        [x_b, y_b, z_b, x_surf, y_surf, z_surf, r_obs]
    """
    rows = []

    for obj in objs:
        x_b = float(obj.get("x", 0.0))
        y_b = float(obj.get("y", 0.0))
        z_b = float(obj.get("z", 0.0))
        class_id = int(obj.get("class_id", -1))

        if CLASS_RADIUS_M is not None and class_id in CLASS_RADIUS_M:
            r_obs = float(CLASS_RADIUS_M[class_id])
        else:
            r_obs = float(DEFAULT_RADIUS_M)

        theta = math.atan2(y_b, x_b)
        x_surf = x_b - r_obs * math.cos(theta)
        y_surf = y_b - r_obs * math.sin(theta)
        z_surf = z_b  # z는 그대로

        rows.append([x_b, y_b, z_b, x_surf, y_surf, z_surf, r_obs])

    if not rows:
        return xp.zeros((0, 7), dtype=float)

    return xp.asarray(rows, dtype=float)


############################################################ 선박의... 상대위치 ##########################################################




