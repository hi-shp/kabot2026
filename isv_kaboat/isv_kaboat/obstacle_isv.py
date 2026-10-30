import math
from typing import List, Dict, Tuple, Optional


def _norm360(a: float) -> float:
    """0~360 범위로 정규화"""
    return (a % 360.0 + 360.0) % 360.0


def _angdiff(a: float, b: float) -> float:
    """
    a→b 최단각 (-180~180)
    a, b 모두 '북 기준 CW+ 0~360' 각도라고 가정.
    """
    d = (b - a + 180.0) % 360.0 - 180.0
    return d


def _bearing_body_to_world(now_heading_deg: float, x: float, y: float) -> float:
    """
    선체좌표(보통 x:전방+, y:좌현+)의 방위각을
    '절대 방위각(북 기준 CW+ 0~360)'으로 변환.

    - bearing_body : 선체 전방 기준 CCW+ (전방=0°, 좌현=+90°)
    - now_heading_deg : 북 기준 CW+ (나침반 헤딩)

    => 절대 방위각(북 기준 CW+) = now_heading_deg - bearing_body
    """
    # 선체좌표에서의 방위각(전방=0°, 좌현=+90°), CCW+
    bearing_body = math.degrees(math.atan2(y, x))  # (-180~180)
    bearing_body = _norm360(bearing_body)          # (0~360)

    # 북 기준 CW+ 절대 방위각으로 변환
    return _norm360(now_heading_deg - bearing_body)


def scan_ranges_to_obstacles(
    ranges: List[float],
    angle_min: float,
    angle_increment: float,
    *,
    range_min: float = 0.0,
    range_max: float = float("inf"),
    class_id: int = 0,
) -> List[Dict]:
    """Convert 2D scan ranges to obstacle dicts in body frame."""
    obstacles: List[Dict] = []
    if not ranges or angle_increment == 0:
        return obstacles

    for idx, value in enumerate(ranges):
        try:
            distance = float(value)
        except (TypeError, ValueError):
            continue

        if not math.isfinite(distance):
            continue
        if distance < range_min or distance > range_max:
            continue

        angle = angle_min + angle_increment * idx
        x = distance * math.cos(angle)
        y = distance * math.sin(angle)

        obstacles.append(
            {
                "x": x,
                "y": y,
                "z": 0.0,
                "range": distance,
                "class_id": class_id,
                "track_id": idx,
            }
        )

    return obstacles


def calculate_heading_by_risk(
    obstacles: List[Dict],
    now_heading_abs: float,        # 현재 절대 헤딩(0~360, 북 기준 CW+)
    angle_to_goal_abs: float,      # 목표점 절대 방위각(0~360, 북 기준 CW+)
    *,
    angle_increment: float = 1.0,  # 후보 각도 해상도(도)
    base_clearance_deg: float = 8.0,   # 장애물 여유(기본) 각
    clearance_gain: float = 18.0,      # 근접 시 여유각 확대 이득(가까울수록 커짐)
    min_consider_dist: float = 0.3,    # 너무 가까운(센싱오류) 최소 거리 컷
    max_consider_dist: float = 25.0,   # 고려할 최대 거리
    distance_weight: float = 1.0,      # 거리 위험 가중
    heading_weight: float = 0.25,      # 현재 헤딩과의 괴리(회전량) 가중
    goal_weight: float = 0.7,          # 목표각 괴리 가중
) -> Tuple[float, float]:
    """
    0~360 절대 각도 후보들에 대해
      - 장애물(거리/각) 위험,
      - 헤딩 변화(회전량) 위험,
      - 목표각 괴리 위험
    을 합산 → 최소 위험의 '절대 목표 방위각'을 반환.
    또한 now_heading 대비 signed 에러(±180)도 함께 반환.

    obstacles: [{x,y,z,range,class_id,track_id}, ...]  (x,y 단위: m, 선체좌표)
    """
    # 후보 각도 목록(절대각, 북 기준 CW+)
    num = int(360.0 / angle_increment)        # 예: 1도 해상도 → 360개
    candidates = [i * angle_increment for i in range(num)]  # 0,1,...,359
    risks = [0.0] * num

    # (1) 장애물 위험도
    # 장애물 각각의 절대 방위각과 거리 기반으로, 근접할수록 여유각 확대 + 삼각형 커널로 가중
    TARGET_CLASS = {0}

    for obj in obstacles:
        cid = obj.get("class_id", None)
        if cid not in TARGET_CLASS:
            continue

        try:
            x = float(obj.get("x", 0.0))
            y = float(obj.get("y", 0.0))
            rng = float(obj.get("range", (x * x + y * y) ** 0.5))
        except Exception:
            continue

        if rng < min_consider_dist or rng > max_consider_dist:
            continue

        # 절대 방위각 기준 장애물 각도 (0~360, 북 기준 CW+)
        obs_bearing_abs = _bearing_body_to_world(now_heading_abs, x, y)

        # 거리 기반 여유각(가까울수록 커짐): base + clearance_gain*(1/r)
        clearance = base_clearance_deg + clearance_gain * (1.0 / max(rng, 1e-3))

        # 커널: 각 차이가 clearance 내에 있으면 (1 - diff/clearance)로 선형 감쇠
        for idx, cand in enumerate(candidates):
            # 장애물 절대각 vs 후보 절대각 차이 (0~180, 최단각)
            sep = abs(_angdiff(obs_bearing_abs, cand))
            if sep <= clearance:
                # 근접 가중(1/r), 선형 커널, 거리 가중
                risks[idx] += (
                    distance_weight
                    * (1.0 / (rng + 1e-5))
                    * (1.0 - sep / clearance)
                )

    # (2) 현재 헤딩으로부터 회전량 위험(급회전 억제)
    for idx, cand in enumerate(candidates):
        turn_amt = abs(_angdiff(now_heading_abs, cand))  # 0~180
        risks[idx] += heading_weight * turn_amt

    # (3) 목표각 괴리 위험(목표를 최대한 따라가되, 장애물이 막으면 우회)
    for idx, cand in enumerate(candidates):
        goal_sep = abs(_angdiff(angle_to_goal_abs, cand))  # 0~180
        risks[idx] += goal_weight * goal_sep

    # 최소 위험 후보 선택
    best_idx = min(range(num), key=lambda i: risks[i])
    best_abs_bearing = candidates[best_idx]  # 절대 방위각(0~360, 북 기준 CW+)

    # PID에 넣을 ±180 에러(= 목적 절대각 - 현재헤딩, 둘 다 북 기준 CW+)
    err_signed = _angdiff(now_heading_abs, best_abs_bearing)

    return best_abs_bearing, err_signed






# import math


# def calculate_angle_risk(
#     filtered_range_list,
#     angle_to_goal,
#     angle_range,
#     distance_weight,
#     angle_weight,
#     goal_weight,
#     angle_increment,
#     critical_distance=0.2,
# ):
#     """각도별 위험 점수를 계산하고 최적의 목표 각도를 결정하는 함수

#     Args:
#         filtered_range_list (list of float): 0부터 360도까지 노이즈가 제거된 라이다 거리 리스트
#         cut_filtered_list (list of float): angle_range에 따라 잘린 filtered_range_list

#         now_heading (float): 현재 헤딩 (각도)
#         angle_to_goal (float): 목표 지점과 현재 헤딩 간의 상대 각도
#         angle_range (list): 위험 장애물을 탐지할 각도 범위 [angle_min, angle_max]

#         distance_weight (float): 거리 위험 점수에 대한 가중치
#         angle_weight (float): 각도 위험 점수에 대한 가중치
#         goal_weight (float): 목표 위험 점수에 대한 가중치
#         angle_increment (float): 각도 증가량 (ex. 0.5도)

#         critical_distance (float): 장애물을 피하기 위한 거리 제한 (default: 0.5m)

#     Returns:
#         float: 장애물을 피하고 목표 지점으로 접근하기 위한 안전한 목표 각도
#     """
#     # Cut the filtered_list based on the desired detection angles(angle_range[0], angle_range[1]).
#     # cut_filtered_list = filtered_range_list[(angle_range[0]/angle_increment):(angle_range[1]/angle_increment)+1]
#     # filtered_range_list.reverse()
#     angle_len = len(filtered_range_list)
#     mid_index = len(filtered_range_list) // 2
#     filtered_range_list = (
#         filtered_range_list[mid_index:] + filtered_range_list[:mid_index]
#     )

#     angle_risk = [0] * angle_len

#     for i in range(angle_len):
#         current_angle = i * angle_increment
#         # 거리 위험도: 거리의 역수로 위험도를 취함(이후 유리함수로 변경해보자)
#         angle_risk[i] += distance_weight * (
#             1 / abs(filtered_range_list[i] + 1e-5)
#         )  # critical_distance

#         # 각도 위험도: 현재 헤딩에서 멀수록 위험도가 높아짐
#         if current_angle < 180:
#             angle_risk[i] += angle_weight * current_angle  # 180도 기준으로 해서 바꾸기
#         else:
#             angle_risk[i] += angle_weight * (360 - current_angle)

#         # 목표 위험도: 목표 각도에서 멀수록 위험도가 높아짐
#         angle_difference = abs(current_angle - angle_to_goal)

#         if angle_difference > 180:
#             angle_difference = 360 - angle_difference

#         angle_risk[i] += goal_weight * angle_difference

#         # other risk factors can be added here

#     # 가장 낮은 위험 점수를 가진 위험도의 index 추출
#     min_risk_index = angle_risk.index(min(angle_risk))
#     desired_angle = min_risk_index * angle_increment

#     return desired_angle
