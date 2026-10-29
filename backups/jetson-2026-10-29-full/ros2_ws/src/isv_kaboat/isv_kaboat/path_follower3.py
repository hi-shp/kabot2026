# path_follower.py
import math
from typing import List, Tuple, Optional
import pymap3d as pm       #en 좌표계 생성에 사용

def _norm180(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0 ##  -180   +180 변환자

class PathFollower3:
    def __init__(self, origin_latlonalt: Tuple[float, float, float], arrive_radius_m: float = 0.1):
        self.origin = origin_latlonalt  # (lat, lon, alt)
        self.arrive_radius = float(arrive_radius_m)

        # 여기에서 경로 지정 (원하는 GPS 좌표 리스트로 수정하세요)
        self.goal_gps_coords: List[Tuple[float, float]] = [
  
            (35.2316100, 129.0827780),
            (35.2316324, 129.0827477),
            (35.2316402, 129.0827229),
            (35.2316642, 129.0827196),
            (35.2316615, 129.0827810),
            (35.2316773, 129.0827648),
            (35.2316836, 129.0827230),
        ]

        # GPS -> ENU 변환
        self.goal_xy: List[Tuple[float, float]] = []
        for lat, lon in self.goal_gps_coords:
            e, n, u = pm.geodetic2enu(lat, lon, 0.0, *self.origin)
            self.goal_xy.append((e, n))

        self.idx = 0  # 현재 타깃 웨이포인트 인덱스

    def done(self) -> bool:
        return self.idx >= len(self.goal_xy)

    def current_target_xy(self) -> Optional[Tuple[float, float]]:
        if self.done():
            return None
        return self.goal_xy[self.idx]

    def update(self, cur_e: float, cur_n: float, now_heading_deg: float) -> float:
        """
        현재 위치/헤딩 기반으로 다음 웨이포인트까지의 '상대 조향각(±180°)'을 리턴.
        웨이포인트에 도착(반경 이내)하면 다음 웨이포인트로 자동 진행.
        """
        if self.done():
            # 더 갈 곳 없으면 0(직진 유지) 리턴
            return 0.0

        tx, ty = self.goal_xy[self.idx]
        dx, dy = tx - cur_e, ty - cur_n
        dist = math.hypot(dx, dy)

        # 도착 체크
        if dist <= self.arrive_radius:
            self.idx += 1
            if self.done():
                return 0.0
            tx, ty = self.goal_xy[self.idx]
            dx, dy = tx - cur_e, ty - cur_n

        # 북쪽 기준 절대 방위각(0~360)
        angle_to_goal = (math.degrees(math.atan2(dx, dy)) + 360.0) % 360.0
        # 현재 헤딩 기준 상대각(±180)
        rel = _norm180(angle_to_goal - now_heading_deg)
        return rel
