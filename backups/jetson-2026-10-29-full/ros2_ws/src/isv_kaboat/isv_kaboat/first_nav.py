import math

def go_first_goal(now_goal, e, n, now_heading):
    """
    현재 위치 (e, n)에서 now_goal까지의 목표각을 계산
    입력:
        now_goal: (goal_x, goal_y) 목표 ENU 좌표
        e, n: 현재 ENU 좌표
        now_heading: 현재 heading (deg, 북쪽 기준)
    출력:
        target_angle: 목표까지의 상대각 (deg)
    """
    goal_x, goal_y = now_goal
    delta_x = goal_x - e
    delta_y = goal_y - n

    # 목표까지 절대각 (북쪽 기준)
    angle_rad = math.atan2(delta_x, delta_y)
    angle_deg = math.degrees(angle_rad)
    angle_to_goal = (angle_deg + 360) % 360

    # 현재 heading 기준 상대각
    target_angle = (angle_to_goal - now_heading + 360) % 360

    return target_angle
