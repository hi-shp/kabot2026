import math


def calculate_angle_risk(
    filtered_range_list,
    angle_to_goal,
    angle_range,
    distance_weight,
    angle_weight,
    goal_weight,
    angle_increment,
    critical_distance=0.2,
):
    """각도별 위험 점수를 계산하고 최적의 목표 각도를 결정하는 함수

    Args:
        filtered_range_list (list of float): 0부터 360도까지 노이즈가 제거된 라이다 거리 리스트
        cut_filtered_list (list of float): angle_range에 따라 잘린 filtered_range_list

        now_heading (float): 현재 헤딩 (각도)
        angle_to_goal (float): 목표 지점과 현재 헤딩 간의 상대 각도
        angle_range (list): 위험 장애물을 탐지할 각도 범위 [angle_min, angle_max]

        distance_weight (float): 거리 위험 점수에 대한 가중치
        angle_weight (float): 각도 위험 점수에 대한 가중치
        goal_weight (float): 목표 위험 점수에 대한 가중치
        angle_increment (float): 각도 증가량 (ex. 0.5도)

        critical_distance (float): 장애물을 피하기 위한 거리 제한 (default: 0.5m)

    Returns:
        float: 장애물을 피하고 목표 지점으로 접근하기 위한 안전한 목표 각도
    """
    # Cut the filtered_list based on the desired detection angles(angle_range[0], angle_range[1]).
    # cut_filtered_list = filtered_range_list[(angle_range[0]/angle_increment):(angle_range[1]/angle_increment)+1]
    # filtered_range_list.reverse()
    angle_len = len(filtered_range_list)
    mid_index = len(filtered_range_list) // 2
    filtered_range_list = (
        filtered_range_list[mid_index:] + filtered_range_list[:mid_index]
    )

    angle_risk = [0] * angle_len

    for i in range(angle_len):
        current_angle = i * angle_increment
        # 거리 위험도: 거리의 역수로 위험도를 취함(이후 유리함수로 변경해보자)
        angle_risk[i] += distance_weight * (
            1 / abs(filtered_range_list[i] + 1e-5)
        )  # critical_distance

        # 각도 위험도: 현재 헤딩에서 멀수록 위험도가 높아짐
        if current_angle < 180:
            angle_risk[i] += angle_weight * current_angle  # 180도 기준으로 해서 바꾸기
        else:
            angle_risk[i] += angle_weight * (360 - current_angle)

        # 목표 위험도: 목표 각도에서 멀수록 위험도가 높아짐
        angle_difference = abs(current_angle - angle_to_goal)

        if angle_difference > 180:
            angle_difference = 360 - angle_difference

        angle_risk[i] += goal_weight * angle_difference

        # other risk factors can be added here

    # 가장 낮은 위험 점수를 가진 위험도의 index 추출
    min_risk_index = angle_risk.index(min(angle_risk))
    desired_angle = min_risk_index * angle_increment

    return desired_angle
