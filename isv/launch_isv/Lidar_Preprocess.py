# def ISV_Lidar(case_string, raw_range):
#     if case_string == "no_obstacle, angle" or case_string == "obstacle, angle":
#         obstacle_angle_list = obstacle_angle(case_string, raw_range)
#         print(obstacle_angle_list)
#         return obstacle_angle_list
#     elif case_string == "obstacle, range":
#         obstacle_range_list = obstacle_range(case_string, raw_range)
#         return obstacle_range_list
#     else:
#         print("Error: case_string is not correct")
#         return None

import math


def detect_and_cluster(raw_range):
    n = len(raw_range)
    a = 8  # 연속된 0의 개수를 판단하는 기준 값, 우선은 8로 설정
    b = 0.2  # 두 값의 차이를 비교하는 기준 비율, 우선은 큰 값 기준 20%로 설정
    threshold = 0.5  # 급격한 경사를 판단하기 위한 임계값, 우선은 0.5로 설정

    # 2.5m 초과하는 값들은 노이즈로 간주하고 0.0으로 변경
    raw_range = [x if x <= 2.5 else 0.0 for x in raw_range]

    i = 0

    # 전체 리스트를 순회하면서 노이즈 제거와 클러스터링 수행
    while i < n:

        if raw_range[i] == 0.0:  # 현재 값이 0.0이라면 (연속된 0.0 값 처리 시작)
            zero_start = i  # 연속된 0.0이 시작되는 위치를 저장
            # 연속된 0.0 값의 끝을 찾기 위한 루프
            while i < n and raw_range[i] == 0.0:

                i += 1
            zero_end = i - 1  # 연속된 0.0 값의 끝 위치를 저장

            zero_length = zero_end - zero_start + 1  # 연속된 0.0 값의 길이를 계산

            if zero_length < a:  # 연속된 0.0 값의 길이가 a보다 작다면 처리
                # left_value: 연속된 0.0 값의 시작점 바로 왼쪽의 0이 아닌 값 찾기
                left_value = next(
                    (
                        raw_range[(zero_start - 1 - j) % n]
                        for j in range(n)
                        if raw_range[(zero_start - 1 - j) % n] != 0.0
                    ),
                    0.0,
                )
                # right_value: 연속된 0.0 값의 끝점 바로 오른쪽의 0이 아닌 값 찾기
                right_value = next(
                    (
                        raw_range[(zero_end + 1 + j) % n]
                        for j in range(n)
                        if raw_range[(zero_end + 1 + j) % n] != 0.0
                    ),
                    0.0,
                )

                # 두 값의 차이가 큰 값의 b% 이내라면 (값 차이가 크지 않다면) 그 구간의 0.0을 평균값으로 대체
                larger_value = max(left_value, right_value)
                if abs(left_value - right_value) / larger_value <= b:
                    new_value = (left_value + right_value) / 2.0
                    raw_range[zero_start : zero_end + 1] = [
                        new_value
                    ] * zero_length  # 0.0 값을 평균값으로 대체

        # else:  # 현재 값이 0.0이 아닌 경우 (노이즈 필터링)
        # 이전 두 값 간의 경사를 계산
        #            prev_slope = abs(raw_range[(i - 1) % n] - raw_range[(i - 2) % n])
        # 다음 두 값 간의 경사를 계산
        #            next_slope = abs(raw_range[(i + 1) % n] - raw_range[(i + 2) % n])
        # 현재 값과 이전 값 간의 경사를 계산
        #            current_slope_left = abs(raw_range[i] - raw_range[(i - 1) % n])
        # 현재 값과 다음 값 간의 경사를 계산
        #            current_slope_right = abs(raw_range[i] - raw_range[(i + 1) % n])

        # 만약 현재 값이 급격한 경사 변화를 보이면 (즉, 노이즈로 의심되면)
        #            if (current_slope_left > threshold * prev_slope) and (current_slope_right > threshold * next_slope):
        # 이전 경사와 다음 경사의 평균값을 계산하여
        #                avg_slope = (prev_slope + next_slope) / 2.0
        # 좌측 경사가 더 크다면 이전 값을 기준으로, 우측 경사가 더 크다면 다음 값을 기준으로 보정
        #                corrected_value = raw_range[(i - 1) % n] + avg_slope if current_slope_left > current_slope_right else raw_range[(i + 1) % n] - avg_slope
        #                raw_range[i] = corrected_value  # 노이즈 값 수정
        i += 1
    # 0.0 값을 10.0으로 변경
    raw_range = [10.0 if x == 0.0 else x for x in raw_range]
    # raw_range = [0.15 if x < 0.15 else x for x in raw_range]

    i = 400
    while i < n - 400:
        right_extension = 0
        if (
            raw_range[i] != 10.0 and raw_range[i - 1] == 10.0
        ):  # 처음으로 10이 아닌 값이 나오는 경우
            leftt_value = raw_range[i]
            left_extension = int((0.4 / leftt_value) * 2020 / (2 * math.pi))
            for j in range(1, left_extension + 1):
                raw_range[(i - j) % n] = leftt_value  # 왼쪽 확장
        # left_value로 설정
        elif raw_range[i + 1] == 10.0 and raw_range[i] != 10.0:
            # if raw_range[i] == 10.0:  # 처음으로 10이 나오는 경우
            rightt_value = raw_range[i]  # 그 직전 값을 right_value로 설정
            right_extension = int((0.4 / rightt_value) * 2020 / (2 * math.pi))
            for j in range(1, right_extension + 1):
                raw_range[(i + j) % n] = rightt_value

        # 왼쪽 끝값 처리
        #  left_extension = int((0.3/leftt_value) * 2020 / (2 * math.pi))
        # 오른쪽 끝값 처리
        # right_extension = int((0.3 /rightt_value) * 2020 / (2 * math.pi))

        # 왼쪽 확장 및 값 변경 (원형 인덱싱 적용)
        # for j in range(1, left_extension + 1):
        #     raw_range[(i - j) % n] = leftt_value  # 왼쪽 확장

        # 오른쪽 확장 및 값 변경 (원형 인덱싱 적용)
        # for j in range(1, right_extension + 1):
        # raw_range[(i + j) % n] = rightt_value  # 오른쪽 확장
        i = i + 1 + right_extension

    return raw_range


# def obstacle_angle(case_string, raw_range):
#     clusters_list = detect_and_cluster(raw_range)
#     obstacle_angle_index = []
#     for i in range(len(clusters_list)):
#         obstacle_angle_index.append(list(clusters_list[i].keys()))

#     if case_string == "no_obstacle, angle":
#         print("no_obstacle, angle")
#         empty_angle_index = [i for i in range(len(raw_range))]
#         for item in obstacle_angle_index:
#             for i in item:
#                 empty_angle_index.remove(i)

#         empty_angle_zone_list = []
#         temp = [empty_angle_index[0]]
#         for i in range(1, len(empty_angle_index)):
#             if empty_angle_index[i] == temp[-1] + 1:
#                 temp.append(empty_angle_index[i])
#             else:
#                 empty_angle_zone_list.append(temp)
#                 temp = [empty_angle_index[i]]
#         empty_angle_zone_list.append(temp)
#         if empty_angle_zone_list[-1][-1] == len(raw_range) - 1 and empty_angle_zone_list[0][0] == 0:
#             empty_angle_zone_list[-1].extend(empty_angle_zone_list[0])
#             empty_angle_zone_list.pop(0)
#         return empty_angle_zone_list
#     elif case_string == "obstacle, angle":
#         print("obstacle, angle")
#         return obstacle_angle_index
#     else:
#         print("Error: case_string is not correct")
#         return None

# def obstacle_range(case_string, raw_range):
#     clusters_list = detect_and_cluster(raw_range)
#     obstacle_range_list = []
#     for i in range(len(clusters_list)):
#         obstacle_range_list.append(list(clusters_list[i].values()))
#     if case_string == "obstacle, range":
#         print("obstacle, range")
#         return obstacle_range_list
#     else:
#         print("Error: case_string is not correct")
#         return None


def main():
    test_data_set_radian = [i for i in range(0, 360)]
    test_data_set_range = [0 for i in range(0, 360)]
    test_data_set_range[5] = 1.26
    test_data_set_range[40] = 2.0
    for i in range(50, 100):
        test_data_set_range[i] = 2.0
    # return_value = ISV_Lidar("obstacle, range", test_data_set_radian, test_data_set_range)
    # print(return_value)


if __name__ == "__main__":
    main()
elif __name__ == "ISV_Lidar":
    print("ISV_Lidar imported")
