#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
import math
import os
import yaml
from math import degrees, atan2

from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


def wrap_to_180(deg: float) -> float:
    """각도를 [-180, 180) 범위로 래핑"""
    return (deg + 180.0) % 360.0 - 180.0


class CourseThreeNavigator(Node):
    """
    코스3

    1) 라이다: 전방 180도(-90~+90)를 선택해 180등분하고(1도당 1값) bin 평균으로 dist_180(180개, m) 생성
    2) IMU: 헤딩을 [-180, 180)로 만든 뒤, IMU 헤딩이 [-180, 0] 범위일 때만 주행 알고리즘 적용
       (즉, IMU는 '상태 필터' 역할만 함. 라이다각도와 더해서 교집합 계산하지 않음)
    3) dist_180에서 가장 먼 거리값(best_dist)의 각도(best_lidar_angle)를 선택해서 키(서보) 목표각을 만든다
    4) best_dist <= 0.3m 이면 정지(쓰러스터 0)
    5) 선회 반경은 조향량에 따라 쓰러스터를 감속해서 제어
    """

    def __init__(self):
        super().__init__("course_three_navigator")

        # YAML(코스1 스타일 유지)
        self.load_params()

        # 상태
        self.imu_heading_deg_360 = 0.0          # 0~360
        self.imu_heading_deg_signed = 0.0       # -180~180
        self.have_imu = False

        # 파라미터(요구사항)
        self.stop_dist_m = 0.3
        self.thrust_steer_gain = 0.15
        self.thrust_min = 5.0

        
        # QoS (네 테스트 코드 스타일)
        lidar_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Sub
        self.create_subscription(LaserScan, "/scan", self.lidar_cb, lidar_qos)
        self.create_subscription(Imu, "/imu", self.imu_cb, imu_qos)

        # Pub
        self.key_pub = self.create_publisher(Float64, "/actuator/key/degree", 10)
        self.thruster_pub = self.create_publisher(Float64, "/actuator/thruster/percentage", 10)

        # 디버그용
        self.last_best_angle = 0.0
        self.last_best_dist = 0.0
        self.last_final_steer = float(self.neutral_deg)
        self.last_final_thrust = float(self.default_thrust)

        self.get_logger().info("Course 3 Navigator start.")

    # -------------------------
    # YAML 로드
    # -------------------------
    def load_params(self):
        try:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            yaml_path = os.path.join(current_dir, "isv_params.yaml")

            with open(yaml_path, "r", encoding="utf-8") as f:
                params = yaml.safe_load(f)

            # 서보 범위/중립
            self.min_deg = float(params["servo"]["min_deg"])
            self.max_deg = float(params["servo"]["max_deg"])
            self.neutral_deg = float(params["servo"]["neutral_deg"])

            # 추진기 기본 출력: state2
            thr = params.get("state_machine", {}).get("thruster_defaults", {})
            if "state2" in thr:
                self.default_thrust = float(thr["state2"])
            else:
                self.default_thrust = 12.0

        except Exception as e:
            self.get_logger().error(f"Failed to load YAML: {e}")
            self.min_deg = 0.0
            self.max_deg = 180.0
            self.neutral_deg = 90.0
            self.default_thrust = 10.0
            self.thrust_steer_gain = 0.15
            self.thrust_min = 5.0

    # -------------------------
    # IMU callback
    # -------------------------
    def imu_cb(self, msg: Imu):
        q = msg.orientation
        # yaw(heading) 계산 (코스1 코드 스타일)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_deg_360 = (degrees(atan2(siny_cosp, cosy_cosp)) + 360.0) % 360.0

        self.imu_heading_deg_360 = yaw_deg_360
        #======수정한부분1=======================
        self.imu_heading_deg_signed = -wrap_to_180(yaw_deg_360)
        #======수정한부분1=======================

        self.have_imu = True

    # -------------------------
    # Lidar callback (매 프레임 제어)
    # -------------------------
    def lidar_cb(self, msg: LaserScan):
        num_ranges = len(msg.ranges)
        if num_ranges == 0:
            return

        # 0) IMU 상태 필터: imu_heading이 [-180, 0]일 때만 주행 알고리즘 적용
        #    범위 밖이면 "정지"로 처리(원하면 '유지'로 바꿀 수 있음)
        # 수정한 부분1
        if (not self.have_imu) or (self.imu_heading_deg_signed > 0.0):
            self.key_pub.publish(Float64(data=float(self.max_deg)))
            self.thruster_pub.publish(Float64(data=3.0))
            self.get_logger().info(
                f"[SEARCH] imu={self.imu_heading_deg_signed:+.1f}° (need -180~0) | steer=max | thrust=3.0"
            )
            return

        # 1) 라이다 전방 180도(-90~+90)를 슬라이싱 (네 기존 방식 유지: mid 기준)
        mid = num_ranges // 2
        half = num_ranges // 4
        start_idx = mid - half
        end_idx = mid + half
        if start_idx < 0 or end_idx > num_ranges:
            return

        front_ranges = np.array(msg.ranges[start_idx:end_idx], dtype=float)
        front_len = len(front_ranges)
        if front_len < 10:
            return

        # 2) INVALID 제거(0 처리)
        invalid = (~np.isfinite(front_ranges)) | (front_ranges <= msg.range_min) | (front_ranges >= msg.range_max)
        front_ranges[invalid] = 0.0

        # 3) 전방 180도를 180등분하고(1도당 1칸), 각 칸 평균거리 dist_180 생성
        dist_180 = np.zeros(180, dtype=float)
        for i in range(180):
            s = int(front_len * i / 180.0)
            e = int(front_len * (i + 1) / 180.0)
            if e <= s:
                e = min(s + 1, front_len)

            chunk = front_ranges[s:e]
            valid = chunk[chunk > 0.0]
            dist_180[i] = float(np.mean(valid)) if valid.size > 0 else 0.0

        # 4) dist_180에서 최대거리 선택
        best_i = int(np.argmax(dist_180))
        best_dist = float(dist_180[best_i])

        # index->각도 매핑: 0..179 => -90..+90 (중심각)
        best_lidar_angle = -90.0 + (best_i + 0.5)
        #======수정한부분1=======================
        best_lidar_angle = -best_lidar_angle
        #======수정한부분1=======================

        self.last_best_angle = best_lidar_angle
        self.last_best_dist = best_dist

        # 5) 정지 조건: best_dist <= 0.3m
        if best_dist <= self.stop_dist_m:
            self.last_final_steer = float(self.neutral_deg)
            self.last_final_thrust = 0.0
            self.key_pub.publish(Float64(data=float(self.neutral_deg)))
            self.thruster_pub.publish(Float64(data=0.0))
            self.get_logger().info(
                f"[STOP] imu={self.imu_heading_deg_signed:+.1f}° | best_angle={best_lidar_angle:+.1f}° "
                f"| best_dist={best_dist:.3f}m <= {self.stop_dist_m:.3f}m"
            )
            return

        # 6) 서보 목표각: 가장 먼 각도 따라가기 (서보 범위 비대칭 고려)
        left_span = float(self.neutral_deg - self.min_deg)
        right_span = float(self.max_deg - self.neutral_deg)

        #굳이 비율로 처리할 필요 없어보이는데----------------------------------- 7번이랑 이거랑 둘 중 하나만 선택해도 될듯
        if best_lidar_angle < 0.0:  # 왼쪽
            steer_offset = (best_lidar_angle / 90.0) * left_span #비율로 처리
        else:  # 오른쪽
            steer_offset = (best_lidar_angle / 90.0) * right_span #비율로 처리

        final_steering = float(self.neutral_deg + steer_offset)
        final_steering = max(float(self.min_deg), min(float(self.max_deg), final_steering))
        
        #이것도 굳이----------------------------------------------------------
        # 7) 추진기: 조향량 클수록 감속(선회반경/안정)
        steer_error = abs(final_steering - float(self.neutral_deg))
        thrust = float(self.default_thrust) - float(self.thrust_steer_gain) * steer_error
        thrust = max(float(self.thrust_min), min(100.0, thrust))

        self.last_final_steer = final_steering
        self.last_final_thrust = thrust

        # 8) Publish
        self.key_pub.publish(Float64(data=final_steering))
        self.thruster_pub.publish(Float64(data=thrust))

        # 9) Debug 로그
        self.get_logger().info(
            f"[RUN] imu={self.imu_heading_deg_signed:+6.1f}° | "
            f"best_angle={best_lidar_angle:+6.1f}° | best_dist={best_dist:6.3f}m | "
            f"steer={final_steering:6.1f} | thrust={thrust:5.1f}%"
        )


def main():
    rclpy.init()
    node = CourseThreeNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
