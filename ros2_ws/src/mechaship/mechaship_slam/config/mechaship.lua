-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map", -- /submap_list의 frame_id (일반적으로 "map")
  tracking_frame = "imu_link", -- IMU를 사용할 경우 "imu_link" / LiDAR만 사용할 경우 "base_scan"
  published_frame = "base_footprint",     -- child_frame for submap >> if odom is supplied : "odom" / else : "base_link"
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,
  publish_to_tf = true,
  use_odometry = false, -- use MCU odom : true / use map odom : false
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-2, -- 0.05 sec. (20 hz)
  trajectory_publish_period_sec = 2e-1, -- 0.2 sec. (5 hz)
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- [ Input ] >> 센서 입력값 필터링, 보정
-- LiDAR
TRAJECTORY_BUILDER_2D.min_range = 0.1 -- 맵핑에 사용할 최소 거리 (M)
TRAJECTORY_BUILDER_2D.max_range = 13. -- 맵핑에 사용할 최대 거리 (M)
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 13.5 -- 최대 거리 초과할 경우 대체할 거리 값 (M)
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1 -- 맵핑에 사용할 데이터 묶음 개수
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05 -- 전처리에 사용할 필터 크기
-- IMU
TRAJECTORY_BUILDER_2D.use_imu_data = true -- use IMU : true / do not use IMU (use LiDAR) : false
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 5.

-- [ Local SLAM ] >> 현재 위치 주변의 빠른 맵핑 (근거리, 실시간 탐색)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

-- [ Global SLAM ] >> 전체 영역에서 일관성 있는 맵핑 (넓은 영역, 장시간 탐색)
-- POSE_GRAPH.optimize_every_n_nodes = 0  -- disable Global SLAM
POSE_GRAPH.constraint_builder.log_matches = true -- logging
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

return options