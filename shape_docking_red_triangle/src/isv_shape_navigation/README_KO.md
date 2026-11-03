# 도형 우선 SLAM 구역 진입 패키지

현재 젯슨의 기존 퓨전·쓰러스터를 사용하는 **붉은 삼각형 추종 모드**와 다른 도형으로 바꾸는 방법은 [FOLLOW_KO.md](FOLLOW_KO.md)를 참고합니다. 기본 실행은 `shape_follow.launch.py`이며, 아래의 SLAM 구역 진입 모드도 별도 유지됩니다.

`hi-shp/kabot2026`의 기존 센서·SLAM·추진기 인터페이스에 연결하는 **독립 ROS2 Python 패키지**입니다. 기존 자율운항 노드를 수정하지 않고 옆에 추가합니다. 확인한 기준 커밋은 `6797c384d4ab1ea99d9f23dc918a448d25d0c89e`이며 구체적인 토픽과 서비스는 `UPSTREAM.json`에 기록했습니다.

현재 순서는 **도형 인식 → 카메라/라이다 거리 연결 → SLAM 지도에 목표 고정 → 도형 앞 접근 → 표시 구역 안쪽으로 이동 → 중립·정지**입니다. ㄷ자 입구를 먼저 찾지는 않습니다. 첨부한 `docking_geometry.py`는 원본 그대로 패키지 안에 보존했고, 다음 단계에서 `REGION_REACHED` 이후 입구 선택·벽 정렬·ㄷ자 진입에 연결할 수 있습니다. 이번 실행 경로에서는 호출하지 않습니다.

## 1. 현재 구현 범위

- 학습 가중치 없이 OpenCV 색상·윤곽선으로 삼각형, 원, 정사각형, 직사각형, 오각형, 십자형을 구분합니다. 기본값은 **빨간 삼각형**입니다. 반사광, 물결, 가림이 있는 현장 정확도는 실측 자료로 조정해야 합니다. `score`는 기하학적 휴리스틱 값입니다.
- 카메라 중심과 도형 폭을 기준으로 같은 방향의 `/scan` 점을 연결합니다. 세 점 이상과 거리 일관성이 있어야 하며, 시간 차이 0.2초 초과·거리 혼합·같은 표식 후보 두 개이면 목표 확정을 보류합니다. 2D 라이다는 수직 위치를 확인할 수 없으므로 표지판/지지대/뒤 벽의 어느 높이를 스캔하는지 설치 시 확인해야 합니다.
- 네 번의 서로 다른 프레임에서 일관된 목표를 확인하면 지도 좌표를 고정합니다. 잠시 도형이 안 보여도 기본 30초 동안 그 SLAM 좌표를 사용합니다. 다른 구역의 같은 도형으로 자동 변경하지 않습니다. SLAM 세션 변경 시 목표와 지도를 초기화합니다.
- `/map`의 장애물과 미관측 칸을 선체 반경만큼 확장한 뒤 A*로 경로를 찾습니다. 벽 너머로 목표를 옮기거나 좁은 틈을 임의 통과하지 않습니다.
- 접근점은 표식에서 2.5m 앞, 구역 진입점은 1.2m 앞입니다. **표식이 구역의 안쪽 또는 뒤 벽에 있고, 그 앞쪽이 진입할 구역이라는 배치**를 가정합니다. 최초 관측 방향을 기준으로 두 점을 정합니다. 표식 뒤 공간이나 실제 출입구를 자동 추론하지 않습니다. 표식 위치가 입구 바로 위인 경우 현장 배치에 맞게 목표 생성 규칙을 바꿔야 합니다.
- 위치·스캔·지도·SLAM 준비 상태가 끊기면 실제 출력 세션을 중지합니다. RC 전환·추진기 응답 실패 시 자동 재무장하지 않습니다. 학습/경기용 완성 항법기가 아닌 저속 초기 구현이며, 조류·관성·제동거리는 아직 실측 모델에 반영하지 않았습니다.

## 2. 젯슨 설치

ROS2 Humble과 원본 `kabot2026`가 `/home/isv/ros2_ws/src`에 설치된 환경을 기준으로 합니다. 이 폴더 **`isv_shape_navigation` 전체**를 그 `src`에 복사합니다. 원본 저장소를 통째로 다시 복사할 필요는 없습니다.

```bash
source /opt/ros/humble/setup.bash
cd /home/isv/ros2_ws
source install/setup.bash
rosdep install --from-paths src/isv_shape_navigation --ignore-src -r -y
colcon build --symlink-install --packages-select isv_shape_navigation
source install/setup.bash
```

원본의 `isv_water_trials`, `mechaship_interfaces`, `rf2o_laser_odometry`, `slam_toolbox`가 먼저 빌드되어 있어야 합니다. YOLO 입력은 나중에 `yolo_msgs`가 빌드된 경우에만 사용합니다. 기본 윤곽선 모드는 YOLO를 import하지 않습니다.

## 3. 먼저 센서와 지도, 인식 확인

터미널 1에서 기존 브링업을 실행합니다. 독립 임무를 실행할 때는 기존 outdoor 실행기를 끄고 추진기 명령 소유자를 하나로 유지합니다.

```bash
ros2 launch mechaship_bringup all_sensors.launch.py use_camera:=true use_outdoor:=false
```

터미널 2에서 SLAM과 새 패키지를 함께 실행합니다. **이 명령은 추진기를 호출하지 않습니다.** SLAM이 이미 실행 중이면 `start_slam:=false`를 사용합니다.

```bash
ros2 launch isv_shape_navigation shape_navigation.launch.py start_slam:=true
```

새 지도에서는 RC로 주변을 먼저 관측해 이동 경로가 지도에 빈 공간으로 보이게 합니다. 미관측 영역을 자율 탐사하는 기능은 포함하지 않았습니다. 저장된 지도로 위치 추정하려면 다음과 같이 실행합니다.

```bash
ros2 launch isv_shape_navigation shape_navigation.launch.py \
  start_slam:=true slam_mode:=localization map_file:=/home/isv/water_trials/maps/course
```

`map_file`은 원본 `tank_mapping.launch.py`와 동일하게 `.posegraph`를 제외한 경로 또는 원본 `.tankmap` 파일을 받습니다. 저장 지도 위치 추정의 준비 여부는 `/tank/session`을 따릅니다.

```bash
ros2 topic echo /shape_navigation/status
ros2 run rqt_image_view rqt_image_view /shape_navigation/debug_image
```

RViz의 Fixed Frame을 `tank_map`으로 놓고 `/map`, `/shape_navigation/path`, `/shape_navigation/goal`을 추가하면 목표와 경로를 볼 수 있습니다. `WAIT_CAMERA` 또는 보정 오류가 보이면 아래 카메라 설정을 먼저 적용합니다.

## 4. 카메라와 좌표 설정

원본 브링업은 카메라 보정 파일을 지정하지 않고, 저장소의 `camera_info.yaml`에도 유효한 카메라 행렬이 없습니다. 그래서 실제 보정값을 임의로 넣지 않았습니다. 다음 둘 중 하나를 사용합니다.

1. 카메라 캘리브레이션 결과를 `usb_cam`의 `camera_info_url`에 적용하여 `/camera/camera_info`에 올바른 `K`, `D`, 영상 크기를 발행합니다. 기본 `plumb_bob`/`rational_polynomial` 핀홀 모델을 지원합니다.
2. 이미 왜곡 보정된 영상 토픽을 사용하고, `config/shape_navigation.yaml`의 `camera_fx`, `camera_cx`, `camera_width`, `camera_height`에 **실측 보정값과 정확한 해상도**를 넣습니다. 원본 왜곡 영상을 그대로 쓰면서 이 값만 임의 설정하면 안 됩니다.

기본 SLAM 위치 `/tank/pose`는 선체 중심이 아닌 `velodyne` 원점의 위치입니다. x는 전방, y는 좌측, 각도는 좌회전이 양수입니다. 카메라의 `camera_x`, `camera_y`, `camera_yaw_deg`를 이 기준에 맞춥니다. 카메라와 2D 스캔은 수평 설치를 가정하며 큰 pitch/roll은 지원하지 않습니다.

라이다 x축과 선체 전방이 다르면 `pose_yaw_offset_deg`에 SLAM yaw에서 선체 전방까지의 각도를 넣고, `scan_yaw_deg`는 **보정된 선체 전방에서 스캔 x축까지의 각도**로 설정합니다. 원본처럼 스캔과 pose가 모두 같은 `velodyne` 기준이면 두 각도는 부호가 반대입니다. 임의의 TF를 발행하거나 기존 SLAM의 TF를 덮어쓰지 않습니다.

`footprint_radius=0.62m`는 1.1m × 0.5m 선체의 중심 근처에 라이다가 있다는 가정입니다. 라이다가 선체 중심에서 떨어져 있으면 **라이다 원점에서 선체의 가장 먼 지점까지의 거리** 이상으로 늘리고, `entry_standoff`도 그 반경 + `clearance` + `goal_tolerance`보다 크게 조정합니다.

설정을 복사해 사용하면 재빌드 없이 바꿀 수 있습니다.

```bash
cp /home/isv/ros2_ws/src/isv_shape_navigation/config/shape_navigation.yaml /home/isv/shape_params.yaml
# 보정값, 센서 위치, 선체 반경을 실제 장비에 맞게 수정
ros2 launch isv_shape_navigation shape_navigation.launch.py params_file:=/home/isv/shape_params.yaml
```

도형만 바꾸는 예입니다. launch 인자를 생략하면 YAML의 값이 적용됩니다.

```bash
ros2 launch isv_shape_navigation shape_navigation.launch.py target_label:=circle target_color:=blue
```

## 5. 실제 추진기 연결

동시에 `outdoor`, `mode_test`, 기존 도킹 코드 등 다른 추진기 송신기를 실행하지 않습니다. 이번 패키지의 파일 잠금은 이번 패키지 중복 실행만 방지합니다. 지도·위치·영상·목표 경로와 센서 설치값을 확인한 뒤 기존 미리보기 프로세스를 종료하고 실행합니다.

```bash
ros2 launch isv_shape_navigation shape_navigation.launch.py \
  params_file:=/home/isv/shape_params.yaml start_slam:=true enable_output:=true
```

별도 터미널에서 RC의 AUTO 상태를 확인하고 시작합니다. SLAM을 별도 프로세스에서 계속 실행하고 있다면 위 명령의 `start_slam:=false`로 중복 실행을 피합니다.

```bash
ros2 service call /shape_navigation/start std_srvs/srv/Trigger '{}'
```

`enable_output`과 시작 서비스가 모두 필요합니다. 좌우 요청은 원본 아두이노 브리지가 요구하는 동시 요청으로 보냅니다. 기본 최대 모터 편차는 서비스 단위 기준 ±100µs입니다. 1500이 서비스상의 중립이고 실제 ESC 1480µs 변환은 기존 펌웨어가 담당합니다. 이 패키지가 별도로 변환하지 않습니다.

```bash
ros2 service call /shape_navigation/stop std_srvs/srv/Trigger '{}'
ros2 topic echo /shape_navigation/status
```

정지 서비스의 응답은 요청 접수이며 실제 MCU 정지 확인은 `output.stop_ack: true`로 구별합니다. Ctrl+C에서도 진행 중인 요청을 처리한 뒤 기존 `/actuators/stop`으로 중립 ACK를 확인합니다. 통신이 끊겨 ACK가 없으면 `STOP_UNCONFIRMED`를 표시합니다. 센서 지연이나 RC 전환 후에는 상태 복구 후 시작 서비스를 다시 호출해야 합니다.

목표를 다시 고르려면 정지 ACK 이후 초기화합니다.

```bash
ros2 service call /shape_navigation/reset std_srvs/srv/Trigger '{}'
```

## 6. 데이터 수집과 나중에 학습하기

```bash
ros2 run isv_shape_navigation shape_record
# 포인트클라우드 저장량을 줄이려면
ros2 run isv_shape_navigation shape_record --without-points
```

기본 저장 위치는 `~/water_trials/shape_bags/shape_날짜_시각`입니다. 카메라 원본·보정 정보·2D 스캔·3D 라이다·GPS·속도·IMU·SLAM 지도/위치/TF·검출 결과·추진기 상태·임무 명령을 기록합니다. 존재하지 않는 선택 토픽은 발행되면 기록합니다. 추가 센서는 `--extra-topic /원하는/토픽`으로 넣습니다. 자동 녹화는 하지 않으므로 직접 실행해야 합니다. 60초 또는 256MiB마다 분할하며 전체 녹화 시간/총용량 제한은 없습니다. 시작 전 2GiB 여유 공간을 확인하고 이후 디스크 용량을 확인하며 Ctrl+C로 종료합니다.

나중에 이미지 자료를 추출합니다.

```bash
ros2 run isv_shape_navigation shape_export_images \
  /home/isv/water_trials/shape_bags/shape_날짜_시각 \
  --output /home/isv/shape_dataset/run01 --every 0.5
```

`images/*.jpg`와 `manifest.jsonl`에 원본 영상 시각·bag 시각·frame·파일명이 나옵니다. 아직 사람이 검수한 라벨이나 학습 가중치는 생성하지 않습니다. 추출 이미지를 라벨링하고 학습한 뒤 원본 저장소의 YOLO에 연결합니다.

실행 설정과 센서 보정/좌표 설정은 latched `/shape_navigation/config`에도 발행·기록됩니다. 직접 JSONL 로그도 남기려면 YAML의 `log_directory`를 지정합니다.

```bash
ros2 launch yolo_bringup yolo.launch.py model:=/home/isv/models/shapes.pt \
  input_image_topic:=/camera/image_raw use_tracking:=False use_3d:=False
ros2 launch isv_shape_navigation shape_navigation.launch.py detector_source:=yolo target_label:=triangle
```

YOLO는 `yolo_msgs/msg/DetectionArray`의 `class_name`, `score`, `bbox.center.position`, `bbox.size`를 사용합니다. 일반 COCO 기본 가중치에는 이 임무의 도형 클래스가 없으므로 직접 학습한 모델이 필요합니다. 색상은 YOLO 모드에서 별도 필터링하지 않습니다. 색상까지 구분하려면 `red_triangle` 등으로 클래스 이름을 학습하고 `target_label`을 그 이름으로 설정합니다. 박스는 원본 영상 픽셀 좌표이며, CameraInfo로 보정한 후 거리 연결에 사용합니다.

녹화 자료를 재생할 때는 센서/SLAM 실시간 실행기를 끄고, 기록한 `/map`, `/tank/pose`, `/tank/session`을 사용합니다.

```bash
ros2 launch isv_shape_navigation shape_navigation.launch.py use_sim_time:=true
ros2 bag play /home/isv/water_trials/shape_bags/shape_날짜_시각 --clock
```

재생에서도 기본값은 미리보기입니다. `use_sim_time=true`에서 실제 출력 활성화는 거부합니다. 원본 SLAM launch는 실시간 설정이므로 bag 재생 시 `start_slam:=false`를 유지합니다.

## 7. ROS 없는 PC에서 실행

이 패키지 폴더 안에서 실행합니다. `node` 이외의 인식·기하·경로·임무 모듈은 ROS 없이 import할 수 있습니다.

```bash
python -m pip install -r requirements-offline.txt
python -m isv_shape_navigation.demo --output demo_output
python -m unittest discover -s tests -v
```

모의 카메라와 스캔을 사용해 표식 위치를 추정하고 두 단계 이동과 최종 정지까지 확인합니다. `summary.json`, `trajectory.csv`, `detected_shape.png`, `route.png`를 생성합니다. 모의 동역학은 실제 보트의 속도나 성능을 나타내지 않습니다.

이번 작업 환경은 Windows이며 ROS2/젯슨/실물 센서는 연결되어 있지 않습니다. 오프라인 인식·경로·임무·추진기 서비스 상태머신과 어댑터 테스트 결과는 `VERIFICATION.md`에 기록합니다. 젯슨의 colcon 빌드와 실제 ROS 통신·수상 주행은 현장에서 검증해야 합니다.
