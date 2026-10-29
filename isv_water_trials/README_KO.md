# ISV 수상 시험 / 역추정 도구

사용자 제공 제원(2026-10-29): 배터리·장비를 포함한 총질량 **25 kg**, 길이 **1.1 m**, 폭 **0.50 m**, 좌우 추진기 중심 간격 **0.32 m**. `config/boat.json`에 저장했습니다. 관성모멘트·추력은 미측정 상태입니다. 직사각형 관성 근사값을 실측값으로 사용하지 않습니다.

## 설치와 기본 실행

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select isv_water_trials mechaship_bringup
source install/setup.bash
ros2 run isv_water_trials preflight
ros2 run isv_water_trials record
```

`preflight`는 센서 조회만 합니다. `record`도 추진기를 제어하지 않습니다. Ctrl+C로 종료하면 `~/water_trials/<UTC 시각>/`에 `raw.jsonl`, `metadata.json`이 저장됩니다. GNSS·VectorNav·IMU·배터리 전압·비상정지·추진기 명령·시험 구간·`/scan`을 기록합니다. 원시 헤더 시각은 보존하며, 동기화 기준은 수신 컴퓨터의 monotonic 시각입니다. 네트워크 지연/센서 시계 오차까지 보정했다는 뜻은 아닙니다. PointCloud2 전체는 기본 JSON에 저장하지 않습니다.

## Velodyne bringup

```bash
ros2 launch mechaship_bringup mechaship_bringup.launch.py \
  velodyne_model:=VLP16 velodyne_ip:=192.168.1.201
```

VLP16은 잠정 기본값입니다. 장비 라벨을 확인한 뒤 `VLP32C` 또는 `VLS128`로 바꿀 수 있습니다. IP도 실장비 설정과 일치해야 합니다. YDLIDAR는 호출하지 않습니다. `/velodyne_packets` → `/velodyne_points` → `/scan` 순서입니다. `velodyne_ring:=-1`은 자동 링 선택이며, 수면 반사·낮은 장애물 감지가 되는지는 실제로 확인해야 합니다. 라이다 오도메트리는 물 위에서 고정 구조물이 부족할 수 있어 기본 OFF입니다(`use_lidar_odom:=true`로 별도 사용). 라이다 TF 장착 위치는 임의 생성하지 않습니다.

센서만 따로 확인할 때(액추에이터를 활성화하지 않음):

```bash
ros2 launch mechaship_bringup velodyne_water_trial.launch.py
ros2 topic hz /velodyne_points
ros2 topic hz /scan
```

이 패키지는 유선 IP나 라이다 내부 설정을 변경하지 않습니다. 유선 링크/전원/주소가 맞아야 패킷이 들어옵니다. 드라이버 프로세스 실행과 실제 데이터 수신은 구분합니다. 기존 bringup의 GPS·IMU·카메라 기본 설정은 유지했습니다. 실제 시험에 쓰는 `/smc_2000/fix`, `/vectornav/raw/common`은 기존 별도 드라이버 구성이 필요할 수 있습니다. `preflight`가 알려주는 토픽 상태를 기준으로 판단하세요.

## 오늘 시험 순서

1. 총질량과 배터리 상태를 확인합니다. GNSS가 고정되고 실제 센서가 발행하는지 확인합니다. 기존 `ros2 run isv_kaboat gps`는 **GPS 스푸퍼**이므로 실험에서 실행하지 않습니다.
2. 정지 상태 30~60초를 기록합니다. 배를 손으로 회전시켜 VectorNav raw yaw와 실제 선수 방향을 비교합니다. 설정의 `vectornav_yaw_offset_deg`를 확인하고 `frame_confirmed`를 true로 바꿉니다. NED raw heading은 북쪽 0°, 시계 방향 +입니다. 분석은 ENU 동쪽 0°, 반시계 방향 +로 변환합니다. 설치 방향이 다르면 반드시 보정합니다. 변환된 RFU `/imu`를 선체 FLU로 간주하지 않습니다.
3. 수면이 잔잔하고 회수 가능한 구역에서 정지 → 직진 → 중립 감속을 양방향으로 각각 3회 이상 수행합니다. 출력은 낮은 단계에서 시작합니다. 다음으로 좌·우 선회를 각각 반복하고 마지막에 지그재그를 반복합니다. 배터리 상태, 풍향·풍속, 조류, 적재 상태를 별도 기록합니다.
4. 각 구간 시작 때 다른 터미널에서 `ros2 run isv_water_trials mark straight` / `coast` / `turn_left` / `turn_right` / `zigzag` / `drift` / `idle`을 보냅니다. 끝은 `mark end`입니다. 구간 표시는 직전 구간을 끝내고 새 구간을 시작합니다.
5. 수집 종료 후 `ros2 run isv_water_trials analyze ~/water_trials/<시각>`을 실행합니다. `report_KO.md`, `report.json`, `kinematics.csv`, `mpc_model_candidate.json`이 생성됩니다.

원본 설정을 보존하고 확인된 시험 설정을 따로 복사하여 사용하는 예:

```bash
cp ~/ros2_ws/src/isv_water_trials/config/boat.json ~/water_trial_boat.json
# 실제 장착 방향 확인 후 해당 복사본을 편집합니다.
ros2 run isv_water_trials record --config ~/water_trial_boat.json
ros2 run isv_water_trials analyze ~/water_trials/<시각> --config ~/water_trial_boat.json
```

## 추진기 명령 기록: 기존 코드 또는 반복 시험

Humble의 일반 토픽 기록만으로 다른 노드가 보낸 PWM 서비스 요청을 관찰할 수는 없습니다. 기존 제어 코드를 사용할 때는 로깅 프록시를 별도 실행하고 **그 제어 노드만** 다음과 같이 remap합니다. 원래 좌·우 서비스에 직접 연결된 다른 제어 노드는 꺼야 합니다.

```bash
ros2 run isv_water_trials command_proxy --config ~/water_trial_boat.json
# 아래 <패키지> <실행파일>은 실제 사용할 기존 제어 노드로 교체합니다.
ros2 run <패키지> <실행파일> --ros-args \
  -r /actuators/throttle_left/set_pulse_width:=/water_trial/throttle_left/set_pulse_width \
  -r /actuators/throttle_right/set_pulse_width:=/water_trial/throttle_right/set_pulse_width
```

프록시는 관측용입니다. 상위 제어기의 비상정지 기능을 대체하지 않습니다. 프록시를 remap하지 않은 코드는 명령 기록에서 누락됩니다. 실제 `status=true` 응답을 받은 좌·우 명령만 분석에 사용합니다.

반복 시험 계획을 먼저 출력(이 명령은 모터를 움직이지 않음):

```bash
ros2 run isv_water_trials trial surge --repeats 3
ros2 run isv_water_trials trial yaw --repeats 3
ros2 run isv_water_trials trial zigzag --repeats 3
```

현장 운전자가 동작을 시작하려면 `--config ~/water_trial_boat.json --execute --operator-confirmed --exclusive-control-confirmed`를 명시해야 합니다. `frame_confirmed=true`, 신선한 GNSS·VectorNav·비상스위치, 기록 구독자, 좌·우 서비스가 필요합니다. 기본 PWM은 1500(중립)~1560이고 `--delta` 범위는 10~100 μs입니다. 센서/비상스위치/응답 실패 시 중립을 3회 시도합니다. **프로세스·전원·통신이 끊기면 소프트웨어 정지 명령이 전달되지 않을 수 있으므로 실물 비상정지와 MCU 측 명령 만료 정지가 필요합니다.** 무인 원격 반복 실행용이 아닙니다. 시험 종료 후 프록시와 반복 시험을 동시에 실행하지 않습니다.

## 추정 항목과 조건

| 항목 | 조건 / 결과 의미 |
|---|---|
| 대지 속도·동/북 속도·궤적 | 유효 GNSS, 실제 시각 간격, 위치 분산 기준 통과 |
| 선체 u/v·전진 가속도 | GNSS+VectorNav, 장착 방향 확인 후 사용 |
| 선수각·yaw rate·yaw acceleration | raw yaw unwrap, raw gyro와 교차 확인, gap 횡단 보간 금지 |
| 선회반경 대용값 | 정상 선회에서 대지 속도/절대 yaw rate; 진짜 선회시험 전술직경과 다름 |
| roll/pitch 주기 | 관측 스펙트럼의 우세 주기. 자연주기·GM 확정값 아님 |
| 전진·선회 PWM 응답 모델 | 기록된 PWM과 변화 구간 필요. MPC 모의용 경험모델이며 PWM을 추력으로 환산하지 않음 |
| 감속의 d1/M, d2/M | `coast` 표시, 중립 응답 확인, frame/still_water_confirmed=true 필요 |
| 유효 전진 질량·저항, 유효 yaw 관성·저항 | 실측 좌·우 추력–PWM–전압 표와 충분한 가진 필요. 단일 축 근사, 부가질량/관성 포함 |

`thrust_calibration_template.csv`에는 가짜 데이터를 넣지 않았습니다. 실측값을 `side=left/right`, `pwm_us`, `voltage_V`, `thrust_N`으로 기록하고 설정에서 절대경로를 지정합니다. 전압·PWM 표 밖으로는 외삽하지 않습니다. 전진 추력은 +, 후진은 -이며 yaw moment는 ENU/FLU 기준 `0.16*(F_right-F_left)`입니다. 25 kg의 실중량을 유효질량으로 강제하거나 추력과 질량을 동시에 임의 역산하지 않습니다.

## MPC 연결과 독립 검증

현재 확인한 젯슨 `isv_kaboat`과 GitHub `isv/launch_isv`에는 MPC 구현이 없습니다. 기존 회피 코드를 MPC로 바꾸거나 실제 추진기에 새 MPC를 연결하지 않았습니다. `mpc_model_candidate.json`은 분석 결과 중 유효한 모델만 포함하며 `deployment_ready=false`입니다.

독립된 두 번째 시험 데이터와 라이다 장착 보정 설정을 사용하여 모의 MPC를 반복 평가할 수 있습니다.

```bash
ros2 run isv_water_trials mpc_replay \
  --model ~/water_trials/<학습시각>/mpc_model_candidate.json \
  --session ~/water_trials/<다른검증시각> \
  --config ~/water_trial_boat.json --goal-east 5 --goal-north 0
```

목표 좌표는 **검증 세션의 첫 GNSS 위치를 원점**으로 한 ENU 미터 좌표입니다. `frame_confirmed`와 `lidar_frame_confirmed`를 확인하고, 실제 `lidar_x_m`, `lidar_y_m`, `lidar_yaw_deg`를 넣어야 합니다. 수면 위 장애물이 충분히 검출되는 링인지 확인하세요. 학습 데이터 재사용을 거부하고, 오래된/불완전한 scan에는 제안을 생성하지 않습니다. 결과 `mpc_replay.json`은 가속도 예측 오차와 추진기 **제안**만 저장합니다. 이 코드는 모터 서비스에 연결하지 않습니다. 현재 모델은 sway·결합력·파도·지연·이동 장애물을 충분히 다루지 못하므로 실배 MPC 완성으로 판단하지 않습니다. 실제 MPC 파일이 제공되면 그 상태/입력/좌표계에 맞춘 연결이 추가로 필요합니다.

## 검증

```bash
cd ~/ros2_ws/src/isv_water_trials
python3 -m unittest discover -s tests -p 'test_identification.py' -v
python3 -m unittest discover -s tests -p 'test_ros_fixture.py' -v
```

모의 센서·가짜 추진기 서비스 시험은 독립 ROS 도메인과 `/fixture` 서비스에서만 진행합니다. 수상 실험 결과를 의미하지 않습니다. 실제 수신 여부와 미검증 항목은 `TESTING.md`에 기록합니다.

모델 근거: https://www.fossen.biz/html/marineCraftModel.html
ROS bag/QoS 근거: https://index.ros.org/r/rosbag2/
