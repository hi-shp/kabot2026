# 수상 시험 기록과 분석

센서와 추진 명령을 기록하고, 수집한 데이터로 선체 응답과 동역학 계수를 추정하는 ROS 2 패키지입니다. 실제 조종 명령과 현재 설정은 [정상 운용 기준](../docs/BOAT_BASELINE_2026-10-30.md)을 참고합니다.

선체 제원은 총질량 25kg, 길이 1.1m, 폭 0.50m, 추진기 중심 간격 0.32m입니다. `config/boat.json`과 현재 센서 구성용 `config/boat_live.json`에 저장되어 있습니다. 관성모멘트와 추력은 실측이 필요합니다.

## 빌드와 센서 연결

젯슨의 새 터미널은 ROS 환경을 자동 적용합니다. 파일 수정 후 빌드는 `cb`를 사용합니다.

```bash
cb --packages-select isv_water_trials mechaship_bringup
ros2 launch mechaship_bringup all_sensors.launch.py
```

Velodyne은 VLP16, IP 192.168.1.201로 설정되어 있습니다. 데이터는 `/velodyne_packets` → `/velodyne_points` → `/scan` 순서로 변환됩니다. `velodyne_ring:=-1`은 자동 층 선택입니다. 현재 점군에는 16개 ring이 있고 `/scan`은 한 층을 사용합니다. 장착 위치·방향과 수면 위 장애물 검출 범위는 현장에서 확인합니다.

라이다만 확인할 때는 다음 명령을 사용합니다.

```bash
ros2 launch mechaship_bringup velodyne_water_trial.launch.py
ros2 topic hz /velodyne_points
ros2 topic hz /scan
```

라이다 오도메트리는 `use_lidar_odom:=true`로 선택할 수 있습니다. 수면에서는 고정 구조물의 수와 배치에 따라 정확도가 달라집니다.

## 센서 기록과 지도

3D 점군과 원시 패킷까지 보존하려면 정상 운용 기준의 rosbag 명령을 사용합니다. 분석용 JSON 기록은 다음과 같습니다.

```bash
ros2 run isv_water_trials preflight
ros2 run isv_water_trials record
```

`preflight`는 센서·설정 상태를 조회합니다. `record`는 GNSS·VectorNav·IMU·scan·추진 명령·구간 표식과 수신 가능한 배터리·정지 상태를 저장합니다. Ctrl+C로 마치면 `~/water_trials/<UTC 시각>/`에 `raw.jsonl`, `metadata.json`이 생성됩니다. 기록 범위는 JSON 분석 데이터이며, 3D 점군과 Velodyne 원시 패킷은 rosbag으로 보관합니다.

원시 메시지의 헤더 시각을 보존하고 수신 컴퓨터의 monotonic 시각으로 데이터를 맞춥니다. 분석 시 네트워크 지연과 센서별 시계 차이도 고려해야 합니다. 기본 UNO 구성에서 `/actuators/stop_requested`는 소프트웨어 제어 상태를 나타냅니다. 물리 EMO는 외부 전원을 차단하며, 감지선은 미연결 상태입니다.

```bash
ros2 run isv_water_trials gps_track --output ~/water_trials/gps_$(date +%Y%m%d_%H%M%S)
```

`gps_latest_url.txt`의 링크로 위치를 지도에서 보고, `gps_track.csv`·`gps_track.geojson`으로 경로를 확인합니다.

## 시험 순서와 좌표 설정

1. 당일 적재 질량·배터리·GNSS 상태를 확인합니다. 위치 데이터는 실제 GNSS를 사용합니다. 기존 `ros2 run isv_kaboat gps`는 고정 위치를 발행하는 GPS 스푸퍼입니다.
2. 정지 상태를 30~60초 기록한 뒤 실제 선수 방향과 VectorNav raw yaw를 비교합니다. 장착 방향에 맞게 `vectornav_yaw_offset_deg`를 보정하고 `frame_confirmed=true`로 설정합니다.
3. 잔잔하고 회수 가능한 구역에서 정지 → 직진 → 중립 감속을 양방향으로 각각 3회 이상 반복합니다. 이어 좌·우 선회와 지그재그를 기록합니다. 풍향·풍속·조류·적재 상태도 함께 남깁니다.
4. 각 구간 시작 때 `mark`를 보내고 수집 후 `analyze`를 실행합니다.

VectorNav raw heading은 NED 기준 북쪽 0°·시계 방향 +이며, 분석은 ENU 기준 동쪽 0°·반시계 방향 +로 변환합니다. RFU 형식 IMU를 사용할 때는 선체 FLU 좌표로 변환해야 합니다.

```bash
ros2 run isv_water_trials mark straight
ros2 run isv_water_trials mark coast
ros2 run isv_water_trials mark turn_left
ros2 run isv_water_trials mark turn_right
ros2 run isv_water_trials mark zigzag
ros2 run isv_water_trials mark drift
ros2 run isv_water_trials mark idle
ros2 run isv_water_trials mark end
ros2 run isv_water_trials analyze ~/water_trials/<시각>
```

새 표식은 직전 구간을 끝내고 다음 구간을 시작합니다. 분석 결과는 `report_KO.md`, `report.json`, `kinematics.csv`, `mpc_model_candidate.json`입니다.

설정은 현재 센서 구성에 맞는 `boat_live.json`을 복사해 조정합니다.

```bash
cp ~/ros2_ws/src/isv_water_trials/config/boat_live.json ~/water_trial_boat.json
# 실제 장착 방향을 확인하고 복사본을 편집합니다.
ros2 run isv_water_trials record --config ~/water_trial_boat.json
ros2 run isv_water_trials analyze ~/water_trials/<시각> --config ~/water_trial_boat.json
```

## 추진 명령 기록과 반복 시험

현재 아두이노 브리지는 `/water_trial/command`에 좌우 명령과 응답 결과를 남깁니다. 기존 제어기의 서비스 호출을 따로 기록할 때는 로깅 프록시를 실행하고 해당 제어 노드의 서비스 경로를 바꿉니다. 추진 명령은 한 제어기에서 보내도록 구성합니다.

```bash
ros2 run isv_water_trials command_proxy --config ~/water_trial_boat.json
# <패키지> <실행파일>은 실제 제어 노드로 바꿉니다.
ros2 run <패키지> <실행파일> --ros-args \
  -r /actuators/throttle_left/set_pulse_width:=/water_trial/throttle_left/set_pulse_width \
  -r /actuators/throttle_right/set_pulse_width:=/water_trial/throttle_right/set_pulse_width
```

프록시를 경유한 요청이 기록되며, 분석에는 좌우 모두 `status=true`로 응답한 명령을 사용합니다. 통신 단절 시 정지는 실물 EMO와 MCU의 명령 만료 기능으로 처리합니다.

반복 시험 계획은 아래 명령으로 미리 확인합니다.

```bash
ros2 run isv_water_trials trial surge --repeats 3
ros2 run isv_water_trials trial yaw --repeats 3
ros2 run isv_water_trials trial zigzag --repeats 3
```

기본 동작은 계획 출력입니다. 실제 실행에는 `--config ~/water_trial_boat.json --execute --operator-confirmed --exclusive-control-confirmed`를 추가합니다. `frame_confirmed=true`, 최신 GNSS·VectorNav·정지 상태, 기록 구독자와 좌우 서비스가 필요합니다. 현장 운전자가 조건을 확인한 뒤 실행합니다.

기본 명령은 중립 1500~1560µs이며 `--delta`는 10~100µs입니다. 센서·정지 상태·응답에 문제가 생기면 중립 명령을 최대 3회 시도합니다. 이 유한 반복 도구와 Ctrl+C까지 송신하는 `mode_test`는 목적과 실행 조건이 다릅니다.

## 추정할 수 있는 항목

| 항목 | 필요한 데이터와 해석 |
|---|---|
| 대지 속도·동/북 속도·궤적 | 유효 GNSS, 실제 시각 간격, 위치 분산 기준 |
| 선체 u/v·전진 가속도 | GNSS와 VectorNav, 장착 방향 보정 |
| 선수각·yaw rate·yaw acceleration | raw yaw 연속화, gyro 교차 확인, 데이터 공백 구분 |
| 선회반경 대용값 | 정상 선회 속도 ÷ 절대 yaw rate. 전술직경은 별도 선회시험으로 산출 |
| roll/pitch 주기 | 관측 스펙트럼의 우세 주기. 자연주기·GM 산출에는 추가 조건 필요 |
| 전진·선회 PWM 응답 | PWM과 변화 구간으로 구한 MPC 모의용 경험모델 |
| 감속 d1/M, d2/M | coast 표식, 중립 응답, frame/still_water_confirmed=true |
| 유효 전진 질량·저항, yaw 관성·저항 | 실측 추력–PWM–전압 표와 충분한 입력 변화. 단일 축 근사이며 부가질량·관성 포함 |

`thrust_calibration_template.csv`에 실측 `side=left/right`, `pwm_us`, `voltage_V`, `thrust_N`을 채우고 설정에 절대경로를 지정합니다. 추정에는 표에 포함된 전압·PWM 범위만 사용합니다. 전진 추력은 +, 후진은 −이며 yaw moment는 ENU/FLU 기준 `0.16*(F_right-F_left)`입니다. 실중량 25kg과 물속에서의 유효질량은 구분해 다룹니다.

## MPC 모의 검증

확인한 기존 코드에서는 회피·방위 제어를 사용하고 있습니다. 이 패키지의 MPC는 기록 데이터로 모델을 평가하는 오프라인 도구입니다. `mpc_model_candidate.json`에는 유효하게 추정된 모델을 담고 `deployment_ready=false`로 저장합니다. 실제 제어기 연결에는 해당 MPC의 상태·입력·좌표계 확인이 필요합니다.

학습과 다른 세션으로 평가합니다.

```bash
ros2 run isv_water_trials mpc_replay \
  --model ~/water_trials/<학습시각>/mpc_model_candidate.json \
  --session ~/water_trials/<다른검증시각> \
  --config ~/water_trial_boat.json --goal-east 5 --goal-north 0
```

목표는 검증 세션의 첫 GNSS 위치를 원점으로 한 ENU 미터 좌표입니다. `frame_confirmed`, `lidar_frame_confirmed`와 실제 `lidar_x_m`, `lidar_y_m`, `lidar_yaw_deg`를 설정하고 장애물 검출 층을 확인합니다.

검증기는 독립 세션과 최신·완전한 scan을 요구합니다. `mpc_replay.json`에 가속도 예측 오차와 추진기 제안을 저장합니다. 실선 적용 전에는 sway·결합력·파도·지연·이동 장애물에 대한 추가 평가가 필요합니다.

## 검증 기록

```bash
cd ~/ros2_ws/src/isv_water_trials
python3 -m unittest discover -s tests -p 'test_identification.py' -v
python3 -m unittest discover -s tests -p 'test_ros_fixture.py' -v
```

모의 검사는 독립 ROS 도메인의 `/fixture` 센서·서비스를 사용합니다. 실물 수신과 운용 확인 결과는 [TESTING.md](TESTING.md)에 구분해 기록했습니다.

- [선박 운동 모델](https://www.fossen.biz/html/marineCraftModel.html)
- [rosbag2와 QoS](https://index.ros.org/r/rosbag2/)
