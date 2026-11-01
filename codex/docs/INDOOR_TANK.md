# 실내 수조 지도·왕복 시험

GPS 없이 `/scan`으로 이동량을 구하고, SLAM Toolbox로 2D 지도를 만듭니다. 지도 좌표는 m, 방향은 °입니다. 지도 원점과 방향은 SLAM 시작 위치를 기준으로 정해집니다.

## 1. PC 센서 화면

PC PowerShell에서 실행합니다. SSH 비밀번호를 입력하면 브라우저가 열립니다.

```powershell
cd C:\Users\Park\Documents\JetsonWaterTrial-20261029\kabot2026
powershell -ExecutionPolicy Bypass -File codex\sensor_viewer\start_viewer.ps1
```

주소: **http://127.0.0.1:8765**

- 라이다: 3D·위에서 보기, 16개 링을 고르게 추려 최대 5,000점·5 FPS로 표시합니다. 원본 수신은 그대로 유지합니다.
- IMU: 큰 나침반은 VectorNav 센서 Yaw를 북쪽 0°·동쪽 90°·남쪽 180°·서쪽 270°로 표시합니다. Roll·Pitch는 보조 값이며 `±180°`로 바꿀 수 있습니다. 회전속도는 °/s, 가속도는 m/s²입니다.
- GPS: 유효한 fix가 들어오면 위치·이동 경로와 지도 링크가 표시됩니다.
- 노드·토픽: 전체 그래프, 발행자·구독자 수, 관측한 수신 주기, 최근 메시지를 확인합니다. 큰 영상·점군은 선택한 스트림 위주로 수신합니다.
- 수조 지도: 현재 위치, A·B 지점, 계획 경로(청록), 2초 등속 예측(분홍)을 표시합니다. 현재 위치 화살표는 라이다 +X 방향입니다.

나침반은 VectorNav 원시 방위각을 사용합니다. 실내 철제 구조물·전류가 만드는 자기장 때문에 북쪽 오차가 생길 수 있습니다. ROS IMU와 VectorNav 원시 방위각은 좌표계가 다릅니다. 각 패널의 프레임과 북쪽 기준 표기를 확인하세요. 원본 메시지에서는 ROS 단위를 유지합니다.

## 2. 센서와 지도 시작

젯슨의 서로 다른 터미널에서 실행합니다. 새 터미널에는 ROS 환경이 자동 적용됩니다. 파일 수정 뒤에는 `cb`로 빌드합니다.

```bash
# 터미널 1: 이미 실행 중이면 유지
ros2 launch mechaship_bringup all_sensors.launch.py

# 터미널 2: 지도 작성 + 라이다 오도메트리
ros2 launch isv_water_trials tank_mapping.launch.py
```

처음에는 보이는 범위가 지도에 들어옵니다. RC로 천천히 움직여 수조 벽과 모서리를 여러 방향에서 관측하세요. 실제 벽 위치·거리와 지도, 배의 이동 방향과 위치 화살표를 비교한 뒤 주행에 사용합니다. 수면 반사와 대칭인 벽은 위치 추정 오차를 만들 수 있습니다.

현재 VLP-16은 케이블이 선미로 나오는 방향입니다. [공식 설명서 그림 9-1](https://docs.clearpathrobotics.com/assets/files/clearpath_robotics_011703-TDS1-e0837d8436bb21caf9d84acc8237d4ea.pdf)의 축은 **+Y=선수, +X=우현, +Z=위쪽**입니다.

[ROS 드라이버](../../velodyne/velodyne_pointcloud/src/lib/rawdata.cpp)의 VLP-16 변환 함수는 `ROS X = 공식 Y`, `ROS Y = -공식 X`, `ROS Z = 공식 Z`를 적용합니다. 따라서 `/velodyne_points`와 `/scan`에서는 **+X=선수, +Y=좌현, +Z=위쪽**입니다. 이 ROS 좌표를 그대로 사용하므로 추가 회전은 **0°**, 왕복 코드의 `--forward-axis-deg` 기본값도 `0`입니다.

위치 기준은 라이다 장착점입니다. 기본 장애물 여유 거리는 0.75m이며, 장착점에서 선체 끝까지의 거리와 회전 공간을 포함해야 합니다.

## 3. 지도 저장·현재 위치·목적지

지도를 충분히 만든 뒤 저장합니다.

```bash
ros2 run isv_water_trials tank_save --name ~/water_trials/tank_maps/tank
```

이미지·YAML과 위치 추정에 쓸 `.posegraph`·`.data`를 함께 저장합니다. 지도 작성 터미널을 Ctrl+C로 종료한 뒤, 저장 지도로 위치를 추정합니다.

```bash
ros2 launch isv_water_trials tank_mapping.launch.py \
  mode:=localization map_file:=$HOME/water_trials/tank_maps/tank
```

PC 화면에서 **현재 위치**를 누르고 지도 위 배의 위치를 클릭합니다. 초기 방향은 지도 +X가 0°, +Y가 90°인 라이다 +X 방향입니다. 실제 배 위치와 화면 위치가 일치하는지 확인합니다.

그다음 **A 지점 → B 지점 → 왕복 경로 저장**을 누릅니다. 벽·미관측 영역과 가까운 지점은 저장 단계에서 안내합니다. A·B는 1m 이상 떨어뜨립니다. 경로는 `~/water_trials/tank/route.json`에 저장됩니다. 지도 작성 세션이 바뀌면 A·B를 다시 선택합니다. 같은 저장 지도를 재사용하면 경로도 재사용할 수 있습니다.

SLAM Toolbox의 지도 저장과 `initialpose` 위치 지정은 [공식 문서](https://github.com/SteveMacenski/slam_toolbox#readme)를 따릅니다.

## 4. 왕복 시험과 기록

먼저 미리보기로 경로·대기 사유를 확인합니다. 이 명령은 추진기 서비스를 호출하지 않습니다.

```bash
ros2 run isv_water_trials indoor_trial
```

실제 왕복은 기존 `mode_test` 등 추진 명령 노드를 Ctrl+C로 끝낸 뒤 실행합니다. 현재 장착 방향은 기본값 `--forward-axis-deg 0`을 사용합니다.

```bash
# 라이다 +X가 선수 방향인 경우
ros2 run isv_water_trials indoor_trial --execute --percent 20 --forward-axis-deg 0
```

배는 현재 위치에서 A로 이동한 뒤 A↔B를 반복합니다. 도착하면 2초간 중립을 유지합니다. `--percent`는 좌우 각 채널의 PWM 명령 한도이며 기본 20%, 설정 범위는 0 초과~30%입니다. 출력 비율과 실제 속도·추력의 비율은 다릅니다.

RC 우선권과 방향 변환은 기존 아두이노 설정을 사용합니다. 위치·스캔이 끊기거나 벽과 가까워지면 AUTO 명령을 중립으로 두고 노드는 대기합니다. 데이터가 회복되면 경로를 다시 계산합니다. Ctrl+C는 중립 응답을 확인한 뒤 왕복 노드를 종료합니다. SSH 연결이 끊겨도 프로세스는 유지됩니다.

전체 센서 원본은 별도 터미널에서 기록합니다. 점군과 패킷이 함께 저장되므로 여유 공간을 확인하세요.

```bash
mkdir -p ~/water_trials/bags
ros2 bag record -a -o ~/water_trials/bags/tank_$(date +%Y%m%d_%H%M%S) \
  --max-bag-size 1073741824
```

왕복 노드는 `~/water_trials/indoor/<시각>/`에 위치·연속 오도메트리·IMU Z축 회전속도·명령·MCU 응답·운항 구간을 기록합니다. 모델 분석에는 AUTO에서 응답이 확인된 기록만 사용합니다.

```bash
ros2 run isv_water_trials indoor_analyze ~/water_trials/indoor/<시각>
```

결과는 `response_model.json`, 검증 구간 예측 CSV와 `analysis.html`입니다. PC로 가져와 확인할 수 있습니다.

```powershell
scp -r isv@10.182.77.20:/home/isv/water_trials/indoor/<시각> .\tank_result
Start-Process .\tank_result\analysis.html
```

분석 항목은 속도·각속도·주행거리, PWM에 대한 전진·선회 응답 계수와 시정수, 학습에 쓰지 않은 마지막 30% 구간의 예측 오차입니다. 출력 변화가 부족하면 계수 산출 보류 사유를 남깁니다. 절대 저항·부가질량·관성모멘트에는 좌우 추진기의 PWM–추력 실측이 추가로 필요합니다. 총질량 25kg, 길이 1.1m, 폭 0.5m, 추진기 간격 0.32m는 기록에 포함됩니다.

## 주요 토픽

| 토픽 | 내용 |
|---|---|
| `/velodyne_points`, `/scan` | 3D 점군, 2D 거리 스캔 |
| `/vectornav/imu`, `/vectornav/raw/common` | IMU, 원시 센서 값 |
| `/gps/fix` | GPS 위치와 fix 상태 |
| `/map` | 2D 점유 지도 |
| `/tank/odom` | 연속 라이다 오도메트리; 속도 분석 기준 |
| `/tank/pose` | 지도상 현재 라이다 위치 |
| `/tank/session` | 현재 지도 세션·작성/위치 추정 모드 |
| `/tank/status` | 왕복 상태·목적지·계획 경로 |
| `/tank/prediction` | 2초 등속 예측; 학습된 물리 모델과 구분 |
| `/actuators/status`, `/water_trial/command` | MCU 상태, 왕복 명령 기록 |

확인 범위: 실제 센서 화면·점군 16개 링·지도 생성·저장·불러오기와 추진 없는 미리보기를 확인했습니다. 실제 왕복 주행과 수조 치수에 대한 지도 정확도는 현장 검증이 필요합니다.
