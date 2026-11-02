# KABOAT 2026

[야외 RC 기록·모델 학습·GPS 순환 주행](codex/docs/OUTDOOR_RC.md): 브링업과 함께 위치·방향·경로를 표시하고, RC 기록으로 모델을 학습합니다. 대시보드에서 현재 모델을 적용하고 여러 목적지를 순서대로 반복 운항합니다.

[PC 센서 화면·실내 수조 SLAM·왕복 시험](codex/docs/INDOOR_TANK.md): 라이다·IMU·전체 토픽을 PC에서 보고, GPS 없이 지도에서 A·B 지점을 정합니다.

## 필수 명령어

**SSH 접속** — PC에서 젯슨에 연결합니다.

```bash
ssh isv@10.182.77.20
```

새 젯슨 터미널은 ROS 환경이 자동 적용됩니다. 파일 수정 후에는 `cb`로 빌드·적용합니다.

**센서 브링업** — GPS·IMU·라이다·아두이노 브리지를 켭니다. SSH가 끊겨도 젯슨에서 계속 실행됩니다.

```bash
ros2 run isv_water_trials tank_session start sensors
```

기존 `ros2 launch mechaship_bringup all_sensors.launch.py`도 같은 센서를 켭니다. 두 방식 중 하나만 사용하세요. 직접 launch한 경우 해당 터미널의 Ctrl+C로 종료합니다. 카메라를 연결했다면 `use_camera:=true`를 추가합니다.

PC 대시보드 프로그램을 켜 두면 브링업 시 자동으로 연결됩니다. `stop all` 뒤에도 화면은 대기 상태로 유지됩니다. 닫아 둔 대시보드 탭은 다음 브링업 때 다시 열립니다. 처음 실행하는 방법은 [PC 센서 화면](codex/docs/INDOOR_TANK.md#1-pc-센서-화면)에 있습니다.

**PC 대시보드** — 저장소 폴더의 PowerShell에서 실행합니다.

```powershell
powershell -ExecutionPolicy Bypass -File codex\sensor_viewer\start_viewer.ps1
```

`http://127.0.0.1:8765/`에서 실제 센서·기록·학습을 확인합니다. 젯슨 주소가 바뀌면 실행 명령에 `-Jetson 새주소`를 붙입니다. 학습 모델을 **현재 모델 적용**으로 선택하고 지도에 지점을 저장한 뒤 **순환 주행 시작**을 누릅니다. 아래 수조 지도 명령은 실내 시험에 사용합니다.

**새 지도 작성** — 센서 브링업 뒤 실행합니다. 지도를 바꾸기 전에는 왕복을 종료하세요.

```bash
ros2 run isv_water_trials tank_session stop trial
ros2 run isv_water_trials tank_session stop mapping
ros2 run isv_water_trials tank_session start mapping mode:=mapping
```

RC로 천천히 이동하며 벽·모서리를 관측합니다. 새 지도 작성은 저장할 때까지 이어집니다. `tank_session`의 매핑 시작·종료에 맞춰 원본 센서 기록도 자동으로 켜지고 꺼집니다.

**지도 저장·고정** — 작성한 지도를 `tank_pnu.tankmap` 하나로 저장하고, 같은 지도에서 자동 위치추정으로 전환합니다.

```bash
ros2 run isv_water_trials tank_session freeze --name ~/water_trials/tank_maps/tank_pnu.tankmap
```

이 명령이 지도 세션을 자동 재시작하므로 따로 `start mapping`을 입력할 필요가 없습니다. 저장 후 벽·격자는 고정되고 배 위치만 갱신합니다. 같은 이름이 있으면 `tank_pnu_20261102.tankmap`처럼 새 이름을 쓰세요.

**저장 지도 불러오기** — 다음 접속부터 마지막으로 고정한 지도를 불러옵니다. 이미 실행 중이면 기존 세션을 유지합니다.

```bash
ros2 run isv_water_trials tank_session start mapping
```

다른 저장 지도를 고르려면 지도 세션을 종료하고 파일을 지정합니다.

```bash
ros2 run isv_water_trials tank_session stop mapping
ros2 run isv_water_trials tank_session start mapping mode:=localization map_file:=~/water_trials/tank_maps/tank_pnu.tankmap
```

초기 위치·방향은 자동으로 찾습니다. **자동 위치 찾기**를 누르면 현재 추정과 경로를 유지하며 주변 정합을 보정합니다. 후보 화살표를 비교하려면 **후보 1·2…**를 선택하고, 실제 위치와 맞으면 **선택 후보를 현재 위치로 지정**을 누르세요. 위치와 방향을 확인한 뒤 왕복을 시작합니다.

**A·B 왕복·응답 기록** — 대시보드에서 **A 지점 → B 지점 → 왕복 경로 저장** 후 실행합니다.

```bash
ros2 run isv_water_trials tank_session start trial --execute --percent 20
```

좌우 출력 상한은 각각 20%이며 선회·회피·도착 때 감속합니다. 응답 그래프는 대시보드에서 확인하고, 센서 원본과 이동 응답·분석 결과는 자동 저장됩니다. SSH가 끊겨도 젯슨의 왕복은 계속됩니다. A·B는 같은 지도에서 재사용하며 새 지도에서는 다시 지정합니다.

문자 상태·오류 로그가 필요할 때만 `ros2 run isv_water_trials tank_session attach trial`로 터미널 로그를 엽니다. 이 화면의 Ctrl+C는 중립을 요청하고 왕복을 종료합니다.

**상태 확인·종료**

```bash
ros2 run isv_water_trials tank_session status
ros2 run isv_water_trials tank_session stop trial  # 왕복 종료·중립
ros2 run isv_water_trials tank_session stop mapping  # 지도 작성·위치추정 종료
ros2 run isv_water_trials tank_session stop sensors  # 센서 브링업 종료
ros2 run isv_water_trials tank_session stop all      # 왕복·기록·지도·센서 전부 종료
```

개별 명령은 필요한 것만 실행하세요. 센서나 매핑을 끄기 전에는 왕복부터 종료합니다. 매핑을 종료하면 원본 센서 기록도 저장을 마치고 종료합니다. 새 지도를 남기려면 종료 전에 `freeze`로 고정 저장하세요. 저장한 `.tankmap` 파일은 종료 후에도 유지됩니다.

위 종료 명령은 `tank_session`으로 실행한 세션에 적용됩니다. `ros2 launch ...`로 직접 실행한 브링업·매핑이나 `mode_test`는 해당 터미널에서 Ctrl+C로 종료합니다.

왕복 기록: `~/water_trials/indoor/`, 원본 bag: `~/water_trials/bags/`, 실행 로그: `~/water_trials/runtime/`. PC 대시보드 실행과 위치추정·분석 기준은 [수조 운용 안내](codex/docs/INDOOR_TANK.md)에 있습니다.

**추진기 시험** — 다른 터미널에서 실행합니다. 아래 명령은 양쪽 50% 직진입니다.

```bash
ros2 run isv_water_trials mode_test --side both --percent 50
```

`--side left`는 좌현, `--side right`는 우현이며 후진은 `--percent -50`입니다. 시험은 Ctrl+C까지 계속 실행됩니다. RC/AUTO 선택은 아두이노가 처리합니다. Ctrl+C를 누르면 중립 응답을 확인하고 종료합니다.

**ESC 초기화** — EMO 해제 후 재초기화가 필요할 때 실행합니다.

```bash
ros2 run isv_water_trials reinitialize_esc
```

실행 중인 시험은 중립을 7초 유지한 뒤 원래 출력을 자동 재개합니다.

**일회 중립 요청** — MCU에 즉시 정지를 요청합니다.

```bash
ros2 run isv_water_trials stop_thrusters
```

실행 중인 시험은 다음 명령부터 다시 출력합니다. 시험을 계속 멈춰 두려면 시험 터미널에서 Ctrl+C를 누릅니다.

## 저장소 구조

젯슨 `/home/isv/ros2_ws/src`의 14개 폴더를 저장소 최상위에 같은 이름과 구조로 보관합니다. `firmware/`에는 현재 UNO에 적용한 펌웨어를, `codex/`에는 문서·백업·로컬 도구·겨울 최종본을 모았습니다.

```text
kabot2026/
├── fusion_viewer/
├── isv_kaboat/
├── isv_water_trials/
├── mechaship/
├── mechaship_example/
├── micro-ROS-Agent-humble/
├── micro_ros_msgs/
├── ouster-ros/
├── rtcm_msgs-master/
├── ublox-ros2/
├── usb_cam/
├── vectornav/
├── velodyne/
├── yolo_ros/
├── firmware/kabot_actuator/
└── codex/
    ├── docs/
    ├── scripts/
    ├── tests/
    ├── firmware/
    ├── deploy/
    ├── backups/
    ├── 로컬보관/
    └── 레거시코드/동계 최종코드/
```

Git 내부 파일, 빌드 결과·캐시, 개인 개발 설정과 실측 GPS 로그는 수집 대상에서 제외했습니다. 원본 소스 ZIP과 드라이버 시험 자료는 함께 보관합니다. 파일별 해시는 [젯슨 소스 목록](codex/docs/JETSON_SOURCE_MANIFEST.json)에서 확인할 수 있습니다.

## 주요 설정과 문서

- 명령 표현: **1000=최대 후진, 1500=정지, 2000=최대 직진**. 현재 ROS 서비스에는 1100/1500/1900으로 변환해 전달합니다.
- 실제 ESC 정지 출력: 양쪽 **1480µs**.
- 배선: 수신기 CH2→UNO D2 좌현, CH3→D3 우현, CH5→D4 모드. ESC 출력은 D9 좌현·D10 우현입니다.
- 선체: 25kg, 길이 1.1m, 폭 0.50m, 추진기 중심 간격 0.32m.

| 문서 | 내용 |
|---|---|
| [정상 운용 기준](codex/docs/BOAT_BASELINE_2026-10-30.md) | 실행 명령·주요 토픽·모드 전환·ESC 초기화·기록 방법 |
| [프로젝트 구조](codex/PROJECT_STRUCTURE.md) | 저장소와 젯슨 경로 대응 |
| [아두이노 설정](codex/firmware/README.md) | 현재 펌웨어 배선·출력·통신 |
| [cb 사용법](codex/docs/CB_USAGE.md) | 빌드와 셸 환경 적용 |
| [겨울 최종 코드](codex/레거시코드/README.md) | 기존 `자율운항 최종코드` 커밋 원본 |

운용 확인 기준은 `boat-stable-20261030` 태그입니다. 이 태그는 폴더 정리 전 상태를 보존합니다.
