# 정상 운용 기준 — 2026-10-30

2026-10-30 사용자가 확인한 정상 운용 상태를 정리했습니다. RC와 코드 명령의 좌우 전후진, 모드 전환, EMO, 센서 브링업과 ESC 초기화를 기준으로 삼습니다. 이후 수정할 때도 이 배선과 동작을 유지합니다.

## 1. 명령값과 중립

앞으로 명령값은 **1000=최대 후진, 1500=정지, 2000=최대 직진**으로 부릅니다. 좌현과 우현은 실제 배의 왼쪽과 오른쪽을 뜻하며, 배선과 방향 보정은 아두이노가 처리합니다.

코드에 전달할 때는 현재 ROS 서비스·시리얼 범위인 **1100~1900**으로 변환합니다. 논리 중립은 **1500**입니다.

| 사용자 표현 | `mode_test --percent` | 실제 ROS 명령값 |
|---:|---:|---:|
| 1000 | -100 | 1100 |
| 1250 | -50 | 1300 |
| 1500 | 0 | 1500 |
| 1750 | 50 | 1700 |
| 2000 | 100 | 1900 |

변환식은 `percent=(사용자값-1500)/5`, `ROS값=1500+round(0.8×(사용자값-1500))`입니다. 현재 서비스에 1000이나 2000을 직접 넣으면 범위 오류가 납니다. 퍼센트는 PWM 명령 폭을 나타냅니다. 실제 추력과 속도는 별도 측정으로 구합니다.

중립값은 적용 위치에 따라 다음과 같습니다.

| 위치 | 현재 값 | 의미 |
|---|---:|---|
| 사용자 표현·ROS 명령 | 1500 | 논리적 정지 |
| RC 수신기 입력 중심 | 좌우 1496µs | 손을 놓은 스틱 기준 위치 |
| ESC 실제 정지·초기화 출력 | 좌우 1480µs | 현재 배에서 정상 확인한 실제 PWM |

## 2. 필수 실행 명령

PC에서 젯슨에 접속합니다.

```bash
ssh isv@10.182.77.20
```

새 젯슨 Bash/SSH 터미널은 ROS 환경이 자동 적용되어 바로 실행 명령을 쓸 수 있습니다. `.bashrc`에서 ROS Humble과 설치된 작업 공간을 불러옵니다. 기본 통신 설정은 `ROS_DOMAIN_ID=50`, `ROS_LOCALHOST_ONLY=0`입니다.

파일 수정 후에는 `cb`로 전체 빌드와 현재 터미널의 환경 적용을 한 번에 처리합니다. 이미 열려 있는 다른 터미널의 환경을 갱신할 때는 `source ~/.bashrc`를 사용합니다.

```bash
cb
# 해당 두 패키지만 빌드할 경우
cb --packages-select isv_water_trials mechaship_bringup
```

**터미널 1 — 전체 센서와 아두이노 브리지:**

```bash
ros2 launch mechaship_bringup all_sensors.launch.py
```

GPS·VectorNav IMU·Velodyne·아두이노를 함께 켭니다. 카메라 연결 시 `use_camera:=true`를 추가합니다. 같은 브링업을 다시 실행하면 기존 세션을 정리하고 새로 시작합니다. Ctrl+C는 해당 센서와 브리지의 하위 프로세스까지 종료합니다. 젯슨을 켠 뒤 브링업은 직접 실행합니다.

**터미널 2 — 다음 중 하나만 실행, 50% 직진:**

```bash
# 좌현만 직진, 우현 정지
ros2 run isv_water_trials mode_test --side left --percent 50
# 우현만 직진, 좌현 정지
ros2 run isv_water_trials mode_test --side right --percent 50
# 양쪽 직진
ros2 run isv_water_trials mode_test --side both --percent 50
```

후진은 `--percent -50`, 30%는 `--percent 30`으로 지정합니다. 선택한 쪽에 시험 출력을 보내고 반대쪽에는 논리 중립 1500을 보냅니다. 추진 명령 노드는 한 번에 하나씩 실행합니다.

`mode_test`는 Ctrl+C까지 계속 송신하며 `--seconds`는 무시합니다. RC/AUTO 선택은 아두이노가 담당합니다. 서비스·USB가 끊기면 재시도하고, SSH 단절(SIGHUP)과 SIGTERM에도 실행을 유지합니다. 전원 차단·SIGKILL·치명적 오류로 종료된 경우에는 다시 실행해야 합니다.

**시험 종료·정지:** 실행한 시험 터미널에서 **Ctrl+C**를 누릅니다. 중립 명령 ACK를 확인하고 AUTO를 해제한 뒤 종료합니다. 연결이 끊겨 ACK를 받지 못하면 `STOP_UNCONFIRMED`로 보고합니다. MCU는 AUTO 명령이 400ms 끊기면 중립으로 전환합니다.

```bash
# 다른 터미널에서 MCU에 즉시 중립 요청
ros2 run isv_water_trials stop_thrusters
# 브링업이 남았을 때 수동 전체 정리
~/bin/stop_bringup
```

`stop_thrusters`는 한 번 중립을 요청합니다. 실행 중인 `mode_test`는 다음 명령으로 다시 출력을 보내므로, 시험을 계속 멈춰 두려면 해당 터미널에서 Ctrl+C를 누릅니다. `stop_bringup`은 독립 시험 프로세스도 함께 정리합니다.

**ESC 재초기화:** EMO 해제 후 필요할 때 다른 터미널에서 실행합니다.

```bash
ros2 run isv_water_trials reinitialize_esc
```

시험 노드가 실행 중이면 중립 응답부터 7초간 중립을 유지한 뒤 원래 출력을 자동 재개합니다. 시험 노드가 없으면 MCU에 정지를 요청합니다. EMO는 ESC 전원을 차단합니다. UNO의 감지선은 미연결 상태이므로, EMO 해제 후 초기화가 필요한 시점에 이 명령을 사용합니다.

## 3. 아두이노의 확정 배선과 동작

보드는 UNO R3, ESC는 Blue Robotics Basic ESC 500입니다. 모드 신호는 **수신기 CH5 → UNO D4**로 연결합니다.

| 기능 | 수신기 → UNO | ESC 출력 | RC 반전 | AUTO 반전 |
|---|---|---|---|---|
| 좌현 | CH2 → D2 | D9 | true | true |
| 우현 | CH3 → D3 | D10 | false | true |
| RC/AUTO 모드 | CH5 → **D4** | 해당 없음 | 해당 없음 | 해당 없음 |

좌우 입력은 독립적으로 처리합니다. 방향 플래그는 현재 배선에서 확인한 값이며, RC와 AUTO를 따로 조정합니다.

- D4가 1700µs 초과이고 세 입력이 모두 유효·최신이면 RC가 우선합니다. 나머지는 AUTO로 처리합니다.
- RC→AUTO에서는 기존 출력·무장을 해제하고 20ms 동안 중립을 유지합니다. 새 AUTO 무장·명령을 바로 받아 다음 명령부터 재개합니다.
- AUTO→RC는 현재 조종기 입력을 따릅니다. 정지 명령 S를 받은 뒤에는 양쪽 스틱을 중립에 0.5초 두면 RC로 복귀합니다.
- RC PWM은 900~2100µs, 최근 100ms 이내 수신값을 사용합니다. 신호가 끊기면 기존 명령을 해제하고, 새 무장 명령으로 AUTO를 시작할 수 있습니다. 무선 단절 후에도 수신기가 PWM을 내보내는 경우에는 수신기의 failsafe 설정을 함께 확인해야 합니다.
- 부팅·USB 브리지 재연결·수신기 신규 연결 때 ESC에 **1480µs를 7초** 보내 초기화합니다. 이 동안 AUTO 무장·추진 요청은 거절되고 송신 노드는 재시도합니다. RC 단독 조종은 UNO 전원과 수신기 연결로 동작합니다.
- 출력은 Timer1 하드웨어 PWM **400Hz**, D9/OC1A·D10/OC1B입니다. AUTO 명령 유효 시간은 400ms, MCU 하드웨어 watchdog은 250ms입니다.

RC 입력은 양쪽 중심 1496µs, 중립 구간 1446~1546µs, 설정 끝점 1000~2000µs입니다. 공통 대칭 폭 ±496µs에 같은 곡선을 사용합니다. 중립 밖에서는 `u=clamp((|입력-1496|-50)/446,0,1)`, 논리 출력 편차는 `round(400u²)`입니다. 출력은 0부터 증가해 입력 1000/1992µs에서 최대에 도달합니다. 끝점은 현재 설정값입니다.

ESC 변환은 논리 1500을 실제 1480µs로 옮기고, 방향 플래그와 양쪽 물리 끝점 1100/1900µs를 적용합니다. 현재 AUTO +50%의 논리 1700은 실제 약1290µs, AUTO -50%의 논리 1300은 실제 약1690µs입니다. 실제 PWM 숫자가 작아도 현재 설치 기준으로 직진일 수 있습니다.

## 4. 주요 토픽과 서비스

현재 코드에서 사용하는 토픽입니다. 2026-10-30 재접속 당시 브링업은 정지 상태였으며, 아래 목록은 코드와 이전 수신 기록을 기준으로 정리했습니다.

| 토픽 | ROS 형식 | 용도 |
|---|---|---|
| `/gps/fix` | `sensor_msgs/msg/NavSatFix` | 위도·경도·고도·fix 상태 |
| `/gps/fix_velocity` | 드라이버 발행 형식 확인 | GPS 속도 |
| `/vectornav/raw/common` | `vectornav_msgs/msg/CommonGroup` | IMU 원시 공통 데이터 |
| `/vectornav/imu` | `sensor_msgs/msg/Imu` | 표준 IMU |
| `/velodyne_packets` | `velodyne_msgs/msg/VelodyneScan` | 원시 라이다 패킷 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 3D 점군 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 선택한 한 ring의 2D 스캔 |
| `/actuators/status` | `std_msgs/msg/String` JSON | 모드·무장·RC 원시 입력·논리 명령·핀 관측 PWM |
| `/water_trial/command` | `std_msgs/msg/String` JSON | 좌우 명령 요청·성공·실패 로그 |
| `/water_trial/mark` | `std_msgs/msg/String` | 시험 구간 표식 |
| `/actuators/stop_requested` | `std_msgs/msg/Bool` | 소프트웨어 제어 상태 |
| `/camera/image_raw`, `/camera/camera_info` | `Image`, `CameraInfo` | 카메라 활성 시에만 사용 |

라이다는 VLP16 설정이며 점군에 16개 ring이 들어옵니다. 이전 실측 ID는 0~15, 자동 선택 `/scan`은 ring 8이었습니다. `/scan`은 선택한 한 층의 데이터입니다. IP 192.168.1.201, UDP 2368, 600rpm을 사용하며 장착 좌표는 별도 보정합니다.

`/actuators/status`에서 `mode_name`, `armed`, `rc_left_us`, `rc_right_us`, `rc_mode_us`, `observed_left_us`, `observed_right_us`, `receiving`, `error`를 확인합니다. `left/right`는 논리 명령, `observed_*`는 UNO 핀의 펄스 관측값입니다. ESC 입력단 신호와 실제 회전은 별도로 확인해야 합니다.

UNO의 EMO 감지선은 미연결 상태로 `estop_known=false`이며 `/sensor/emo/status` 메시지도 미발행 상태입니다. 기존 보드의 `/sensor/battery/voltage`, `/smc_2000/fix`는 해당 장치 연결이 필요합니다.

| 서비스 | 형식 | 의미 |
|---|---|---|
| `/actuators/arm` | `std_srvs/srv/SetBool` | AUTO 무장/해제 |
| `/actuators/throttle_left/set_pulse_width` | `mechaship_interfaces/srv/ThrottlePulseWidth` | 좌현 논리 1100~1900 |
| `/actuators/throttle_right/set_pulse_width` | 같은 형식 | 우현 논리 1100~1900 |
| `/actuators/stop` | `std_srvs/srv/Trigger` | 중립·AUTO 무장 해제 |
| `/water_trial/reinitialize_test` | `std_srvs/srv/Trigger` | 실행 중 mode_test의 중립 7초 후 자동 재개 |

자율운항 노드에서 서비스를 직접 호출할 때는 좌우 요청을 동시에 보냅니다. 브리지는 0.1초 이내의 두 요청을 묶어 한 시리얼 명령으로 전달합니다. 간단한 추진 시험은 이 조건을 처리하는 `mode_test`를 사용합니다.

```bash
ros2 topic list -t
ros2 topic echo /actuators/status
ros2 topic echo /gps/fix --once
ros2 topic hz /scan
ros2 service list -t
```

## 5. 센서 기록·GPS 지도·분석

**전체 센서 원시 데이터는 rosbag으로 기록**합니다. 별도 터미널에서 실행하고 Ctrl+C로 저장을 마칩니다.

```bash
mkdir -p ~/water_trials
ros2 bag record -o ~/water_trials/bag_$(date +%Y%m%d_%H%M%S) --regex '^/(gps/.*|vectornav/.*|velodyne_packets|velodyne_points|scan|actuators/.*|water_trial/.*|camera/.*|tf|tf_static|diagnostics)$'
```

간단한 JSONL 기록·구간 표시·GPS 지도용 파일은 다음 명령입니다.

```bash
ros2 run isv_water_trials record --output ~/water_trials
ros2 run isv_water_trials mark straight --note '직진 구간'
ros2 run isv_water_trials mark coast --note '중립 관성 구간'
ros2 run isv_water_trials gps_track --output ~/water_trials/gps_$(date +%Y%m%d_%H%M%S)
```

`record`는 GPS·IMU·scan·명령을 JSON으로 기록합니다. 3D 점군과 원시 Velodyne 패킷까지 보존할 때는 rosbag을 사용합니다. 분석에는 세션의 `metadata.json`, `raw.jsonl`이 필요합니다. GPS 도구는 `gps_track.csv`, `gps_track.geojson`, `gps_latest_url.txt`를 만듭니다. 마지막 파일의 Google Maps 링크로 위치를 확인합니다.

```bash
ros2 run isv_water_trials analyze ~/water_trials/<실제_세션_폴더>
ros2 run isv_water_trials avoid_preview
```

분석 결과는 `report.json`, `report_KO.md`, `kinematics.csv`, `mpc_model_candidate.json`입니다. `avoid_preview`는 `/scan`으로 회피 방향을 제안하는 조회 도구입니다. 동역학 식별과 MPC replay는 기록 데이터를 사용합니다. 자율 회피와 MPC의 수상 성능은 별도 시험으로 평가해야 합니다.

`boat_live.json`에는 질량 25kg, 길이 1.1m, 폭 0.50m, 추진기 중심 간격 0.32m가 저장되어 있습니다. 추력 환산·센서 장착 좌표·동역학 식별에는 실측이 필요합니다. 실좌표와 원시 로그는 젯슨의 시험 폴더에 보관합니다.

## 6. 기준 소스와 복원 위치

저장소 최상위는 젯슨 `~/ros2_ws/src`의 14개 폴더 구조를 따릅니다. 문서·도구·보관 사본은 `codex/`에 모았습니다. 이번 수집의 파일별 해시는 [젯슨 소스 목록](JETSON_SOURCE_MANIFEST.json)에 있습니다.

| 저장소 경로 | 젯슨 위치/역할 |
|---|---|
| `firmware/kabot_actuator/` | `/home/isv/boat_firmware/kabot_actuator/`, UNO 운용 소스 |
| `firmware/kabot_actuator/board_config.h` | 실제 빌드에 쓰는 배선·중립·방향 기준 |
| `isv_water_trials/` | `/home/isv/ros2_ws/src/isv_water_trials/` |
| `codex/deploy/jetson/mechaship_bringup/` | 폴더 정리 전 배포용 사본 |
| `mechaship/mechaship_bringup/` | 현재 젯슨과 같은 경로의 브링업 소스 |
| `codex/scripts/ros2_shell_helpers.bash` | `~/.config/isv/ros2_helpers.bash`, `.bashrc`에서 통신 기본값·cb 자동 로드 |
| `codex/scripts/stop_bringup.py` | `~/bin/stop_bringup`, 수동 프로세스 정리 |

설정을 복원할 때는 `board_config.h`의 배선·방향·ESC 중립값을 기준으로 합니다. `actuators.json`은 참고 사본이며 `codex/firmware/generate_config.py`는 수동 실행 도구입니다.

아두이노 업로드는 젯슨에서 수행합니다.

```bash
# /home/isv/boat_firmware에 업로더와 tests까지 배치된 경우
cd ~/boat_firmware
bash jetson_upload.sh --build-only
# 브리지/시리얼 점유를 정리하고 업로드할 때
bash jetson_upload.sh --flash
```

기본 동작은 빌드입니다. `--flash`는 기존 flash를 두 번 읽고 EEPROM을 백업한 뒤 새 이미지를 업로드·검증합니다. 업로드 시 UNO가 재시작됩니다.

폴더 정리 전 운용 기준 태그는 `boat-stable-20261030`입니다. 해당 태그를 별도 디렉터리에 체크아웃하고 현재 소스와 비교한 뒤 필요한 패키지를 배포합니다.

## 7. 확인 범위

- 사용자 확인: 현 상태의 RC/AUTO 좌우 전후진, 모드 전환, EMO 해제 후 기동, 시험 코드·초기화 정상.
- 전체 소스 수집: 젯슨 소스·펌웨어 1,322개 항목과 최상위 폴더 14개 일치. 파일 내용과 심볼릭 링크는 `JETSON_SOURCE_MANIFEST.json`으로 확인.
- 이전 부분 감사: 36개 중 35개 해시 일치, 나머지는 CMake 주석 한 줄 차이. 당시 결과는 `BASELINE_RUNTIME_SHA256.json`에 보관.
- 로컬 unittest: 총 52개 중 32개 통과, ROS/POSIX 환경이 필요한 20개 제외. 수치 모델은 모의 데이터로 검증.
- 젯슨 모의 서비스 검사 6개 통과: SIGHUP/SIGTERM 유지, 외부 stop 무시, 초기화 후 자동 재개, 일회 중립 후 재개, Ctrl+C 중립.
- 확인 방법: 사용자 실물 확인, 소스 비교, 모의 서비스 검사. 이전 문서의 실패·미검증 기록은 각 기록 시점의 상태.
