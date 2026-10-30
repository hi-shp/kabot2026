# 정상 운용 기준 — 2026-10-30

이 문서는 사용자가 RC 좌우 전후진, RC/AUTO 전환, 코드 좌우 전후진, EMO, 센서 브링업, 시험 노드와 ESC 초기화의 정상 작동을 확인한 시점의 기준입니다. 이후 수정 시 이 좌우·방향·중립·모드 동작을 보존합니다. 문서화 작업에서는 운용 코드를 변경하거나 추진 시험을 실행하지 않았습니다.

## 1. 앞으로 사용하는 명령 표현

사용자가 말하는 **1000=최대 후진, 1500=정지, 2000=최대 직진**은 운용상의 논리 표현입니다. 좌현/우현은 현재 확인된 실제 배의 좌현/우현을 뜻합니다. RC와 AUTO의 배선·방향 보정은 아두이노가 담당합니다.

현재 정상 코드의 ROS 서비스·시리얼 입력 범위는 **1100~1900**, 중립은 **1500**입니다. 편의 표현을 바꾸기 위해 정상 API나 ESC 설정을 변경하지 않았습니다.

| 사용자 표현 | `mode_test --percent` | 실제 ROS 명령값 |
|---:|---:|---:|
| 1000 | -100 | 1100 |
| 1250 | -50 | 1300 |
| 1500 | 0 | 1500 |
| 1750 | 50 | 1700 |
| 2000 | 100 | 1900 |

변환은 `percent=(사용자값-1500)/5`, `ROS값=1500+round(0.8×(사용자값-1500))`입니다. **1000/2000을 현재 서비스에 그대로 넣으면 범위 오류**입니다. 퍼센트는 PWM 명령 폭의 비율이며, 실측 추력·속도 비율이 아닙니다.

서로 구분할 세 중립값:

| 위치 | 현재 값 | 의미 |
|---|---:|---|
| 사용자 표현·ROS 명령 | 1500 | 논리적 정지 |
| RC 수신기 입력 중심 | 좌우 1496µs | 손을 놓은 스틱 기준 위치 |
| ESC 실제 정지·초기화 출력 | 좌우 1480µs | 현재 배에서 정상 확인한 실제 PWM |

## 2. 필수 실행 명령

PC에서 접속합니다. 비밀번호는 문서나 Git에 저장하지 않습니다.

```bash
ssh isv@10.182.77.20
```

젯슨의 각 ROS 터미널에서 환경을 적용합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=50 ROS_LOCALHOST_ONLY=0
```

수정 후 전체 빌드·현재 셸 적용은 `cb`입니다. 빌드만 하며 모터를 시험하지 않습니다.

```bash
cb
# 해당 두 패키지만 빌드할 경우
cb --packages-select isv_water_trials mechaship_bringup
```

**터미널 1 — 전체 센서와 아두이노 브리지:**

```bash
ros2 launch mechaship_bringup all_sensors.launch.py
```

GPS·VectorNav IMU·Velodyne·아두이노가 기본 활성입니다. 카메라는 기본 비활성이며 연결 시 `use_camera:=true`를 추가합니다. 같은 브링업 재실행 시 기존 세션을 정리하고 새 세션을 시작합니다. 이 터미널의 Ctrl+C는 해당 센서·브리지 하위 프로세스까지 종료합니다. 젯슨 부팅 후 자동 실행 서비스는 별도로 설정되어 있지 않습니다.

**터미널 2 — 다음 중 하나만 실행, 50% 직진:**

```bash
# 좌현만 직진, 우현 정지
ros2 run isv_water_trials mode_test --side left --percent 50
# 우현만 직진, 좌현 정지
ros2 run isv_water_trials mode_test --side right --percent 50
# 양쪽 직진
ros2 run isv_water_trials mode_test --side both --percent 50
```

후진은 같은 명령에서 `--percent -50`입니다. 30%는 `--percent 30`입니다. 선택하지 않은 쪽에는 논리 중립 1500을 계속 보냅니다. **추진 명령 노드를 중복 실행하지 않습니다.**

`mode_test`는 시간 제한 없이 계속 송신합니다. 이전 `--seconds` 인자는 호환상 받지만 무시합니다. 호스트에서 RC/AUTO나 조종기 연결 여부를 판단해 송신을 막지 않습니다. 아두이노만 RC 우선권을 결정합니다. 서비스·USB가 끊기면 재시도하고, SSH 단절의 SIGHUP과 SIGTERM을 무시합니다. 전원 차단·SIGKILL·복구 불가능한 프로세스 오류까지 생존한다는 의미는 아닙니다.

**시험 종료·정지:** 실행한 시험 터미널에서 **Ctrl+C**를 누릅니다. 중립 명령 ACK를 확인하고 AUTO를 해제한 뒤 종료합니다. 연결이 끊겨 ACK를 받지 못하면 `STOP_UNCONFIRMED`로 보고합니다. MCU는 AUTO 명령이 400ms 끊기면 중립으로 전환합니다.

```bash
# 다른 터미널에서 MCU에 즉시 중립 요청
ros2 run isv_water_trials stop_thrusters
# 브링업이 남았을 때 수동 전체 정리
~/bin/stop_bringup
```

`stop_thrusters`는 영구 정지 잠금이 아닙니다. **계속 송신 중인 mode_test는 다음 새 명령으로 다시 무장·출력**하므로, 지속 정지는 해당 시험의 Ctrl+C를 사용합니다. `stop_bringup`은 독립 시험 프로세스도 정리하는 수동 관리 명령입니다.

**ESC 재초기화:** EMO 해제 후 필요할 때 다른 터미널에서 실행합니다.

```bash
ros2 run isv_water_trials reinitialize_esc
```

실행 중인 시험 노드가 있으면 종료하지 않고, 중립 ACK부터 7초간 중립을 보낸 뒤 원래 명령을 자동 재개합니다. 이때 노드 종료나 별도 resume 요청은 필요 없습니다. 시험 노드가 없으면 MCU 정지 요청만 보냅니다. EMO는 ESC 전원을 물리적으로 끊으며 UNO에는 감지선이 없으므로, 소프트웨어가 EMO 해제 시점을 자동 인지하지는 못합니다.

## 3. 아두이노의 확정 배선과 동작

보드는 UNO R3, ESC는 Blue Robotics Basic ESC 500입니다. **수신기 CH5와 UNO D5는 다른 번호**입니다.

| 기능 | 수신기 → UNO | ESC 출력 | RC 반전 | AUTO 반전 |
|---|---|---|---|---|
| 좌현 | CH2 → D2 | D9 | true | true |
| 우현 | CH3 → D3 | D10 | false | true |
| RC/AUTO 모드 | CH5 → **D4** | 해당 없음 | 해당 없음 | 해당 없음 |

좌우 입력은 독립이며 조이스틱 혼합을 하지 않습니다. 현재 방향 플래그는 실제 정상 확인한 배선 기준이므로 한쪽 변경을 다른 경로에 복사하지 않습니다.

- D4가 1700µs 초과이고 세 입력이 모두 유효·최신이면 RC가 우선합니다. 그 외는 AUTO입니다. 스위치 위치 교체는 적용하지 않았습니다.
- RC→AUTO에서는 기존 출력·무장을 폐기하고 실제 중립 20ms를 거칩니다. 새 AUTO 무장·명령을 즉시 받을 수 있으며 이전 3초 대기는 없습니다. 계속 송신 중이면 다음 새 명령으로 재개합니다.
- AUTO→RC는 현재 RC 입력을 따릅니다. 정지 명령 S를 받은 뒤 RC 복귀에는 양쪽 스틱 중립 0.5초가 필요합니다. 영구 정지 잠금은 없습니다.
- RC PWM은 900~2100µs, 최신성 100ms로 판정합니다. 유효 PWM 소실 시 기존 명령을 해제하며 이후 새 호스트 무장으로 AUTO 운용이 가능합니다. 수신기가 무선 단절 후에도 유효 PWM을 유지하면 PWM만으로 RF 단절을 알 수 없습니다.
- 부팅·USB 브리지 재연결·유효 수신기 신규 연결 때 ESC에 **1480µs 중립을 7초** 보냅니다. 초기화 중 AUTO 무장·추진 명령은 거절하며 송신 노드는 재시도합니다. UNO 전원이 유지되면 RC 단독 조종에 ROS 브링업은 필요 없습니다.
- 출력은 Timer1 하드웨어 PWM **400Hz**, D9/OC1A·D10/OC1B입니다. AUTO 명령 유효 시간은 400ms, MCU 하드웨어 watchdog은 250ms입니다.

RC 입력은 양쪽 중심 1496µs, 중립 구간 1446~1546µs, 설정 끝점 1000~2000µs입니다. 중심 기준 공통 대칭 폭은 ±496µs로 같은 곡선을 사용합니다. 중립 밖에서 `u=clamp((|입력-1496|-50)/446,0,1)`, 논리 출력 편차는 `round(400u²)`입니다. 중립에서 0으로 시작하며 3배 최소 출력·누적 가속·끝점 점프는 없습니다. 입력 1000/1992µs에서 최대에 도달합니다. 끝점은 설정값이며 이번 문서 작업에서 다시 측정하지 않았습니다.

ESC 변환은 논리 1500을 실제 1480µs로 옮기고, 방향 플래그와 양쪽 물리 끝점 1100/1900µs를 적용합니다. 현재 AUTO +50%의 논리 1700은 실제 약1290µs, AUTO -50%의 논리 1300은 실제 약1690µs입니다. 실제 PWM 숫자가 작아도 현재 설치 기준으로 직진일 수 있습니다.

## 4. 주요 토픽과 서비스

다음은 현재 코드의 토픽 이름입니다. **2026-10-30 재접속 시에는 브링업이 정지되어 실제 센서 발행은 확인하지 않았습니다.** 이전 수신 확인 및 사용자 정상 확인과 구분합니다.

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
| `/actuators/stop_requested` | `std_msgs/msg/Bool` | 소프트웨어 상태, 물리 EMO 센서가 아님 |
| `/camera/image_raw`, `/camera/camera_info` | `Image`, `CameraInfo` | 카메라 활성 시에만 사용 |

라이다는 **VLP16 설정, 16개 ring 점군**입니다. 이전 실측 ring ID는 0~15였고 자동 선택 `/scan`은 ring 8이었습니다. `/scan`은 16개 층을 합친 데이터가 아닙니다. IP 192.168.1.201, UDP 2368, 600rpm 설정이며 장착 좌표 보정은 별도입니다.

`/actuators/status`에서 `mode_name`, `armed`, `rc_left_us`, `rc_right_us`, `rc_mode_us`, `observed_left_us`, `observed_right_us`, `receiving`, `error`를 확인합니다. `left/right`는 논리값이며 `observed_*`는 UNO 핀에서 관측한 펄스입니다. ESC 커넥터 외부 측정이나 실제 회전 피드백이 아닙니다.

UNO에 EMO 감지 입력이 없어 `estop_known=false`입니다. `/sensor/emo/status`는 알려진 상태가 없으면 메시지를 내보내지 않습니다. 기본 UNO 브링업에서는 기존 보드의 `/sensor/battery/voltage`, `/smc_2000/fix`도 사용하지 않습니다.

| 서비스 | 형식 | 의미 |
|---|---|---|
| `/actuators/arm` | `std_srvs/srv/SetBool` | AUTO 무장/해제 |
| `/actuators/throttle_left/set_pulse_width` | `mechaship_interfaces/srv/ThrottlePulseWidth` | 좌현 논리 1100~1900 |
| `/actuators/throttle_right/set_pulse_width` | 같은 형식 | 우현 논리 1100~1900 |
| `/actuators/stop` | `std_srvs/srv/Trigger` | 중립·AUTO 무장 해제 |
| `/water_trial/reinitialize_test` | `std_srvs/srv/Trigger` | 실행 중 mode_test의 중립 7초 후 자동 재개 |

직접 서비스를 사용하는 자율운항 노드는 좌우 요청을 **동시에** 보내야 합니다. 브리지는 0.1초 이내의 두 요청을 묶어 한 시리얼 명령으로 전달합니다. 순차 `ros2 service call` 두 개는 이 조건을 만족하지 못할 수 있으므로 시험에는 위 `mode_test`를 사용합니다.

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

`record`는 수동 기록기이며 추진 명령을 보내지 않습니다. GPS·IMU·scan·명령 등을 저장하지만 **3D 점군·원시 Velodyne 패킷은 저장하지 않으므로 전체 기록은 rosbag을 사용**합니다. 생성된 세션의 `metadata.json`, `raw.jsonl`을 보존합니다. GPS 도구는 `gps_track.csv`, `gps_track.geojson`, `gps_latest_url.txt`를 만들며 마지막 파일의 Google Maps 링크로 위치를 볼 수 있습니다.

```bash
ros2 run isv_water_trials analyze ~/water_trials/<실제_세션_폴더>
ros2 run isv_water_trials avoid_preview
```

분석 결과는 `report.json`, `report_KO.md`, `kinematics.csv`, `mpc_model_candidate.json`입니다. `avoid_preview`는 `/scan` 기반의 조언만 발행하며 모터를 움직이지 않습니다. 식별·MPC replay는 오프라인 도구이고, 현재 추진 정상 확인이 자율 장애물 회피·MPC 수상 성능까지 검증했다는 뜻은 아닙니다.

배 제원은 `boat_live.json`에 질량25kg, 길이1.1m, 폭0.50m, 추진기 중심 간격0.32m로 저장되어 있습니다. 추진력 환산·센서 장착 좌표·해상 시험 기반 동역학 식별은 별도입니다. 실좌표와 시험 원시 로그는 Git에 올리지 않습니다.

## 6. 기준 소스와 복원 위치

| 저장소 경로 | 젯슨 위치/역할 |
|---|---|
| `firmware/kabot_actuator/` | `/home/isv/boat_firmware/kabot_actuator/`, UNO 운용 소스 |
| `firmware/kabot_actuator/board_config.h` | 실제 빌드에 쓰는 배선·중립·방향 기준 |
| `isv_water_trials/` | `/home/isv/ros2_ws/src/isv_water_trials/` |
| `deploy/jetson/mechaship_bringup/` | 실제 설치 소스의 배포용 사본 |
| `mechaship/mechaship_bringup/` | 기존 패키지와 추가 설치 항목을 보존한 사본 |
| `scripts/ros2_shell_helpers.bash` | `~/bin/ros2_shell_helpers.bash`, `.bashrc`의 cb 정의 |
| `scripts/stop_bringup.py` | `~/bin/stop_bringup`, 수동 프로세스 정리 |

현재 `actuators.json`은 설정 참고 사본입니다. `firmware/generate_config.py`는 운용 빌드에서 자동 실행되지 않으며 `board_config.h`를 대신하지 않습니다. 이 기준을 복원할 때 기존 배선·방향·ESC 중립값을 임의로 재생성하지 않습니다.

아두이노 업로드는 젯슨에서만 수행합니다. 이번 문서화에서는 업로드하지 않았습니다.

```bash
# /home/isv/boat_firmware에 업로더와 tests까지 배치된 경우
cd ~/boat_firmware
bash jetson_upload.sh --build-only
# 브리지/시리얼 점유를 정리하고 업로드할 때
bash jetson_upload.sh --flash
```

업로더는 기본 빌드만 수행하며 `--flash`에서 기존 flash 이중 읽기·EEPROM 백업 후 새 이미지 업로드·검증을 수행합니다. UNO가 재시작되므로 운용 중 단순 문서 수정에 사용할 명령은 아닙니다.

기준 태그는 `boat-stable-20261030`으로 보존합니다. 복원은 해당 태그를 별도 디렉터리에 체크아웃해 소스와 비교한 뒤 필요한 패키지를 배포합니다. 작업 중 파일을 강제로 덮어쓰는 reset은 사용하지 않습니다.

## 7. 확인 범위

- 사용자 확인: 현 상태의 RC/AUTO 좌우 전후진, 모드 전환, EMO 해제 후 기동, 시험 코드·초기화 정상.
- 재접속 감사: 펌웨어·ROS·설정·브링업 파일36개 중35개 SHA-256 일치. 나머지 배포용 CMake 파일은 주석 한 줄만 다르고 실행·설치 구문은 같습니다. 비교 결과는 `BASELINE_RUNTIME_SHA256.json`에 보존합니다. 실제 flash 새 읽기는 하지 않았습니다.
- 문서 작업 전 로컬 unittest: 총52개 중32개 통과, ROS/POSIX 환경 필요20개 제외. 수치 모의 검증은 실제 수상 주행이 아닙니다.
- 직전 젯슨 ROS 가짜 서비스 검사6개 통과: SIGHUP/SIGTERM 유지, 외부 stop 무시, 초기화 후 자동 재개, 일회 중립 요청 후 재개, Ctrl+C 중립. 실제 모터 서비스와 분리한 검사였습니다.
- 이번 문서화에서 새 추진·수상 시험이나 실행 중 노드 변경은 하지 않았습니다. 과거 문서의 미검증/실패 기록은 당시 상태이며 이 문서가 현재 운용 기준입니다.
