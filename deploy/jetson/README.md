# 젯슨 필수 명령

현재 배선·방향·실제 중립값·토픽·기록 명령의 기준은 [정상 운용 안내](../../docs/BOAT_BASELINE_2026-10-30.md)입니다. 사용자가 말하는1000/1500/2000과 실제 ROS API1100/1500/1900을 구분합니다. 실제 ESC 정지 출력은 양쪽1480µs입니다.

각 ROS 터미널에서:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=50 ROS_LOCALHOST_ONLY=0
```

전체 빌드와 현재 셸 적용은 `cb`입니다. 터미널1에서 기본 GPS·IMU·Velodyne·아두이노를 켭니다.

```bash
ros2 launch mechaship_bringup all_sensors.launch.py
```

카메라 연결 시 `use_camera:=true`를 추가합니다. Ctrl+C는 해당 브링업의 센서·브리지 하위 프로세스까지 정리합니다. 남은 세션의 수동 정리는 `~/bin/stop_bringup`이며 독립 시험 프로세스도 정리합니다.

터미널2에서 **아래 중 하나만** 실행합니다.

```bash
ros2 run isv_water_trials mode_test --side left --percent 50
ros2 run isv_water_trials mode_test --side right --percent 50
ros2 run isv_water_trials mode_test --side both --percent 50
```

후진은 `--percent -50`, 30%는 `--percent 30`입니다. 시험은 Ctrl+C까지 계속 송신하며 `--seconds`는 무시합니다. RC/AUTO 선택은 MCU가 처리하고, 서비스/USB 단절 시 호스트는 재시도합니다. SSH SIGHUP·SIGTERM은 무시하며 Ctrl+C에서는 중립 ACK 후 종료합니다. 전원 차단과 SIGKILL까지 생존하는 것은 아닙니다.

```bash
# 일회 중립 요청: 계속 실행 중인 시험은 새 명령을 보내므로 지속 정지는 시험의 Ctrl+C 사용
ros2 run isv_water_trials stop_thrusters
# EMO 해제 후 필요 시: 실행 중 시험을 유지하고 중립7초 후 원래 출력을 자동 재개
ros2 run isv_water_trials reinitialize_esc
```

UNO에는 물리 EMO 감지 입력이 없습니다. 외부 stop 요청은 현재 mode_test를 일시정지시키지 않습니다. 초기화 요청은 유지보수용 명시 요청이며 노드를 종료하지 않습니다.

| 데이터 | 주요 토픽 |
|---|---|
| GPS 위치·속도 | `/gps/fix`, `/gps/fix_velocity` |
| IMU | `/vectornav/raw/common`, `/vectornav/imu` |
| 라이다 원시·3D·선택 ring | `/velodyne_packets`, `/velodyne_points`, `/scan` |
| MCU 상태·원시 RC·핀 PWM | `/actuators/status` |
| 명령 기록 | `/water_trial/command` |

라이다는 VLP16 설정의16개 ring 점군이며 `/scan`은 선택한 한 층입니다. 카메라는 기본 비활성이고 기존 MCU 배터리·EMO·SMC GPS 토픽은 기본 UNO 구성에서 수신하지 않습니다.

전체 센서 기록과 지도용 GPS 파일:

```bash
mkdir -p ~/water_trials
ros2 bag record -o ~/water_trials/bag_$(date +%Y%m%d_%H%M%S) --regex '^/(gps/.*|vectornav/.*|velodyne_packets|velodyne_points|scan|actuators/.*|water_trial/.*|camera/.*|tf|tf_static|diagnostics)$'
ros2 run isv_water_trials gps_track --output ~/water_trials/gps_$(date +%Y%m%d_%H%M%S)
```

각 기록은 별도 터미널에서 실행합니다. `gps_latest_url.txt`의 지도 링크, `gps_track.csv`, `gps_track.geojson`을 사용합니다. 추가 JSON 기록·구간 표시·오프라인 분석은 정상 운용 안내에 정리했습니다.
