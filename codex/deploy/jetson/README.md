# 젯슨 실행 명령

새 Bash/SSH 터미널은 ROS 환경과 통신 설정(`ROS_DOMAIN_ID=50`, `ROS_LOCALHOST_ONLY=0`)이 자동 적용됩니다. 파일 수정 후에는 `cb`로 빌드·적용합니다. 기존 터미널의 환경을 새로 불러올 때는 `source ~/.bashrc`를 사용합니다.

배선과 설정은 [정상 운용 기준](../../docs/BOAT_BASELINE_2026-10-30.md)을 참고합니다. 사용자 표현 1000/1500/2000은 ROS 명령 1100/1500/1900으로 변환하며, 실제 ESC 정지 출력은 양쪽 1480µs입니다.

## 센서와 아두이노 브리지

터미널 1에서 실행합니다.

```bash
ros2 launch mechaship_bringup all_sensors.launch.py
```

GPS·IMU·Velodyne·아두이노를 함께 켭니다. 카메라 연결 시 `use_camera:=true`를 추가합니다. Ctrl+C로 해당 브링업과 하위 프로세스를 종료합니다. 남은 세션은 `~/bin/stop_bringup`으로 정리하며, 이 명령은 독립 실행 중인 시험 프로세스도 종료합니다.

## 추진기 시험

터미널 2에서 다음 중 하나를 실행합니다.

```bash
ros2 run isv_water_trials mode_test --side left --percent 50
ros2 run isv_water_trials mode_test --side right --percent 50
ros2 run isv_water_trials mode_test --side both --percent 50
```

`left`는 좌현, `right`는 우현, `both`는 양쪽입니다. 후진은 `--percent -50`, 30% 출력은 `--percent 30`으로 지정합니다.

시험은 Ctrl+C까지 계속 실행되며 `--seconds`는 무시합니다. RC/AUTO 선택은 아두이노가 담당합니다. 서비스·USB 단절 시 젯슨은 재시도하고, SSH 단절(SIGHUP)과 SIGTERM에도 실행을 유지합니다. 전원 차단이나 SIGKILL로 종료된 경우에는 다시 실행해야 합니다.

Ctrl+C를 누르면 중립 응답을 확인한 뒤 종료합니다. 응답 확인에 실패하면 `STOP_UNCONFIRMED`가 표시됩니다.

```bash
# 일회 중립 요청
ros2 run isv_water_trials stop_thrusters
# ESC 재초기화: 중립 7초 후 기존 시험 출력 자동 재개
ros2 run isv_water_trials reinitialize_esc
```

`stop_thrusters` 요청 뒤에도 실행 중인 시험은 새 출력을 보냅니다. 시험을 계속 멈춰 두려면 해당 터미널에서 Ctrl+C를 누릅니다. EMO 해제 후 재초기화가 필요하면 `reinitialize_esc`를 실행합니다. UNO의 EMO 감지선은 미연결 상태이므로 해제 시점은 사용자가 판단합니다.

## 주요 토픽

| 데이터 | 토픽 |
|---|---|
| GPS 위치·속도 | `/gps/fix`, `/gps/fix_velocity` |
| IMU | `/vectornav/raw/common`, `/vectornav/imu` |
| 라이다 패킷·3D 점군·2D 스캔 | `/velodyne_packets`, `/velodyne_points`, `/scan` |
| 모드·RC 입력·PWM 관측값 | `/actuators/status` |
| 추진 명령 기록 | `/water_trial/command` |

라이다는 VLP16 설정이며 점군에 16개 ring이 들어옵니다. `/scan`은 그중 한 층을 사용합니다. 카메라는 선택 사용하며, 기존 MCU의 배터리·EMO·SMC GPS 데이터는 별도 보드 연결이 필요합니다.

## 센서 기록과 GPS 지도

각 명령은 별도 터미널에서 실행합니다.

```bash
mkdir -p ~/water_trials
ros2 bag record -o ~/water_trials/bag_$(date +%Y%m%d_%H%M%S) --regex '^/(gps/.*|vectornav/.*|velodyne_packets|velodyne_points|scan|actuators/.*|water_trial/.*|camera/.*|tf|tf_static|diagnostics)$'
ros2 run isv_water_trials gps_track --output ~/water_trials/gps_$(date +%Y%m%d_%H%M%S)
```

`gps_latest_url.txt`의 링크로 현재 위치를 지도에서 확인합니다. 경로는 `gps_track.csv`와 `gps_track.geojson`에 저장됩니다. JSON 기록·구간 표시·분석 방법은 정상 운용 기준에 있습니다.
