# 젯슨 실행 명령

아두이노 기본 배선·방향·중립값은 [펌웨어 안내](../../firmware/README.md)를 기준으로 합니다. **수신기 CH5 → UNO D4**가 모드 입력이며 UNO D5는 사용하지 않습니다.

각 터미널에서 먼저 실행합니다.

```bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=50 ROS_LOCALHOST_ONLY=0
```

터미널 1 — GPS·VectorNav IMU·Velodyne·아두이노 상태 수신:

```bash
ros2 launch mechaship_bringup all_sensors.launch.py
```

카메라를 연결했다면 `use_camera:=true`를 추가합니다. 재실행하면 기존 같은 브링업을 종료하고 새 터미널에서 시작합니다. **이 터미널에서 Ctrl+C를 누르면 해당 브링업의 센서·브리지·하위 프로세스를 종료**합니다. 남은 프로세스는 SIGTERM·SIGKILL로 정리합니다. 수동 정리 명령은 `~/bin/stop_bringup`입니다. `cb`는 전체 작업 공간을 빌드하고 현재 셸에 적용합니다.

터미널 2 — PWM 기준 30% 전진, 세 가지 중 하나 선택(Ctrl+C까지 계속, 정지 명령은 출력 일시정지):

```bash
# 좌현만 전진, 우현 중립
ros2 run isv_water_trials mode_test --side left --percent 30
# 우현만 전진, 좌현 중립
ros2 run isv_water_trials mode_test --side right --percent 30
# 양쪽 전진
ros2 run isv_water_trials mode_test --side both --percent 30
```

젯슨 시험 노드는 모드·수신기 상태와 관계없이 명령을 계속 보냅니다. 아두이노가 RC 선택 시 젯슨 명령을 무시합니다. 시간 제한 없이 연결 단절 때도 재시도하며 서비스 대기 문구는 출력하지 않습니다. `mode_test`는 SSH 끊김(SIGHUP)·SIGTERM을 무시하고 Ctrl+C(SIGINT)에서 중립 응답을 확인한 뒤 종료합니다. 외부 stop 토픽·일시정지 서비스를 사용하지 않습니다.

다른 터미널 — MCU 중립 요청(계속 송신 중인 시험의 정지는 Ctrl+C):

```bash
ros2 run isv_water_trials stop_thrusters
```

계속 실행 중인 `mode_test`는 이 중립 요청 뒤에도 새 명령을 송신하므로, 시험 종료는 해당 터미널의 Ctrl+C를 사용합니다. USB·통신 단절로 응답이 없으면 정지 확인 실패를 표시합니다. AUTO 통신 감시 시간은 400ms입니다.

EMO 해제 후 `ros2 run isv_water_trials reinitialize_esc`를 실행하면 기존 시험 노드를 종료하지 않고 7초간 중립을 유지한 뒤 출력을 재개합니다. EMO 감지 입력이 없어 해제 시점은 자동 감지하지 못합니다.

| 데이터 | 토픽 |
|---|---|
| GPS 위치·속도 | `/gps/fix`, `/gps/fix_velocity` |
| IMU | `/vectornav/raw/common`, `/vectornav/imu` |
| 라이다 | `/velodyne_packets`, `/velodyne_points`, `/scan` |
| 아두이노 모드·RC 입력·출력 상태 | `/actuators/status` |

```bash
mkdir -p ~/water_trials
ros2 bag record -o ~/water_trials/bag_$(date +%Y%m%d_%H%M%S) --regex '^/(gps/.*|vectornav/.*|velodyne_packets|velodyne_points|scan|sensor/.*|water_trial/.*|actuators/.*)$'
```

무게 25kg·길이 1.1m·폭 0.50m·추진기 중심 간격 0.32m는 `boat_live.json`에 저장되어 있습니다. 이번 변경에서 실제 30% 추진 시험은 실행하지 않습니다.
