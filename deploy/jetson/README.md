# 젯슨 실행 명령

센서 실행(현재 카메라는 기본 OFF):

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch mechaship_bringup all_sensors.launch.py
```

GPS·VectorNav IMU·Velodyne·MCU 통신을 실행합니다. 이미 실행 중이면 중복 실행하지 않습니다. 카메라 연결 후에는 끝에 `use_camera:=true`를 붙입니다. 추진기 활성화·자율운항은 실행하지 않습니다. 빌드가 필요할 때만 `cb`를 실행하면 전체 빌드와 현재 터미널 환경 적용까지 합니다.

다른 터미널에서도 먼저 `source ~/ros2_ws/install/setup.bash`를 실행합니다.

```bash
# 수신 여부·주기·GPS fix 확인
ros2 topic echo /sensor_status

# 센서 원본 전체 기록: Ctrl+C로 종료, 16레이어 라이다 포함
mkdir -p ~/water_trials
ros2 bag record -o ~/water_trials/bag_$(date +%Y%m%d_%H%M%S) --regex '^/(gps/.*|vectornav/.*|velodyne_packets|velodyne_points|scan|sensor/.*|sensor_status|water_trial/.*|camera/.*)$'

# 물리량 분석용 기록: 별도 터미널, Ctrl+C로 종료
ros2 run isv_water_trials record --config ~/ros2_ws/src/isv_water_trials/config/boat_live.json

# 구간 표시: 기록기를 켠 상태에서 별도 터미널로 실행
ros2 run isv_water_trials mark idle
ros2 run isv_water_trials mark straight
ros2 run isv_water_trials mark coast
ros2 run isv_water_trials mark end

# 기록 종료 후 SESSION을 출력된 실제 폴더 경로로 바꾸어 분석
ros2 run isv_water_trials analyze SESSION

# 시험 시퀀스 미리보기: 추진기는 움직이지 않음
ros2 run isv_water_trials trial surge --repeats 3
```

`mark`는 구간 이름만 남깁니다. 분석용 기록기는 GPS·IMU·단일 레이어 `/scan` 등을 저장합니다. 16레이어 원본은 위 rosbag으로 함께 저장합니다. 추진 명령 기록·축 방향 확인·수상 시험 데이터가 없으면 추력·관성·저항계수 추정은 확정할 수 없습니다.

| 데이터 | 토픽 |
|---|---|
| GPS 위치 / 속도 | `/gps/fix` / `/gps/fix_velocity` |
| IMU 원본 / 표준 IMU | `/vectornav/raw/common` / `/vectornav/imu` |
| 라이다 전체 16레이어 | `/velodyne_points` |
| 라이다 원본 패킷 | `/velodyne_packets` |
| 2D 스캔 한 레이어 | `/scan` |
| 배터리 / 비상정지 | `/sensor/battery/voltage` / `/sensor/emo/status` |
| 실제 수신 상태 | `/sensor_status` |

2026-10-29 실제 수신: GPS 5Hz, IMU 20Hz, 라이다 약 10Hz. 배터리·비상정지 텔레메트리는 아직 수신되지 않습니다. 아두이노 RC/자동 모드 전환과 자율운항 명령 통신은 별도이며, 현재 아두이노 펌웨어와 통신 규약은 확인되지 않았습니다. VN-100의 `/vectornav/gnss`, 위치·선속 토픽은 실제 GPS 측정으로 사용하지 마세요. 위치는 `/gps/fix`를 사용합니다.

라이다는 실제 패킷의 VLP16 제품 ID(0x22)와 포인트클라우드 ring 0~15 수신을 확인했습니다. `/scan`은 ring 8(0부터 세면 8번, 9번째 레이어) 하나입니다. 다른 레이어는 `velodyne_ring:=번호`로 지정합니다. 설치 각도는 별도 확인해야 합니다.

라이다 LAN은 USB 이더넷 `enx80691ae85b26`, 젯슨 `192.168.1.100/24`, 센서 `192.168.1.201:2368`입니다. `isv-velodyne-usb` 프로필로 저장했습니다. GPS와 IMU는 `/dev/serial/by-id/` 경로로 USB 연결 순서를 구분합니다.

젯슨 적용 파일은 이 폴더의 `mechaship_bringup/`입니다. 저장소 루트의 기존 bringup은 WTRTK 구성이므로 젯슨 원본과 동일하지 않습니다. 기존 코드와 오늘 수정 전 백업은 보존했습니다.
