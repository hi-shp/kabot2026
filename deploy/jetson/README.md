# 젯슨 수상 시험 실행

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch mechaship_bringup all_sensors.launch.py use_arduino:=true
```

GPS, VectorNav IMU, Velodyne VLP16, 아두이노 상태 수신을 한 번에 시작합니다. 카메라는 연결할 때만 `use_camera:=true`를 추가합니다. 시작 시 자율운항은 무장되지 않으며, 조종기가 RC 모드이면 RC 조작을 유지합니다. `use_arduino:=false`로 아두이노 연결만 끌 수 있습니다. 기존 `cb`는 전체 ROS 작업 공간을 빌드하고 현재 셸에 적용합니다.

| 확인 항목 | 토픽 |
|---|---|
| GPS 위치·속도 | `/gps/fix`, `/gps/fix_velocity` |
| VectorNav IMU | `/vectornav/raw/common`, `/vectornav/imu` |
| 라이다 16층·기본 단면 | `/velodyne_points`, `/scan` |
| 아두이노 모드·명령·RC 입력·D9/D10 출력 펄스 | `/actuators/status` |
| 자율운항 정지 요청 | `/actuators/stop_requested` |

`/scan`은 VLP16의 ring 8(0부터 세면 9번째)입니다. `/actuators/stop_requested`는 RC 모드·미무장·통신 단절 등을 포함하는 소프트웨어 정지 요청이며, 별도 배선된 물리 비상정지가 아닙니다. 배터리 전압과 물리 비상정지 신호는 아직 확인된 센서 토픽이 없습니다. VN-100의 GNSS·위치·속도 출력은 이 배의 실제 GNSS 측정값으로 사용하지 않습니다.

```bash
# 현재 토픽과 아두이노 상태 확인
ros2 topic list
ros2 topic echo /actuators/status

# 다른 추진 명령 프로그램을 종료한 뒤, 별도 터미널에서 RC/AUTO 전환 시험
ros2 run isv_water_trials mode_test --seconds 60
# AUTO + 양쪽 스틱 중립 0.5초 확인 후 좌우 1540µs(최대 PWM 편차의 10%).
# RC 전환 시 수동 우선, AUTO 복귀 시 중립 확인 후 재개. Ctrl+C/60초 후 AUTO 해제.
# 한쪽만 시험: --side left 또는 --side right. 통신·신호 이상은 재개 없이 종료.

# 센서와 명령 기록. Ctrl+C로 종료
mkdir -p ~/water_trials
ros2 bag record -o ~/water_trials/bag_$(date +%Y%m%d_%H%M%S) --regex '^/(gps/.*|vectornav/.*|velodyne_packets|velodyne_points|scan|sensor/.*|sensor_status|water_trial/.*|actuators/.*)$'

# 물리량 추정용 기록 및 분석
ros2 run isv_water_trials record --config ~/ros2_ws/src/isv_water_trials/config/boat_live.json
ros2 run isv_water_trials analyze SESSION
```

무게 25 kg, 길이 1.1 m, 폭 0.50 m, 추진기 중심 간격 0.32 m는 `boat_live.json`에 저장했습니다. 추력 보정값·관성모멘트·IMU 설치 방향·라이다 설치 각도는 아직 측정하지 않았습니다. `trial` 실행은 프레임 확인과 별도 운전자 확인을 요구합니다. 모터 전원이 분리된 상태에서 아두이노 RC 입력과 D9/D10 중립 PWM까지만 확인했으며, 빵판·ESC·스러스터의 실제 회전은 확인하지 못했습니다.

아두이노 배선·원본 백업·RC/AUTO 전환 절차는 [펌웨어 안내](../../firmware/README.md)를 참고하세요.
