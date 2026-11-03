# 기존 젯슨 출력으로 도형 따라가기

기존 `~/ros2_ws` 파일은 수정하지 않고 별도 오버레이에 설치하는 모드입니다. 기본 목표는 **red triangle / 클래스 8**입니다. 기존 SLAM 구역 진입 모드는 `shape_navigation.launch.py`로 유지하며, 이번 첫 추종 모드는 `shape_follow.launch.py`로 별도 실행합니다. 이번 추종 모드에는 ㄷ자 진입·GPS 웨이포인트·후진 시퀀스를 넣지 않았습니다.

## 사용하는 기존 데이터와 출력

- `/fusion/roi_center`: 실제 젯슨과 같은 `[u,v,class_id,track_id,x,y,z,R]` 8열. x 전방, y 왼쪽, z 위쪽인 카메라 기준 좌표입니다.
- `/yolo/tracking`: 목표 이름·클래스 번호·신뢰도·박스 위치를 함께 확인합니다. 퓨전 메시지 자체에는 신뢰도나 촬영 시각이 없어, 최근 YOLO 프레임과 대응되는 행만 사용합니다. 기존 퓨전이 `/yolo/detections`를 쓰는 경우 새 패키지 YAML의 `yolo_topic`도 같은 값으로 바꿉니다.
- `/scan`: 시간과 프레임을 확인하고 선체 주변/전방 장애물에서 정지합니다.
- 조향은 설치된 `isv_water_trials.tank_core.guidance()`를 **직접 호출**합니다.
- 출력은 설치된 `isv_water_trials.actuator_config.command_pwm()`를 **직접 호출**하고, 기존 `/actuators/arm`, `/actuators/stop`, 좌우 `/actuators/throttle_*/set_pulse_width` 서비스를 사용합니다. 좌우는 100ms 안에 짝이 되도록 동시에 요청합니다.
- 서비스 중립은 1500µs입니다. 실제 ESC 중립 1480µs와 반전은 현재 펌웨어가 처리합니다. 이 코드에서 중복 보정하지 않습니다. 첨부 코드의 `/motor_left_pwm`, `/motor_right_pwm` 토픽과 PWM 표는 사용하지 않습니다.

기존 YOLO·퓨전·센서 실행기를 그대로 사용합니다. 이 launch는 기존 실행기를 시작·중지하거나 기존 설정을 바꾸지 않습니다. 다른 자율운항 제어기와 동시에 쓰러스터를 제어하지 않습니다.

## 실행

별도 설치 위치는 `/home/isv/shape_docking_red_triangle`입니다. 각 터미널에서 다음 환경을 적용합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/shape_docking_red_triangle/install/setup.bash
export ROS_DOMAIN_ID=50
```

처음에는 출력 없이 확인합니다.

```bash
ros2 launch isv_shape_navigation shape_follow.launch.py
ros2 topic echo /shape_follow/status
```

`WAIT_SCAN`, `WAIT_TARGET`이면 기존 센서/YOLO/퓨전 데이터가 아직 없거나 설정한 토픽·프레임과 맞지 않는 상태입니다. `VERIFY_TARGET`은 서로 다른 검출 프레임 4개를 기다리는 상태입니다. 하나의 목표가 안정적으로 확인되면 `FOLLOW_TARGET`, 가까워지면 `HOLD_DISTANCE`가 됩니다. 기본 정지 거리는 카메라/라이다 기준 수평 거리 1.5m이고, 1.7m를 넘어서면 다시 따라갑니다. `OBSTACLE_STOP`, 유실, 시간 불일치, 중복 표적에서는 중립 요청을 계산합니다.

센서 좌표와 경로를 확인한 뒤 실제 출력을 사용할 때만 기존 미리보기를 종료하고 실행합니다.

```bash
ros2 launch isv_shape_navigation shape_follow.launch.py enable_output:=true
# RC AUTO, 최신 MCU 상태, 안정적인 목표/스캔이 확인된 다음:
ros2 service call /shape_follow/start std_srvs/srv/Trigger '{}'
# 정지:
ros2 service call /shape_follow/stop std_srvs/srv/Trigger '{}'
```

유실·통신 오류·RC 전환 후에는 자동 재무장하지 않습니다. 복구 후 시작 서비스를 다시 호출합니다. 정지 응답은 요청 접수이며 실제 중립 확인은 상태의 `output.stop_ack`로 확인합니다. 실제 출력은 시뮬레이션 시간에서 차단됩니다.

## 다른 도형으로 바꾸기

**Python 코드를 고칠 필요가 없습니다.** 실행 중인 추종 노드를 정지한 뒤 `target_label`만 바꿔 다시 실행합니다.

```bash
ros2 launch isv_shape_navigation shape_follow.launch.py target_label:="blue circle"
ros2 launch isv_shape_navigation shape_follow.launch.py target_label:="green cross"
ros2 launch isv_shape_navigation shape_follow.launch.py target_label:="red triangle"
```

현재 젯슨 `best3.pt`의 이름/번호는 다음과 같습니다. 번호는 이름으로 자동 선택하므로 따로 고치지 않습니다.

| 번호 | 이름 | 의미 |
|---|---|---|
| 0 | blue circle | 파란 원 |
| 1 | blue cross | 파란 십자 |
| 2 | blue triangle | 파란 삼각형 |
| 3 | green circle | 초록 원 |
| 4 | green cross | 초록 십자 |
| 5 | green triangle | 초록 삼각형 |
| 6 | red circle | 빨간 원 |
| 7 | red cross | 빨간 십자 |
| 8 | red triangle | 빨간 삼각형 |

자주 사용하는 기본값을 바꾸려면 **새 패키지의** `config/shape_follow.yaml`에서 `target_label`을 수정합니다. 파라미터는 실행 중 변경하지 않고 노드를 다시 실행합니다. 모델에 없는 사각형·새 표식은 이름만 바꿔서 인식할 수 없습니다. 해당 클래스로 학습한 새 모델을 사용하고 `class_names`를 새 모델의 번호 순서와 일치시켜야 합니다. 받은 다른 ZIP의 10클래스 데이터는 부표 때문에 번호가 달라 현재 9클래스 번호를 그대로 쓰면 안 됩니다.

설정의 카메라/라이다 위치·방향과 선체 반경은 실제 설치에 맞춰야 합니다. 이 모드는 기존 퓨전 좌표를 사용하며 SLAM 지도로 장애물을 우회하거나 도킹 입구를 추론하지 않습니다. 현장 검증 전에는 실제 추종 성능이나 제동거리가 확인된 상태가 아닙니다.
