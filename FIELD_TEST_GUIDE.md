# KABOAT 2026 실선 시험 가이드

작성 기준: 2026-10-09 Asia/Seoul. 첫 시험 목표는 PASSIVE 수동 기록이다.
GPS, 카메라, 기존 대회 알고리즘 실행이 필수는 아니다. 아래 명령의 사용자,
호스트, 설치 경로는 현장 값으로 지정한다. ROS2 배포판은 임의로 가정하지 않는다.

## 최초 설치 또는 업데이트

현재 IDE의 /home/soonhong/kaboat는 별도 시뮬레이션 저장소다. 이번 코드의
저장소는 hi-shp/kabot2026, 브랜치는 이미 존재하는 field-calibration이다.

처음 설치할 때만:

```bash
git clone -b field-calibration --single-branch \
  https://github.com/hi-shp/kabot2026.git ~/kabot2026
```

기존 ~/kabot2026 checkout이 있을 때:

```bash
cd ~/kabot2026
git remote -v
git status --short --branch
git fetch origin field-calibration
git branch --list field-calibration
```

변경사항이 있다면 경로를 확인하고 기존 작업을 보존한다. 강제 checkout,
reset, clean을 사용하지 않는다. 로컬 branch가 있으면:

```bash
git switch field-calibration
git merge --ff-only origin/field-calibration
```

로컬 branch가 없고 원격 branch만 있으면 원격 branch를 추적하는 checkout:

```bash
git switch --track origin/field-calibration
```

전체 repository를 기존 ros2_ws/src 아래에 넣지 않는다. 기존 ROS 및 보트
workspace를 source한 상태에서 별도 overlay에 새 패키지 하나만 연결한다:

```bash
export FIELD_WS="$HOME/field_calibration_ws"
mkdir -p "$FIELD_WS/src"
test ! -e "$FIELD_WS/src/field_calibration" && \
  ln -s "$HOME/kabot2026/field_calibration" "$FIELD_WS/src/field_calibration"
cd "$FIELD_WS"
colcon list
rosdep install --from-paths src/field_calibration --ignore-src -r -y
colcon build --symlink-install --packages-select field_calibration
source "$FIELD_WS/install/setup.bash"
ros2 pkg executables field_calibration
```

이미 연결 경로가 있으면 readlink -f로 대상이 올바른지 확인한다. colcon list에
기존 mechaship/isv 패키지가 중복 검색되면 먼저 src 배치를 수정한다.
모든 새 터미널에서 같은 ROS underlay → 기존 보트 workspace → field overlay
순서로 source한다. 노트북에도 새 패키지만 같은 방법으로 설치한다.

## 1. 선박 전원 연결

추진기 출력을 안전하게 차단한 상태에서 전원·배터리·센서 케이블을 확인한다.
실측 중립각, 실제 출력 범위, 물리 EMO 및 RC 우선권은 현장 책임자가 확인한다.
이 패키지는 actuator enable/disable을 호출하지 않는다. 기존 bringup은 MCU가
발견되면 actuator를 자동 활성화하므로 센서 전용 실행과 구분한다.
MCU watchdog·통신 단절 정지를 입증하지 못했으므로 자동 시험 구동은 제공하지 않는다.

## 2. SSH 접속

```bash
export BOAT_HOST=실제사용자@실제선박주소
ssh "$BOAT_HOST"
```

SSH 단절 후에도 기록을 유지하려면 선박에서 tmux를 사용한다:

```bash
tmux new -s field
# 재접속 후: tmux attach -t field
# 분리: Ctrl+B, D
```

기록 패키지는 모터 명령을 발행하지 않는다. 기존 teleop의 SSH 단절 정지는
이 패키지가 보장하지 않으므로 물리 RC/EMO로 제어한다.

## 3. ROS2 환경 확인

```bash
ls /opt/ros
# 실제 배포판의 setup.bash를 선택해 source한다.
export ROS_SETUP=/실제/ros/설치/setup.bash
source "$ROS_SETUP"
export BOAT_SETUP=/실제/기존/ros2_ws/install/setup.bash
source "$BOAT_SETUP"
source ~/field_calibration_ws/install/setup.bash
printenv ROS_DISTRO ROS_DOMAIN_ID RMW_IMPLEMENTATION
python3 --version
python3 -c 'import rclpy; print(rclpy.__file__)'
ros2 doctor --report
ros2 node list
```

rclpy import가 실패하면 Python 버전과 ROS C extension의 버전을 먼저 맞춘다.
현장 ROS의 Python으로 빌드해야 한다. 노트북과 선박의 ROS_DOMAIN_ID/RMW,
방화벽 및 DDS 통신 환경을 맞춘다. 개발 테스트용 domain 191을 선박에 적용하지 않는다.

## 4. 센서 드라이버 실행

이미 실행 중이면 중복 실행하지 않는다. 기존 현장 서비스/launch가 있다면
그 경로를 우선 사용한다. 설치된 경로를 찾는다:

```bash
ros2 pkg prefix --share mechaship_bringup
ros2 pkg executables ydlidar_ros2_driver
ros2 pkg executables iahrs_ros2_driver
export BRINGUP_SHARE="$(ros2 pkg prefix --share mechaship_bringup)"
```

필요할 때만 각 선박 터미널에서 센서 전용 실행:

```bash
ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node --ros-args \
  -r __node:=ydlidar_ros2_driver_node \
  --params-file "$BRINGUP_SHARE/param/mechaship_lidar.yaml"
```

```bash
ros2 run iahrs_ros2_driver iahrs_ros2_driver_node --ros-args \
  -r __node:=iahrs_ros2_driver \
  --params-file "$BRINGUP_SHARE/param/mechaship_imu.yaml" \
  -r imu/data:=/imu -r imu/mag:=/mag
```

드라이버 버전이 lifecycle을 사용하면 ros2 lifecycle get으로 확인하고 그
설치 버전의 configure/activate 절차를 따른다. 이 repository의 YDLidar
실행 파일은 일반 Node 구현이지만 launch에는 LifecycleNode로 기재되어 있다.

RF2O가 없을 때만 설치된 기존 rf2o_laser_odometry를 독립 실행한다:

```bash
ros2 launch field_calibration gps_free_odometry.launch.py
```

출력은 /field/rf2o_odom, base_footprint, odom이며 TF 발행은 없다.
base_scan/imu_link의 실제 TF가 필요하다. 기존 state publisher의 SDF 전달이
설치 버전에서 동작하는지 확인한다. 임의 identity TF로 센서 위치를 대신하지 않는다.
이미 RF2O가 정상이라면 그 토픽(/scan/odom 또는 /odom_rf2o)을 그대로 사용한다.

## 5. LiDAR / IMU / RF2O 토픽 확인

```bash
ros2 topic list -t
ros2 topic info /scan --verbose
ros2 topic info /imu --verbose
ros2 topic hz /scan
ros2 topic hz /imu
ros2 topic echo /scan --once --qos-reliability best_effort
ros2 topic echo /imu --once --qos-reliability best_effort
ros2 topic info /scan/odom --verbose
ros2 topic info /odom_rf2o --verbose
ros2 topic info /field/rf2o_odom --verbose
# 존재하는 경로로 바꿔 실행:
ros2 topic echo /field/rf2o_odom --once
ros2 run tf2_ros tf2_echo base_footprint base_scan
ros2 run tf2_ros tf2_echo base_footprint imu_link
ros2 topic info /actuator/thruster/percentage --verbose
ros2 topic info /actuator/key/degree --verbose
ros2 topic echo /sensor/emo/status --once
```

센서를 손으로 천천히 좌회전시켜 raw /imu.angular_velocity.z와 LiDAR heading이
함께 CCW 양수인지 확인한다. /imu/yaw_refined는 부호 반전된 도 단위이므로
새 모니터 입력으로 사용하지 않는다. 자세 초기 yaw는 로그 진단에서만 제거한다.
위치·heading은 odom 기준을 유지한다. IMU 장착은 수평·z-up 확인이 필요하며
기울어진 장착은 단순 yaw 부호/오프셋 설정만으로 보정할 수 없다.

## 6. PASSIVE field test 실행

현장 설정을 저장소 밖에 복사한다:

```bash
mkdir -p ~/field_config
cp "$(ros2 pkg prefix --share field_calibration)/config/field_test.yaml" ~/field_config/boat.yaml
```

boat.yaml에서 실제 scan/imu/odom/command 토픽과 frame을 맞춘다.
독립 gps_free launch를 썼으면 odom_topic: /field/rf2o_odom,
base_frame: base_footprint로 설정한다. 기존 독립 RF2O라면
odom_topic: /odom_rf2o, base_frame: base_link다. logger는 설정한 토픽을
자동으로 기록 목록에 추가하고 실제 발견한 메시지 타입만 저장한다.

```bash
ros2 launch field_calibration field_test.launch.py \
  config:="$HOME/field_config/boat.yaml" profile:=indoor test_mode:=manual_log
```

기본 PASSIVE이며 모터 publisher는 없다. RF2O 공분산이 0이면 기본 INVALID다.
기록은 계속된다. 정지/알려진 이동으로 검증 후 실내에서만
allow_unknown_covariance: true를 선택할 수 있다. 그러면 quality는
LIMITED_UNKNOWN_COVARIANCE다. outdoor는 미지 공분산을 허용하지 않는다.
고정 특징이 없는 수면에서 정상 궤적처럼 표시하기 위해 이 조건을 해제하지 않는다.

## 7. 데이터 기록 시작

launch가 시작되면 자동 기록이 시작된다. /field/recording_status를 확인한다:

```bash
ros2 topic echo /field/recording_status --once --qos-reliability best_effort
ros2 topic echo /field/state --once --qos-reliability best_effort
df -h ~/field_data
```

매 실행마다 ~/field_data/UTC_ID/에 bag과 CSV가 생성된다. missing_topics는
현재 메시지를 받지 못한 목록이다. 실제로 RC 조작을 하면서 command count가
증가하는지 확인한다. RC 명령이 ROS에 반영되지 않는 하드웨어이면 명령은
결측으로 남고 모델 식별이 불가하다. 0 또는 중립으로 임의 채우지 않는다.

## 8. 실내 수조 시험

수조 벽 등 고정 특징을 보며 stationary를 30–60초 먼저 기록한다.
단일 실험 중 라벨 변경은 다음과 같이 한다:

```bash
ros2 param set /calibration_node test_mode stationary
```

시험별 독립 폴더가 필요하면 Ctrl+C로 기록을 마무리한 후 다시 launch한다:

```bash
ros2 launch field_calibration field_test.launch.py \
  config:="$HOME/field_config/boat.yaml" profile:=indoor test_mode:=stationary
```

stationary → straight → coast → port_turn → starboard_turn → zigzag 순서로
수동 RC 조작을 기록한다. 각 시험은 책임자가 확인한 저출력·중립각·안전거리로
수행한다. port/starboard는 실측 서보 방향에 따라 판단하며 90도가 실제
중립이라는 가정으로 움직이지 않는다. 프로파일은 조작 안내/로그 라벨이며
명령을 자동 실행하지 않는다. 정지 중 IMU bias·가짜 이동량·센서 주기는
motion_identifier --summarize로 확인한다. 충분히 멈춘 데이터만 bias로 사용한다.

## 9. 야외 감지 시험

```bash
ros2 launch field_calibration field_test.launch.py \
  config:="$HOME/field_config/boat.yaml" profile:=outdoor test_mode:=manual_log
```

위치추정이 INVALID여도 scan/imu/명령은 계속 기록한다. 표적 거리·각도 안정성은
저장 scan으로 사후 확인한다. 사람을 자동 회피 장애물로 사용하지 않는다.
GAP 판단 토픽이 현장에 별도로 있으면 record_topics에 실제 토픽을 추가한다.
새 패키지는 기존 대회 노드를 시작하거나 변경하지 않는다. 판단 노드가 motor
명령도 발행한다면 PASSIVE 관측을 위해 무작정 함께 실행하지 않는다.

## 10. 데이터 기록 종료

먼저 RC로 안전하게 정지한 후 launch 터미널에서 Ctrl+C를 한 번 누른다.
rosbag metadata.yaml과 metadata.json의 closed: true를 확인한다. CSV는 줄마다
flush되고 기본 1초마다 fsync되며 bag은 256 MiB로 분할된다. 전원 강제 차단,
디스크 장애나 SIGKILL의 완전 보존을 보장하지는 않는다. 정상 종료를 우선한다.

## 11. 노트북 실시간 시각화

노트북의 ROS/overlay 환경을 source하고 선박과 DDS 통신을 맞춘다:

```bash
# 선박에서 확인한 실제 값을 사용:
export ROS_DOMAIN_ID=실제숫자
unset ROS_LOCALHOST_ONLY
ros2 topic echo /field/state --once --qos-reliability best_effort
ros2 run field_calibration trajectory_monitor
```

4Hz Matplotlib 창에 실제 궤적 실선, 예상 궤적 점선, 선수방향,
servo–IMU/model yaw-rate, thruster–surge, 센서/기록 상태와 과거 예측 오차가
표시된다. 미보정 예측은 UNCALIBRATED다. INVALID 구간은 선을 끊는다.
viewer는 숫자 토픽만 구독하며 scan/이미지는 구독하지 않는다. SSH X-forwarding,
VNC, GUI 프레임 전송은 사용하지 않는다. 노트북이 끊겨도 선박 logger는 유지된다.
DDS가 통신되지 않으면 현장 네트워크 설정을 확인하되 기록을 중단하지 않는다.

## 12. 저장 데이터 확인

```bash
ls -lt ~/field_data
export EXPERIMENT=/실제/field_data/실험ID
cat "$EXPERIMENT/metadata.json"
ros2 bag info "$EXPERIMENT/bag"
wc -l "$EXPERIMENT"/*.csv
ros2 run field_calibration motion_identifier --summarize "$EXPERIMENT"
```

노트북으로 가져올 때는 rsync/scp로 폴더 전체를 복사한다. bag 종료가 끝난
실험을 복사한다. 강제 종료로 metadata.yaml이 없으면 백업 후 노트북에서:

```bash
ros2 bag reindex "$EXPERIMENT/bag"
```

sqlite 파일 자체가 손상되면 reindex로 복구되지 않을 수 있다.

## 13. 모델 추정 및 사후 재생

독립된 여러 수동 시험을 훈련용/holdout용으로 분리한다. 명령 변화, 양방향 선회,
감속이 충분하고 VALID 상태가 연결되는 데이터가 필요하다:

```bash
ros2 run field_calibration motion_identifier \
  --train /실제/train1 /실제/train2 \
  --validate /실제/holdout \
  --servo-neutral 실측각도 --throttle-neutral 실측중립값 \
  --output "$HOME/field_config/motion_model.json"
```

모델이 부족하면 UNCALIBRATED JSON과 exit code 2다. 모델 JSON의 독립 검증
0.5/1/2초 위치·heading·yaw-rate RMSE와 baseline을 비교한다. 식별되었다는
이유만으로 정확도를 보장하지 않는다. 승인할 수 있는 결과라면 boat.yaml의
model_path에 절대 경로를 지정하고 다음 실험을 시작한다.

재생은 모터와 분리된 노트북 domain에서만 한다. 원본 bag에는 actuator 명령이
들어 있으므로 연결된 선박 domain에서 전체 bag을 재생하지 않는다:

```bash
export ROS_DOMAIN_ID=192
export ROS_LOCALHOST_ONLY=1
# 저장한 숫자 결과 확인; 센서/actuator를 발행하지 않음:
ros2 bag play "$EXPERIMENT/bag" --clock --topics \
  /field/state /field/command /field/prediction /field/prediction_error /field/test_profile
# 같은 개발 domain의 다른 터미널:
ros2 run field_calibration trajectory_monitor
```

센서로 다시 계산하려면 새 replay.yaml에서 output_root를 별도 폴더로,
use_sim_time을 true로 설정하고 새 field_test를 시작한다:

```bash
ros2 launch field_calibration field_test.launch.py \
  config:=/절대경로/replay.yaml use_sim_time:=true
```

재생 토픽은 실제
bag info로 확인한 /scan, /imu, 실제 odom, /tf, /tf_static과 명령을 명시한다.
명령은 반드시 안전한 이름으로 remap한다:

```bash
ros2 bag play "$EXPERIMENT/bag" --clock --topics \
  /scan /imu /scan/odom /tf /tf_static \
  /actuator/thruster/percentage /actuator/key/degree \
  --remap /actuator/thruster/percentage:=/field/replay/throttle \
          /actuator/key/degree:=/field/replay/steering
```

replay.yaml의 throttle_topic/steering_topic도 /field/replay/*로 지정한다.
저장된 /field/state와 새 계산 결과를 동시에 재발행하지 않는다. 같은 시간축과
좌표계에서만 비교한다. 실제 선박 데이터로 모델을 식별하고 확인하는 것이
다음 권장 작업이다.
