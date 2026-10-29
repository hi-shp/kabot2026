# 젯슨 ROS 워크스페이스 전수 조사

기준: 2026-10-29, `/home/isv/ros2_ws/src`. 삭제는 하지 않습니다. 공개 소스 백업은 [backups](../backups/README.md)에 보존했습니다. 단축 명령 설정은 백업 커밋·푸시 확인 뒤 별도 변경으로 적용합니다.

## 확인 범위와 근거

정리 전 14개 최상위 소스 폴더에서 ROS package.xml 34개, Python 파일 265개를 수집했습니다. 일반 파일 1,279개를 바이트 단위로 백업하고 SHA-256을 확인했습니다. 내부 상대 심볼릭 링크 4개, 원본 ZIP 2개, 제외 파일은 inventory에 따로 기록했습니다. Python 파일 전부를 AST로 분석하여 import, 토픽/서비스, launch 노드, setup.py 실행 엔트리를 뽑았습니다. package.xml의 조건부 의존성은 정적 후보이며 실제 활성 의존성과 동일하다고 단정하지 않습니다.

`.bash_history`, `.bashrc`와 시스템 셸 설정, ROS launch 로그 96개 세션, 과거 빌드 기록 위치를 읽었습니다. 사용자/시스템의 ROS 관련 자동 시작 서비스는 조사한 위치에서 찾지 못했습니다. 히스토리는 입력한 명령의 흔적이며, 그 명령의 성공·수상 시험 완료를 입증하지 않습니다. launch 로그의 `process started`도 센서 데이터 정상 수신을 입증하지 않습니다. 원시 히스토리/로그는 공개하지 않고 필요한 명령과 프로세스 이름만 요약했습니다.

모든 파일의 명세는 [`inventory.json`](../backups/jetson-2026-10-29-full/inventory.json)에 있습니다. 이 조사는 소스·의존성과 과거 실행 경로의 정적 조사입니다. 실물 배를 구동하거나 모든 노드를 import하여 시험하지 않았습니다.

## 과거 bringup과 시험 방식

실행 기록에서 다음이 확인됩니다. 횟수는 저장된 히스토리의 출현 횟수이며 신뢰도나 성공 횟수가 아닙니다. 마지막 두 bringup과 수상 시험 명령은 이번 개발 중 확인한 기록도 포함합니다.

| 명령 | 히스토리 횟수 | 확인 내용 |
|---|---:|---|
| `ros2 launch vectornav vectornav.launch.py` | 14 | IMU를 별도로 실행한 흔적 |
| `ros2 run micro_ros_agent micro_ros_agent ...` | 9 | MCU 연결용 agent를 별도로 실행한 흔적 |
| `ros2 launch usb_cam camera.launch.py` | 8 | 카메라를 별도로 실행한 흔적 |
| `ros2 launch yolo_bringup yolo.launch.py` | 6 | YOLO/추적을 별도로 실행한 흔적 |
| `ros2 launch ouster_ros sensor.launch.xml` | 6 | Ouster를 사용/시험한 흔적. 지금 장착 여부는 미확인 |
| `ros2 launch fusion_viewer fusion_viewer.launch.py` | 6 | 영상/라이다 융합 노드의 별도 실행 흔적 |
| `ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py` | 4 | VLP16 구성으로 실행한 과거 흔적 |
| `ros2 launch mechaship_bringup mechaship_bringup.launch.py` | 1 | 지금 확인한 bringup. 기존 전체 시스템 시작 경로라고 단정할 수 없음 |

launch 로그에서는 `vectornav`와 `vn_sensor_msgs`가 각각 30회, `usb_cam_node_exe` 25회, `os_driver` 16회, YOLO/추적/디버그 프로세스 각각 8회, Velodyne 세 노드 각각 7회, `viewer_node` 5회의 시작 기록이 있습니다. `actuator_enable_node`와 `ublox_gps_node` 시작은 각각 1회입니다.

micro-ROS agent의 직렬 포트 입력 흔적은 `/dev/ttyTHS1` 5회, `/dev/ttyTHS0` 2회와 `ttyTH1`, `ttyTH2` 각 1회입니다. 뒤의 두 포트명은 오타일 가능성도 있으므로 실제 장치 설정으로 복사하지 않습니다. 현재 MCU 연결 포트가 무엇인지는 실물 연결과 실제 토픽으로 별도 확인해야 합니다.

저장소의 `명령어 모음.txt`는 `/home/ubuntu/ros2_ws` 기준이고, 실제 젯슨은 `/home/isv/ros2_ws`입니다. 파일에는 Foxglove, GNSS 시각화·발행, tmux, 빌드 방법이 적혀 있습니다. 프로세스 강제 종료나 build/install/log 삭제 명령도 있지만 이번 작업에서는 실행하지 않습니다.

## 현재 bringup과 실제 제어 코드의 차이

젯슨의 `mechaship_bringup.launch.py`는 기본적으로 actuator 설정, ublox GPS, USB 카메라, iahrs IMU, 로봇 상태 publisher, Velodyne 노드를 구성합니다. 라이다 오도메트리는 기본 OFF입니다. **micro-ROS agent, VectorNav, YOLO, fusion_viewer와 자율운항 제어 노드를 전부 포함하는 launch는 아닙니다.**

현재 `isv_kaboat` 자율운항 코드는 `/smc_2000/fix`, `/vectornav/raw/common`, 좌·우 `/actuators/throttle_*/set_pulse_width` 서비스를 사용합니다. 기존 bringup의 ublox 출력 remap은 `/gps/fix`, IMU는 iahrs 계열입니다. 따라서 bringup 하나를 실행했다고 위 자율운항 코드의 모든 입력이 준비되는 것은 아닙니다. 서로 다른 세대의 구성이 섞여 있습니다. 임의로 센서 모델이나 포트를 추정하여 설정을 통일하지 않았습니다.

이번에 추가한 Velodyne launch는 드라이버/포인트클라우드/scan 세 프로세스 시작까지 확인했지만 UDP packet timeout 상태였습니다. 실제 VLP16 여부, IP, 유선 링크 및 scan 수신은 미확인입니다. 수상 시험 preflight는 필요한 GNSS·VectorNav·비상스위치가 없을 때 NOT_READY로 종료합니다.

## 패키지 분류와 보존 상태

아래 모든 항목을 현재 소스와 백업에 보존합니다. 과거 사용 또는 현재 의존성이 확인되지 않았다는 이유로 삭제하지 않습니다.

| 폴더 / 패키지 | 역할·근거 | 정리 판단 |
|---|---|---|
| `isv_kaboat` | 기존 차동 추진기 자율운항·도킹·VO·센서 확인 코드 | 보존. 실제 사용할 실행파일 선택 필요 |
| `isv_water_trials` | 수상 시험 기록·반복 시험·역추정·MPC 모의 평가 | 보존 |
| `mechaship_bringup`, `mechaship_system`, `mechaship_interfaces`, `mechaship_description` | 현재 bringup, MCU 서비스, 사용자 정의 메시지/서비스, 로봇 형상 | 보존 |
| `velodyne`의 5개 패키지 | VLP16 등 드라이버·포인트클라우드·scan | 현재 사용 후보. 보존 |
| `vectornav`, `vectornav_msgs` | 기존 코드가 직접 사용하는 IMU와 메시지 | 보존. 과거 별도 실행 기록 있음 |
| `micro-ROS-Agent-humble`, `micro_ros_msgs` | MCU 통신 | 보존. 과거 별도 실행 기록 있음 |
| `ublox-ros2`의 4개 패키지, `rtcm_msgs-master` | 현재 bringup의 GPS 드라이버와 의존성 | 보존. 출력 토픽 차이 확인 필요 |
| `iahrs_ros2_driver`의 2개 패키지 | 현재 bringup의 IMU 경로 | 보존. VectorNav와 세대 차이 확인 필요 |
| `usb_cam` | 카메라 | 보존. 현재 bringup·과거 기록 있음 |
| `yolo_ros`의 3개 패키지, `fusion_viewer` | 검출·추적·영상/라이다 융합 | 보존. 기존 자율운항 입력일 수 있음 |
| `ouster-ros`의 2개 패키지 | 다른 라이다와 메시지 | 보존. 과거 실행 기록 있고 현재 사용 여부 미확인 |
| `mechaship_example` | 배터리·비상스위치·추진기·조타 등 시험 예제 | 보존. 인터페이스 세대 차이 있음 |
| `mechaship_teleop`, `mechaship_yolov8` | 예전 조이스틱/키보드/YOLO 경로 | 보존. 현재 인터페이스와 맞지 않는 import가 있음 |
| `mechaship_slam`, `rf2o_laser_odometry` | 지도·레이저 오도메트리 | 보존. 물 위 정확도와 사용 여부 미확인 |

설치된 `mechaship_interfaces.msg`에는 `Classification`, `ClassificationArray`, `Detection`, `DetectionArray`, `Goal`, `Heading`, `Load`, `RelHeading`, `StartHeading`이 있습니다. 서비스에는 `Battery`, `Key`, `RGBColor`, `ThrottlePercentage`, `ThrottlePulseWidth`가 있습니다. 예전 teleop의 `ToneTopic`, `RgbwLedColor` import는 현재 생성된 인터페이스 목록에 없습니다. Python 구문 통과와 실행 가능 여부가 다른 구체적인 사례입니다.

`isv_kaboat`의 `gps` 엔트리는 `GPS_Spoofer`를 실행합니다. 실제 GNSS 확인 도구로 혼동하면 안 됩니다. `thrust` 엔트리는 직접 PWM을 보내는 기존 추력 시험 코드입니다. 전수 조사 중에는 실행하지 않았습니다. 센서 확인과 추진기 시험을 명확히 구분해야 합니다.

## cb 조사

현재 `.bashrc`, `.bash_aliases`, `.profile`, 시스템 bash 설정과 조회한 profile.d/fish 설정에서 `cb`의 정의를 찾지 못했습니다. 새 interactive bash에서도 `cb`는 정의되지 않았습니다. `.bashrc`는 ROS Humble과 기존 `install/setup.bash`를 source하고 있습니다.

히스토리에서는 인자 없는 `colcon build` 14회, `--symlink-install --packages-up-to velodyne` 3회, `--symlink-install` 전체 빌드 1회가 확인됩니다. 단순 `cb` 입력 흔적은 1회이나 성공 증거는 없습니다. 저장소 명령 모음에는 삭제 후 symlink 빌드가 있지만 현재 요청은 삭제 금지입니다. 따라서 새 `cb`의 기본값은 기존 build/install을 유지하는 **전체 `colcon build` → 성공 시 `source install/setup.bash`**로 설정합니다. 실패하면 환경 적용을 건너뛰고 실패 코드를 반환해야 합니다.

## 미검증 사항

현재 실제 장착된 모든 센서의 모델·포트, MCU 펌웨어와 명령 만료 정지, 실행해야 하는 자율운항 파일, 카메라·도킹·YOLO의 현장 사용 여부, 실제 MPC 원본 위치는 아직 확정되지 않았습니다. 소스 백업이 수상 주행 성공이나 모든 예제 동작을 보장하지 않습니다. 이번 조사와 단축 명령 설정은 파일/패키지 삭제를 포함하지 않습니다.
