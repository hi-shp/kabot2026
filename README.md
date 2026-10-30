# KABOAT 2026

## 필수 명령어

**SSH 접속** — PC에서 젯슨에 연결합니다.

```bash
ssh isv@10.182.77.20
```

새 젯슨 터미널은 ROS 환경이 자동 적용됩니다. 파일 수정 후에는 `cb`로 빌드·적용합니다.

**전체 브링업** — GPS·IMU·라이다·아두이노 브리지를 켭니다.

```bash
ros2 launch mechaship_bringup all_sensors.launch.py
```

카메라를 연결했다면 `use_camera:=true`를 추가합니다. Ctrl+C로 해당 브링업과 하위 프로세스를 종료합니다.

**추진기 시험** — 다른 터미널에서 실행합니다. 아래 명령은 양쪽 50% 직진입니다.

```bash
ros2 run isv_water_trials mode_test --side both --percent 50
```

`--side left`는 좌현, `--side right`는 우현이며 후진은 `--percent -50`입니다. 시험은 Ctrl+C까지 계속 실행됩니다. RC/AUTO 선택은 아두이노가 처리합니다. Ctrl+C를 누르면 중립 응답을 확인하고 종료합니다.

**ESC 초기화** — EMO 해제 후 재초기화가 필요할 때 실행합니다.

```bash
ros2 run isv_water_trials reinitialize_esc
```

실행 중인 시험은 중립을 7초 유지한 뒤 원래 출력을 자동 재개합니다.

**일회 중립 요청** — MCU에 즉시 정지를 요청합니다.

```bash
ros2 run isv_water_trials stop_thrusters
```

실행 중인 시험은 다음 명령부터 다시 출력합니다. 시험을 계속 멈춰 두려면 시험 터미널에서 Ctrl+C를 누릅니다.

## 저장소 구조

젯슨 `/home/isv/ros2_ws/src`의 14개 폴더를 저장소 최상위에 같은 이름과 구조로 보관합니다. `firmware/`에는 현재 UNO에 적용한 펌웨어를, `codex/`에는 문서·백업·로컬 도구·겨울 최종본을 모았습니다.

```text
kabot2026/
├── fusion_viewer/
├── isv_kaboat/
├── isv_water_trials/
├── mechaship/
├── mechaship_example/
├── micro-ROS-Agent-humble/
├── micro_ros_msgs/
├── ouster-ros/
├── rtcm_msgs-master/
├── ublox-ros2/
├── usb_cam/
├── vectornav/
├── velodyne/
├── yolo_ros/
├── firmware/kabot_actuator/
└── codex/
    ├── docs/
    ├── scripts/
    ├── tests/
    ├── firmware/
    ├── deploy/
    ├── backups/
    ├── 로컬보관/
    └── 레거시코드/동계 최종코드/
```

Git 내부 파일, 빌드 결과·캐시, 개인 개발 설정과 실측 GPS 로그는 수집 대상에서 제외했습니다. 원본 소스 ZIP과 드라이버 시험 자료는 함께 보관합니다. 파일별 해시는 [젯슨 소스 목록](codex/docs/JETSON_SOURCE_MANIFEST.json)에서 확인할 수 있습니다.

## 주요 설정과 문서

- 명령 표현: **1000=최대 후진, 1500=정지, 2000=최대 직진**. 현재 ROS 서비스에는 1100/1500/1900으로 변환해 전달합니다.
- 실제 ESC 정지 출력: 양쪽 **1480µs**.
- 배선: 수신기 CH2→UNO D2 좌현, CH3→D3 우현, CH5→D4 모드. ESC 출력은 D9 좌현·D10 우현입니다.
- 선체: 25kg, 길이 1.1m, 폭 0.50m, 추진기 중심 간격 0.32m.

| 문서 | 내용 |
|---|---|
| [정상 운용 기준](codex/docs/BOAT_BASELINE_2026-10-30.md) | 실행 명령·주요 토픽·모드 전환·ESC 초기화·기록 방법 |
| [프로젝트 구조](codex/PROJECT_STRUCTURE.md) | 저장소와 젯슨 경로 대응 |
| [아두이노 설정](codex/firmware/README.md) | 현재 펌웨어 배선·출력·통신 |
| [cb 사용법](codex/docs/CB_USAGE.md) | 빌드와 셸 환경 적용 |
| [겨울 최종 코드](codex/레거시코드/README.md) | 기존 `자율운항 최종코드` 커밋 원본 |

운용 확인 기준은 `boat-stable-20261030` 태그입니다. 이 태그는 폴더 정리 전 상태를 보존합니다.
