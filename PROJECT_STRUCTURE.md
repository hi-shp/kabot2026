# 프로젝트 구조와 실제 젯슨 경로

현재 배의 확정 배선·방향·중립, 실행 명령과 토픽은 [정상 운용 기준](docs/BOAT_BASELINE_2026-10-30.md)을 참고합니다.

현재 실제 운용/빌드 workspace는 젯슨의 `/home/isv/ros2_ws`입니다. 이 저장소 전체를 그대로 src에 중첩하여 복제하는 것과 같은 구성이 아닙니다.

| 경로 | 용도 |
|---|---|
| `docs/BOAT_BASELINE_2026-10-30.md` | 사용자 정상 확인 상태, 필수 명령·토픽·논리값 변환·복원 기준 |
| `docs/BASELINE_RUNTIME_SHA256.json` | 재접속 시 주요 소스36개 SHA-256 비교 결과(실좌표·비밀번호 제외) |
| `firmware/kabot_actuator/` | 현재 UNO 운용 펌웨어. board_config.h가 실제 빌드 설정 기준 |
| `isv/` | 기존 저장소의 자율운항 코드. 실제 젯슨의 `isv_kaboat`과 세대/인터페이스가 다름 |
| `mechaship/` | 기존 저장소의 장치/launch 코드. GPS 구성은 실제 젯슨과 차이가 있음 |
| `isv_water_trials/` | 실제 젯슨에 추가한 수상 시험/분석 ROS 패키지 |
| `deploy/jetson/mechaship_bringup/launch/` | 실제 젯슨에 적용한 bringup/Velodyne launch 파일 |
| `scripts/ros2_shell_helpers.bash` | 젯슨에 적용한 cb 함수 원본 |
| `scripts/audit_jetson_workspace.py` | 원본을 변경/삭제하지 않는 소스 스냅샷 및 정적 조사 수집기 |
| `tests/test_cb_helper.bash` | cb의 빌드·환경 적용·실패 전파 시험 |
| `backups/jetson-2026-10-29-full/` | 정리 전 실제 젯슨 소스 전체 백업과 파일별 inventory. COLCON_IGNORE로 활성 빌드 제외 |
| `docs/WORKSPACE_AUDIT_2026-10-29.md` | 패키지/파일 전수 조사, 과거 bringup/시험 기록, 현재 구성 차이 |
| `docs/CB_USAGE.md` | cb 사용법·설치 경로·실제 빌드 검증 |

백업에는 Git 내부·생성 결과·원시 측정 로그·개인/AI 규칙을 포함하지 않습니다. 기존 소스, build/install/log, 다른 센서/대회 코드는 삭제하지 않았습니다. 이 문서는 코드/배포 경로를 설명하며 로컬 전용 AI 지침 파일이 아닙니다.
