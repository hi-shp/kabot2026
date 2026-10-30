# 프로젝트 구조

저장소 최상위는 젯슨 `/home/isv/ros2_ws/src`의 폴더 구조를 따릅니다. 각 ROS 패키지의 소스·설정·모델·시험 자료와 원본 소스 ZIP을 같은 경로에 보관합니다.

| 저장소 경로 | 젯슨 경로 또는 용도 |
|---|---|
| `<src의 각 폴더>/` | `/home/isv/ros2_ws/src/<같은 폴더>/` |
| `firmware/kabot_actuator/` | `/home/isv/boat_firmware/kabot_actuator/`, 현재 UNO 운용 소스 |
| `codex/docs/` | 운용 안내·전수 조사·소스 목록 |
| `codex/scripts/` | cb 설정·소스 수집·프로세스 정리 도구 |
| `codex/tests/` | 셸 함수·프로세스 종료 검사 |
| `codex/firmware/` | 펌웨어 설명·빌드/업로드 도구·검사 코드 |
| `codex/deploy/` | 이전 배포용 사본 |
| `codex/backups/` | 2026-10-29 젯슨 소스 백업 |
| `codex/로컬보관/` | 폴더 정리 직전 로컬 파일 |
| `codex/레거시코드/동계 최종코드/` | `d310548` 커밋 원본 전체 |

젯슨 최상위 폴더 14개와 파일별 해시는 [JETSON_SOURCE_MANIFEST.json](docs/JETSON_SOURCE_MANIFEST.json)에 기록했습니다. 수집 대상은 소스와 자료이며, Git 내부 파일·빌드 결과·캐시·개인 개발 설정·실측 GPS 로그는 제외했습니다. `codex/COLCON_IGNORE`는 보관 사본의 중복 빌드를 방지합니다.

겨울 최종본은 원본 커밋의 파일 경로를 보존하고, 공개 사본의 NTRIP 설정 두 파일에서 계정 값만 가렸습니다. 원본·공개 사본 해시는 [WINTER_SOURCE_MANIFEST.json](레거시코드/WINTER_SOURCE_MANIFEST.json)에 있습니다.

실행 명령과 토픽은 [정상 운용 기준](docs/BOAT_BASELINE_2026-10-30.md), 자주 쓰는 명령은 [최상위 README](../README.md)를 참고합니다.
