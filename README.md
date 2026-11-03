# 젯슨 적용 전 백업

2026-11-03 로컬 버전 적용 전 젯슨 소스입니다. 원래 홈 폴더의 상대 경로를 유지했습니다.

- `ros2_ws/src`: 기존 ROS 소스
- `boat_firmware`: 기존 아두이노 펌웨어 소스·산출물
- `shape_docking_red_triangle`: 별도 도킹 작업
- `.local/share/kabot_sensor_viewer`: 기존 대시보드 서버

소스 1417개를 원본 바이트와 실행 권한 그대로 보존했습니다. 빌드·설치·캐시·Git 내부 파일은 수집에서 제외했습니다. 개인 셸 설정, GPS 로그, 원본 ZIP 등 공개 제외 목록은 `SNAPSHOT_MANIFEST.json`에 있으며 PC의 원본 압축 백업에 보관했습니다. 실험 bag·모델은 기존 젯슨 저장 위치에 보존합니다.
