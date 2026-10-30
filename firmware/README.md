# 현재 아두이노 펌웨어

`kabot_actuator/`는 젯슨 `/home/isv/boat_firmware/kabot_actuator/`에서 가져온 UNO R3 운용 소스입니다. 8개 파일의 SHA-256을 젯슨 원본과 비교했습니다.

- 입력: D2 좌현, D3 우현, D4 RC/AUTO 모드
- 출력: D9 좌현, D10 우현
- ESC: Blue Robotics Basic ESC 500, 정지·초기화 1480µs
- 논리 명령: 1100~1900, 중립 1500

설정은 [`board_config.h`](kabot_actuator/board_config.h)를 기준으로 합니다. 상세 동작과 업로드 방법은 [펌웨어 안내](../codex/firmware/README.md), 빌드·업로드 도구는 [`codex/firmware/jetson_upload.sh`](../codex/firmware/jetson_upload.sh)에 있습니다.
