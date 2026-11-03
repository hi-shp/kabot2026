# KABOAT 2026 대시보드

```powershell
powershell -ExecutionPolicy Bypass -File codex\sensor_viewer\start_viewer.ps1
```

SSH 비밀번호를 입력하면 **http://127.0.0.1:8765/**가 열립니다. 주소 변경 시 `-Jetson 새주소`를 붙입니다. PC 실행 환경은 `%LOCALAPPDATA%\KabotSensorViewer`에 설치합니다.

젯슨에서 `ros2 run isv_water_trials tank_session start sensors`를 실행하면 센서와 RC 기록·학습이 시작됩니다. PC 대시보드 프로그램을 켜 둔 상태에서는 브링업에 맞춰 자동 연결됩니다. 주행은 지도 아래 **순환 주행 시작 / 주행 정지**로 조작합니다.

라이다 3D·2D, IMU 자세·각속도·가속도, GPS, 전체 노드·토픽과 메시지를 표시합니다. 화면 각도는 °, 거리는 m입니다. 점군은 최대 5,000점·5 FPS이며, 링별로 고르게 추려 표시합니다. 저사양 PC에서는 `-MaxPoints 2000 -Hz 3`으로 낮출 수 있습니다.

[RC 학습·모델 적용·GPS 순환 주행](../docs/OUTDOOR_RC.md)을 참고하세요. 기존 [수조 화면](../docs/INDOOR_TANK.md)은 `/sensors`에서 엽니다.

위쪽 **스크린샷 / 영상 시작 / 영상 종료**로 전체 페이지와 각 패널을 함께 저장합니다. 스크롤 아래까지 한 번에 촬영하고, 같은 화면에서 패널별 파일을 만듭니다. 사진은 원본 크기 PNG, 영상은 고화질 H.264 MP4입니다. 영상은 실제 촬영된 프레임과 시간 간격을 그대로 저장하며 화면에 실제 fps를 표시합니다. 패널 영상은 녹화 종료 후 잘라 저장하므로 전체 영상과 시점·프레임이 같습니다. 저장 위치는 `C:\Users\Park\바탕화면\2026 활동\카봇 전국대회\대시보드기록`이며, 날짜별로 구분합니다. 영상 종료 후 저장이 끝날 때까지 프로그램을 켜 두세요.

로컬 검사는 `python -m unittest discover -s codex/sensor_viewer -p 'test_*.py'`로 실행합니다. 브라우저 검사는 별도 시험 데이터를 사용합니다. 2026-11-03 대시보드 검사 118개를 통과했고, 젯슨의 별도 ROS 도메인에서 기록 노드와 대시보드 수신부의 연결을 확인했습니다. 실제 센서·추진기 운용은 현장에서 확인합니다.
