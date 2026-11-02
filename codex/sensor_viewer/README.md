# KABOAT 2026 대시보드

```powershell
powershell -ExecutionPolicy Bypass -File codex\sensor_viewer\start_viewer.ps1
```

SSH 비밀번호를 입력하면 **http://127.0.0.1:8765/**가 열립니다. 주소 변경 시 `-Jetson 새주소`를 붙입니다. PC 실행 환경은 `%LOCALAPPDATA%\KabotSensorViewer`에 설치합니다.

젯슨에서 `ros2 run isv_water_trials tank_session start sensors`를 실행하면 센서와 RC 기록·학습이 시작됩니다. PC 대시보드 프로그램을 켜 둔 상태에서는 브링업에 맞춰 자동 연결됩니다. 주행은 지도 아래 **순환 주행 시작 / 주행 정지**로 조작합니다.

라이다 3D·2D, IMU 자세·각속도·가속도, GPS, 전체 노드·토픽과 메시지를 표시합니다. 화면 각도는 °, 거리는 m입니다. 점군은 최대 5,000점·5 FPS이며, 링별로 고르게 추려 표시합니다. 저사양 PC에서는 `-MaxPoints 2000 -Hz 3`으로 낮출 수 있습니다.

[RC 학습·모델 적용·GPS 순환 주행](../docs/OUTDOOR_RC.md)을 참고하세요. 기존 [수조 화면](../docs/INDOOR_TANK.md)은 `/sensors`에서 엽니다.

로컬 검사는 `python -m unittest discover -s codex/sensor_viewer -p 'test_*.py'`로 실행합니다. 브라우저 검사는 별도 시험 데이터를 사용하며 ROS·추진기에 연결하지 않습니다. 현재 변경의 젯슨 운용 확인은 연결 후 진행합니다.
