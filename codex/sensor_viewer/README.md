# PC 센서 화면

```powershell
powershell -ExecutionPolicy Bypass -File codex\sensor_viewer\start_viewer.ps1
```

SSH 비밀번호를 입력하면 **http://127.0.0.1:8765**가 열립니다. Python 가상환경은 PC의 `%LOCALAPPDATA%\KabotSensorViewer`에 준비됩니다. 젯슨에는 조회용 ROS 노드만 추가합니다.

라이다 3D·2D, IMU 자세·각속도·가속도, GPS, 전체 노드·토픽과 메시지를 표시합니다. 화면 각도는 °, 거리는 m입니다. 점군은 최대 5,000점·5 FPS이며, 링별로 고르게 추려 표시합니다. 저사양 PC에서는 `-MaxPoints 2000 -Hz 3`으로 낮출 수 있습니다.

[수조 지도·목적지·왕복 시험 명령](../docs/INDOOR_TANK.md)을 참고하세요. 지도 클릭은 경로 저장과 초기 위치 설정에 사용하며, 추진은 `indoor_trial --execute`에서 시작합니다.

검증: 실제 점군·IMU·GPS 상태와 ROS 그래프를 받았고, Chrome의 3D/2D 전환·토픽 선택·단위 변환·좁은 화면을 확인했습니다. 변환·경로·응답 모델의 자동 검사는 `python -m unittest discover -s codex/sensor_viewer -p test_tank.py -v`로 실행합니다.
