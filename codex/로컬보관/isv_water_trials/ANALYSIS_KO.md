# 확인한 기존 코드와 적용 범위

2026-10-29 젯슨 `/home/isv/ros2_ws/src`를 직접 읽었습니다. 기존 코드 수정본과 삭제된 파일이 다수 있어 사용자 변경은 되돌리지 않았습니다. 새 분석 도구는 별도 `isv_water_trials` 패키지로 추가합니다.

- `isv_kaboat/isv_kaboat/kaboat2025*.py`, `VO_dong.py`, `isv_orgin.py`: GNSS와 VectorNav 자세, 라이다/영상 융합 결과로 좌·우 PWM 서비스를 호출합니다. 회피/방위 제어 코드이며 확인 범위에 MPC 최적화기는 없습니다. 모든 파일 전체를 형식 검증했다는 뜻은 아닙니다.
- `isv_kaboat/setup.py`: GNSS 출력, 주기 확인, IMU 출력, 융합 확인, 추력 확인 실행파일을 등록합니다. 자율운항 스크립트가 전부 console entry로 등록된 상태는 아닙니다.
- `thrust_check.py`: 1500 μs 중립, 1600 μs 전진, 1400 μs 후진을 시간 지연으로 실행합니다. 응답 완료 확인 없이 `call_async` 후 `sleep`하며 executor를 돌리지 않고, GNSS·IMU·PWM의 동기 기록을 만들지 않습니다. 수상 물리량 역추정에는 별도 기록/응답 확인이 필요합니다. 기존 파일은 보존했습니다.
- `imu_convert.py`: yaw 음수에 360을 더해 표시합니다. 주석의 CCW 표현과 VectorNav NED raw 관례가 다를 수 있습니다. raw yaw는 ENU로 변환하고 raw gyro 부호를 교차 확인하도록 했습니다.
- `vectornav_msgs/msg/CommonGroup.msg`: `group_fields`로 실제 활성 필드를 판단해야 합니다. 필드 미활성 0을 유효 측정값으로 취급하지 않습니다.
- `vectornav/.../vn_sensor_msgs.cc` 및 설정: `use_enu=true`는 FRD를 RFU로 바꿉니다. RFU 값을 선체 FLU로 그대로 사용하지 않습니다. 새 분석은 raw CommonGroup을 사용합니다.
- `GPS_Spoofer.py`: 실제 GNSS 토픽과 같은 `/smc_2000/fix`에 고정 위치를 발행합니다. 실험에서 실행하면 안 되며 preflight에서 이름과 중복 발행자를 검사합니다. 이름을 숨긴 스푸퍼까지 탐지한다고 보장하지 않습니다.
- `mechaship_interfaces/srv/ThrottlePulseWidth.srv`: `uint16 pulse_width -> bool status`. 좌우 응답을 확인하고 N 단위 추력과 구분합니다.
- `mechaship_example/emo_switch_example.py`: `/sensor/emo/status`의 true를 눌림으로 사용합니다. 실물 배선의 논리가 같은지 현장에서 확인해야 합니다.
- `mechaship_bringup`: 젯슨 버전은 ublox/iahrs이고 GitHub 버전은 WTRTK/NMEA/NTRIP 등을 사용해 서로 다릅니다. GitHub 전체 bringup을 젯슨에 덮어쓰지 않고 각각의 라이다 부분만 바꿨습니다. 실제 포트/모델/패킷은 아직 미확인입니다.
- `velodyne`: VLP16/VLP32C/VLS128 드라이버·포인트클라우드·LaserScan 변환 패키지는 설치되어 있습니다. 기존 bringup이 필수 호출하던 YDLIDAR는 젯슨에 없습니다. Velodyne launch로 교체했고 기존 GPS·카메라·IMU 경로는 유지했습니다.
- GitHub `hi-shp/kabot2026`의 `isv/launch_isv`: 단일 조타/추진기 계열 코드도 포함되어 있어 젯슨의 차동 추진기 인터페이스와 같다고 가정하지 않습니다. 사용자 MPC 원본의 정확한 파일 확인이 추가로 필요합니다.

원본 bringup 백업: `/home/isv/water_trial_backups/20261029/mechaship_bringup.launch.py.before`.

시험 데이터·실제 위치 좌표·장비 인증정보는 Git에 넣지 않습니다. 제원과 공개 코드·익명 모의시험만 버전 관리합니다.
