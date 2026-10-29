# 젯슨 bringup 수정본

이 폴더는 2026-10-29 젯슨에 실제로 적용한 launch 파일입니다. 저장소의 원래 bringup은 WTRTK/NMEA/NTRIP 구성이며 젯슨의 기존 bringup은 ublox/iahrs 구성이므로, 두 파일을 같은 것으로 취급하지 않았습니다.

젯슨에는 이 폴더의 `mechaship_bringup/launch/*.py` 두 파일을 `/home/isv/ros2_ws/src/mechaship/mechaship_bringup/launch/`에 적용했습니다. 원본은 `/home/isv/water_trial_backups/20261029/`에 보존했습니다. GPS·IMU·카메라 설정과 기존 사용자 코드는 덮어쓰지 않았습니다.

공통 추가 의존성은 `launch`, `launch_ros`, `ament_index_python`, `velodyne_driver`, `velodyne_pointcloud`, `velodyne_laserscan`, `python3-yaml`입니다. 젯슨의 기존 `package.xml`에 누락된 항목을 추가했습니다. 저장소 쪽 package.xml은 기존 YDLIDAR 실행 의존성만 Velodyne으로 바꾸었습니다.

`use_lidar_odom`은 기본 false입니다. 물 위의 라이다 오도메트리를 정확한 선속계로 간주하지 않습니다. 실제 라이다 장비 모델, IP, 설치 높이·각도와 패킷 수신 여부는 별도 확인해야 합니다.

시험 코드·제원은 루트의 `isv_water_trials/`를 사용합니다. 사용법과 미검증 항목은 해당 패키지의 문서를 확인하세요. 이 폴더 자체는 ROS 패키지가 아닙니다.
