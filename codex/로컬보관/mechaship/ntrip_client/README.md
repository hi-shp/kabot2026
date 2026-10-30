# ROS NTRIP Client

## 기능

NTRIP 서버에서 RTCM 보정 데이터를 받아 ROS 토픽으로 발행합니다. 가상 NTRIP 서버를 사용할 때는 NMEA 위치 메시지를 구독해 서버로 전달합니다.

## 브랜치

* [ros](https://github.com/LORD-MicroStrain/ntrip_client/tree/ros) — ROS 1 버전
* [ros2](https://github.com/LORD-MicroStrain/ntrip_client/tree/ros2) — ROS 2 버전

## 소스 빌드

1. [ROS 2 설치와 작업 공간 설정](https://docs.ros.org/en/foxy/Tutorials/Configuring-ROS2-Environment.html)

2. `ntrip_client` 폴더를 작업 공간의 `src`에 넣습니다.

3. 의존 패키지를 설치합니다: `rosdep install --from-paths ~/your_workspace/src --ignore-src -r -y`

4. 빌드하고 환경을 적용합니다:
    ```bash
    cd ~/your_workspace
    colcon build
    source ~/your_workspace/install/setup.bash
    ```
    일반 ROS 작업 공간은 새 터미널에서도 환경을 적용합니다. 현재 배의 젯슨은 자동 적용되며, 파일 수정 후 `cb`를 사용합니다.

## 인터넷 NTRIP 서버 연결

```bash
ros2 launch ntrip_client ntrip_client_launch.py
```

실행 인자:

- **host**: NTRIP 서버의 호스트 이름 또는 IP 주소
- **port**: 서버 포트. 기본값 `2101`
- **mountpoint**: 서버의 mountpoint
- **ntrip_version**: 최초 HTTP 요청의 `Ntrip-Version` 헤더 값
- **authenticate**: 인증 사용 여부. true이면 `username`, `password` 지정
- **username**: 인증용 사용자 이름. `authenticate=true`일 때 사용
- **password**: 인증용 비밀번호. `authenticate=true`일 때 사용
- **ssl**: SSL 사용 여부. true일 때 `cert`, `key`, `ca_cert` 적용
- **cert**: 인증서 기반 인증에 사용할 클라이언트 인증서
- **key**: 인증서 기반 인증에 사용할 개인키
- **ca_cert**: 자체 서명 인증서 또는 별도 CA 체인을 위한 CA 파일
- **rtcm_message_package**: RTCM 메시지 패키지. 노드 기본값 `mavros_msgs`, 이 저장소 launch 기본값 `rtcm_msgs`

## 시리얼 NTRIP 장치 연결

[MicroStrain 3DM-RTK](https://www.microstrain.com/inertial-sensors/3dm-rtk) 같은 장치에서 시리얼로 보정 데이터를 받습니다.

```bash
ros2 launch ntrip_client ntrip_serial_device_launch.py
```

실행 인자:

- **port**: 장치를 연결한 시리얼 포트
- **baudrate**: 시리얼 속도. 기본값 115200baud
- **rtcm_message_package**: RTCM 메시지 패키지. 노드 기본값 `mavros_msgs`, 이 저장소 launch 기본값 `rtcm_msgs`

## 토픽

* **/rtcm**: 서버에서 받은 보정 데이터를 [RTCM 메시지](http://docs.ros.org/en/noetic/api/mavros_msgs/html/msg/RTCM.html)로 발행합니다. [microstrain_inertial_driver](https://github.com/LORD-MicroStrain/microstrain_inertial) 등의 노드에서 사용합니다.
    * 메시지 형식은 [`mavros_msgs/RTCM`](https://github.com/mavlink/mavros/blob/ros2/mavros_msgs/msg/RTCM.msg) 또는 [`rtcm_msgs/Message`](https://github.com/tilk/rtcm_msgs/blob/master/msg/Message.msg) 중 `rtcm_message_package`로 선택합니다.
* **/nmea**: [NMEA 위치 메시지](http://docs.ros.org/en/api/nmea_msgs/html/msg/Sentence.html)를 받아 서버로 전달합니다. 가상 NTRIP 서버와 NTRIP 장치 연결에 필요합니다.
* **/fix**: `/nmea`와 같은 용도로 전역 위치를 전달하며, NMEA 이외 형식의 위치 입력을 받습니다.

## 라이선스

MIT License를 따릅니다. 배포본의 `LICENSE`를 참고합니다.

Copyright (c)  2024, MicroStrain by HBK
