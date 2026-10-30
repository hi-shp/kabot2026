# cb: 전체 빌드와 현재 셸 환경 적용

새 젯슨 Bash/SSH 터미널은 ROS Humble과 설치된 작업 공간을 자동으로 불러옵니다. 통신 설정은 `ROS_DOMAIN_ID=50`, `ROS_LOCALHOST_ONLY=0`입니다. 터미널을 열면 바로 브링업과 노드를 실행할 수 있습니다.

파일 수정 후에만 다음 명령으로 빌드·현재 셸 환경을 갱신합니다.

```bash
cb
```

`cb`는 어느 경로에서 실행해도 `~/ros2_ws`로 이동합니다. ROS Humble 환경에서 전체 패키지를 빌드하고, 성공하면 현재 터미널에 `install/setup.bash`를 적용합니다.

기존에 열어 둔 터미널에는 한 번 다음을 실행하면 됩니다.

```bash
source ~/.bashrc
```

일부 패키지만 지정할 때는 colcon 인자를 그대로 전달합니다.

```bash
cb --packages-select isv_water_trials mechaship_bringup
```

기존 build/install 구성으로 `colcon build`를 실행합니다. 전달한 인자는 그대로 사용하며, `--install-base`를 지정하면 해당 위치의 setup.bash를 적용합니다. 빌드 실패나 setup.bash 누락은 오류로 반환합니다. 설치 환경은 빌드 성공과 파일 확인을 마친 뒤 적용합니다. 센서와 주행 노드는 별도 실행 명령으로 관리합니다.

## 설치 위치

- 관리 원본: [`scripts/ros2_shell_helpers.bash`](../scripts/ros2_shell_helpers.bash)
- 젯슨 적용 파일: `/home/isv/.config/isv/ros2_helpers.bash`
- 자동 로드: `/home/isv/.bashrc` 끝의 `# ISV_ROS2_CB_HELPER` 블록.
- 변경 전 bashrc: `/home/isv/code_backups/shell-before-cb-20261029/.bashrc.before`.
- 기본 workspace: `~/ros2_ws`. 다른 경로가 필요하면 `ISV_ROS_WS`로 지정합니다.
- 기본 underlay: `/opt/ros/humble/setup.bash`. 필요하면 `ISV_ROS_SETUP`으로 지정합니다.

`.bashrc`가 ROS 환경과 helper를 불러오고, helper가 통신 기본값과 `cb`를 설정합니다.

## 현재 적용 확인(2026-10-30)

환경 변수를 비운 새 일반 Bash와 로그인 Bash에서 `ros2`, `isv_water_trials`, `cb` 함수와 통신 설정 50/0을 확인했습니다. 젯슨과 로컬 helper의 SHA-256도 일치합니다.

## 이전 빌드 검증(2026-10-29)

- `bash -n` 구문 검사: PASS.
- 가짜 colcon을 사용한 별도 Bash 시험: 기본 전체 빌드 호출, 인자 전달, 현재 셸 환경 반영, 공백을 포함한 custom install 경로, 실패 코드 7 전달과 환경 적용 차단 PASS.
- 실제 젯슨 `cb`: **34 packages finished [10.7s]**, 종료 코드 0.
- 실제 빌드 후 같은 셸의 setup 적용 메시지 확인.
- 새 interactive Bash에서 `cb`가 function으로 자동 등록됨을 확인.
- 빌드 뒤 원본 소스/소스 압축파일 1,279개를 스냅샷의 SHA-256과 다시 비교: 변경/누락 0개.
- 적용 helper와 저장소 원본 SHA-256 일치: `64b82379aecc4725bd12427c74b6048e413b6bc49e5b6d18981cf54b439b6057`.
- 시험 로그: `/home/isv/code_backups/cb_build_20261029.log` (로컬 보존, Git 미포함).

셸 함수 검사는 [`tests/test_cb_helper.bash`](../tests/test_cb_helper.bash)로 재현할 수 있습니다. 센서 수신과 수상 동작은 별도 운용 확인이 필요합니다. 기존 메시지 형식과 브링업 구성은 [전수 조사 보고서](WORKSPACE_AUDIT_2026-10-29.md)에 정리했습니다.
