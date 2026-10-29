# cb: 전체 빌드와 현재 셸 환경 적용

2026-10-29 젯슨 `isv` 계정에 설정했습니다. 새 Bash/SSH 터미널에서는 다음 한 줄이면 됩니다.

```bash
cb
```

`cb`는 현재 위치와 관계없이 `~/ros2_ws`로 이동하여 ROS Humble 환경을 불러오고, `colcon build`로 전체 패키지를 빌드합니다. 빌드가 성공한 경우에만 같은 셸에서 `install/setup.bash`를 source합니다. 별도 프로세스에서 source한 뒤 부모 터미널에 적용됐다고 표시하는 방식이 아닙니다.

기존에 열어 둔 터미널에는 한 번 다음을 실행하면 됩니다.

```bash
source ~/.bashrc
cb
```

일부 패키지만 지정할 때는 colcon 인자를 그대로 전달합니다.

```bash
cb --packages-select isv_water_trials mechaship_bringup
```

기본 명령은 `--symlink-install`이나 폴더 삭제를 자동으로 추가하지 않습니다. 현재 젯슨의 기존 build/install 구성을 유지합니다. `--install-base`를 별도로 지정하면 그 위치의 setup.bash를 적용합니다. 빌드 실패 시 비정상 종료 코드를 그대로 반환하고 workspace setup을 source하지 않습니다. 빌드가 성공했어도 setup.bash가 없으면 성공으로 표시하지 않습니다. ROS 노드 실행·정지, 추진기 구동, sudo, 파일/폴더 삭제는 포함하지 않습니다.

## 설치 위치

- 관리 원본: [`scripts/ros2_shell_helpers.bash`](../scripts/ros2_shell_helpers.bash)
- 젯슨 적용 파일: `/home/isv/.config/isv/ros2_helpers.bash`
- 자동 로드: `/home/isv/.bashrc` 끝의 `# ISV_ROS2_CB_HELPER` 블록.
- 변경 전 bashrc: `/home/isv/code_backups/shell-before-cb-20261029/.bashrc.before`.
- 기본 workspace: `~/ros2_ws`. 다른 경로가 필요하면 `ISV_ROS_WS`로 지정합니다.
- 기본 underlay: `/opt/ros/humble/setup.bash`. 필요하면 `ISV_ROS_SETUP`으로 지정합니다.

기존 startup 내용은 유지하고 자동 로드 블록만 추가했습니다. 현재 계정에서는 원래 cb 정의를 찾지 못했으므로 새 Bash 함수로 추가했습니다. 설정 파일을 source하는 동작은 모터·센서 노드를 시작하지 않습니다.

## 검증 결과

- `bash -n` 구문 검사: PASS.
- 가짜 colcon을 사용한 별도 Bash 시험: 기본 전체 빌드 호출, 인자 전달, 현재 셸 환경 반영, 공백을 포함한 custom install 경로, 실패 코드 7 전달과 환경 적용 차단 PASS.
- 실제 젯슨 `cb`: **34 packages finished [10.7s]**, 종료 코드 0.
- 실제 빌드 후 같은 셸의 setup 적용 메시지 확인.
- 새 interactive Bash에서 `cb`가 function으로 자동 등록됨을 확인.
- 빌드 뒤 원본 소스/소스 압축파일 1,279개를 스냅샷의 SHA-256과 다시 비교: 변경/누락 0개.
- 적용 helper와 저장소 원본 SHA-256 일치: `64b82379aecc4725bd12427c74b6048e413b6bc49e5b6d18981cf54b439b6057`.
- 시험 로그: `/home/isv/code_backups/cb_build_20261029.log` (로컬 보존, Git 미포함).

현재 셸 시험은 [`tests/test_cb_helper.bash`](../tests/test_cb_helper.bash)로 재현할 수 있습니다. 실제 빌드는 모든 패키지의 런타임/센서/수상 동작까지 검증하는 것은 아닙니다. 기존 코드의 메시지 세대 차이와 bringup 범위는 [전수 조사 보고서](WORKSPACE_AUDIT_2026-10-29.md)에 기록했습니다.
