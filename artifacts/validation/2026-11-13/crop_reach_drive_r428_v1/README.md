# r428 동일 조건의 current crop reach 제한 주행 준비

**하네스 준비 전용. 최종 source freeze·preflight·주행은 아직 하지 않았다.** r426의 29검사/313저장 current 비교와 공통5파일 exact 이관 이후, r418과 같은 조건에서 제한 주행을 딱1회 준비한다. 원 r418의10.10m 정체·준비 오류·archive·평가 결과는 수정하지 않는다. 새 후보의 저장 양수127→237은 부모 발행이나 실제 진행 성공이 아니다.

## 유지하는 조건과 유일한 제어 변경

- 최종 vehicle YAML: 축거128mm·윤거123.9mm·타이어폭26.1mm, 후방 marker90mm/4mask, C1 X75.4/Z134.6mm·native10Hz/원 laser_frame/verified-only.
- r418과 같은 `cad_runtime_r377_v1`, `g1_crop_edge_component_trial=true`, `g1_current_corner_budget=r396_v1`, 기존 process/fresh/native/retained 설정. 새 launch 옵션은 없다.
- old610 시설, blank20/30, 두220mm 상대, road_level, g1_open_1p5, 80sim초, RTF0.25, 차량·센서 물성/속도/접촉/간격/노면/STOP/종료 판정은 동일하다. actor·expected header·launch selection·시설 geometry·header comparator는 r418 바이트 그대로다. 시설 모델 이름 `eval_r406_*`도 유지한다.
- 제어 행동 변경은 [r426 integration](../crop_current_reach_r426_v1/integration.json)의 공통5파일뿐이다. parent/evaluator 2파일 변경과 crop scope/candidate/proof 3파일 추가다. r423 세 SOLO 게이트 확장, 임계값·TTL 완화는 포함하지 않는다. 상세 위험 범위와 실물 미적용은 [r426 RESULT](../crop_current_reach_r426_v1/RESULT.md)를 따른다.

`prepare.py`는 r418 동결 common 목록과 현재 전체 목록의 delta가 위5파일 base/staged 해시와 exact 일치해야 한다. 나머지 common은 동일해야 하며 추가 변화·삭제·binary 차이를 자동 승인하지 않는다. `source_review_actual_v1.json`은 승인 전 목록이고 root가 별도 승인표에 모든 경로·해시·이유를 확인한다. firmware builder는 r418에 이미 포함된 원문을 유지한다.

artifact 차이는 새 경로/설명, r426 계산과 부모 발행의 별도 감사, STOP 관측 분리다. `run1/report.json`/accepted_whole_run 등 원 평가 판정을 바꾸지 않는다. header 원문 exact와 구조JSON strict-decode 동등을 별도로 기록한다. 처음부터 r418 `preflight_revision_v3`의 indent fixture를 사용해 compact→compact를 차이라고 기대한 이전 오류를 반복하지 않는다. freeze는 integration/review CLI 경로를 먼저 absolute resolve한다.

## 순서: build → source 승인·freeze → preflight → 주행1회

1. **빌드:** 실행 담당만 `build_v1/`에 기록한다. repo root/기존 install overlay에서 `colcon build --symlink-install --packages-select arena_autonomy arena_vehicle_interface arena_bringup`을 한 번 수행한다. `build.exit=0`, build.log 및 import 경로를 회수한 뒤 다음 단계로 간다. 이 하네스 준비 담당은 WSL 빌드/ROS를 실행하지 않는다.
2. **source 검토:** 빌드 후 `prepare.py --inventory`로 delta를 기록한다. root가 승인표의 `status=reviewed`와 각 `reason`을 작성한다. 목록이 이미 있으면 같은 경로에 덮지 않는다. 목록과 실제 source가 달라지면 새 승인표로 검토한다.
3. **freeze:** root 승인 후 아래 명령으로 `prepared.json`/`sources.zip`을 새로 만든다. build 완료 및 정확한5파일 delta를 다시 확인한다. **freeze 전 preflight 금지.** 최종 source/입력은 이때 동결하며 이후 common/하네스를 수정하지 않는다.
4. **preflight:** ROS setup/기존 install을 source하고 `prepare --check` 이후 같은16검사를 한 번 수행한다. 실제 launch 선언/양 node flag 전달과 FollowTrace 직렬화만 확인하며 노드·시뮬은 시작하지 않는다. 정책313/plan184를 다시 재생하지 않는다. skip/실패/소스 변경이면 주행하지 않는다.
5. **주행:** root의 자원 배정·승인 후 아래 run.py를1회만 실행한다. registered/run1이 있으면 재실행을 거절한다. 성공까지 자동 반복하지 않는다.

```bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
# 빌드 완료 후 inventory가 아직 없을 때만 실행:
python3 -B artifacts/validation/2026-11-13/crop_reach_drive_r428_v1/prepare.py --inventory artifacts/validation/2026-11-13/crop_reach_drive_r428_v1/source_review_actual_v1.json
# root가 actual 목록을 검토하여 별도 approved 파일을 작성한 다음:
python3 -B artifacts/validation/2026-11-13/crop_reach_drive_r428_v1/prepare.py --freeze artifacts/validation/2026-11-13/crop_reach_drive_r428_v1/source_review_approved_v1.json --integration artifacts/validation/2026-11-13/crop_current_reach_r426_v1/integration.json
```

```bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
python3 -B artifacts/validation/2026-11-13/crop_reach_drive_r428_v1/prepare.py --check
mkdir artifacts/validation/2026-11-13/crop_reach_drive_r428_v1/preflight_v1
timeout --signal=TERM --kill-after=10s 180s python3 -B -m pytest -q -p no:cacheprovider artifacts/validation/2026-11-13/crop_reach_drive_r428_v1/test_preflight.py artifacts/validation/2026-11-13/crop_reach_drive_r428_v1/test_launch_selection.py --junitxml=artifacts/validation/2026-11-13/crop_reach_drive_r428_v1/preflight_v1/junit.xml 2>&1 | tee artifacts/validation/2026-11-13/crop_reach_drive_r428_v1/preflight_v1/pytest.log
python3 -B artifacts/validation/2026-11-13/crop_reach_drive_r428_v1/prepare.py --check
```

exit0/16pass/skip0 및 source check를 회수한 뒤 root가 다음1회를 배정한다.

```bash
python3 artifacts/validation/2026-11-13/crop_reach_drive_r428_v1/run.py
```

## 실행 중 종료와 회수

원 startup150wall초/전체1500wall초/STOP120wall초와 green 이후20sim초 margin을 유지한다. 실행 담당은 `run1/trajectory.jsonl`의 실제 진행·접촉·이탈과 소유 process를 관찰한다. r418과 같은 **8sim초 무진행**, 접촉/이탈 또는 원 종료 조건이면 아래 소유 STOP helper를 한 번 호출하고 원 evaluator의 STOP·정착·자식 회수를 기다린다. 진단을 위해 CPU 부하를 추가하거나 RTF/속도를 바꾸지 않는다.

```bash
python3 artifacts/validation/2026-11-13/crop_reach_drive_r428_v1/stop_owned.py
```

STOP helper는 r418의 소유 run.py/ROS node 식별자만 바꿨다. `/autonomy/enable false → /actuation/stop`의 ACK·마지막 MCU/출력을 `early_stop.json`에 남긴다. helper의 정확한 service 요청시각은 원 helper가 기록하지 않으므로 새로 추정하지 않는다. `stop_sequence.json`은 helper의 관측과 **원 evaluator**의 stop_request_clock_ns/stop_latch_timing/stop_checks를 나란히 보존한다. helper 이후 evaluator보다 먼저 보이는 software_stop 래치를 원인 MCU 고장으로 단정하지 않으며 원 `no_mcu_latch_before_stop=false`를 성공으로 덮지 않는다. 서로 다른 clock 수신 시점만으로 정확한 인과를 증명하지 않는다.

회수 자료는 원 report/trajectory/trace/follow_decisions/actuation_trace/stop_trace, runtime_audit/stop_sequence/early_stop, launch_command/extra, beam/RTF 감사, registered/prepared/integrity/archive, 자식 exit/강제회수/잔여PID다. 새 current 계산 수락·인접 부모 발행·guard/MCU·실제 진행을 분리한다. 같은 첫 U자코너에서 다시 멈추면 그대로 실패를 보존하고 다음1회를 자동 시작하지 않는다. 한 대표 후 root가 다음 미완료를 결정한다. 실차나 전체 조건 보장이 아니다.

원시 센서·상세 결과/ZIP의 기존 Git 제외 정책을 따른다. `build_v1/`은 실행 담당 전용이며 준비 담당이 기록을 수정하지 않는다.
