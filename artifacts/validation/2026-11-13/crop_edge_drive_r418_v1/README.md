# r418 crop-edge 명시 선택 제한 주행 준비

**준비 단계, 미실행·미동결.** r417 공통 연결의 검증·root 통합과 소스 delta 검토가 끝나기 전 `sources.zip`/`prepared.json`을 만들지 않는다. 이 폴더 외 수정, WSL/ROS/Gazebo/장치 실행은 없다. 원 [r406](../final_runtime_drive_r406_v1/RESULT.md)의 자료·실패·archive는 보존한다.

## r406과 달라지는 선택

제어/scene 선택의 차이는 **`g1_crop_edge_component_trial:=true` 하나**다. `launch_adapter.SELECTION`에 이를 추가하고, expected FollowTrace의 `follow_config.crop_edge_component_trial`만 true로 추가했다. 공통 기본값은 r417의 false를 유지한다. 부모 evaluator 인자는 config/actor/output의 새 경로만 바뀐다. artifact node-entry나 동적 candidate chain은 추가하지 않는다.

- 최종 YAML의 축거128mm·윤거123.9mm·타이어폭26.1mm·후방 마커90mm/4가림, 원 `laser_frame`/`verified_only`/motion verified, native C1 10Hz를 유지한다.
- `cad_runtime_r377_v1`, 두 계산 자식, `g1_current_corner_budget=r396_v1`, 원 process/fresh/native/recheck/retained 설정과 g1_open_1p5 속도·가속·물성을 유지한다.
- 두220mm 상대·old610 시설·ID20/30 누락·road_level·80sim초·RTF0.25·접촉/노면/거리/STOP/종료 판정은 r406과 같다. actor JSON과 facility geometry helper는 바이트 동일하다. 시설 모델 이름 `eval_r406_*`도 그대로 두어 scene까지 불필요하게 바꾸지 않는다.
- `stop_owned.py`는 소유 run.py 경로와 ROS 노드 이름만 r418로 바꿨다. 원 STOP 호출·확인·실패 보존 본문은 동일하다. 8sim초 무진행/접촉/이탈 때 소유 STOP, startup150/전체1500/STOP120wall초·green 초과20sim초의 원 제한을 유지한다.

r414의 저장254 비교/후반88 개선은 필요한 이력 전체를 복원한 실주행 결과가 아니다. 이번1회는 실제 연결과 정체 이후 진행을 확인하는 별도 주행이다. 같은 원성공/실패 기준으로 판단하며 원 자료와 결과를 덮지 않는다.

## 기록과 감사

`run.py`는 기존 평가기를 호출하고 원 `run1/report.json`을 그대로 남긴다. `accepted_whole_run` 등 평가 판정은 이 감사가 수정하지 않는다. 기존 current-corner의 진단/계산 허가/부모 발행과 새 crop-edge 진단을 구분한다.

`audit_runtime.py`는 r409의 `compare_headers`를 **원문 그대로 복사한 `header_comparison.py`**에서 재사용한다. `known_structure_sectors_json` 한 필드의 표기 공백만 엄격 decode하며 중복 키·비유한 수·schema 오류·타입/배열순서/다른 값 차이를 허용하지 않는다. `header_exact`와 `configuration_equivalent`를 별도 기록한다. 원 r406의 `header_exact=false`, r409의 의미 동등 결과는 바꾸지 않는다. `candidate_effect_demonstrated`처럼 부모 발행을 주행 성과로 읽을 수 있는 새 필드는 쓰지 않는다.

r417의 진단은 일반 출력의 `normal[2].partial_observation.crop_edge_component` 등에 들어오며 별도 `applied` bool은 없다. 감사는 `call.output` 안의 실제 경로·진단 있는 판단 수·그중 양수 최종 계획 출력 수와 partial allowed 분포만 기록한다. 진단 존재, 부모 발행, 독립 guard/MCU 적용, 실제 진행은 같은 판정이 아니다. raw trace를 보존하고 최종 주행 성과는 원 report/평가 trace와 대조한다.

## r417 의존성과 동결 순서

r417 준비 경로는 `../crop_edge_runtime_r417_v1/staged`이다. 그 담당자가 FollowConfig 기본false, planner/guard ROS bool, g1_follow/simulation launch, verified simulation 환경 제한과 원 정책 본문 보존을 검증한다. r418은 그 알고리즘/저장254 비교를 반복하지 않는다. 실제 launch 경로는 `course.launch.py → 공통 g1_follow → artifact local/demo → 공통 simulation`이며, 새 인자가 빠지지 않는지 아래 preflight가 확인한다.

root 통합 이후 source inventory를 새 파일로 만들고 각 delta를 검토한다. 병렬 [r416](../runtime_installed_profile_r416_v1/integration.json)의 실장 firmware builder6파일은 archive 계보에 명시하되 시뮬 선택/동작 변경으로 분류하지 않는다. 다른 source 변화도 자동 묵인하지 않는다. r406 `registered.json` 입력 목록과 비교하므로 `added_to_snapshot`은 새 구현이라는 뜻과 다르다.

```bash
python3 -B artifacts/validation/2026-11-13/crop_edge_drive_r418_v1/prepare.py \
  --inventory artifacts/validation/2026-11-13/crop_edge_drive_r418_v1/source_review_actual_v1.json
```

root가 이 목록의 모든 경로·before/after hash를 검토한 뒤 별도 `source_review_approved_v1.json`에 `status=reviewed`와 각 행의 구체적인 `reason`을 남긴다. freeze는 목록/순서/hash가 실제 delta와 exact 일치하는지 검사한다. r417의 실제 `integration.json` 위치를 아래 인자로 지정한다. 해당 영수증의 common hash 및 r416 영수증도 확인한다. 필요한 공통/native/overlay 빌드가 끝난 뒤 최종 freeze하며, 실행 중 입력을 바꾸지 않는다.

```bash
python3 -B artifacts/validation/2026-11-13/crop_edge_drive_r418_v1/prepare.py \
  --freeze artifacts/validation/2026-11-13/crop_edge_drive_r418_v1/source_review_approved_v1.json \
  --integration artifacts/validation/2026-11-13/crop_edge_runtime_r417_v1/integration.json
```

이 단계에서만 `prepared.json`과 `sources.zip`이 새로 만들어진다. 같은 이름이 있으면 덮지 않는다. 원 r406 archive hash, 실제 common/설정/하네스/시설 입력, 승인 목록·이관 영수증을 보존한다. r417 동결 전 투영 source를 실행 승인본으로 고정하지 않는다.

## 실행 담당 명령 — root 자원 배정 후만

기존 WSL ROS Jazzy 및 root가 지정한 설치 overlay/native extension을 사용한다. ROS setup의 `PYTHONPATH`를 지우지 않는다. 이 preflight에는 시뮬·노드 시작이 없고 실제 launch 선언/치환, 양쪽 Node parameter 표현식, 실제 FollowTrace 파일 직렬화만 있다. 현재 구성은 순수10개와 launch/trace5개, 총15검사를 예상한다. 정책254개 재생/전체 repo 검사는 넣지 않는다. 외부180초 상한, skip/실패/source mismatch 때 주행으로 넘어가지 않는다.

```bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
# root가 지정한 기존 install/setup.bash도 source한다.
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
python3 -B artifacts/validation/2026-11-13/crop_edge_drive_r418_v1/prepare.py --check
python3 -m pytest -q -p no:cacheprovider \
  artifacts/validation/2026-11-13/crop_edge_drive_r418_v1/test_preflight.py \
  artifacts/validation/2026-11-13/crop_edge_drive_r418_v1/test_launch_selection.py
```

통과 후 root가 승인한1회만 같은 repository cwd에서 실행한다. `registered.json`/`run1/`이 있으면 재실행을 거절한다. 실행 중 경쟁 빌드/계산/Gazebo 부하를 넣지 않는다.

```bash
python3 artifacts/validation/2026-11-13/crop_edge_drive_r418_v1/run.py
```

소유 시험의 원8sim초 무진행 또는 접촉/이탈 종료는 아래 동일 STOP 경로를 쓴다. 이는 실패를 완료로 바꾸거나 평가 프로세스를 강제 종료하는 명령이 아니다. 기존 평가기의 STOP·차체 정착·자식 회수/잔여 PID·강제회수 여부를 확인하고 자원을 반환한다.

```bash
python3 artifacts/validation/2026-11-13/crop_edge_drive_r418_v1/stop_owned.py
```

회수: 원 `run1/report.json`, trace/follow_decisions, `runtime_audit.json`, `launch_command/launch_extra.json`, `beam_replacement/rtf_audit.json`, `early_stop.json`(해당 시), `registered/prepared/integrity.json`, archive/hash와 정상 종료 자료. 원 약10.13m 정체 통과 여부·진단/발행·접촉/이탈·STOP/종료·소스 불변을 구분한다. 한 대표를 회수한 뒤 root가 다음 미완료를 정하며 전체 조건/실시간/실차 성공으로 확대하지 않는다.
