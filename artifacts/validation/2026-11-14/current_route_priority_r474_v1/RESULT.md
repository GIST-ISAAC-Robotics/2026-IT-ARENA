# r474 현재 경로 우선 정책: 저장 입력 비교와 집중 반례

최종 v2는 저장 r469의 217개 plan에서 **원 0→양수 102, 기존 양수→0 0, 기존 양수 증가 21/감소 0**이다. 대응하는 새 plan output을 원 실제 current에 공급한 113회에서는 **0→양수 2, 양수→0 0**이다. 집중 반례 16개와 기존 STOP/세대 무효화 2개가 통과했다. 이는 저장 시뮬 입력 계산 비교이며 새 명령 발행·폐루프 완주·실차 검증은 아니다. 원 r469의 `passed=false`와 실패 로그는 변경하지 않았다.

## 입력과 버전

- 원 입력: `../solo_full_lap_r469_v1/run1/follow_decisions.jsonl`의 실제 `runtime_evaluator_header`, 제출/완료 217쌍, 실제 current 113개. generation/reset 순서와 센서/속도/epoch 수치를 보존했다.
- 후보: r466 통합 후 공통 source 480파일을 `stage`로 동결했다. marker 기본 OFF를 유지했다. `source_manifest.json`과 `final_receipt.json`에 파일별 SHA가 있다. 종료 시 stage 변경 0, 공통과 stage 차이 0, 원 trace 불변이다.
- v1은 현재 side present를 별도 veto로 남겼다. ref12.72에서 기존 .35가 0으로 변했다. 현재 baseline은 1.4m/s, 실제 swept clearance 1.28m로 양수였지만, 앞선 early return의 선택 이력과 side-present veto가 기존 GUARD 연속 조건을 끊었다. root가 presence/view veto만 제거한 v2를 검증했다. 새 세부 한계나 last-choice 예외를 추가하지 않았다.
- `source_manifest_v1.json`, `current_route_priority_v1.py`, `replay_v1/`은 v1 원 결과다. `replay_v2/`가 최종이다. 공통 구현 4파일은 root 소유이며 이 담당은 수정하지 않았다.

## 결과 구분

| 항목 | 최종 v2 |
|---|---:|
| plan 수 | 217 |
| 원 0→양수 | 102 |
| 원 양수→0 | 0 |
| 원 양수 증가 / 감소 | 21 / 0 |
| side state 변화 / handoff 변화 | 0 / 0 |
| 전체 output exact | 2 |
| 원 plan + 원 actual current | 113 |
| 위 current raw exact | 99 |
| 부모 감속 cap을 분리한 child 결과 exact | 113 |
| 새 plan + 원 actual current | 113 |
| 위 current 0→양수 / 양수→0 | 2 / 0 |

전체 output exact가 작은 이유는 새 policy/선택 진단과 기존 측방 제한 우회가 의도적으로 달라지기 때문이다. side mode와 side 이력 자체는 보존했다. ref12.72 v2는 GUARD 상태 그대로 1.4m/s, steering .0513810016rad이며 원 baseline 경로의 양수를 사용한다. 옆에 반환이 있다는 사실만으로 정지시키지 않는다는 뜻이고, 움직이는 차량에 대한 충돌 회피 보증은 아니다.

첫 정지 actual current seq823의 실측 속도는 원값 **.3673502520864255m/s**다. 원 plan .35/current 0(`released_rebase_current_body_gap_rejected`)에서 새 plan .6/current .6(`rebase_current_follow`)으로 바뀐다. 새 plan과 같은 제출 입력을 `(generation, sequence)`로 연결했으며 실측값을 .35로 자르거나 바꾸지 않았다. 실제 원 plan을 그대로 공급한 보존 검사와 구분한다.

원 actual current raw 차이 14건은 모두 `runtime_dual_state._cap_ready`의 **child 완료 뒤 부모 감속 cap** 때문이다. 기록된 `new_plan_deceleration_cap`을 분리하고 `checked_target_mps`를 원 child 속도로 대조하면 113/113 exact다. 원 raw 기록을 고치지는 않았다. `comparison_details.json`에 14개 trace seq와 원 cap이 있다. 후보 current 출력은 evaluator 결과이며 부모의 최종 발행 결과가 아니다.

정체 첫/중간/끝 ref19.22/24.32/29.32 plan은 모두 .55 양수다. 이 세 plan의 제출 입력을 그대로 current로 재검사한 국소 대표도 .55다. **해당 새 양수 뒤의 실제 후속 current와 발행은 존재하지 않으며**, 이 세 대표로 후속 주행을 재현했다고 주장하지 않는다.

## 집중 반례와 유지 경계

`test_current_route_priority.py`: **16 passed (2.86s)**.

- 실제 첫 current .36735에서 양수 재검사, 입력 불변, 독립 guard 필요/런타임 비발행 표기 유지.
- 실제 저장 scan의 정면 finite return을 .08m로 이동하면 actual occupied swept 검사에서 0(`rebase_current_gap_hold`).
- 현재 전방 ±100°의 모든 반환을 +Inf로 바꾸면 0(`current_route_front_observation_lost`).
- 경로 제거, 경로 취득시각 만료, epoch 불일치, dt=0, 실측속도 NaN은 0.
- 원 0/STOP 출력은 새 경로로 부활시키지 않는다.
- 비활성 선택, baseline 0, 과거 경계 사용, 현재 source 불일치, partial rejected는 early select 대상이 아니다.
- side present는 현재 양수 baseline을 별도 거부하지 않으며 mode/이력을 보존한다. 실제 점유가 경로를 침범할 때 차단하는 검사는 위 실제 finite-return 반례가 담당한다.

기존 `test_runtime_dual_contract.py`의 `test_generation_invalidation_clears_deceleration_cap`, `test_stop_between_selection_and_publish_cancels_pending_ack`: **2 passed (0.24s)**. 새로운 동기 writer·sensor·ROS 실행을 추가하지 않았다.

## 이미 측정된 계산시간

| 기존 기록 | n | median | max | 첫 호출 |
|---|---:|---:|---:|---:|
| r469 원 live worker 기록 | 217 | 52.554ms | 81.073ms | 42.175ms |
| r474 v1 저장 plan 재생 | 217 | 17.789ms | 67.496ms | 33.361ms |
| r474 v2 저장 plan 재생 | 217 | 18.127ms | 65.990ms | 37.004ms |

추가 성능시험은 하지 않았다. r469 live는 native source와 부하 조건도 다르므로 52.554→18.127을 정책 단독 개선률이나 Jetson 실시간 보증으로 사용하지 않는다. v1/v2는 같은 217 저장 plan을 각각 한 번 계산한 wall time이다. current 별도 시간 계측은 없다.

## 실행·실패 보존·회수

각 LF wrapper `run_replay_once.sh`, `run_replay_v2.sh`, `run_tests_once.sh`, `run_stop_tests.sh`를 WSL Ubuntu-24.04에서 한 번씩 실행했다. 각 evaluator/pytest 전경 호출은 exit 0이다. 계산 준비 중 Windows cp949 decode 오류와 미생성 wrapper 호출 exit 1은 `preparation_errors.json`에 보존했다. 명시 UTF-8로 수정했고 실패 호출에서는 evaluator가 시작되지 않았다.

최종 `/proc` 회수 wrapper 진입은 두 번 모두 `Wsl/Service/E_UNEXPECTED`, exit 1로 실패했다. 따라서 **전경 작업 정상 종료는 확인했으나 최종 PID0은 확인하지 못했다.** background/ROS/Gazebo/Jetson은 시작하지 않았고 임의 WSL 재시작도 하지 않았다. 이 제한을 root에 전달하고 WSL 소유권을 반환했다. `resource_return.json`에 조회 미확인 상태를 남겼으며 `final_receipt.json`의 Windows 해시 확인으로 입력 불변을 별도로 확인했다. 이후 실행 여부와 WSL 복구는 root가 맡는다.

## 검토 자료

- 최종 결과: `replay_v2/summary.json`, `plan_rows.json`, `new_plan_actual_current_rows.json`, `legacy_current_rows.json`, `same_input_current.json`
- 비교 분리/기존 시간: `comparison_details.json`; 재집계는 `python -B summarize.py`이며 evaluator를 실행하지 않는다.
- source/입력/종료: `source_manifest.json`, `final_receipt.json`, `resource_return.json`
- 집중 시험: `test_current_route_priority.py`, `pytest_v1.log`, `pytest_stop_v1.log`

현재 확인한 범위에서 최종 v2의 추가 blocker는 발견하지 않았다. 새 문턱이나 예외를 늘리지 않고, 준비된 r475가 실제 진행·가속·접촉·STOP과 종료를 확인할 다음 단계다.
