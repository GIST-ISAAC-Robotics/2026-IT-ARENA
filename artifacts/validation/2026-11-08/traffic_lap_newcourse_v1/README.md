# 새 코스 교통 한 랩 시험 v1 — 실행 결과

**결론: 실패.** 등록된 1회 실행(`two_actor_course_lap/run1`)에서 자차는 첫 코너를 나온 직후 중심선 진행 **10.90 m**(목표 44.64 m)에서 멈췄습니다. 이후 시험이 끝날 때까지 **75.0 s 동안 정지**했습니다. 시험 끝 무렵에는 각본으로 움직이는 두 번째 상대 `eval_second_lead`가 한 바퀴를 돌아 뒤에서 정지한 자차와 **겹쳤고 접촉이 기록**되었습니다. 하네스·입력 무결성·노면·정지·종료 검사는 통과했습니다. 따라서 이번 결과는 측정 실패가 아니라 주행 판정 실패입니다. 이 결과를 한 랩 완주, 전체 경기, 실차 성공으로 해석하지 않습니다.

계획과 사전 목표는 [PLAN.md](PLAN.md)에 있습니다. 이 결과에는 `src/`, `scripts/`, `config/`, 제어기·물성·판정 변경이 없습니다. 실행 래퍼도 수정하지 않았습니다.

## 1. 실행한 명령과 종료값

| 단계 | 명령 | 종료값 / 근거 |
| --- | --- | --- |
| 전체 | `wsl.exe -e bash artifacts/validation/2026-11-08/traffic_lap_newcourse_v1/run_live.sh` (저장소 루트, 1회) | **1** (`run_live.outer_exit.txt`) |
| 선행 회귀 확인 | `grid_pack_newcourse_v1/regression.exit` = 0 | 통과 |
| 기하 검사 | `python3 -B geometry.py` (`environment.sh` 경유) | 0 (`geometry_v1.exit.txt`, `geometry.json`) |
| 소스 등록 | `python3 -B register.py` | 0 (`register.exit.txt`, `registered 880`) |
| 주행·평가 | `run.sh two_actor_course_lap` → `run.py` → `scripts/validate_g1_traffic.py` | `HARNESS_EXIT=1 SURFACE_EXIT=0 INTEGRITY_EXIT=0 LEFTOVER_PGREP=1 ASSESSMENT_EXIT=1 WALL_S=1152` (`two_actor_course_lap/run1.exit`) |

- 실행 전 `pgrep` 점검에서 `gz sim`/`ros2 launch` 프로세스는 없었고(`preflight_processes_run1.txt` 0 B), 실행 후에도 남은 프로세스가 없었습니다(`postflight_processes_run1.txt` 0 B, `LEFTOVER_PGREP=1`).
- `HARNESS_EXIT=1`은 검사기가 `passed=false`로 끝났다는 뜻입니다. 같은 report의 `harness_valid=true`, `run_valid=true`, `error=null`, `abort=null`이므로 실행기 오류가 아닙니다.
- 시뮬레이션은 1회만 실행했고 재실행하지 않았습니다. 등록 시각은 `2026-11-07T20:21:54Z`입니다.

## 2. 등록 소스 불변

- 등록 입력은 880개이며 `src`, `scripts`, `config`, 이 폴더의 helper가 포함됩니다. `snapshot_sha256=1a48c7e0…4cfb`입니다.
- 실행 후 무결성 검사 `postflight_integrity_run1.json`의 결과는 `{"inputs": 880, "changed": []}`입니다.
- report에서도 `source_unchanged=true`, `integrity.source_unchanged=true`가 확인됩니다.

## 3. 원 report와 보충 assessment 대조

창: green `7.401` s부터 end `103.402` s까지 96.0 s입니다. 실행 조건은 `official_v2026_11_06`, G1/G2, B0390, JR60P20, 단일 모터축 28카운트·8.970588:1·65 mm, `g1_open_1p5`, `medium_v1`입니다.

| 항목 | 원 report (`run1/report.json`) | 보충 (`assessment_run1.json`) | 판정 |
| --- | --- | --- | --- |
| 진도 | `progress.at_window_end_m=10.90`; `full_lap=false` | `course.progress_m=10.90`(s 41.66→52.56), `one_lap=false` | **실패** |
| 지속 정지 | `longest_stationary_while_lead_moves_s=75.045`, `no_permanent_stall=false` | `longest_low_speed_s=75.045`, `sustained_motion=false` | **실패** |
| 최고속도 | — | `peak_mps=1.338`(trace 약 10 Hz 표본) | 기록 |
| 랩 | 0랩 | 0랩 | **실패** |
| 주 상대 `eval_g1b_actor` 간격 | 표본 최소 0.530 m, swept 하한 0.529 m, overlap 0 | `minimum_gap/swept_gap/no_overlap=true` | 통과(80 mm·150 mm 모두) |
| 주 상대 접촉 | `actor_contact_messages_with_ego=0`, `no_actor_contact=true` | — | 통과(event-only) |
| **추가 상대 `eval_second_lead`** | `harness.valid=true`; gates `registered_gap/complete_gap_samples/complete_swept_intervals=true`, **`no_ego_contact=false`, `sampled_gap=false`, `swept_gap=false`**; 최소 0.0 m, swept −0.0003 m; `behavior_gates_passed=false` | `additional_valid=false`, `no_contacts=false` | **실패** |
| 추가 상대 접촉 | `combined_contacts.valid=true`, raw 1498, ego events 6874, 첫 접촉 **102.867 s**, 마지막 104.364 s(창 끝과 정지 요청 이후까지 지속); 차체·네 바퀴 모두 접촉 | — | **실패** |
| 최종 간격 | `final_gap=false`(창 끝 주 상대 간격 4.67 m) | — | 원 판정 실패 |
| 도로 밖 차체 | `road.outside_samples=0`, wall 0 | — | 통과 |
| 노면(최대 도로밖 바퀴) | `two_wheel_surface=true` | surface `samples=51290`, `max_off_wheels` any/center/full = **0/0/0**, `wall_wheel_samples=0` | 통과 |
| 신호 | `official_signal_evidence.passed=true`; green 적용 7.400 s, 첫 출발 7.677 s; `false_start=false` | — | 통과 |
| 정지(STOP) | `stop_checks` 6항목 모두 true; `stop_request_sim_s=103.843`; final MCU `ESTOP_LATCHED/software_stop`; 조밀 자세 증거 0.5 s 완전, 최대 0.0017 m/s | `stop_and_shutdown=true` | 아래 해석 참고 |
| 정상 종료 | `shutdown_clean=true`, `process_exit_codes=[0,0,0,0,0]`, `remaining_test_pids=[]` | `process_clean=true` | 통과 |
| 하네스 | `harness_valid=true`(명령 4500, 최대 간격 0.023 s, 추종 오차 0.012 m), 두 상대 모두 | `valid=true` | 통과 |
| 최종 | `passed=false`, `behavior_gates_passed=false` | `passed=false` | **실패** |

**STOP 해석:** report에는 `failure_cleanup_stop` 키가 없습니다. 정지 요청(103.843 s)은 실패 때문에 중간에 일으킨 정리 정지가 아니라, 창이 끝난 뒤(end 103.402 s) 예정대로 보낸 시험 종료 STOP입니다. 그렇다고 이 STOP을 **"정상 완주 STOP"으로 해석하면 안 됩니다.** 자차는 그 전에 이미 75 s 동안 정지해 있었고, 추가 상대와의 접촉도 STOP 이전인 102.867 s에 시작되었습니다. 결국 이번 STOP은 정지·접촉 실패가 이미 발생한 상태에서 수행된 예정 종료 정지입니다. 정지·종료 계통 자체는 정상 동작했습니다.

**과거 5대 출발 사례와의 관계:** 이번 시험의 초기 간격은 주 상대 1.40 m, 추가 상대 7.40 m입니다(`geometry.json`). 과거 5대 출발 그리드에서 문제가 된 30 mm 초기 간격 조건은 이번에 해당하지 않습니다. 이번 추가 상대 gate 실패는 출발 간격이 아니라 아래 2절의 랩 겹침 때문에 발생했습니다.

## 4. 첫 중요 실패와 원인(저장 입력 기준)

1. **정지 시작:** 자차는 1.5 m/s 교통 속도 상한 아래에서 주 상대를 0.5 m/s 전후로 뒤따랐습니다. 첫 왼쪽 코너(약 s 6.3, 위치 x≈9.8, y≈13.2)를 돈 뒤 **t=28.12 s**, s=7.92(x=8.585, y=13.194, yaw −3.086)에서 멈췄습니다. 이후 창이 끝날 때까지 다시 움직이지 않았습니다. 그 직전 코너 진입부(24.88–25.18 s, s≈6.2)에도 약 0.4 s의 짧은 HOLD(trace `HANDOFF_FOLLOW/HOLD`)가 있었으나, 그때는 곧바로 FOLLOW로 복귀했습니다.
2. **제어 원인:** `follow_decisions.jsonl`에서 28.12 s 이후의 모든 호출(3816회)은 `behavior=HOLD`, `reason=follow_boundary_unconfirmed`였고, FOLLOW 복귀는 0회였습니다. 같은 구간의 경계 후보 사유는 `boundary_candidate_reason=wall_segment_short`(764회)였고, 조각 경계 증거는 `fragmented_boundary_evidence.reason=expected_wall_folded`(763회), `boundary_selection=unfiltered`였습니다. 직전에는 `bilateral_observed_corner_distance`와 `observed_history_support`로 경계를 잡고 있었습니다. 즉 코너를 빠져나온 직후 추종 경계를 확인하지 못해 정지한 상태가 해제되지 않았습니다.
   - 같은 시간 보호층은 `safety_reason=clear`(clearance 0.96 m)였고 `/drive`가 0이었습니다. 따라서 보호층 개입이 아니라 상위 제어기가 정지를 명령한 것입니다. MCU는 ACTIVE→ARMED로 바뀌었습니다.
   - 주 상대는 이 시점 0.55 m 앞에 있었고, 이후 계속 멀어져 창 끝에는 4.67 m 앞에 있었습니다. 상대가 길을 막아서 생긴 정지가 아닙니다.
   - `corridor_candidate.reason=not_consistent_inward_corner`이며, `selector_mode=SOLO`입니다.
3. **추가 상대 접촉의 경위(각본 설계 한계):** `eval_second_lead`는 s 49.26→97.26으로 48 m를 진행합니다. 97.26 − 44.64 = **52.62**이므로, 한 바퀴를 돈 끝 지점이 정지한 자차의 s 52.56과 겹칩니다. 이 상대는 무한질량 set_pose 상자라서 정지한 자차를 뒤에서 통과하며 102.867 s부터 접촉했습니다.
   - `geometry.py`는 상대–도로/벽, 상대끼리의 겹침, 초기 간격만 검사합니다. 자차가 크게 지체될 때 1.2랩 각본이 자차 위치를 덮는 경우는 확인하지 않습니다.
   - 이 접촉은 자차의 능동 충돌이 아니라 정지 실패에서 파생된 결과입니다. 다만 등록 판정에 따라 접촉·간격 gate 실패로 그대로 기록합니다.

## 5. 한계

- 단일 seed 0, 시뮬레이션 1회입니다. 실차, 전체 경기, 5랩 결과가 아닙니다. 각본 상대는 차량 동역학이 없는 이동 상자입니다.
- 접촉 증거는 event-only이며, 메시지가 없다고 해서 연속적인 무접촉이 증명되지는 않습니다.
- 최고속도는 trace 약 10 Hz 표본 기준이며 정밀 최대값이 아닐 수 있습니다.
- 4절 원인 분석은 저장된 결정 기록을 읽은 것입니다. `wall_segment_short`/`expected_wall_folded`가 왜 해제되지 않는지 코드 수준 원인이나 재현 확인은 수행하지 않았습니다(추측 없이 미확인으로 남깁니다).

## 6. 다음 확인점(판단은 GPT 주 에이전트)

- s≈7.9 코너 출구의 `follow_boundary_unconfirmed` HOLD 지속 문제입니다. 저장 입력(`follow_decisions.jsonl` 28.0–28.2 s 전후 호출)을 replay해 경계 확인 조건과 해제 경로를 분리해야 합니다. 이 위치에서 상대 추종이 없는 단독 주행도 같은 HOLD에 걸리는지 구분할 필요가 있습니다.
- 다음 등록에서는 기하 검사 범위를 정해야 합니다. 예: 1.2랩 각본 상대가 지체·정지한 자차 위치를 덮는 경우를 사전에 다룰지, 또는 상대 진행량과 랩 겹침을 제한할지입니다.
- 원 뒤추종 판정(`final_gap` 등)과 대표 목표는 계속 구분합니다.

## 7. 원시 근거

| 파일 | sha256 |
| --- | --- |
| `two_actor_course_lap/run1/report.json` | `a565c35c16c53aab67ddac3fcafd7f1563d91d5d0da6e9c92dc684c1ee6c6c16` |
| `two_actor_course_lap/assessment_run1.json` | `0c7748b57d841ee39e5aea739915071954ef898d593278292bbef92fe3720040` |
| `two_actor_course_lap/surface_run1.json` | `41b65789480505804aae665956647297f576560dc0ce6e24adbc47ffd89bed35` |
| `two_actor_course_lap/registered.json` | `3a0f9b7378b3c596911b3bcf7d756c57d9618af42e13a9ccbe87801dc06d1016` |
| `geometry.json` | `ff087f498ff65dabd2a8140521d2f15f68a646ede1c3b57feb5fd319f03fe537` |

그 밖의 원시 기록은 `run1/trace.json`, `run1/follow_decisions.jsonl`(약 158 MB, 5220줄), `run1/autonomy_status.jsonl`, `run1/actor_poses.jsonl`, `run1/ego_poses.jsonl`, `run1/stop_trace.json`, `run1/simulation.log`, 그리고 `run1_console.log`, `surface_run1.log`, `assessment_run1.log`, `geometry_v1.log`, `register.log`입니다. 대용량 원시 파일의 Git 제외 정책은 기존 규칙을 따릅니다.
