# 교통 한 랩 r21(관측 경로 예산 보완) 실행 결과

**결론: 실패.** 등록된 1회 실행(`two_actor_course_lap/run1`)에서 자차는 이전 시험이 멈춘 첫 코너 뒤 10.90 m 지점을 **멈추지 않고 통과**했습니다. 그러나 다음 코너에 들어가던 **18.86 m**(목표 44.64 m)에서 다시 멈췄습니다. 그 뒤 창이 끝날 때까지 **59.33 s 동안 정지**했습니다. 접촉과 겹침은 0입니다. 하네스·입력 무결성·노면·STOP·정상 종료 검사도 통과했습니다. 따라서 이 결과는 실행 실패가 아니라 **주행 판정 실패**입니다. 한 랩 완주, 전체 경기, 실차 성공으로 해석하지 않습니다.

계획은 [PLAN.md](PLAN.md)를 따릅니다. 이전 시험 [traffic_lap_newcourse_v1](../traffic_lap_newcourse_v1/README.md)의 결과는 바꾸지 않습니다. 변경은 [corner_path_budget_v1](../corner_path_budget_v1/README.md)의 r21(`local_path.py`) 하나입니다. 이번 작업에서 `src/`·`scripts/`·`config/`·`tests/`·helper는 수정하지 않았습니다.

## 1. 실행과 종료값

| 단계 | 명령 / 근거 | 결과 |
| --- | --- | --- |
| 전체 | `wsl.exe -e bash artifacts/validation/2026-11-08/traffic_lap_budget_v1/run_live.sh`(저장소 루트) | `run_live.outer_exit.txt`: `OUTER_EXIT=1` |
| 기하 | `geometry.py` | `geometry_v1.exit.txt` 0, `geometry.json` |
| 등록 | `register.py` | `register.exit.txt` 0, `registered 881` |
| 주행·평가 | `run.sh two_actor_course_lap` | `HARNESS_EXIT=1 SURFACE_EXIT=0 INTEGRITY_EXIT=0 LEFTOVER_PGREP=1 ASSESSMENT_EXIT=1 WALL_S=1152` (`two_actor_course_lap/run1.exit`) |

- `HARNESS_EXIT=1`은 `passed=false` 때문입니다. report에서 `harness_valid=true`, `run_valid=true`, `error=null`, `abort=null`이 확인되므로 실행기 오류가 아닙니다.
- 실행 전후 `pgrep` 결과는 모두 0 B였습니다(`preflight/postflight_processes_run1.txt`). 남은 프로세스는 없습니다.
- 실행과 대기는 Claude Haiku 5.5 보조가 맡았습니다(자체 보고 `claude-haiku-5-5`).
- **중단된 첫 기동(시뮬레이션 미시작):** 06:19 첫 기동에서 실행 셸이 보조 에이전트의 턴 종료와 함께 끊겼습니다. 이때 남은 것은 0 B `geometry_v1.log`뿐이었습니다. exit 파일, 등록, `two_actor_course_lap/`, Gazebo는 모두 생기지 않았습니다. WSL 안에 남은 프로세스가 없음을 확인했고 `run1`도 없었으므로, 06:48에 같은 명령을 한 번 다시 기동했습니다. 0 B `geometry_v1.log`는 재기동이 덮어썼습니다. 주행 시도로는 위 1회만 존재합니다.

## 2. 출처 불변

- 등록 입력은 881개입니다(이전 880개에 r21 출처 계약 1개가 추가됨).
- `postflight_integrity_run1.json`은 `{"inputs": 881, "changed": []}`입니다.
- report에서도 `source_unchanged=true`가 확인됩니다. 실행 사본은 `run1/source_snapshot.zip`에 있습니다.

## 3. 판정 대조

창은 green 7.401 s부터 end 103.401 s까지 96.0 s입니다.

| 항목 | 원 report | 보충 `assessment_run1.json` | 판정 |
| --- | --- | --- | --- |
| 진도 | `progress.at_window_end_m=18.86`(s 41.66→60.52), `full_lap=false` | `one_lap=false` | **실패**(이전 10.90 m보다 7.96 m 더 진행) |
| 지속 정지 | `no_permanent_stall=false` | `longest_low_speed_s=59.33`(43.81 s~창 끝), `sustained_motion=false` | **실패** |
| 0.5 s 넘는 저속 구간 | — | 출발 뒤 1회뿐(44.01 s부터). 이전 정지 지점(28.1 s, progress 10.88 m)은 0.24→0.60 m/s로 통과 | 10.90 m 탈출 |
| 최고속도 | — | 1.338 m/s | 기록 |
| 주 상대 간격 | 표본 최소 0.519 m, swept 하한 0.518 m, overlap 0, 접촉 메시지 0 | `minimum_gap/swept_gap/no_overlap=true` | 통과 |
| 추가 상대 `eval_second_lead` | harness valid, gates 모두 true, 최소 1.680 m, `behavior_gates_passed=true` | `additional_valid=true` | 통과 |
| 접촉 | `combined_contacts`: raw 0, events 0; `actor_contacts.jsonl` 0 B | `no_contacts=true` | 통과(event-only. 메시지 부재가 무접촉의 증명은 아님) |
| 최종 간격 | `final_gap=false`(창 끝 주 상대 간격 2.83 m) | — | 원 판정 실패 |
| 도로·노면 | `road.outside_samples=0`; surface 51,422 표본, 도로밖 바퀴 0, wall 0 | `surface=true` | 통과 |
| 중심선 오차 | 창 안 최대 0.155 m | — | 기록 |
| STOP | `stop_checks` 6항목 true; `stop_request_sim_s=103.841`; `failure_cleanup_stop` 없음; 변위 1.5e-5 m | `stop_and_shutdown=true` | 예정 종료 STOP |
| 정상 종료 | `shutdown_clean=true`, exit codes `[0,0,0,0,0]`, `remaining_test_pids=[]`, Gazebo 정지 서비스 확인 | `process_clean=true` | 통과 |
| 최종 | `passed=false` | `passed=false` | **실패** |

**STOP 해석.** 정지 요청은 창 끝(103.401 s) 뒤에 보낸 예정 종료 STOP입니다. 실패 때문에 중간에 보낸 정리 정지는 아닙니다. 다만 자차는 그 전에 이미 59 s 동안 정지해 있었으므로 이 STOP을 "정상 완주 STOP"으로 해석하지 않습니다.

## 4. 새 정지 지점 (집계만, 원인 조사는 별도)

- **위치·시각:** 첫 HOLD 판단은 43.72 s입니다. 자차는 43.81 s에 `drive=0`이 되었고, 43.91 s 이후 s=15.88(x 7.82, y 6.14, yaw −2.16)에 정지했습니다. 직전 42.1–43.7 s에 yaw가 −1.57에서 −2.06으로 바뀌고 있었으므로 **다음 코너 진입 중**이었습니다.
- **보호층·상대:** 보호층은 `safety_reason=clear`(588/588 trace 표본)이고 `/drive`가 0이었습니다. 즉 보호층 개입이 아니라 상위 제어기의 정지입니다. MCU 상태는 ACTIVE→ARMED였습니다. 주 상대는 정지 시점 0.72 m 앞에 있었고, 창 끝에는 2.83 m 앞으로 멀어졌습니다.
- **판단 사유(43.82 s 이후 3,031 call):**
  - 양수 속도는 0회이고, 전부 `handoff_follow|follow_boundary_unconfirmed`였습니다.
  - 새 스캔 607회의 사유 분포는 다음과 같습니다. `boundary_candidate_reason=local_path`(선호 왼쪽 벽 후보 ahead 약 2.22 m)이지만 `boundary_selection=unfiltered`, `fresh_boundary_accepted=false`였습니다. `fragmented_boundary_evidence.reason=anchor_not_observed`, `corner_distance_evidence.reason=not_consistent_inward_corner`, `corridor_candidate.reason=not_consistent_inward_corner`(3,031회)였습니다.
  - 그 밖의 값은 `selector_mode=SOLO`, `selector_reason=baseline_not_normal`입니다.
- **직전 흐름:** 43.62 s에는 `observed_history_support`(후보 `wall_segment_short`)로 0.55 m/s 주행 중이었습니다. 43.72 s부터 경계를 확정하지 못했습니다.
- 이전 시험의 정지는 "반대 벽 코너 근거 0.628 < 0.65 m"였습니다. 이번 정지는 사유 구성이 다릅니다(선호 벽 후보는 있으나 미채택, 코너 증거는 `not_consistent_inward_corner`). 왜 미채택되었는지는 이 보고서에서 조사하지 않았습니다.

## 5. 범위와 한계

- 폐루프 1회입니다. 각본 상대는 자율주행 차량이 아니라 이동 상자이며, 실차 경기 증명이 아닙니다.
- 같은 실패를 자동으로 재실행하지 않았습니다. 다음 원인 조사와 수정은 별도 작업입니다.
- `follow_decisions.jsonl`(162 MB) 전체를 출력하지 않고, 위 사유 수만 스트리밍으로 집계했습니다.
