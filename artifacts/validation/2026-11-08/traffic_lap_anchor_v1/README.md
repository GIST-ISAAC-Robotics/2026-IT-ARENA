# 교통 한 랩 r22(코너 경로 앵커 보완) 실행 결과

**결론: 실패.** 등록된 1회 실행(`two_actor_course_lap/run1`)에서 자차는 **18.98 m**(목표 44.64 m)까지 진행한 뒤 멈췄습니다. 같은 조건의 r21(18.86 m)보다 0.12 m만 더 진행한 것이며, 18.86 m 지점의 정지를 **실질적으로 넘었다고 보기 어렵습니다.** 정지 뒤 창이 끝날 때까지 **59.09 s 동안 저속**이었습니다.

접촉과 겹침은 0입니다. 하네스·입력 무결성·노면·STOP·정상 종료 검사도 통과했습니다. 따라서 실행 실패가 아니라 **주행 판정 실패**입니다.

계획은 [PLAN.md](PLAN.md)를 따릅니다. 변경은 [corner_path_anchor_v1](../corner_path_anchor_v1/README.md)의 r22(`local_path.py`·`local_follow.py`) 두 파일입니다. 이번 작업에서 `src/`·`scripts/`·`config/`·`tests/`·helper·docs는 수정하지 않았습니다. 원인 조사, 코드 수정, 재실행도 하지 않았습니다.

## 1. 실행과 종료값

| 단계 | 명령 / 근거 | 결과 |
| --- | --- | --- |
| 전체 | `wsl.exe -e bash artifacts/validation/2026-11-08/traffic_lap_anchor_v1/run_live.sh`(저장소 루트, 1회) | `run_live.outer_exit.txt`: `OUTER_EXIT=1` |
| 기하 | `geometry.py` | `geometry_v1.exit.txt` 0 |
| 등록 | `register.py` | `register.exit.txt` 0, `registered 882` |
| 주행·평가 | `run.sh two_actor_course_lap` | `HARNESS_EXIT=1 SURFACE_EXIT=0 INTEGRITY_EXIT=0 LEFTOVER_PGREP=1 ASSESSMENT_EXIT=1 WALL_S=1159` (`two_actor_course_lap/run1.exit`) |

- `HARNESS_EXIT=1`은 `passed=false` 때문입니다. report에서 `harness_valid=true`, `run_valid=true`, `error=null`, `abort=null`, `process_exit_codes=[0,0,0,0,0]`가 확인되므로 실행기 오류가 아닙니다.
- 실행 전후 `pgrep` 파일은 모두 0 B였습니다. `LEFTOVER_PGREP=1`은 일치하는 프로세스가 없다는 뜻입니다. `remaining_test_pids=[]`, `shutdown_clean=true`입니다.
- 실행과 대기는 Claude Haiku 5.5 보조가 맡았습니다. 이 보조는 foreground 에이전트 호출 안에서 결과 파일이 생길 때까지 대기했습니다(자체 보고 `claude-haiku-5-5`). 기동은 1회뿐이고, 중단·재기동은 없었습니다.

## 2. 출처 불변

- 등록 입력은 882개입니다. r21의 881개에 r22 출처 계약 1개가 추가되었습니다.
- `postflight_integrity_run1.json`은 `{"inputs": 882, "changed": []}`입니다.
- report에서도 `source_unchanged=true`가 확인됩니다. 실행 사본은 `run1/source_snapshot.zip`에 있습니다.

## 3. 판정 대조

창은 green 7.401 s부터 end 103.401 s까지 96.0 s입니다.

| 항목 | 원 report | 보충 `assessment_run1.json` | 판정 |
| --- | --- | --- | --- |
| 진도 | `progress.at_window_end_m=18.98`(s 41.66→60.64), `full_lap=false`, projection jump 0 | `one_lap=false` | **실패**(r21 18.86 m 대비 +0.12 m) |
| 지속 정지 | `no_permanent_stall=false`, `longest_stationary_while_lead_moves_s=59.09` | `longest_low_speed_s=59.09`, `sustained_motion=false` | **실패** |
| 최고속도 | — | 1.338 m/s(r21 1.338) | 기록 |
| 주 상대 간격 | 표본 최소 0.519 m, swept 하한 0.518 m, overlap 0, 접촉 메시지 0 | `minimum_gap/swept_gap/no_overlap=true` | 통과(하한 80 mm 초과) |
| 최종 간격 | `final_gap=false`(창 끝 2.97 m) | — | 원 판정 실패 |
| 추가 상대 `eval_second_lead` | harness valid, 등록 표본 최소 1.68 m | `additional_valid=true` | 통과 |
| 접촉 | 자차와의 actor contact 메시지 0, `first_contact_sim_s=null`, 벽 충돌 표본 0 | `no_contacts=true` | 통과(event 기반. 메시지 부재가 무접촉의 증명은 아님) |
| 노면 | road outside 0. 바퀴 중심 off 0. `surface_run1.json`: 차체·조향 footprint 밖 0, 벽 바퀴 0, 벌점 사건 0 | `surface=true` | 통과(최대 두 바퀴 잔디 조건 충족) |
| 신호 | 7단계 수신, 첫 출발 7.683 s(green 7.400 뒤), `false_start=false` | — | 통과 |
| STOP·종료 | `stop_request_sim_s=103.833`, 안정 정지(최대 0.0007 m/s), 명시 MCU STOP, `actuation_stop_passed=true`, Gazebo 서버 정지 확인 | `stop_and_shutdown=true` | 통과(예정 STOP. 실패 정리 STOP 아님) |
| 판단 기록 | footer accepted=written 5,221, dropped·failed·pending·capture_errors 0. call 4,839 | — | 완전 |

## 4. 정지 지점 기록(사실만)

`run1/follow_decisions.jsonl`의 기록 출력만 옮깁니다. 원인 해석과 다음 조치는 GPT 판단에 맡깁니다.

- 43.32–44.00 s: `local_follow`, 0.40–0.60 m/s. 선호 왼쪽 `local_path`를 `unfiltered`로 채택했고, 경로 앞 길이는 2.58→2.23 m였습니다. 저장 재생에서 r21이 멈췄던 43.72·43.82 s도 이번에는 채택했습니다. 43.92 s는 `wall_segment_short`로 기억 경로 지지를 썼습니다.
- **44.02 s부터 창 끝까지 `follow_boundary_unconfirmed`(HOLD, 0 m/s) 3,020판단**입니다. 44.02 s 이후 `local_follow` 판단은 없습니다.
  - 44.02 s 후보는 `bilateral_reference_support`, 앞 1.21 m였습니다.
  - 44.12 s 후보는 `local_path`/`unfiltered`, 앞 1.40 m였지만 `fresh_boundary_accepted=false`였습니다. 당시 측정 속도는 0.16 m/s였습니다.
  - 같은 판단에서 `fragmented_boundary_evidence`는 `anchor_not_observed`, `corner_distance_evidence`와 `corridor_candidate`는 `not_consistent_inward_corner`였습니다.
- 정리하면, r22는 이전 정지 판단(43.72 s)을 지나게 했습니다. 그러나 약 0.3 s 뒤 같은 `follow_boundary_unconfirmed`로 다시 정지했습니다. 저장 재생에서 본 "정지 구간 양수 명령"은 폐루프 통과로 이어지지 않았습니다.
- 이번 기록에는 후보 경로의 `path_anchor_error_m`와 어느 검사(앵커·형상 일치·기억 일치 등)가 거부했는지가 남지 않았습니다. 어느 조건이 막았는지는 이 README에서 판정하지 않습니다.

## 5. 범위

이동 상자는 상대 실차 동역학이나 자율주행 차량이 아닙니다. 이 결과는 등록된 시뮬레이션 1회이며, 한 랩 완주·경기 전체·실차 성공의 근거로 확대하지 않습니다. 실패 run은 재실행하지 않았습니다. r20·r21 결과와 원시 기록은 그대로 보존합니다.
