# 교통 코너 구간 44초 (r23) 결과

상태: **사전 등록 44초 구간의 1회 실행이 사전 기준을 모두 통과했습니다.** 이 결과는 등록 코너 구간만의 결과입니다. 전체 랩 완료, 경기 전체, 실차 증명이 아니며, r22 96초 실패(`../traffic_lap_anchor_v1`)를 소급해 성공으로 바꾸지 않습니다. 계획과 사전 기준은 [PLAN.md](PLAN.md)에 있습니다.

## 실행

명령은 `wsl.exe -e bash -lc '… bash artifacts/validation/2026-11-08/traffic_anchor_prefix_v1/run_live.sh; echo OUTER_EXIT=$? > …/run_live.outer_exit.txt'`이고 1회만 실행했습니다. 실행·대기·회수는 Claude Haiku 5.5 보조가 맡았습니다.

| 항목 | 결과 |
|---|---|
| outer | `OUTER_EXIT=0` |
| `geometry_v1` | 0 |
| `register` | 0 (등록 입력 884개) |
| `run1.exit` | `HARNESS_EXIT=0 SURFACE_EXIT=0 INTEGRITY_EXIT=0 LEFTOVER_PGREP=1 ASSESSMENT_EXIT=0 WALL_S=598` |
| 잔존 프로세스 | postflight 목록이 비어 있습니다. 회수 후 다시 확인한 `pgrep`도 exit 1(없음)이었습니다. |

## 핵심 지표 (`two_actor_corner_prefix/assessment_run1.json`)

| 항목 | 값 |
|---|---|
| 창 | green 7.402 s → 51.403 s (44 s) |
| 보충 연속 진행 | **22.63 m** (기준 ≥21.5). s 41.66→64.29. 투영 점프 0, 짝맞춤 21,740 표본 |
| 원 first_curve 평가 | 끝 s 64.29 ≥ 출구 63.14. 순 방향 변화 179.4°(기준 ≥90°) |
| 최장 저속(<0.05 m/s) | 0.0 s. 첫 움직임 8.089 s 이후 9 s부터 최소 지면속도 0.254 m/s |
| 최고 속도 | 1.338 m/s |
| 이전 정체 지점 | r22는 진행 18.98 m(s 60.64, 44초 부근)에서 정체했습니다. 이번 실행은 44·48 s에도 약 0.48 m/s로 주행하며 이 지점을 지났습니다. |
| 간격, 주 상대 | 최소 표본 0.511 m, 쓸림 하계 0.511 m. 겹침 0 |
| 간격, 추가 상대 | 최소 표본 1.650 m, 쓸림 하계 1.650 m |
| 접촉 | 주 상대 메시지 0. 추가·결합 접촉 사건 0(원시 메시지 0) |
| 노면 | 26,019 표본, 벽 바퀴 0. any/center/full 모두 최대 잔디 바퀴 0. 차체 도로 밖 0 |
| 각본 유효 | `harness_valid`, 추가 상대 `behavior_gates_passed`, 거부 자세 0 |
| 보호층 | trace 10 Hz 436행 모두 `safety_reason=clear`. `/drive/safe`가 `/drive`보다 0.05 m/s 넘게 낮은 행 0. 출발 후 `/drive` 0 명령 0 |
| STOP·종료 | `stop_checks` 6항목 true, `shutdown_clean` true |
| 입력 해시 | 보고서 `source_unchanged` true. postflight changed 0/884 |
| 원 보고 | `raw_pass` true. 원 `raw_gates` 13개 모두 true. 원 판정은 별도로 보존합니다. |

`path_reason_summary.txt`에 따르면 trace의 `path_wall`, `wall_fallback`, `boundary_*` 필드는 이 실행에서 모두 None입니다. 행동은 `FOLLOW`/`handoff_follow`가 432/436행입니다. **경로 표현별 앵커 사유(`follow_decisions.jsonl` 2,254 call)는 아직 분석하지 않았습니다.**

## 준비 내용과 바꾼 helper

- 출발점은 `traffic_lap_anchor_v1`의 `register.py`, `run.py`, `run.sh`, `run_live.sh`, `geometry.py`, `assess.py`, 설정 사본입니다. 변환은 `prepare_inputs.py`(exit 0)가 했습니다.
  - case는 `two_actor_corner_prefix`이고 `first_curve`, 44 s, 두 schedule `to_s=44`입니다.
  - acceptance에서 `minimum_laps`·`maximum_actor_laps`를 뺐습니다. 대신 `required_exit_s_m=18.5`, `maximum_actor_s_m=27.0`, `minimum_heading_change_deg=90`, `minimum_progress_m=21.5`를 두었습니다.
- `register.py`는 r22 run1 `registered.json`의 args를 그대로 쓰고 config, case, overlay 경로만 바꿨습니다. 또 r23 계약 after 해시를 assert하고, scope에 계약 경로와 sha256을 기록합니다.
- `geometry.py`는 두 상대 모두 `first_curve`가 공통 case 규칙(출구 63.14, 한계 71.64)으로 검사되는지 assert합니다. 확인한 시작/끝 s는 43.26→65.26, 49.26→71.26이고 1 cm 도로/벽 표본은 각 2,201개입니다. 초기 간격은 1.40/7.40 m입니다.
- `assess.py`는 `one_lap` 대신 `prefix_progress`를 씁니다. 이 값은 `full_course_progress(line, window_tuples(ego,t0,end), window_tuples(actor,t0-.11,end), actor_start_s, .20)`의 보충 평가입니다.
- 사전 점검:
  - `geometry_preflight` exit 0.
  - `preflight_assess.py` exit 0. r22 저장 자세에서 18.98 m를 재현해 저장값과 일치했습니다.
- 제어기, 기준, 물성, 소스, manifest는 바꾸지 않았습니다.
- `../corner_path_anchor_transition_v1`에 `parent_review.json`과 README 보충을 추가했습니다. vehicle_interface 3모듈 해시가 같다는 확인입니다.

## 남은 한계·미분석

- 결과 범위는 44초 등록 구간, 각본 상대(이동 상자, 무한 질량), 단일 seed 1회입니다. 전체 랩, 피추월·추월, 경기 운영, 실차로 확대하지 않습니다.
- 상대 속도가 0.5 m/s라서 자차도 대부분 약 0.5 m/s로 추종했습니다. 속도 개선의 근거가 아닙니다.
- `follow_decisions.jsonl` 기반 경로 표현 전환과 `path_anchor_error_m` 사용 구간 분석은 하지 않았습니다. 사용량 한도 인계로 남깁니다.
- 44초 이후 구간(남은 랩)은 이번 범위 밖이며 검증하지 않았습니다.
