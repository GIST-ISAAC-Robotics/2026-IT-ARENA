# r520 굴곡 벽 대표 1회 — 초기 정체·조기 STOP·시험 무효 실패

2026-11-14, GPT. r518·r508·r521 공통 후보와 선택한 굴곡 벽으로 단1회 실행했다. **진행0 m**였고 무진행8.504 sim초가 관측되어 등록된 STOP 절차를 호출했다. 실제 STOP·14자식 회수·잔여PID0을 확인했다. 원 결과는 `passed=false`, `run_valid=false`, `harness_valid=false`, `shutdown_clean=false`이며 그대로 보존했다. 재실행이나 조건 완화는 하지 않았다.

**기술 실패:** 파생 월드의 상대 재질 경로가 새 폴더 기준으로 해석되어 `aruco_id0.png`와 `aruco_id45.png`를 찾지 못했다. 실행 준비에서 `uri` 외 재질 참조의 경로 기준을 놓친 문제다. 네 개의 시작 중 오류가 원 `shutdown_clean=false`에 포함돼 있다. 14개 자식의 exit0·강제 정리 없음과 구분한다. 벽 collision 정합은 통과했지만 모든 runtime 자산이 원래 의미를 유지했다고 주장할 수 없다. 원 파생 SDF와 실패 로그를 수정하지 않았고 후속 상대 경로 수정은 새 버전에서 해야 한다.

## 입력과 실행

- [준비 조건](PLAN.md): 두 벽 world-y3.0–11.5 m, 도로측 nominal면 유지5 mm, 양벽 같은 위상±30 mm·1 m 주기, 50 mm chord, 600 mm마다20 mm 틈. 이 수치는 현장 미실측 가정이다.
- [최종 선택](source_selection.json): r515 기준 기존 생산7파일 변경·신규 `current_wall_reacquire.py`1파일, 기존 시험2파일 변경·신규 시험3파일. r521의 root220+11검사와 r518/r508 선행 검사는 반복하지 않았다.
- 실제 import8모듈·경쟁 프로세스0·evaluator 파생 벽 연결 후1,444입력을 동결했다. `prepared.json` SHA `8bc5ce5cbce1e71acb9ea45d88210d2174653a8ba9b86e1cc28f9288e34d8c6f`, `sources.zip` SHA `e4999f1651414b74a58d430734216d1314c3cd901402a49aee5616cee9bf9f06`. 압축본 내부1,443파일을 동결 해시와 대조했다.
- 파생 SDF SHA `9bf5ba7491153b8ef045a96e2428f44bffc9d308f4d3431c25bee1f30ee166a2` 불변. 실제 runtime 벽475개 collision digest는 파생본과 같다. 초기 actor 간격은 실제0.296499724490 m/준비0.296499724510 m로 일치했다([기하 영수증](runtime_geometry_receipt.json)).
- owner PID13264, UTC03:27:11–03:31:02, wall231초. GREEN7.401초, 원 기록 종료16.895초로 관측 구간9.494 sim초다. 계획한12 sim초 전체 실행을 완료한 것으로 보지 않는다.
- sim15.909에서 무진행8.504초를 확인해 helper STOP을 호출했다. `/autonomy/enable=false`와 `/actuation/stop` 성공, `ESTOP_LATCHED`·출력0을 확인했다([early_stop.json](early_stop.json)). 평가기의 종료 전 STOP이므로 원 `no_mcu_latch_before_stop=false`도 그대로 보존한다.

## 결과와 판정 경계

| 항목 | 관측 결과 |
|---|---|
| 원 평가 | FAIL: `progress`, `no_permanent_stall`, `final_gap`, `data_coverage` 실패 |
| 진행·정체 | 원 진행0 m·최초 이동 없음. 별도 표본 최장정지9.416 sim초, 최고속도0.000608 m/s |
| 접촉·노면 | actor접촉/표본겹침/벽접촉0, 바퀴중심 도로밖0 |
| 명령 | 저속helper0.1 m/s 실제 발행1회, 표본 `/drive` 최고0.099999994 m/s, `/drive/safe` 최고0 |
| STOP | helper정지확인true, dense pose 안정true·최대0.000513 m/s. 원 STOP6항목 중5개true, 조기래치항목false |
| 종료 | 14/14자식 exit0·강제정리 없음·PID0. 재질경로 오류4개 때문에 원 `shutdown_clean=false` |
| 무결성 | 실행 전후1,444입력 변경0·ZIP불변·원 결과 불변 |
| 별도 목표 | FAIL. `final_state=FOLLOW`/`final_follow=true`는 상태 이름 조건이며 정상 주행 복귀를 증명하지 않음 |

## 최초 래치와 재획득 명령

1. sim7.507 trace: `same_scan_missing_wait`, `/drive=0`, `/drive/safe=0`.
2. sim7.607 trace: 최초 `known_obstacle_latched`, clearance0. 당시 `WAIT_GREEN`·`green_candidate`이며 아직 첫 controller 계획 전이다.
3. sim7.720 첫 plan(seq369, runtime sequence1): LocalFollow의 `current_wall_reacquire_trial`로0.1 m/s·0.3 rad. 현재각 clearance0.56 m, 직전각0.32 m를 기록했다.
4. sim7.730 `route_publication`1회: 같은 저속 명령이 발행됐다. 보호층은 계속0을 출력했다.
5. 이후91개 계획은 `parallel_hold_baseline_unavailable`로0이다. 내부 baseline 사유는 `follow_partial_history_warmup`77, `follow_lateral_metric_outside_trial`12, `follow_lateral_metric_observation_unconfirmed`2회다. 정상 FOLLOW 양수 계획0, 명시 재획득→정상 FOLLOW 인계0이다.

따라서 이번 helper 사용은 추월 중 재획득·복귀 성공이 아니라 LocalFollow 시작 시 양수 후보1회다. `known_obstacle_latched`는 기록된 보호층 사유이며 어느 실제 반환이 충돌 판정을 만들었는지는 아직 귀속하지 않았다. 초기 래치가 첫 helper 발행보다 먼저였다는 시간 순서는 확인했지만, 이를 새 벽·r521·입력 처리 중 하나의 원인으로 확정하지 않았다. 재질 오류가 이 래치의 원인인지도 미확정이다.

## 재현용 발췌와 한계

[failure_inputs/summary.json](failure_inputs/summary.json)에 최초 시각·계획·저속 발행·정상 복귀와 점유 판정의 구분·원문 해시를 정리했다. [guard_timeline.json](failure_inputs/guard_timeline.json)은 직전/최초 래치 trace와 평가용 actor 상대 좌표다. 정확한 래치 전이 시각은 약100 ms trace 간격으로만 한정된다.

로컬 Git 제외 원문은 `failure_inputs/controller_prefix.jsonl`(최초5계획까지 header/reset 포함), `raw_scan_prefix.jsonl`(시작부터 최초래치 뒤까지), `selected_raw_scans.jsonl`, `front_raw_body_points.jsonl`이다. 전방 점은 기록된 extrinsic만 적용한 미보정 관측용ROI이며 guard 임계값이나 실제 guard witness가 아니다. 원 `run1/follow_decisions.jsonl`, `scans_received.jsonl`, `source_snapshot.zip`도 보존했다. 전체 guard 내부 witness와 callback 수신 순서는 원 시험에 기록되지 않았으므로 완전한 guard 상태 재현이 이미 가능하다고 주장하지 않는다.

발췌는 [저장 로그 도구](execution_v1/extract_first_guard.py)로 수행했다. 제어/guard 재생·추가 주행·runtime 자산 수정은 하지 않았다. WSL은 회수 완료 후 root에 반환했고 이후 작업은 Windows의 저장 파일만 사용했다. root의 후속 공통 통합은 이 시험 당시 소스 압축본과 분리한다.

근거: [원 결과](run1/report.json), [실행 요약](execution_summary.json), [별도 목표](assessment_summary.json), [무결성](integrity.json), [STOP](early_stop.json), [최종 PID](execution_v1/owned_processes_after.json), [실행 검토 범위](root_execution_review.json). 이 무효 실패를 현장 굴곡 대응·전체 교통·실차 성공 근거로 사용하지 않는다.
