# r469 단독 1회 결과

**원 판정은 실패다.** actor0 조건에서 44.64 m 중 9.90 m 진행한 뒤 지속 정체했다. 8 sim초 무진행 기준을 관측한 시점의 정체 시간은 8.790 sim초였으며, 소유 STOP을 한 번 발행해 회수했다. 원 `run.exit=1`, `report.passed=false`, `full_lap=false`, `no_permanent_stall=false`를 보존한다. 벽 접촉 표본은 0, 노면 판정은 통과했다. 재주행은 하지 않았다.

## 입력·빌드와 적용 범위

r465의 actor0·old610 시설 3개·차량/C1·속도·물성·RTF .25·80 sim초·정체/STOP/종료 기준을 유지했다. 원본 recipe 13개는 byte exact이며, r465 대비 공통 소스 delta는 r467/r468 통합 5개와 정확히 일치했다. r466/r470 stage는 포함하지 않았다. 빌드/import/고정 CLI 검사는 exit 0, import 23 modules, native SHA 불변이었다. 원 r465 실패·실행 영수증은 복사하지 않았다.

- [검토 범위](source_scope.json), [검토 배정과 실제 검토 수행자](root_source_review.json), [실행 전 검토](root_execution_review.json)
- prepared SHA: `ee8a745e85cc5ca1a54da61181b2ae33e92ed73b8d59c23fe7db1e8c16602ac3`
- sources.zip SHA: `78e2296cb02301c0f4440e773ba274471c4b57ffe54583a04eb297c5dc415bd5`
- 실행 종료 시 [1347개 입력 불변](execution_v1/check_after_run.json). 자원 반환 뒤 공통 변경을 막거나 다시 비교하지 않았다.

## 실제 r468 허용과 첫 정지

아래 reference/at 시각은 simulation seconds다. compute 시간은 host wall 경과 시간이며 두 시간축을 변환하지 않았다. [대표 원 필드](first_stop_evidence.json), [시간축](timing_rows.json), [r468 계획→현재→발행 연결](r468_application.json).

| trace call / 계획 reference | 계획 | 현재 재검사 | 부모 발행 |
|---|---|---|---|
| 803 / 18.52 | .35, released_declared_structure_motion | 18.526 / .35 | 18.54 / .35 |
| 807 / 18.62 | .35, 같은 이유 | 18.625 / .35 | 18.64 / .35 |
| 811 / 18.72 | .35, 같은 이유 | 18.722 / .35 | 18.74 / .35 |
| 815 / 18.82 | .35, 같은 이유 | 18.824 / .35 | 18.84 / .35 |
| 819 / 18.92 | .35, released_bounded_rear_sweep | 18.924 / .35 | 18.94 / .35 |
| 823 / 19.02 | .35, released_bounded_low_speed_body_gap | 19.026 / **0**, released_rebase_current_body_gap_rejected | 19.026 / 0, not_published_as_checked |
| 827 / 19.12 | .35, 같은 body-gap 이유 | 19.124 / .35 | 19.129 / **실제 .1**, checked .35 |
| 832 / 19.22부터 | **0**, parallel_hold_reference_not_straight | 추가 current 없음 | 양수 발행 없음 |

r468 계획 구조 허용 4회는 모두 양수 현재 결과와 실제 .35 부모 발행으로 이어졌다. 별도의 `current_structure_risk`는 RELEASED에서 3회 수용됐다(807/811/815). 전체 모드의 같은 helper 진단은 6회이며 나머지 3회는 기존 SOLO 경로이므로 r468 효과에 합산하지 않는다. r465의 9.54 m보다 진행했지만 .36 m 차이 전체를 한 변경의 인과 효과로 단정하지 않는다.

첫 current 거절은 충돌 표본 거절이 아니라 **body-gap 검사 입구의 실측속도 범위 이탈**과 일치한다. call 823의 계획 실측속도는 .306125210 m/s였으나 current는 .367350252 m/s였고, 원 `current_body_gap.reason=invalid_motion_or_scan`이다. 동결 소스 `released_body_gap.py:10–13`은 `0 <= speed <= .35`, 목표 `0 < target <= .35`, `0 < dt <= .1`을 요구한다. 당시 scan 존재, current dt .086 s, 목표 .35로 기록됐으며 속도 상한을 넘는다. 이 입구에서 region 검사 전 반환한다. 연결 함수는 `race_runtime_policy/retained_recheck/_released.py:53–57`이다. 다음 call 827은 실측 .122450084 m/s로 같은 current 검사에 수용됐다.

이 일시 정지와 후속 고착은 구분한다. call 832부터 마지막까지 **102회 모두 계획 자체 0**, RELEASED/retained 상태, selector choice `parallel`, selector reason `large_motion_region_unobserved`다. 첫 행은 2850개 query 중 일반 unobserved 365, left masked 1501, tail 13.925 mm, front mask 0이었다. `selector_unknown_filled=false`, `selector_hidden_rear_risk_unresolved=true`이며 최종 fallback은 `parallel_hold_reference_not_straight`다. 이후 current 재검사나 양수 발행은 없다. 이 일반 미관측의 세부 광선/기하 원인과 경로 유지 설계는 미해결로 주 담당에게 넘긴다. 새 예외나 판정선 수정은 하지 않았다.

## 이전 상태 전환과 전달·지연

[상태 전환 원 근거](mode_transitions.json): 12.42 s에 SOLO→GUARD, 양쪽 각각 4개 측방 반환과 rearward ds 약 -.327…-.310 m가 기록됐다. 13.22 s에 GUARD→RELEASED로 전환할 때 양측 present=false, seen/passed=true, vanish=rear, clear는 2/3이었다. 이후 정체까지 RELEASED를 유지했다. actor0 시험이므로 다른 차량 actor는 없지만, selector 로그 자체는 각 반환의 물체 ID를 기록하지 않는다. 이 자료만으로 벽·시설·차체의 어느 표면인지 확정하지 않는다.

plan 제출/완료는 각각 217개, 식별자 누락·중복 0, call 원 입력/완료 출력 불일치 0이다. footer는 accepted/written 1166, dropped/failed/pending/capture_errors 모두 0, writer alive=false다. 부모 route publication 113개 중 양수는 111개다. 별도 discard는 `retained_route_not_proven` 2개, `new_plan_superseded_current_check` 3개다. 원 행을 시간축에 보존했으며 첫 정체와 같은 원인으로 합치지 않았다.

plan wall 계산 217회의 median/p95/max는 52.55/68.58/81.07 ms, 100 ms 초과 0이다. current 113회는 21.58/36.84/112.78 ms, 100 ms 초과 1회다. 첫 current 거절은 7.73 ms, 지속 정체 102회 계획은 median 60.88 ms·max 81.07 ms였다. MCU `command_expired` 표본은 0이며 drive/zero_target 갱신이 기록됐다. fault diagnostic 표본 0은 모든 지연이 없다는 보장이 아니다. 정책의 명시적 0을 계산 폭주나 전체 simulation scheduling 실패로 바꾸어 설명할 근거는 없다.

## STOP·정상 회수와 중단 인계

소유 STOP exit 0 뒤 원 evaluator가 종료를 확인했다. STOP 후 .5 sim초 동안 변위 0.00000404 m, 최대 속도 .00040449 m/s, 벽 접촉 0이다. software STOP이 evaluator 자체 STOP보다 먼저 들어갔으므로 원 abort `mcu_latched_before_stop: software_stop`와 해당 stop-check false를 그대로 보존한다.

자식 14개 전부 정상 회수, launch return 0, forced cleanup 없음, 잔여 test PID 없음이다. [소유 프로세스 확인](execution_v1/owned_processes_after.json)은 2026-11-13 15:36:09 UTC (한국 시각 11/14 00:36:09)에 owned PID 0을 기록했다. Windows WSL 호출 PID 52212도 exit 1로 회수했다. **WSL과 공통 동결 소유권은 주 담당에게 반환 완료**했으며 그 뒤에는 Windows 원 로그 집계만 했다. Jetson·실물·새 ROS/Gazebo 실행은 없다.

사용자의 FAST 변경용 일시 중지에 따라 이 결과와 [최종 영수증](final_receipt.json)을 봉인하고 종료한다. 재개 시 본 보고서와 `first_stop_evidence.json`을 읽어 최초 current 속도 입구와 후속 plan 일반 미관측을 분리해 분석할 수 있다. 다음 주행이나 수정은 주 담당의 후속 설계·자원 배정에 따른다. [기계 집계](execution_summary.json), [원 로그 SHA](raw_integrity.json); 원 실패는 보존하며 성공 재실행은 하지 않았다.
