# r452 — GUARD .1 후보 연결 후 고정 대표 1회

주 담당의 `root_execution_review.json` 검토 뒤 고정 wrapper로 1회만 실행했다. **20.18 m에서 8.854 sim초 무진행으로 owned STOP을 1회 호출했으며 미완주다. 새 `selector_guard_crawl_retry` 호출·수용·부모 양수 발행은 모두 0이다.** 이번 시험으로 새 후보의 탈출 효과를 입증하지 않았다. 공통 소스와 동결 입력은 바꾸지 않았으며 같은 주행을 반복하지 않았다.

## 실행과 원 판정

- r441과 같은 config/actor/기하·물성·센서·속도·80 sim초 목표/8 sim초 무진행 STOP 조건이다. 실제 공통 차이 26파일과 진단용 `g1_complete_plan_trace:=true`는 원 승인 목록에 기록했다.
- STOP 판단 표본은 sim 43.853초, 마지막 진행 증가 sim 34.999초, 진행 20.18 m였다. helper는 `/autonomy/enable=false`, `/actuation/stop` 응답을 받았고 exit 0이다.
- 원 `report.json`은 `passed=false`, `harness_valid=false`, `abort=mcu_latched_before_stop: software_stop`, runner exit 1을 그대로 보존한다. owned helper STOP이 평가기 STOP보다 먼저 도착한 82 latch 표본도 소급 제거하지 않았다. 원 STOP 검사 중 `no_mcu_latch_before_stop=false`이며 안정 정지·명시 MCU STOP·deadline 없음·벽 충돌 없음은 참이다.
- 관측 구간의 접촉 이벤트·벽 충돌·샘플 겹침·차체 노면 이탈·3바퀴 이상 이탈은 0이다. 조기 종료한 시뮬레이션의 결과이며 80 sim초 전체, 완주, 실차 안전 증거가 아니다.

## 후보와 실제 움직임을 구분한 집계

새 GUARD .1 후보와 current crop/crawl은 미호출이다. 따라서 새 후보 시작 후 진행거리·연속 .1 지속시간·정상 가속 복귀는 해당 없음으로 기록한다. 전체 current 재검사는 264건이며 `current_relative_body_retry`는 20건이다. 전체 부모 route 기록 264건 중 retained 양수 발행은 255건이며 이것을 새 후보의 발행 수로 세지 않는다.

기존 current 상대 차체 검사 구간에서는 sim 20.055–20.865초 동안 drive 약 .1 m/s가 연속 표본으로 관측됐고, 이 0.810초 동안 진행은 10.03→10.11 m(+.08 m)였다. 이후 간헐적인 .1 명령과 0이 반복되다가 sim 22.683초에 drive/safe .2, 22.885초에 .4 m/s로 복귀했다. 실제 ground 속도도 이후 .15/.35 m/s를 넘는다. 이는 새 GUARD 후보 효과가 아니며 상세 표본 구간은 집계 JSON에 보존했다.

마지막 정체 이후 reference sim 34.999–45.720초의 고유 plan 108건은 전부 0이었다. normal 이유는 `parallel_hold_reference_not_straight`, selector 이유는 `guard_curve_rejected_large_motion_masked_toward_left` 각 108건이다. 기존 .25 재검사는 같은 masked 이유 108건, .15 재검사는 `guard_curve_rejected_toward_left` 108건이다. 이 구간 current 재검사와 부모 양수 발행은 0이다. 원 측정 모터축 속도는 0이 106건, 음수 2건이며 음수만으로 전체 미진입을 설명하지 않는다. 진입 조건의 원인과 수정은 별도 r456에 분리한다.

`actuation_diagnostics=[]`, `command_expired` 표본 0이다. 최종 MCU는 `ESTOP_LATCHED/software_stop`, target 0, mean wheel 0이며 원 최종 출력도 STOP이다. 독립 guard/MCU와 parent route 기록은 각각 원자료로 집계했으며 계산 수용을 실제 적용으로 간주하지 않았다.

## 완전 기록·종료·무결성

`runtime_plan_submitted=381`, `runtime_plan_completed=381`이며 `(generation, sequence)` 기준 모두 고유하고 양방향 미대응 0, 제출 wall시각 불일치 0이다. 계산은 generation 360에서 이루어졌고 시작/STOP reset 기록은 보존했다. call 386에는 반복 current 결과가 포함되므로 고유 plan 수로 사용하지 않았다. 실제 evaluator header 1건이 기록됐다. seq는 연속이며 footer의 accepted/written 각 1832, dropped/failed/pending/capture_errors 모두 0, worker_alive=false다. 제출·완료 기록은 적용 증명이 아니다.

기존 `runtime_audit.json`은 `known_structure_sectors_json`의 문자열/객체 표현 때문에 header exact=false/equivalent=true를 보고한다. 원 감사 실패를 수정하지 않았으며, 이번 새 raw evaluator header가 후속 재현의 직접 입력이다.

자식 14/14 정상 종료, process exit `[0,0,0,0,0]`, unaccounted/remaining test PID 0, 별도 `/proc` 소유 PID/partition 검사 0이다. `integrity.json`의 1,371 입력 변경 0, prepared/archive 불변이며 wrapper 종료 뒤 `prepare --check`도 통과했다. WSL 소유권은 주 담당의 후속 r456 저장 입력 분석·검증 배정에 따라 유지한다. Gazebo/ROS 실행은 끝났으며 Jetson·실물은 사용하지 않았다.

근거: [집계](analysis_v1/summary.json), [원자료 SHA 봉인](analysis_v1/original_evidence_sha256.json), [읽기 전용 집계 코드](postrun_analysis.py), [원 판정](run1/report.json), [STOP 순서](stop_sequence.json), [입력 무결성](integrity.json), [소유 프로세스](execution_v1/owned_processes_after.json). 원 실패 로그와 r441/r452 source archive를 보존한다.
