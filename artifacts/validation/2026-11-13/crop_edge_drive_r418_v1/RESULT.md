# r418 최종 기하·경계 성분 제한 주행 결과

2026-11-13. **최종 기하와 공통 runtime의 주행1회는 약10.10m에서 정체해 미완주했다.** r406과 같은 시설·두220mm 상대·속도·물성·평가 기준을 유지하고 `g1_crop_edge_component_trial=true`만 명시했다. 저장 입력의 전진 판단 증가는 실제 지속 진행으로 이어지지 않았다.

## 실행과 결과

- 공통3패키지 빌드와 최종 사전15검사를 통과한 뒤 단1회 실행했다. 모델 capacity 오류 뒤에는 이미 진행 중인 동일 실행을 이어 회수했으며 주행을 새로 시작하지 않았다.
- 8sim초 무진행 뒤 소유 STOP을 요청했다. 원 `run1/report.json`의 `passed=false`, `harness_valid=false`, `no_permanent_stall/full_lap/data_coverage=false`를 유지한다. `mcu_latched_before_stop: software_stop`은 소유 STOP이 원 평가기의 종료 요청보다 먼저 적용된 순서를 포함하므로 원 주행 정체의 MCU 고장으로 해석하지 않는다.
- 종료 STOP 정착·actuation 정지·Gazebo 정지 확인을 통과했다. 벽 접촉 표본0, actor 접촉/평가 겹침0, 최대두바퀴 노면 기준을 확인했다. 자식5개 exit0·강제회수/잔여PID0·동결1334입력/준비물/archive 불변이다.
- header 원문 exact는false이고, r409의 엄격한 구조JSON 공백 비교에서 의미 동등은true다. crop 옵션ON과 기존 current 후보 선택을 확인했다. 원 exact 판정을 성공으로 덮지 않는다.
- 계획 출력에 crop 성분 진단148회, 그중 양수146회가 기록됐다. 기존 현재 거리 후보의 계산 수락/부모 발행은각2회다. 계획 진단·부모 발행·보호층 통과·실제 진행은 각각 다른 관측이다.

## 남은 정체의 범위

평가 trace의28–37sim초89표본에서는 `/drive`와 `/drive/safe`가 모두0이었고, 보호층 사유는 `clear`85회·`same_scan_missing_wait`4회였다. 마지막 대표에서는 MCU가ARMED이고 상위 상태가 `ROUTE_CURRENT_LARGE_MOTION_REGION_UNOBSERVED`다. 따라서 이 구간은 r406처럼 독립 보호층의 `unbounded_missing`만으로 설명할 수 없다. 경로 계산의 양수 결과와 현재 경로 재검사·발행의 연결을 [r421 저장 분석](../crop_edge_stall_r421_v1/REVIEW.md)에서 분리한다. 서로 다른주기 표본을 정확히 같은 호출의 연속 처리로 합산하지 않는다.

같은 조건을 그대로 반복하지 않는다. 현재 검사 쪽 새 관측 결과의 연결을 확인하기 전 코너 옆차 대표를 자동 실행하지 않으며, 접촉·구동 실패를 숨기거나 평가 기준/물성을 낮추지 않는다. 실차·전체10Hz·전체 경기 검증은 아니다.

## 주행 전 시험 도구 오류 보존

첫 사전 검사는 주 에이전트가 동결 전 잘못 배정해 `prepared.json` 부재로3통과/2실패/10오류를 남겼다. 상대 integration 경로와 절대 경로를 섞은 준비 도구 오류도 archive 생성 전에 발견해 경로resolve만 수정했다(`preparation_correction_v1`). 동결 후 검사는14통과/1실패였는데 expected JSON이 이미compact인 상황에서 compact 재직렬화가 원문과 달라질 것이라고 가정한 fixture 오류였다. `preflight_revision_v3`은 동등한 indent형식으로 그 기대만 수정했고15검사/skip0을 통과했다. 원로그·원fixture·동결본을 보존하며 제어/평가의 수치나 판정은 변경하지 않았다.

근거: `run1/report.json`, `run1/trace.json`, `run1/follow_decisions.jsonl`, `runtime_audit.json`, `early_stop.json`, `integrity.json`, `prepared.json`, `build_v1/`, `preflight_v2/`, `preflight_v3/`. 원시 로그·대형ZIP의 기존 Git 제외 정책을 유지한다.
