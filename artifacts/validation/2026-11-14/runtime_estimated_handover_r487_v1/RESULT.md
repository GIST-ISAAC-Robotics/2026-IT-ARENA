# r487 최신 공통의 명시 estimated 입력 계약 확인

2026-11-14, GPT. **기존 r444 변환을 그대로 사용한 4대표 plan/current 총8회에서 시간·scope 계약 거절은 없었다.** 입력과 `false / host_estimated_v1 / unverified_estimate`는 8/8 유지됐다. 실제 센서 입력·주행 성공의 검증은 아니다.

## 입력 출처와 범위

검토한 r444/r446/r447 보존 자료에서 r447 `bundle/inputs/selected.jsonl`은 시뮬 원 수치이며, estimated 입력은 기존 r444 함수로 실행 중 생성했다. 보존된 실제 실물 Evaluator 입력으로 오인하지 않았다. r447의 99개를 반복하지 않고 기존 대표 r437:643 및 r426:369/680/614만 선택했다.

원 `estimated_record`·`estimated_header` 함수가 들어 있는 `contract_reference.py`를 byte 그대로 재사용했다. 계약 필드와 기존 crop scope 표시 외에는 원 scan/ranges/angles/acquisition times/wheel/history/epoch/기하/측정속도 수치가 모두 그대로임을 Windows에서4개 대조했다. 새로운 센서값·실물 시각·출력 기대값을 합성하지 않았다.

plan은 각 사례에서 새 Evaluator로 시작한다. current는 **기존 저장 plan_output과 원 plan_input/current**를 `current_request(..., crop_current_selection=True)`로 묶었다. 새 plan의0을 바꿔 current에 넣지 않았다. 따라서 이 시험은 최신 plan/current의 명시 계약 입구 및 기존 저장 판단과의 호환성 확인이며, 최신 plan→현재관측→실제 발행의 연속 주행 재생은 아니다.

## 실제 결과

| 대표 | plan 결과 | current 결과 m/s | current 이유 |
|---|---|---:|---|
| r437:643 | 0, `follow_partial_history_warmup` | .1 | `rebase_current_follow`, estimated crop 단일 목표 수락 |
| r426:369 | 0, `follow_boundary_unconfirmed` | .45 | `rebase_current_follow` |
| r426:680 | 0, `follow_boundary_unconfirmed` | .09332669202147063 | `rebase_current_follow`, estimated corner 단일 목표 수락 |
| r426:614 | 0, `follow_boundary_unconfirmed` | 0 | `route_current_large_motion_region_unobserved` |

새 plan의0은 초기 이력/경계 확인 결과이며 시간/scope 오류가 아니다. 음수 측정 사례의 current0도 기존 영역 거절로 유지됐다. current4개 모두 `independent_guard_required=true`, `runtime_authorized=false`다. r426:369의 .45는 과거 r447의 .15와 다르므로 옛 출력 exact를 주장하지 않으며, 최신 공통 계산 결과로 기록한다. 이번 작업은 정책 변경 전후 동등성 시험이 아니다.

동작을 바꾸지 않는 Python line trace로 실제 도달 지점을 기록했다. `current_route_priority.observed_command` 및 `rebase` 입구는 실행됐으나 현재 경로 수락 조건은 통과하지 않았다. r482 `CurrentLaneSelector`도 실행됐지만 `baseline_not_normal`로 복귀해 measured-region 재사용에는 도달하지 않았다. **r481 수락 분기와 r482 재사용 분기 실행 검증은 미도달**로 남긴다. 이를 도달시키기 위한 추가 prefix 재생·입력 변경·시험 반복은 하지 않았다.

## 공통·실물 진입 연결

동결한 공통은 r481 v2의 `local_follow.py` / `race_runtime_policy/current_route_priority.py` 최종 SHA, r482의 `current_lane_preference.py` 최종 SHA와 일치한다. [source_check.json](source_check.json)에 근거가 있다. r444/r446의 실물 명시 진입은 G1G2Node/native 두 계산 runtime이고, r401/r407 profile 및 r453 운영 선택은 `timing_verified=false`·`host_estimated_v1`을 유지한다. r482 [실물 코드 대조](../jetson_plan_cost_r482_v1/HARDWARE_PATH.md)의 같은 selector/proposal 연결도 그대로다.

현재 경로 우선 함수에는 simulation scope만 허용하는 별도 분기가 없으며, 실제 scope 검사는 기존 evaluator/scene 생성 계약을 거친다. 이번8회에서 그 계약 입구가 정상 작동한 것을 확인했다. 실제 센서 오차/추정시각 정확도·신규 분기 전체 커버리지·동시 ROS 부하를 확인한 것은 아니다.

## 실행·회수

- 격리 경로 `/home/q/it-arena-r487-a`, 동결241파일. 기존 r470 ARM binary를 C++/setup SHA가 같은 것을 확인하고 재사용했다. 새 빌드/패키지 설치0.
- 호출8회, 추가 warmup0, 성능 행렬0. line trace가 있으므로 실행 시간을 성능으로 발표하지 않는다.
- child PID7164 exit0, 강제종료false, process group 잔존false, 최종 소유 작업0. 원본/동결 source 변경0, native 원본 불변.
- 센서/포트/MCU/ROS/구동 출력/WSL 접근0, 전력·네트워크 설정 변경0. 저장 계산 전송에 기존 SSH만 사용했다. **Jetson은 root에 반환했다.**
- [요약](summary.json), [실행 영수증](execution_receipt.json), [동결 manifest](manifest.json), [실행 helper](run_smoke.py), 로컬 보존 `remote_result/results/report.json`에 근거를 남겼다.

회수 archive SHA는 `c47c188e6b0b0ecf3beed368b001e6331ec01d92a44216b87c3e960707849586`다. 기존 r447 원 실패·출력·원 입력은 변경하지 않았고 공통 코드도 수정하지 않았다.
