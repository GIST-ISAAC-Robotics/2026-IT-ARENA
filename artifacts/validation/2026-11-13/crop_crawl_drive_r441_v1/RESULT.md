# r441 단일 제한 주행 결과

2026-11-13, GPT가 Ubuntu-24.04의 동결 실행 묶음을 **1회** 실행했습니다. 원 판정은 **exit 1 / 미완주 실패**이며 그대로 보존합니다. 추가 주행은 하지 않았습니다.

- 코스 진행량 20.09 m, 마지막 진행 증가 34.203 sim s. 관찰 시점 42.555 sim s에 8.352 sim s 무진행을 확인하여 기존 owned STOP을 호출했습니다. 최대 80 sim s를 채우지 않았습니다.
- 이번 기록에는 `current_crop_component_retry`, 새 `crawl_retry`가 **각각 0회**입니다. 따라서 새 0.1 m/s crop 수락→부모 양수 발행→차체 진행의 연결은 **미검증**입니다. 이번 진행량을 r440 효과로 설명하지 않습니다.
- 370개 plan 호출 중 current 재검사 257개, route publication 257개를 수집했습니다. 기존 상대 차체 미반환 재검사 진단 14개, 기존 corner distance budget 진단 2개가 있습니다. 예를 들어 seq 627의 0.1 m/s는 기존 `current_relative_body_retry`이고, 새 crop/crawl이 아닙니다.

## 정체 원인

seq 895부터 998까지, reference 34.12–44.42 sim s의 **104개 plan이 연속으로 0**을 반환했습니다. 기존 GUARD 곡선 후보가 후측방 자기 가림 꼬리 때문에 거절되고, 평행 대체 경로도 `parallel_hold_reference_not_straight`로 거절됐습니다. current crop 단계에는 도달하지 않았습니다.

- 기존 GUARD의 0.35→0.25→0.15 m/s 검사 순서는 유지됐습니다. 104개 모두 `unobserved=0`, `masked_front_samples=0`이나, 0.15 후보의 `masked_tail_extent_m`는 **0.010044–0.012757 m**로 기존 0.01 m 한계를 초과했습니다. 0.25 후보도 모두 거절됐습니다.
- 첫 지속 정체 seq 895: 실측 0.061225 m/s, 0.35 후보 꼬리 0.014418 m, 0.25 후보 0.014055 m, 0.15 후보 0.010577 m입니다. 마지막 seq 998: 실측 0, 각각 0.015021 / 0.014573 / 0.010886 m입니다.
- 마지막 plan의 평행 기준 직선 적합 residual은 0.465648 m입니다. 별도 `partial_observation=fully_observed`와 swept clearance 0.72 m는 GUARD 전체 검사의 통과를 뜻하지 않습니다.
- 마지막 양수 route publication은 seq 893, reference 34.024 / publication 34.026 sim s, checked 0.15→actual 0.1 m/s, 기존 `rebase_current_present_guard`입니다.

## 시간·정지·관측 범위

이번 WSL 실행에서 기록한 **wall 시간** 계산비용은 plan 370개 median/p95/max **20.71/44.80/55.44 ms**, current 257개 **16.71/29.67/82.19 ms**입니다. plan+current 계산시간 합은 **38.28/52.19/99.70 ms**이고 queue/전달/전체 명령 지연과 동일하지 않습니다. publication의 reference→발행 간격은 **sim 시간** median/p95/max 80/80/100 ms입니다. 새 crawl 분기는 미호출이므로 그 추가 비용의 주행 계측도 아닙니다.

설정 RTF는 0.25입니다. 관찰 UTC 사이 458.387 wall s 동안 저장 trajectory는 40.084 sim s 진행했습니다(비율 약 0.0874; Gazebo 내부 RTF 계측은 아님). 원 validator의 전체 wall elapsed는 526.669 s로 시작/회수 등의 구간을 포함합니다. Windows 후처리 시간과 Jetson 성능으로 확대하지 않습니다.

`command_expired` 상태 표본 0개, actuation diagnostics 0개, 원 `no_deadline_diagnostic=true`입니다. owned helper의 `/autonomy/enable=false`와 `/actuation/stop` ACK 후 0 출력/`ESTOP_LATCHED`를 확인했습니다. 이 STOP이 validator 자체 STOP보다 먼저 도착했으므로 원 `no_mcu_latch_before_stop=false`, abort=`mcu_latched_before_stop: software_stop`입니다. 판정선을 수정하거나 이를 전체 주행 성공으로 바꾸지 않았습니다.

무진행 STOP 필요 판정에 사용한 마지막 trajectory 표본은 42.555 sim s이고, 첫 MCU `software_stop` latch는 44.440 sim s입니다. 서비스 요청의 정확한 sim 시각은 별도 기록되지 않았습니다. 8 sim s 조건 감지와 실제 MCU latch를 같은 순간으로 취급하지 않습니다.

validator STOP 이후 0.5 sim s의 dense pose 증거는 정상이며 변위는 0.000007734 m입니다. 벽 접촉 표본·actor 접촉 이벤트·sampled overlap·노면 밖 표본은 각각 0, 관측 wheel center 최대 이탈 수는 0입니다. 조기 종료 구간의 표본/이벤트 결과이며 전체 코스·실차 안전 증명이 아닙니다. 원 harness와 behavior gate의 실패도 그대로 남겼습니다.

## 동결·회수와 증거

- 원 validator의 자식 프로세스 14/14 정상 종료, 5개 수집 프로세스 exit 0, forced cleanup 없음. 별도 `/proc` 감사에서 PID 776 및 `GZ_PARTITION=arena_autonomy_test_776` 잔여 작업 0개입니다.
- 실행 전후 동결 입력 **1,352개 변경 0**, prepared/archive 불변입니다. r437 대비 공통 차이는 사전 승인한 r434 4개 + r440 1개입니다. r434 preview 기본 OFF이며 이번 launch에 패널을 추가하지 않았습니다.
- runtime header는 `known_structure_sectors_json`의 JSON 공백 차이 때문에 **exact=false**, 값·타입·순서 유지 **equivalent=true**입니다. 원 audit를 수정하지 않았습니다.
- 실행 자원 및 공통 소스 동결을 root에게 반환했습니다. 아래 분석은 동결 archive와 원자료를 Windows에서 읽은 결과입니다.

주요 증거: [원 보고서](run1/report.json), [원 판정 로그](run1/follow_decisions.jsonl), [종료 무결성](integrity.json), [소유 PID 회수](execution_v1/owned_processes_after.json), [STOP ACK](early_stop.json), [런타임 audit](runtime_audit.json), [기계 집계와 원자료 SHA](analysis_v1/summary.json), [선정 원본 행](analysis_v1/selected_original_rows.json), [wall 계산 표본](analysis_v1/compute_wall_samples.json), [읽기 전용 집계 스크립트](postrun_analysis.py).

다음 판단은 저장된 GUARD 입력에서 기존 0.35/0.25/0.15 증거와 양측 위협 상태를 먼저 복원할 수 있는지 확인하는 것입니다. reference 직선 조건·1 cm 한계·seen/present/passed/vanish를 바꾸지 않으며, 새 주행이나 공통 수정을 이 결과로 자동 승인하지 않습니다.

## 후속 저장 비교용 추출

[GUARD 원본 106행](analysis_v1/guard_original_rows.json)과 [복원 가능성·추가접근·평가용 actor 대조](analysis_v1/guard_followup.json)를 따로 남겼습니다. 후보 재계산은 하지 않았습니다.

- 마지막 GUARD 진입은 seq 891 / 33.92 sim s입니다. 진입과 마지막 행 모두 양쪽 `seen=true, present=true, passed=false, vanish=null`입니다. 따라서 위협 상태가 단순 과거 기억만 남았다고 단정할 수 없습니다.
- 평가 정답상 두 actor 중심은 첫 지속 정체에서 2.52/2.87 m, 마지막에서 4.24/5.24 m 떨어져 있습니다. 인접 측방 actor와는 떨어져 있으나 센서 위협 표본의 물체 동일성은 확정하지 않았습니다. 이 정답은 평가에만 사용하며 제어 입력이나 위협 삭제 근거로 사용하지 않습니다.
- 원 0.35 후보에서 왼쪽 추가 접근이 약 0.0225 m입니다. mask 거절이 먼저 반환하므로 `current_approach` 검사는 아직 실행되지 않았습니다. 0.1 후보가 mask 한계를 만족하더라도 기존 양쪽 누적/추가접근 검사를 통과해야 합니다. 0.25/0.15의 추가접근 수치는 원 로그에 별도로 저장되지 않았습니다.
- 원 scan/history/reference/epoch/속도/조향과 header/source는 있으나 selector 내부 전체 상태의 독립 snapshot은 없습니다. 선행 plan/reset 순서를 복원하고 원 0.35/0.25/0.15 결과의 동일성을 확인한 후 동일 호출의 실제 상태에서 0.1을 한 번 비교하는 것이 최소 경로입니다. 아직 exact 복원 가능성을 실행으로 확인하지 않았습니다.
- 104개 정체의 0.15 꼬리가 1 cm를 조금 넘고 실제 속도가 0 또는 0.0612 m/s이므로 제한 저장 비교의 가치는 있습니다. 다만 0.1 추가는 전체 GUARD 검사를 한 번 더 수행하는 비용이며, 기존 0.25/0.15를 생략하거나 deadline 여유를 추정하지 않습니다.
