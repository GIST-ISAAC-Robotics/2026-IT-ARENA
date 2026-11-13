# r445 GUARD 저장 비교 — baseline 복원 불일치에서 중단

2026-11-13. GPT가 r441 동결 소스·native 바이너리·원 입력만 복사한 독립 stage에서 순차 plan 재생을 확인했습니다. **최초 실질 baseline 불일치인 trace seq 441 / reference 11.52 sim s에서 중단했으며, 0.1 m/s 후보 계산은 0회입니다.** 결과 필드를 제외하거나 내부 상태를 원 출력에 맞춰 채우지 않았습니다. 공통 파일·설치·빌드·새 주행·실물 접근은 없습니다.

## 확인된 결과

- r441 동결 archive SHA: `c032a4cf5b689bde9e150103a65847d783f9d64c56c2b15513840345d653ee5d`.
- 원 `follow_decisions.jsonl` SHA: `9a160ab2daeaa43722d55447a6e882d8572aa85e98459415797ff00fb5b3f5aa`.
- v3에서 **실제 고유 plan 호출 38개 중 37개 exact**, 재전달 기록 1개도 exact입니다. 수신기록 기준으로 39개 중 38개 exact, 다음 1개에서 중단했습니다. 결과 JSON의 `unique_plan_calculations=37`은 cache에 저장된 **exact 계산 수**이며, 불일치한 마지막 실제 호출까지 합치면 38개입니다.
- 불일치 seq 441 / worker generation 361, sequence 76에서 실제 출력에는 `recent_boundary_confirmation=current_observation_after_memory_expiry`가 추가됐습니다. `known_structure.hidden_memory_added/released/points`도 각각 재생 24/28/34, 원 기록 16/16/37로 달랐습니다. 속도·조향만 같다는 이유로 동일 판정을 내리지 않았습니다.
- 직전 수집 plan은 worker 72 / reference 11.32 sim s입니다. 원 trace 전체 중첩에서 worker 74/75의 입력·출력은 발견되지 않았습니다. 빠진 선행 계산이나 전달 상태가 원인일 가능성은 있지만, 이 자료만으로 해당 계산의 실행·내용을 확정할 수 없습니다. 11.42 입력이나 내부 기억을 추정 복원하지 않았습니다.
- 따라서 정체 구간 GUARD 상태의 exact 복원, 0.1 후보 수용 수, 추가 접근 거절, 추가 계산비용은 **미확인**입니다. baseline 자체가 맞지 않는 상태에서 0.1 성공률을 만드는 것은 하지 않았습니다.

## 어댑터 오류와 환경 오류 보존

1. v1은 Evaluator 생성 전 header `runtime_policy` 누락으로 종료했습니다. 원 trace는 이 설정을 별도 `runtime_factory_policy` 행에 기록합니다. baseline/후보 계산 0회이며 [원 로그](run.log)와 [종료 감사](handoff_audit.json)를 보존했습니다.
2. v2는 별도 policy 행을 동결 `runtime_node_backend.evaluator_header`의 실제 경로로 결합했습니다. [파생 header](result_v3/derived_header.json)와 [키별 출처](result_v3/derived_header_sources.json)에 근거를 남겼습니다. seq 369와 371이 동일 worker 결과의 재전달인데 plan을 두 번 호출한 어댑터 오류에서 중단했습니다. [v2 원 결과](result_v2/summary.json)를 보존했습니다.
3. v3은 `(stage, generation, sequence)`별 실제 계산을 한 번만 수행합니다. 재전달은 input·원 output·계산/제출/수신 시각 네 값이 모두 같은지 확인하고 보존 출력과 exact 대조합니다. generation 변경 시 실제 worker 방식으로 reset을 적용하며 재전달은 상태를 전진시키지 않습니다. 원 trace의 **370개 call 기록은 고유 plan 결과 353개 + 재전달 17개**입니다. 17쌍의 입력·출력·시각 모두 exact입니다([감사](v3_delivery_preflight.json)). r441의 370개 비용 표본은 수신기록 기준이므로 고유 계산 횟수와 구분해야 합니다.
4. v3 최초 두 WSL 호출은 `Wsl/Service/E_UNEXPECTED`였습니다. [첫 오류](v3_launch_error_1.json)·[재확인 오류](v3_launch_error_2.json)를 보존했습니다. root가 기존 승인으로 Ubuntu-24.04만 재시작한 뒤, 21:15:23 KST에 전용 root 부재와 r445 프로세스 부재를 확인해 실행 소유권을 다시 배정했습니다. 그 후 v3을 실행했으며 앞선 미확인 상태를 소급 성공으로 바꾸지 않습니다.

## 센서 위협과 벽 형상 조사 경계

원 동결 코드 `pass_release_trial.threat_view`는 같은 scene의 유한 반환을 `m3_execution_trial.side_return_masks`로 선택합니다. 현재 경로에 투영한 횡위치가 고정 벽 offset/tolerance 띠 밖이면서 내부 창에 있고, 차체 측방보다 바깥이며 compact component인 점을 위협으로 셉니다. `lateral_intent.compact_return_mask`는 source index 단절 또는 인접점 간격으로 묶음을 끊습니다. 따라서 가려져 작게 분할된 벽 반환이나 현재 경로와 다른 벽 형상이 차량 크기의 위협 묶음처럼 남을 가능성은 있습니다. **이는 코드상 가능성이며 이번 정체의 실제 오분류를 증명한 결과가 아닙니다.**

r441 원 기록에서는 마지막 양쪽 `seen=true, present=true, passed=false, vanish=null`이고, 원 0.35 후보의 왼쪽 누적/추가 접근이 한계를 넘어 검사 대상입니다. mask 거절이 먼저 반환하므로 `observed_guard_approach`에는 도달하지 않았습니다. 0.1이 mask를 통과해도 그 추가 접근 검사를 모두 통과해야 합니다. `present`를 단순 과거 기억이라고 삭제하거나, 긴 반환열을 벽이라고 확정해 장애물 검사에서 제외할 근거는 확보하지 않았습니다.

root가 허용한 후속 센서 전용 분석에서는 기록된 scan/history/path로 `ReturnGrid/side_return_masks`를 복원했고 원 evidence 점수·위치가 일치했습니다. [센서 독립 조사 결과](SENSOR_FINDINGS.md)에 따라 **단순 벽 조각 오인으로 지울 근거는 부족**합니다. 오른쪽 전경 반환은 인접 배경 선보다 약 14 cm 앞에 있고, 왼쪽도 인접 유한 반환과 큰 깊이 단절이 있습니다. 전체 plan 복원이나 0.1 후보 수용 증거는 아닙니다. 다음 우선순위는 실제 plan worker의 누락 없는 계산 입력/출력 연결이며 새 주행·공통 연결은 이 결과에 포함되지 않습니다.

## 실행·원본 회수

v3 자식 exit 2, PID 781 회수, 잔여 소유 작업 0, **226개 stage 입력 변경 0**입니다. 모든 `arena_autonomy` import가 전용 `/home/jinhyeong/it-arena-r445-guard-c/src/arena_autonomy` 아래임을 확인했습니다. WSL을 root에게 반환했습니다. 이후 집계는 Windows 읽기 전용입니다.

주요 증거: [v3 결과](result_v3/summary.json), [개별 baseline 행](result_v3/baseline_rows.json), [최초 불일치 4개 필드](first_genuine_mismatch_details.json), [누락 worker 입력 검색](missing_worker_input_audit.json), [import 경로](result_v3/module_paths.json), [종료 감사](handoff_audit_v3.json), [원 source 목록·SHA](stage_v3/manifest.json), [v3 준비 영수증](prepared_v3.json). v1·v2·v3 묶음은 각각 별도 보존했습니다.
