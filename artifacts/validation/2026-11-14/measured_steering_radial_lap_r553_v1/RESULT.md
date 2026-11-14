# r553 단독 주행 결과

2026-11-14 GPT 실행·회수. 승인된 PC Gazebo 단독 주행 1회, PID **44714**. **원 판정 FAIL**을 보존했다. 기존 19.56 m와 25.81 m 정체 위치를 지나 **25.94 / 44.64 m**까지 진행했으나 새 정체로 완주하지 못했다. 재실행하지 않았다.

## 주요 결과

- 원 `passed=false`, `run_valid=true`, `harness_valid=true`. GREEN 후 62.630 sim초, 원 최장 정지 9.996초/총 정지 10.900초. 기존 19.56 m는 t=35.137, 25.81 m는 t=58.377에 통과했다.
- 벽 overlap **0 / 31,568** 평가 표본, 노면 밖 바퀴 중심 최대 2개, 3개 이상 0표본. 원 `two_wheel_surface=true`, 과거 body-only road gate는 false 그대로다. 표본 기하 검사이며 연속 실물 무접촉 증거가 아니다.
- 실제 활성 RTF **0.08480**: guard reference 7.42→70.02 sim초 / 평가 수신 wall 738.167초. 전체 원 wall time 845.980초. 설정 RTF는 기존 0.25이다.
- 조향 **7/7** 실제 launch/SDF 검사 통과. controller/guard 중앙 상한 `.28475436568260193 rad`, MCU/actuator 물리 중앙 `.28475438456006436`, plugin/양쪽 joint 내륜 `.30666234131856146`. 바퀴각 실측으로 확대하지 않는다.
- 발행 1,131개 중 양수 1,028개(90.89%), 상한 근접 583개, 상한 초과 0개(허용차 1e-6 rad). 최대 절대 발행 조향 `.28475436568260193`.
- GREEN-window trajectory t=7.443→69.983의 621표본을 다음 표본까지 유지하여 시간 가중한 `/drive` 양수 비율 **83.21%**, `/drive/safe` **83.05%**. 양수는 >1e-6 m/s. safe 양수이면서 <=.100001 m/s인 저속 최장 **9.860초**, 총26.082초. safe 0 최장10.096초/총10.600초. 원 ground stationary 정의와 구분한다.
- helper plan 기록360개, helper 없는 정상 양수 FOLLOW plan257개. 계획 기록 수이며 실제 이동 횟수가 아니다. full 상태 원문3,526행, follow trace3,927행, trace drop/failed/pending/capture error 모두0.

## 새 정체 발췌

[transition.json](stall_extract/transition.json)에 원 current/compute 입력·call·plan·guard·trajectory를 분리 저장했다. 마지막 양수 발행 seq3484 t59.82 retained=true .1/−L; 최초0 발행 seq3487 at59.824 `not_published_as_checked`. 대응 call3486 ref59.82는 `candidate`/`local_follow` .1/−L다. 이후103 call은 모두 양수 candidate이며 0-output candidate/discarded는 없어 absence를 기록했다.

최초 실제 drive0 trajectory는 t59.887, [동시점 guard 원문](stall_extract/first_zero_drive_guard.json) ref59.88는 clear/safe0/command_stale=false다. [최초0 발행 current 입력](stall_extract/first_zero_publication_call_current_input.json), [compute 입력](stall_extract/first_zero_publication_call_compute_input.json), [원 plan](stall_extract/first_zero_publication_call_plan.json)을 주 담당에게 먼저 전달했다. 원인 해석은 별도 분석 범위다.

기존 발췌기는 0-output candidate가 없어 StopIteration으로 종료했다. 이는 종료 후 저장 입력 발췌 오류이며 주행 실패와 구분한다. [초기 오류](extract_stall_initial_failure.json)·초기 도구를 보존하고 실제0 publication 기준으로 수정했다. 원시/원report 변경 및 제어기 재생은 없다.

## STOP·정상 종료·입력

마지막 진행 t59.887 이후 t68.769에 **8.882 sim초 무진행**을 관측해 기존 STOP을 1회 실행했다. disable/STOP 모두 수락, ESTOP_LATCHED/출력0을 확인했다. polling·서비스 처리 중 추가 시간도 보존한다. 외부 STOP에 따른 원 `mcu_latched_before_stop: software_stop` 및 `no_mcu_latch_before_stop=false`를 그대로 남겼다. 다른 STOP5개 true와0.5 sim초 dense pose 정지 검증 통과를 구분한다.

**14/14 child 정상 종료**, 강제 종료·잔존PID·미회수reader0, Gazebo 종료응답 성공, `shutdown_clean=true`. 독립 사후 PID0 및 process/integrity/collection exit0을 확인하고 WSL을 주 담당에게 먼저 반환했다. 이후 Windows 집계만 수행했다. Jetson 접근0.

필수 입력457개/관측990개/ZIP 모두 불변, actual source map451개와 snapshot의 누락·해시 불일치0이다. 미사용 UI race_ops_node/page는 package 전체 수집에 포함된 관측 항목으로 필수 입력과 구분했다. `arena_autonomy`만 빌드(9.68초,exit0), native 바이너리 불변, 실제 import11개를 확인했다. r552 최종5소스 manifest와195+12 native검사 근거를 재사용했다. measuredYAML·actor0·시설3·80sim상한·8sim무진행·속도·노면·판정선·full상태는 r550와 같다. 첫 직선2벽만 변형된 가정 민감도 월드이며 실측/전구간 구불벽으로 주장하지 않는다.

- [원 report](run1/report.json) SHA `b416f2a622514f6a6b648d07eec77d8f83e6111348652cbdaaaf512121e7d017`.
- [prepared](prepared.json) SHA `5e91fe976a2ba9cb4b57973ccd7878aea24ca2d4dfabba55776ac57110d36aff`.
- [sources.zip](sources.zip) SHA `0881ac6243f8670af52bf3e0e0fd73e8332d95323ca0483176a69b0e68605ba1`.
- [실제 입력 manifest](actual_execution_inputs.json) SHA `5452d49d9aa73fd955963686d3ad9049f4ccc655dc07a2dd02f22dd5b3b92643`.
- [전체 수집](evidence_collection.json), [보충 집계·정의](run_supplement.json), [조향 영수증](steering_runtime_receipt.json), [사후 PID](execution_v1/owned_processes_after.json), [수집 종료코드](execution_v1/collection_exit.json).

이전 r544/r550 실패·원시·ZIP와 이번 원 실패를 보존했다. 다음 별도 시험은 새 승인·별도 경로에서만 수행한다.
