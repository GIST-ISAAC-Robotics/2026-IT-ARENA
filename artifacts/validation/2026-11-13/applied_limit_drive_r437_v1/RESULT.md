# r437 조향 연결 수정 후 제한 주행 결과

2026-11-13, GPT. **기존0.45 rad보다 큰 조향에서 발생하던 명령 만료 경로는 이번 대표에서 나타나지 않았지만, 주행은10.08 m에서 정체해 미완주했다.** 주 담당 검토 뒤 고정 주행1회만 수행했다. 원 `passed=false`, `harness_valid=false`, `run_valid=false`, `no_permanent_stall/full_lap/data_coverage=false`와 로그를 보존하며 재실행하지 않았다.

## 준비·조건

r431 대비 검토된 공통14파일(r430 네 파일·r429 여덟 파일·r436 두 파일, 기존11변경/신규3)만 포함했다. r429/r430은 integration의 stage SHA와 정확히 일치했다. 새 colcon3패키지 빌드와 실제5모듈 import 경로 확인은 exit0, 기존 사전 검사16개는 실패0/skip0이다. 자기 install underlay에 대한 colcon 경고는 원 build.log에 보존했다.

config/actor/header/launch 선택·물성·속도·판정 기준은 설명/새 경로 외에 r431과 같다. 80 sim초·RTF0.25·8 sim초 무진행 STOP을 유지했다. 수동 패널 모드는 활성화하지 않았고 speed cap은 기존 maximum 기본값, 설치 profile export는 실행하지 않았다. 따라서14파일을 함께 통합한 대표이며 조향 한 파일만 달라진 엄밀한 단일 변수 비교로 설명하지 않는다.

동결1,347입력의 prepared SHA는 `8dc275de3c88cf192ed30ea55ba769f4defb9c4cdc1425e996e5285967854697`, archive SHA는 `e0342d9476e3fe50148403638200f485a60a12742b35a5f97fd7a9d6ebf9b3db`다. [준비 manifest](preparation_manifest.json)와 [주 담당 실행 검토](root_execution_review.json)에 근거를 남겼다.

## 조향 수용과 원 실패의 구분

- MCU 기록에서 조향0.4578436017036438 rad가 적용된19.260–19.460 sim초의50개 상태가 모두 ACTIVE·`output_ready=true`다. 속도 명령은0.649999976 m/s이고 accepted seq913→922, deadline19.350→19.530초로 갱신됐다.
- 전체 MCU 기록의 `command_expired`와 `FAULT_LATCHED`는 각각0개다. 출력기는 최종 STOP까지 상태를 수용했으며 최종 사유는 `software_stop`, 마지막 상태시각32.030초다. r431의 `last_status_us=19.377` 정체/만료와 다르다.
- 이는 이미 선택한 차량 조향 한계의 출력기 전달을 이번 대표에서 확인한 결과다. 실물 조향·실차 watchdog·전체 조건 검증은 아니다.

## 정체·STOP·회수

출발 신호7.401초 뒤31.141초에서 원 case 관측을 마쳤고, 최종 진행은10.08 m다. 원 evaluator의 wall 실행시간은381.224초다.

마지막 진행 증가 관측은21.289초다.29.273초 표본에서 무진행7.984초, 다음30.487초 확인에서9.198초로8초 기준 초과를 확인해 소유 STOP helper를 한 번 호출했다. **정확히8초 시각에 STOP 요청했다고 기록하지 않는다.** 원 helper에 서비스 요청 정확시각이 없어 최초 호출 시각을 추정하지 않는다. 관측·호출 기록은 [stop_requested.json](execution_v1/stop_requested.json)에 보존했다.

`/autonomy/enable false`와 `/actuation/stop`은 모두 ACK, helper exit0이다. 첫 MCU latch는31.142초의 `ESTOP_LATCHED/software_stop`, 후행 evaluator STOP pose시각은31.533초(`/clock`31.534초)다. 따라서 원 `abort=mcu_latched_before_stop: software_stop`와 `no_mcu_latch_before_stop=false`는 유지하되, helper 이후의 래치를 앞선 주행 정체 원인으로 해석하지 않는다. 각 노드의 clock 수신 시각만으로 정확한 서비스 지연을 계산하지 않는다.

STOP 안정화·명시 MCU STOP·연속 pose 검사를 통과했고 변위는0.000151639 m다. 벽 접촉 표본0, actor contact 이벤트0, 평가 road outside 표본0이며 기존 two-wheel surface gate는 true다. 자식5개 exit0·launch14/14 clean·강제회수0·잔여PID/소유 partition0을 확인했다. 원 run exit1은 실패 판정을 보존한 정상 회수이며 성공 코드로 바꾸지 않았다.

시험 중 frozen 입력 변경0, prepared/archive 불변을 회수 후 확인했다. WSL/Gazebo는 반환했다. 이후 주 담당이 공통 코드를 변경할 수 있으므로 이 불변 판정은 **해당 주행과 반환 직전 검사 시점**의 결과다.

## 마지막 정체 창의 저장 입력 집계

창은 마지막 진행 증가21.289초부터 첫 MCU STOP latch31.142초까지다. plan/current는 input reference, parent는 `at_s`로 골랐으며 서로 다른 노드의 callback 시각이 같다고 간주하지 않았다. 새 재생·주행 없이 원 follow/actuation 자료만 집계했다. 상세 위치·대표 입력은 [stall_window.json](execution_v1/stall_window.json)에 있다.

| 단계 | 저장된 결과 |
|---|---|
| plan98회 | `local_follow`96, `follow_partial_transformed_gap_uncovered`1, `follow_release_wait`1 |
| current96회 | 모두 `route_current_large_motion_region_unobserved`, 양수 출력0 |
| crop retry96회 | 모두 `current_crop_full_region_rejected`; 최종 수락0·양수 부모 발행0 |
| crop proof72회 | `accepted=true / bounded_same_crop_current_reach_risk`였으나 최종 `large_motion_masked_toward_left`로 거절 |
| crop proof24회 | `accepted=false / different_current_whole_component`; 최종 `large_motion_region_unobserved`로 거절 |
| parent 이벤트96개 | 모두 speed0·retained=false. 이벤트 존재를 양수 경로 발행으로 세지 않음 |

대표 seq643(JSONL644행)은 current reference21.522초·측정속도0, core 재검사0.2 m/s를 냈지만 전체 영역1,628질의 중 미관측429·masked_left680을 기록했고, crop proof만 통과한 뒤 `large_motion_masked_toward_left`로 최종 거절됐다. seq645(646행)는 reference21.622초·측정속도0, 전체1,566질의/미관측370·masked_left680이며 crop proof에서 다른 전체 성분으로 거절됐다. 여기서 새 후보 설계나 허가 기준 변경은 하지 않았다.

plan 입력 모터속도는98회 중 양수1(+0.061225 m/s)/0값97회, **current96회는 모두0**이다. 이 창의 current 거절을 음수속도 허용범위 문제로 설명할 근거가 없다. 같은 창 MCU의 고유 feedback seq986개에는 양수7/0값978/음수1(−0.061225 m/s)이 있으나, 이 음수 표본을 current 입력과 동일 시각으로 연결하지 않는다.

전체 실행에서도 crop retry96/수락0/양수 부모발행0이다. 별개의 기존 corner-distance-budget은 수락6·부모발행6이며 새 crop 후보 성공과 합산하지 않는다. `known_structure_sectors_json`의 JSON 공백 차이로 header exact는 false, strict 구조 비교 equivalent는 true이며 원 audit의 두 판정을 모두 보존한다.

원 r431 실패 원본은 수정하지 않았다. 이번 결과는 조향 전달 오류 확인과 남은 current 전체 영역 거절을 분리하는 근거이며, 기존10.10 m 정체를 해소했다고 주장하지 않는다. 후속 저장 입력 원인 분석과 변경 판단은 주 담당에게 넘겼다.

근거: [실행 영수증](execution_v1/receipt.json), [원 report](run1/report.json), [원 runtime audit](runtime_audit.json), [STOP 순서](stop_sequence.json), [입력 무결성](integrity.json), [잔여 프로세스 확인](execution_v1/owned_processes_after.json).
