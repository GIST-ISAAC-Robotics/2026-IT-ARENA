# r411 현행 runtime 정책의 Jetson 계산 비용 — 1차 범위 제안

> 아래는 준비 당시 계획이다. 이후 새 WSL80·Jetson380 계산과 회수를 완료했다. 정확 일치 실패의 분류와 비용/한계는 [최종 결과](RESULT.md)를 따른다.

2026-11-13. **계획과 저장 입력/소스 계보 조사만 완료했다.** 번들·재생기·실행 명령은 아직 만들지 않았다. 공통 소스/다른 문서 변경, 제어기 import/계산, native 빌드, WSL/Jetson/센서/ROS/Gazebo 실행은 없다. 주 에이전트가 아래 범위를 확정한 뒤에만 구현·동결 묶음을 준비한다.

## 권장 범위

**최신 r406 최종4가림 기록의 계획 prefix37개와 독립 현재 입력153개를 사용한다.** 동일 현행 common/native에 거리 후보 `off`/`r396_v1`만 다르게 선택하여 각 arm에서 한 번씩 계산한다. 전체380호출이며 ABBA·반복 부하·r382 옛572개 전수 재계측은 하지 않는다. 계획 prefix는 원시 입력/순서를 복원할 수 있는 구간만 사용하고, 현재 계산은 기록된 원 계획 결과를 고정 입력으로 삼는다.

계획과 현재 결과를 연결해 새 폐루프를 만들지 않는다. 현재153개 중 각각의 `compute.plan.output`은 당시 저장된 원 계획이다. 이번에 새로 계산한 prefix 계획 결과로 이를 바꾸면 원인과 입력이 달라지므로 금지한다.

| 단계 | 선택 이유와 원 기록 | 범위·한계 |
|---|---|---|
| 계획 OFF/ON 각37 | generation361, 공유 sequence1..73 안의 계획37개, reference7.72..11.32초. 원 결과 양수35/0출력2, 모두SOLO | 원265 call행을 그대로265번 계산하지 않는다. 중복11행 제거 후 고유254개 중217개는 제외 이유를 기록한다. 후반 코너/GUARD/RELEASED 계획 비용을 대표하지 않는다. |
| 현재 OFF/ON 각153 | 원 `current_rebase`153개 전체. SOLO140/GUARD13, 원 선택ON 결과 양수145/0거절8, 후보 적용1 | 기록된 완료/부모 처리 결과의 표본이다. 당시 제출·계산한 모든 current의 전수나 실제 발행·주행 비율이 아니다. RELEASED 표본0. |

세 가지 필수 표본은 이 범위에 모두 있다. 일반 양수는 current index0부터 포함한다. 0거절은 index42/43/44의 `guard_current_new_side` 및 뒤쪽 `route_current_large_motion_region_unobserved`5개다. 거리후보 적용은 **index137, trace_seq646, current sequence296, reference21.522초, 기록 목표0.09509513866308608 m/s**다. 원 기록의 측정속도0은 시뮬레이션 원 입력 그대로이며 실제 encoder 관측으로 승격하지 않는다.

후보 적용 비용 표본은 원 기록상1개뿐이다. 해당 호출의 비용/추가량을 개별로 보고하고 p95 모집단처럼 포장하지 않는다. 선택 결과가 실제 ARM에서 달라지면 그 사실을 별도로 남긴다. 비용 개선·적용 비율을 만들기 위해 과거5가림 성공 표본을 자동 보충하지 않는다.

## 복원 가능성과 제외 근거

입력은 `../final_runtime_drive_r406_v1/run1/follow_decisions.jsonl`이며 SHA256은 `fb17ea629430f184f53172738ca21c51e4c5f3c02a5a27bd7bf9c99930e2f01b`, 크기는48,434,282 B다. 원 실패/STOP 판정은 유지한다. 결과 문서와 header 감사는 입력 출처 확인용이며 정답 위치/평가자 상태를 제어기에 전달하지 않는다.

- 원 call265행의 `(compute.generation, compute.sequence)`를 구별하면 계획254개다. 중복11행은 입력/출력의 canonical JSON이 모두 같다. **같은 계획을 다시 사용한 행을 새로운 stateful 계획 호출로 세지 않는다.**
- `runtime_dual_process.py`와 `runtime_dual_state.py`는 계획/현재가 하나의 sequence를 증가시킨다. 계획 ID만의 공백은 누락 근거가 아니다. 두 단계 ID를 합치면 generation361의1..437 중407개가 원 입력과 함께 확인된다.
- 공유 sequence 누락은30개다. current78/251/290/297/319의5개는 `parallel_discard`에서 단계/거절 사유만 확인된다. 해당 입력은 없다. 나머지25개는 trace만으로 단계/입력까지 확인되지 않는다. 전체 목록은 `lineage.json`의 `sequence_coverage`에 있다. footer의 dropped0/capture_errors0은 파일 기록 손실이 없다는 근거이지 모든 자식 계산 입력이 저장됐다는 근거가 아니다.
- 최초 복원 불가 번호는74다. 따라서1..73을 온전히 설명할 수 있는 **처음37계획**만 원 순서로 복원한다. 초기 reset은 기록된 `WAIT_GREEN`, generation361이다. 이후217계획을 fresh evaluator에서 이어 계산하고 원 자식 전체 이력이라고 주장하지 않는다. 입력이 없는 번호에 임의 상태/스캔을 끼워 넣거나 새 reset을 삽입하지 않는다.
- 현재153개에는 각각 원 `compute.plan.input/output`, 현재 `input`의 원 scan/점별 시각/gyro·wheels 이력/reference/epoch/이전 조향/실제 측정속도/dt/profile이 있다. 현재 evaluator는 `restore_call`→`motion_scene`→공통 rebase를 해당 자료와 config로 구성하므로, 이후 계획 이력의 누락과 별도로 **독립 현재검사 입력**을 복원할 수 있다. 이는 정적 데이터/호출 경로 확인이며 native 재현 통과는 아직 아니다.
- G1G2 trace writer는 `data.output=item.result.output`으로 원 plan 결과를 저장하고 `current_rebase.output`을 따로 둔다. 따라서 부모의 current 합성 명령을 원 계획 oracle로 혼동하지 않는다. 계획 후보상태는 순서대로 유지하며 중복 사용행은 다시 실행하지 않는다.

`lineage.json`은 전체254개 계획의 선택/제외를 식별할 ID·trace 행·원시각·입력/출력 해시와 현재153개 매핑을 담는다. 원 센서 배열이나 전체 trace를 새 문서에 복제하지 않았다. r410 담당자에게 공용 sequence, 중복 제거,37개 prefix 한계, 원 plan 출력 위치를 직접 공유했다. 작업 파일 소유권은 별개다.

## 소스와 기존 근거의 구분

현행 `src/arena_autonomy/arena_autonomy/race_runtime_policy/`는 **23 Python 파일**이다. r398의 원 알고리즘 ledger17개 + package init/evaluator2개 + r405의 corner_budget/corner_candidate/range_mask_component/profile_evidence4개다. 조사 시점23개 모두 r406 `prepared.json.frozen_inputs`의 해시와 같으며, ledger17개는 r398 staged 해시와도 같다. 선택·프로세스·trace 경계7파일도 r406 동결 해시와 같다. 전체 import 폐포/native 원문은 범위 승인 후 별도 manifest로 동결해야 한다. 이 계보 조사는 실행용 최종 source freeze가 아니다.

| 기존 자료 | 확인한 사실 | 이번에 반복하지 않는 것 |
|---|---|---|
| r382 `recovery_r1` | r375 top-level 호출의 옛 저장572개+17반례. 적용93개 p50/p95/max89.648/91.296/91.768 ms, 원 pipeline 보고 오류와 x86 재현 실패 보존 | r375 옛 wrapper/기하 계측을 현행 패키지 비용으로 인용하지 않는다.572/589호출·ABBA·query probe를 다시 실행하지 않는다. |
| r398 `review/README.md`와 `review/verify_native_v1/report.json` | README는 준비 시점 기록이지만 실제 report는 native plan24/current223 비교 통과. current 모드는 **SOLO208/GUARD15** | 존재하지 않는 root README를 근거로 쓰지 않는다. 전체가SOLO라는 후속 문장도 쓰지 않는다. 기존20검사/plan24/current223 이관검증을 새 성과로 반복하지 않는다. |
| r405 RESULT/integration |46검사·223 OFF/ON exact, 원 양수119→213·적용94·비대상129·3반례 통과. 실제 공통 연결 | 원5가림 r377 코퍼스와 이번4가림 r406을 섞지 않는다. 같은223/원795 대조를 다시 하지 않는다. |
| r404 실제 수신 | C1/RGB/IMU와 guard0명령·정상 종료 근거. 실제 encoder 없음 | 센서 없이 저장 wheels를 실센서에 끼워 넣어10Hz나 전체 계획 처리율을 만들지 않는다. |

r398/r405 검증은 이미 된 연결 근거이며 이번 목적은 **그 이후 현행 common과 최종4가림 입력의 Jetson 비용**이다. `JETSON_RUNTIME_PLAN_2026_11_12.md`의 r232187.48ms나 r358 계획약132/current약61ms도 구성·입력·계측 범위가 다른 역사 자료다. 이번 결과의 before 값이나 개선율 분모로 사용하지 않는다.

## 승인 후 구현할 좁은 계측 계약

1. 원 r406 header의 Follow/Execution/MotionConfig를 그대로 사용한다. 원4가림 JSON의 공백을 포함한 기록 원문도 보존한다. `runtime_factory_policy`와 명시 native 등록에 따라 evaluator header를 구성하고 추가한 필드의 출처를 기록한다. final YAML에서 다른 값을 다시 생성해 덮지 않는다.
2. OFF는 `current_corner_budget=off`, ON은 `r396_v1` 및 현행 validator가 요구하는 verified simulation/native2slot 명시 환경만 다르다. 원 `laser_frame`, `simulation_acquisition`, timing_verified=true/verified_only, 원 source/reference/epoch/dt를 유지한다. `host_estimated_v1`로 바꾸지 않는다. 실제 Jetson에서 계산한다는 사실은 입력이 실센서라는 뜻이 아니다.
3. 기존 `Evaluator`와 `Lane`를 재사용한다. 순서는 계획OFF37→계획ON37→현재OFF153→현재ON153, 한 번에 자식1개다. arm별 fresh evaluator를 만들고 계획은 원 순서·reset을 지킨다. startup·첫 호출 비용을 별도 표시하며 임의 warmup 반복은 넣지 않는다. 실행 순서/캐시/온도 영향이 있는1회 계측임을 남긴다.
4. artifact의 최소 timing wrapper로 `Evaluator.__call__` 전체 시간을 측정한다. restore/scene/공통 baseline/선택 거리제안·최종 core·성분 검사/freeze를 포함한다. wrapper 밖 입력 deepcopy·결과 diff·파일 기록 시간은 분리한다. Lane의 기존 began/completed/received는 JSON decode/encode와 IPC/회수 지표로 따로 보존한다. 측정용 전역 monkeypatch나 공통 알고리즘 수정은 하지 않는다.
5. p50/p95/max, >100ms/>200ms 개수와 원행을 plan/current 및 OFF/ON·SOLO/GUARD·양수/0·후보 비대상/제안거절/최종검사거절/적용으로 나눈다. 원 기록과 새 ARM 결과의 분포를 각각 적고, 현재 같은 입력의 ON−OFF 비용은 paired difference로 표시한다. p95끼리 더하거나 차감하여 종단 지연을 만들지 않는다. 표본1은 개별값으로 보고한다.
6. 계산 입력의 byte/canonical hash·원 generation/sequence·source epoch/time, 전체 raw 출력, native ABI/소스/실제 import 경로와 child 종료를 기록한다. numeric backend fallback이나 외부 프로젝트 import는 실패다. native 빌드/의존 import 사전 확인은 승인 후 새 독립 root에서만 한다. 과거 동결 bundle에 현재 파일을 덮지 않는다.

## 정확 비교와 중단 경계

- 비교는 원 r406 x86 기록과 현행 ARM, 현행 ARM OFF/ON의 관계를 구별한다. 원 계획37개/현재ON153개의 exact 비교를 별도 보고한다. 기존 `comparable_decision`이 정규화하는 측정 processing_ms 위치 이외에 새 필드를 빼거나 tolerance를 확대하지 않는다. raw는 그대로 보존한다.
- ARM/x86 exact 차이가 있으면 첫 경로뿐 아니라 전체 diff와 원값을 저장한다. command/state/reason/time/허가 boolean, region 개수, 경로·기하 부동소수 차이를 별도로 집계한다. r382처럼 정확 재현 실패를 비용 완료와 분리하며, 이후 수치가 맞을 때까지 설정/입력을 바꾸지 않는다. ARM 적용 개수가 원1과 다르면 이식/재현 차이로 명시한다.
- 같은 ARM에서 계획OFF/ON 결과·선택기 상태 동일, current 비대상 불변, 미적용 후보는 기존 diagnostic 추가 외 동일, 적용 후보는 원 단일 제안/최종 검사·원시각 보존이어야 한다. 이 불변을 깨면 원 실패를 저장하고 중단한다. x86 차이만으로 원 결과를 성공으로 소급하지 않는다.
- 원 데이터/header/source 누락·해시 변화, 입력 변조, import/native 오류, 자식 예외·비정상 종료·잔여 PID, 출력 수/순서 불일치가 있으면 중단한다. 복원 불가 표본을 조용히 제거하거나 current의 원 plan 출력을 새 결과로 교체하지 않는다.
- 100ms/200ms는 계산 예산 참고다. 저장 입력을 직렬로 호출하는 시험에서 명령 만료 통과/실패를 판정하지 않는다. 계획과 현재의 실제 겹침·부모50Hz·센서/영상·독립 보호·MCU lease·실차 처리는 이번 계측 밖이다.

후속 실행 제안은 fresh Jetson root1개, 단독 소유, native build 최대180초·import30초·계산/비교 최대180초다. 계산 입력380개를 한 번씩만 수행하며 상한 초과는 실패/회수한다. 전력/클럭/스레드 정책을 자동 변경하지 않고 시작/종료 상태만 기록한다. 최종 명령·경로·source manifest·중단/회수 스크립트는 **이번 범위 승인 뒤** 준비한다. 지금은 이 PLAN과 lineage만 인계하고 대기한다.
