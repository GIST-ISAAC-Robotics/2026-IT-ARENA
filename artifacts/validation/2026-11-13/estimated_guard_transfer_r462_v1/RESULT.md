# r462 최신 r459 관측 접근의 estimated 계약 이관 대표 결과

**동일 ARM에서 실제 저장 current5쌍과 국소 plan checkpoint1쌍의 전체 출력이 exact였고, `timing_verified=true` 위장 입력은 거절됐습니다.** r459 이후 새 bounded GUARD .15가 명시 false/host-estimated 계약에서도 유지되는 대표 근거입니다. 이 범위에서 추가 정책 수정은 필요하지 않았습니다. 실물 profile 활성화·실제 센서 입력·구동 확인을 완료했다는 뜻은 아닙니다.

승인된 계산은 **정상 current10회 + 국소 checkpoint2회 + 잘못된 계약 거절1회 =13회**입니다. 새 native 빌드·전체plan prefix·r44799개 반복·ROS·센서·ESP·ARM/drive·전력·WSL 작업은 하지 않았습니다. 모든 계산은 신규 `/home/q/it-arena-r462-a`에서 했으며 원 r447 폴더와 common/r457/r460은 변경하지 않았습니다.

## 실제 저장 대표

| 원 기록 | 두 계약의 출력 | 확인한 경계 |
|---|---|---|
| r457 seq1360, plan(361,514),32.52→32.523s | .15, `rebase_current_present_guard` | plan과 current의 `observed_with_mask_budget` 유지 |
| r457 seq1986, plan(361,843),48.12→48.122s |0, `route_current_large_motion_masked_toward_right` | 새 정체의 원 거절 보존; 원인 분석은 r463 별도 |
| r452 seq519, plan(360,76) | .35, `rebase_current_present_guard` | 기존 양수를 .1로 일괄 낮추지 않음 |
| r452 seq539, 측정 −.061225042m/s | .209617158, `rebase_current_parallel` | 음수 부호 보존·기존 양수 유지 |
| r452 seq531 |0, `guard_current_new_side` | 실제 저장 유한 측방 반환의 기존 새 위협 거절 보존 |

마지막 행은 기록된 관측 반환의 거절 대표이며 물리 접촉의 증명은 아닙니다. 어느 ray/기하/속도/epoch도 장애물 반례를 만들 목적으로 변경하지 않았습니다. 5개 모두 실제 원 plan 입력·출력과 후속 current를 사용하며, 원 r452 정체의 없는 current는 만들지 않았습니다.

국소 checkpoint는 r456 당시 실제 호출에서 복제한 (360,573;35.22s)의 owner/baseline/selected를 사용했습니다. 같은 시점의 원 raw 입력으로 두 계약의 scene을 각각 복원하되 owner 상태는 유지했고, 두 결과는 .15 및 `guard_curve_current_approach_observed_with_mask_budget`로 전체 exact였습니다. 이는 `_guard_curve` 분기의 국소 동등성입니다. 전체 상태형 plan의 재생이나 실물 plan→출력 증명으로 확대하지 않습니다.

## 계약 보존과 출력 비교

r444 변환기의 원문/AST를 재사용했습니다. Windows에서 raw snapshot11개의 metadata3개를 원상복원하면 전체 원값과 일치함을 확인했습니다. 실행한 current10회에서는 request hash 불변, checkpoint2회에서는 owner 상태 hash 불변을 확인했습니다. wheel/gyro 값·음수 속도·scan 배열·점별 시각·기하·epoch는 원 저장 시뮬 값입니다.

estimated current5개와 checkpoint는 모두 **false / host_estimated_v1 / unverified_estimate**를 유지했고 명시 scope는 `host_estimated_crop_v1`입니다. current5개는 부모 full-scan equality도 true였습니다. 정상 current의 `independent_guard_required=true`, `runtime_authorized=false`도 유지했습니다. 공간 증거의 `basis=observed_with_mask_budget`가 시각을 verified로 승격한 것은 아닙니다. verified flag만 true로 바꾼 입력은 계산 입구에서 `estimated observation requires unchanged false/host-estimated input timing`으로 거절됐습니다.

**GUARD 기존 .15 단계와 estimated crop/corner 보완의 단일 .1 목표는 서로 다른 분기입니다.** .1은 원 현재 판단0인 관측 보완의 후보 목표이며 정상 양수 전체의 상한이 아닙니다. 이번 세 양수 .15/.35/.209617은 그대로 유지됐습니다. 실제 하드웨어 노드의 명시 scope·native2slot/r396 선택, planner/독립 guard 양쪽의 일관된 profile은 별도 운용 조건입니다. 공통6파일 이관만으로 실물 선택이 자동 적용됐다고 주장하지 않습니다.

같은 ARM의 current5쌍은 최종 명령뿐 아니라 region 수치·진단까지 **전체 exact**입니다. 반면 원 x86 저장 결과 비교는1개 exact, 기존 comparator4개 통과/1개 실패입니다. r452 seq519에는 r459 이전 진단 차이가 섞여 있어 최신 r459의 같은 current 저장 결과와도 **재계산 없이** 대조했습니다. 정책 진단 차이를 분리한 뒤에도 `region.queries`5550→5551, `masked_left`266→267이 current/route proof에서 각각 나타나4개 count 차이와 tinyfloat가 남았습니다. comparator false를 보존했고 허용 오차/판정선을 바꾸거나 재실행하지 않았습니다. 이 region 값도 같은 ARM의 simulation/estimated 사이에는 exact이므로 추정시각 계약의 격차라고 단정할 근거는 없습니다. [원 전체 비교](remote_result/results/summary.json), [같은 정책의 x86 비교](same_policy_cross_platform.json).

## 처리시간과 회수

전체 current Evaluator 호출의 wall time이며 restore/scene/recheck/freeze가 포함됩니다. 생성·입력 hash·결과 저장·IPC는 포함하지 않았습니다. 추가 warmup0, 첫 호출 포함, simulation5회 후 estimated5회의 고정 순서입니다.

| 대표 | simulation ms | estimated ms |
|---|---:|---:|
| r457:1360, 첫 호출 |69.05|69.02|
| r457:1986 |53.23|53.16|
| r452:519, dataset 첫 호출 |124.31|124.19|
| r452:539 |27.33|27.23|
| r452:531 |24.83|24.77|

estimated n=5 중앙53.16ms/max124.19ms,100ms 초과1개입니다. 국소 checkpoint와 잘못된 계약 거절의 시간은 별도 계측하지 않았습니다. 선택된5건의 비용이며 전체ROS10Hz/센서 주행/시간 정확도의 증거가 아닙니다. [원 처리시간](timing.json).

r447 native 소스3개+setup.py가 현재와 같음을 확인하고 기존 ARM binary를 신규 경로에 복사했습니다. SHA는 `a2afb70525be1e86f63ae072abc4fbaeec10d65f0596e5f642f135ba6a1de394`이며 새 빌드나 패키지 변경은 없습니다. 처음 SSH 준비 명령은 한국어 주석의 인코딩 때문에 Paramiko dispatch 전에 실패했습니다. 원 exit1/오류를 보존하고 ASCII 명령으로 수정해 extraction/unpickle을 완료했으며 정책 계산을 중복 시작하지 않았습니다.

실제 계산 자식 PID60593/60599와 요약 PID60606은 모두 exit0, forced0, 잔존 process group0입니다. 사후 관련 프로세스0/열린 센서 포트0, 입력 source239파일 변경0을 확인했고 Jetson 계산 자원은 주 담당에게 반환했습니다. 회수 ZIP SHA는 `7b06a6c05139887d5855f64de8b1924842585150fc340a6961684a3c21274037`입니다. [실행 receipt](remote_result/execution_receipt.json), [자원 반환](remote_result/resource_return.json), [준비 manifest](source_manifest.json), [실행 검토](execution_review.json).
