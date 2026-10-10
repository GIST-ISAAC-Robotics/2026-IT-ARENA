# 격자 실행기 관문(Gate A)·플러그인 명령 전환 계측(Gate B)·1 ms 반복 분리(Gate C) — 사전 등록

작성: 2026-10-09, 실행 전. 이 절(“사전 등록”)은 `pre_run_freeze.sha256`으로 동결하며 결과는 아래 “결과” 절에 덧붙인다.
역할: GPT 주 에이전트가 범위·수용 조건·최종 검토, Claude Opus 5.5가 조사·구현·분석·문서, 시뮬레이션 실행·대기·회수는 사용자 지정에 따라 Claude Sonnet 5.5 서브에이전트가 맡는다(런타임에서 Haiku 5.5 미지원 확인 후 사용자 결정).

## 시작 상태와 보존

- HEAD `25dea60`, 시작 dirty 상태·수정 파일 해시: `start_git_status.txt`, `start_head.txt`, `start_tracked_modified_sha256.txt`, 관련 파일 `start_relevant_sha256.txt`. 기존 dirty 상태는 이전 작업/사용자 소유다.
- 이전 플러그인 바이너리 `538a187d…`를 `plugin_before/`에 보존(Git 제외). 새 빌드 `c48d4a99…`(`plugin_v1.sha256`, `build_v1.log`, `ctest_v1.log`).
- 10/8 `scheduled_phase_v1`의 6회 원시·보고·분석은 읽기만 한다.

## 변경(계약 `config/tests/scheduled_command_r2_sha256.json`, 부모 r1)

| 파일 | 변경 | 기본 동작 |
|---|---|---|
| `scripts/scheduled_command.py` | `audit_all_ticks`(origin 이후 모든 tick의 지연 ≤10 ms·누락 0·연속), `ScheduleFaultInjection`(평가기 poll 보류) | 기존 `GridSchedule`·창 감사 불변 |
| `scripts/probe_braking_contact.py` | 격자 모드에서 전체 tick 감사와(켠 경우) 추적 유효성을 `input_valid`에 결합. `--evaluation-schedule-fault {late,miss}` + 목표/지연(가드: sim_grid·출력 이름 `inject`·등록 창 안 유지값 tick·late 10<d<20 ms, miss 20≤d<40 ms). 벽시계 poll·콜백·GC·자체 `/clock` 수신 시간 계측. `--plugin-command-trace` | 기본 poll 경로·인자 불변 |
| `simulation.launch.py` | `evaluation_command_trace`(기본 false, true면 증거 경로 필수, 실행 사본 플러그인 요소 1개만 추가) | 기본 모델 XML byte 동일 |
| `single_motor_drive.cpp` + `command_trace.hpp` | 켠 경우에만 수신 콜백(기존 잠금 안)에서 수신 번호·벽시계·직전 PreUpdate iteration 기록, PreUpdate(기존 잠금 구역 안)에서 표지 복사, 적용값 전환 시 이벤트를 같은 iteration PostUpdate에서 `/model/arena_car/eval/command_trace`로 발행. 1 s sim마다 요약 | 꺼져 있으면 분기만 통과. 값 샘플 문장·잠금 범위·구동식 불변 |

새 평가 도구: `run_probe_batch.py`(순차 실행·0이 아닌 종료/잔존에서 중단·status 기록), `audit_schedule_gate.py`(관문 분류), `analyze_evaluator_timing.py`, `analyze_apply_trace.py`. 사전 회귀: 전체 `tests/` 1256개 통과(`pytest_pre_v1.log`).

이벤트 의미: `rx_seq_first`=그 값을 처음 실어 온 메시지의 채널 누적 번호, `rx_after_iteration/sim_ns`=그 수신 직전에 잠금을 지난 PreUpdate, `iteration/sim_ns`(apply)=새 값을 구동/조향 계산에 처음 넘긴 PreUpdate의 `UpdateInfo`, `steps_after_rx`=둘의 iteration 차, `rx_superseded_total`=어떤 PreUpdate에도 보이기 전에 덮인 수신 전환 누적. `first_update`=플러그인 첫 PreUpdate(스폰 관측). 시간 계측 벽시계는 모두 `CLOCK_MONOTONIC`이다. `sim_ns`와 `iteration×dt`의 관계는 실행 자료에서 확인해 기록한다.

## 공통 조건

분할 노면 `official`, `passive_damping`(평가 전용 평균축 저항 근사, MD20A 회로 아님), 1 ms 기본 월드, `sim_grid` 위상 10 ms, 등록 프로필 `braking_ab_v1/command_profile.json`(3.5 s), 기본 legacy 이외 차량 물성·센서·보호·MCU·판정선 불변, G1 꺼짐.

## Gate A — 실패 전파(계획 `plan_gate_a.json`)

순서: `a1_base`(정상 기준, 기대 통과) → `a2_inject_late`(프로필 2920 ms 0 유지 tick을 15 ms 늦게 발행, 기대 실패) → `a3_sentinel`(시작되면 안 됨).
- 기대 실패 인정 조건: 종료코드≠0, `report.passed=false`, `input_valid=false`, 주입 `applied=true`(실제 보류 poll>0, 목표 tick 지연≥15 ms), 실패 검사가 정확히 `lateness_within_limit`·`all_ticks_lateness_within_limit`뿐, 명시 STOP 확인·`software_stop`·안정 정지·정상 종료·잔존 0·소스 불변·다른 입력 유효성 모두 참.
- 관문 통과: `audit_schedule_gate.py` 판정 `passed=true`(a1 pass, a2 기대 실패, 배치가 a2에서 멈추고 a3 미시작, 사례 후 잔존 없음).
- 자연 지연: a1/a2의 5 ms 이상 tick을 `analyze_evaluator_timing.py`로 범주화한다. 범주는 관측 패턴이며 원인 확정이 아니다.
- a1이 자연 지연이나 다른 이유로 실패하면 배치는 거기서 멈춘다. 이때는 원인 기록을 먼저 하고, 새 가설이나 명확한 환경 실패가 있을 때만 새 이름으로 1회 재시도한다. 판정선(10 ms)은 완화하지 않는다.

## Gate B — 계측 보존·영향

- 순수: 헤더 시나리오·덮인 전환·감시 만료·요약·이벤트 순번, launch 기본 byte 동일, 추적 호출의 분기 보호, 무변화 step 비용 상한(하네스 측정).
- 실행: a1(새 빌드, 추적 끔)의 `runtime_vehicle.sdf`·`runtime_world.sdf`가 10/8 R1과 byte 동일하고 진단 키 집합이 같을 것. c 실행은 a1 대비 실행 사본 차이가 추적 요소 하나뿐일 것. RTF는 참고값으로만 비교한다(물리 결과로 계측 영향을 판정하지 않는다. 결과 자체가 비재현 대상이기 때문).
- 추적 유효성: 이벤트 순번 1부터 연속, `first_update` 1개, 요약 ≥1, 속도·조향 적용 이벤트 존재, 수신기 오류 0. 프로필 창의 속도 16·조향 11 전환과 개수·값이 대응해야 정렬 분석에 쓴다.

## 실행 전 가설(10/8 원시 자료 회고, `retro_divergence_v1.json`)

1 ms 실행의 첫 진단 표본(플러그인 첫 PostUpdate)은 0.002 s 또는 0.003 s 두 부류였다(10/8 1 ms 실행 13개 중 0.002 s 9개, 0.003 s 4개). R1(0.002)과 R5(0.003)는 0.023 s 차체 높이부터 달랐고, 같은 부류의 R5·R3(위상만 다름)는 첫 명령 차이(조향 전환 직후 0.376 s) 전까지 12자리 정밀도에서 같았다. R1·R5의 바퀴/차체 속도 차는 첫 속도 전환 직후 1e-6, 0.638 s에 1e-3을 넘었고, 이는 출력 단계 첫 1 ms 차이(0.755 s)보다 앞선다. 다만 조향의 플러그인 적용 시점은 관측되지 않았다.
- H1(스폰/초기 상태): 차량 생성 step이 벽시계 의존(`create` 서비스)으로 ±1 step 달라 초기 미세 상태가 다르고, 분할 노면 주행이 이를 증폭한다.
- H2(적용 시점): 출력 노드 벽시계 10 ms 타이머·전송 지연 때문에 일부 속도/조향 전환이 다른 step에 적용된다.
- H3(미추적 입력/물리 비결정성): H1·H2로 설명되지 않는 차이.

## Gate C — 1 ms 동일 조건 반복(계획 `plan_c.json`, Gate A·B 충족 후)

`c1_trace`, `c2_trace`(위 공통 조건 + `--plugin-command-trace`). `analyze_apply_trace.py` 등록 분류:
1. 적용 이벤트 개수/값이 다르면 `applications_not_comparable`.
2. 스폰 iteration이 다르면 `spawn_differs_initial_state_not_identical`(H1과 양립; H2 분리 불가).
3. 스폰 같고 적용 step이 모두 같을 때: 진단이 끝까지 같으면 `reproduced_at_print_precision_given_same_spawn_and_applications`(H1·H2 밖 비결정성 미관측), 다르면 `diverged_without_traced_application_difference`(H3 후보).
4. 스폰 같고 적용 차이가 있을 때: 첫 진단 분기가 첫 적용 차이 이후면 `divergence_onset_not_before_first_application_difference`(H2와 양립), 이전이면 `divergence_precedes_first_application_difference`.
물리 지표 허용치는 10/8과 같다(속도 max(0.05 m/s,10%), 거리 max(0.01 m,20%), 시간 max(0.05 s,20%)). 분류는 관측된 선후 관계이며 인과 확정이 아니다. 탐색 비교(관문 아님): a1·c1·c2를 10/8 R1/R5와 같은 sim 시각으로 대조해 같은 스폰 부류에서 진단이 일치하는지 본다.

추가 실행 규칙: C 2회 뒤 스폰이 달라 H2를 분리할 수 없고 그 분리가 꼭 필요할 때에 한해 1회만 추가한다(같은 스폰을 강제하는 수단은 쓰지 않음). 같은 결과를 얻으려는 반복은 하지 않는다. 신규 Gazebo 총 4회 목표, 상한 6회(명확한 환경/정상 종료 실패 재시도 1회 포함). 0.25 ms·G1 폐루프·왼쪽 가림·오른쪽·G2·고속은 하지 않는다.

## 실행 방법

```bash
bash artifacts/validation/2026-10-09/command_apply_trace_v1/run_batch.sh \
  artifacts/validation/2026-10-09/command_apply_trace_v1/plan_gate_a.json status_gate_a_v1.json
python3 -B scripts/audit_schedule_gate.py --status <base>/status_gate_a_v1.json --output <base>/gate_a_v1.json
```

C는 `plan_c.json`, `status_c_v1.json`으로 같은 실행기를 쓴다. 환경은 `env.sh`.

## 실행 기록(사전 등록 뒤 추가)

- **시도 0(실패, Gazebo 미시작):** `run_batch.sh`가 ROS 설정 source 전에 `set -u`를 켜 `/opt/ros/jazzy/setup.bash`의 미정의 변수에서 즉시 종료했다. status·사례 출력이 생기지 않았고 잔존 프로세스도 없었다(실행 담당 서브에이전트 보고). 실행 래퍼 결함이며 등록 조건·판정과 무관하다. 원본 `run_batch.sh`는 보존하고 source 뒤 `set -u`로 고친 `run_batch_v2.sh`를 쓴다. Gazebo 없이 `--help`/잘못된 인자 계획으로 래퍼 연기 시험을 했다(2번째 사례 rc 2에서 중단, 3번째 미시작, 잔존 0). 신규 Gazebo 실행 횟수에는 넣지 않는다.
- **Gate A(v2, 2회):** a1 통과, a2 기대 실패, a3 미시작으로 관문 통과(`gate_a_v2.json`).
- **Gate C 1차(2회):** c1 통과. c2는 정지·종료·추적은 유효했지만 등록 창 뒤 0 유지 tick(상대 4.24 s)이 13 ms 늦어 r2 전체 tick 감사에서 무효(배치 중단). c1 스폰 iteration 2, c2는 3으로 달라 등록 분류는 `spawn_differs_initial_state_not_identical`이다. 두 실행 모두 속도 16·조향 11 전환을 포착했고 전부 수신 후 첫 PreUpdate에 적용됐다. c1·c2 적용 step은 속도 4개·조향 5개가 ±1 ms 달랐다.

### 추가 1회 등록(c3, 실행 전)

- 근거: 위 등록 규칙(스폰이 달라 적용 시점 효과를 분리할 수 없음). 새 관측: 같은 스폰 부류의 a1(추적 끔)과 c1(추적 켬)은 상대 1.081 s까지, R1과 a1은 0.557 s까지, R5와 a2는 0.658 s까지 진단이 같았고 각 분기는 명령 전환 직후였다.
- 가설 H2′: 같은 스폰에서는 물리 계산이 결정적이며, 진단 분기는 첫 플러그인 적용 step 차이 이후에만 시작한다.
- 실행: `plan_c3.json`의 `c3_trace` 1회(조건·소스는 c1과 동일, 평가기 GC 완화 같은 변경 없음).
- 판정: c3가 유효하고 c1과 스폰이 같으면 등록 분류를 적용한다. `divergence_onset_not_before_first_application_difference` 또는 `reproduced_at_print_precision_given_same_spawn_and_applications`면 H2′와 양립, `diverged_without_traced_application_difference`·`divergence_precedes_first_application_difference`면 H2′ 반증(H3 후보). 스폰이 다르거나 c3가 무효이면 보조 분석만 하고 더 실행하지 않는다(신규 Gazebo 5회째, 상한 6회).

## 결과(실행 후 작성)

요약: `gate_summary_v1.json`. 신규 Gazebo 5회(a1·a2·c1·c2·c3), 모두 명시 STOP 확인·`software_stop`·안정 정지·정상 종료·잔존 0, 실행 소스 r2·플러그인 `c48d4a99…` 일치(`final_checks.sh`, `pytest_final_v1.log` 1256개 통과). README 동결본은 모두 현재 파일의 앞부분 그대로다(`readme_prefix_check_v1.txt`).

| 실행 | 판정 | 등록 창 / 전체 tick 최대 지연 | RTF | 비고 |
|---|---|---:|---:|---|
| a1_base | 통과 | 6 / 9 ms | 0.0968 | 새 빌드·추적 끔 |
| a2_inject_late | 기대 실패(rc 1) | 15 / 15 ms | 0.0972 | 주입 tick만 실패, 보류 poll 63 |
| c1_trace | 통과 | 6 / 6 ms | 0.0969 | 스폰 iteration 2 |
| c2_trace | 무효(rc 1) | 0 / **13 ms** | 0.0975 | 프로필 뒤 0 유지 tick 지연, 스폰 3 |
| c3_trace | 통과 | 4 / 4 ms | 0.0979 | 스폰 2 |

- **Gate A 통과**(`gate_a_v2.json`): 기대 실패는 주입한 두 검사만 실패했고 다른 입력·STOP·종료 검사는 모두 참이었다. 배치는 a2에서 멈췄고 a3는 시작하지 않았다.
- **자연 지연(좁힘, 미확정):** 5 ms 이상 늦은 tick은 모두 평가기 Python 2세대 GC 정지 구간 안이었다. 실행당 2세대 GC 8회 중 20 ms 이상 6회가 약 42→142 ms로 커졌고, 각 정지 동안 노드 시계가 RTF만큼(약 4→14 ms) 도약했다. c2의 13 ms 지연은 142.6 ms 정지 직후다. OS·DDS·브리지 지연은 배제하지 않았고, 10/8 R4/R6에는 GC 계측이 없었다(`timing_gc_*_v1.json`).
- **Gate B:** 세 추적 실행 모두 속도 16·조향 11 전환을 포착해 단계와 대응시켰다. 모든 전환은 수신 뒤 첫 PreUpdate에 적용됐고(`steps_after_rx=1`), 덮인 수신 전환은 0이었다. 적용−출력 0~+1 step, 적용−MCU 6~7 ms, `simTime = iteration×dt`. 추적 끔 a1은 10/8 R1과 실행 사본 차량 byte 동일·월드는 임시 메시 경로 외 동일(`preservation_a_v2.json`), 진단은 같은 스폰 부류에서 상대 557 ms까지 같았다. 추적 켬 c1과 끔 a1은 1081 ms까지 같았다. 장부 비용은 무변화 step당 약 13~15 ns(하네스)다.
- **Gate C(등록 쌍 c1–c3, 같은 스폰):** 적용 step은 656 ms 전환(속도·조향)에서 처음 1 ms 달랐고, 진단은 그 직후 표본인 658 ms에서 처음 갈라졌다 → `divergence_onset_not_before_first_application_difference`(H2′와 양립). 이후 속도 5/16·조향 3/11 전환이 ±1 ms 달랐고 7개 주요 지표가 모두 허용치를 넘었다(0명령 전 최대 차 0.546/0.010 m/s, 0명령 후 0.999/0.411, 좌우 1.999/0.752, 정지 이동 0.294/0.051 m). c1–c2는 스폰이 달라(2/3) 보조 자료다.
- **보조 대조(`onset_context_v1.json`):** 같은 스폰 부류의 모든 쌍(R1–a1/c1/c3, R5–a2/c2, a1–c1/c3, a2–c2, c1–c3)에서 분기는 명령 전환 직후에 시작했다. a1–c1은 975 ms 전환의 출력 단계 시각이 1 ms 달랐지만 진단은 1081 ms까지 같았다(추적한 c1은 그 전환을 출력과 같은 step에 적용). 출력 단계 시각은 적용 시각의 대용치로 충분하지 않다.

해석 한계: 추적 있는 같은 스폰 쌍은 1개다. 분류는 선후 관계이며 인과 증명이 아니다. 스폰 step 변동(H1)과 적용 step 변동(H2)은 모두 벽시계 의존 경로(`create` 서비스, 출력 노드의 벽시계 10 ms 타이머·전송)에서 생긴다고 보이지만 그 경로 자체는 이번에 바꾸거나 검증하지 않았다. 분할 노면·`passive_damping` 조건에서 1 ms 차이가 크게 증폭된다는 관측을 실차나 다른 조건으로 확대하지 않는다.
