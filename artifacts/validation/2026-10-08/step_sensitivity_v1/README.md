# 수동 제동 근사의 물리 step 민감도와 선택형 왼쪽 가림 대조

2026-10-08 후속. [0속도 처리 대조](../zero_speed_v1/README.md)의 `passive_damping` 결과가 1 ms 물리 step에서 수치적으로 크게 왜곡됐는지를 한정된 범위에서 확인한다. GPT 주 에이전트가 범위와 최종 검토를 맡고, Claude Opus 5.5가 구현·분석·문서를, Claude Sonnet 5.5 서브에이전트가 Gazebo 실행·대기·결과 회수를 맡는다.

## 실행 전 등록 (Gazebo 실행 전 작성)

### 고정 입력과 유지 범위

- 입력: [`command_profile.json`](../braking_ab_v1/command_profile.json). 보존 파일에서 직접 계산한 SHA-256은 `a792af4a13cebaa5450bb0188b72b955c91488e2b95c7be69ce8f9daac055a08`이며, 재사용할 1 ms 실행 3개의 `report.json` 기록값·실행 소스 목록 값과 일치했다. 분석기는 실행마다 이 대조를 다시 수행한다(`profile_file_matches_report`). 제동 창은 파일에 기록된 프로필 상대 `1.75~2.85 s`, 슬롯 3 시작 자세, 프로필 종료 후 1초 0명령, 명시 STOP 후 1초 관측이다.
- `/drive` → 독립 LiDAR 보호층 → `/drive/safe` → 가상 MCU 경계, 모터축 단일 28카운트·15:1·65 mm 피드백, 차량 설정·질량·마찰·토크·Kp/Ki·제동 상한·보호 판정선·센서 주기·기본 월드를 바꾸지 않는다. 기본 구동 모드는 `legacy`, G1 기본 꺼짐을 유지한다.
- 실물 MD20A 회로·하드웨어 안전·실차 제동거리 시험이 아니다. 오른쪽 가림/G2/추월/고속/실차 MCU 통합/새 센서/출발 신호 변경은 범위 밖이다.

### 구현한 평가 전용 경로

| 파일 | 변경 | 기본값 |
|---|---|---|
| `src/arena_bringup/launch/simulation.launch.py` | `evaluation_physics_step` 인자. `0.0005`/`0.00025`만 허용하고 `runtime_artifacts` 증거 경로가 없으면 거부. 실행 사본 월드에서 **값이 정확히 `0.001`인 `max_step_size` 하나**만 바꾸며, 물리 요소가 여러 개이거나 기준값이 다르거나 이미 바뀐 월드는 거부 | 빈 값. 실행 사본 월드가 직전 launch와 byte 단위로 같음(회귀) |
| `scripts/probe_braking_contact.py` | `--physics-step` 전달, ARM 완료/종료 시각의 실제 시간·시뮬레이션 시간 기록 | 생략 시 launch 인자 동일 |
| `scripts/validate_g1_traffic.py` | 선택형 `--zero-speed-mode`, `--physics-step`. 비기본값은 `--record-braking-diagnostics`가 필수 | `legacy`/생략 시 인자 추가 없음 |
| `scripts/analyze_physics_step.py` | 아래 지표·유효성·분류 | 판정 보고 전용 |
| `config/tests/physics_step_r0_sha256.json` | `zero_speed_r0_sha256.json`의 후속 계약. launch 변경과 두 평가 스크립트의 변경 전/후 해시 등록. 구동 플러그인 해시는 그대로 | 이전 계약 소급 수정 없음 |

`real_time_factor`(1.0), 센서 `update_rate`, 접촉 센서 200 Hz, 명령 발행 간격(≥19 ms), 구동 진단 발행 비교식(`now-lastPublish<0.02`)은 바꾸지 않는다.

### step 변경 때 함께 달라질 수 있는 것

다음은 물리엔진 단독 효과와 구분해 측정·보고한다.

1. **구동 플러그인 내부 이산화**: 평균속도 PI 적분, 40 ms 토크 응답의 `1-exp(-dt/τ)`, 조향 서보 PI, 오도메트리 적분이 모두 `info.dt`로 계산된다. 연속시간 등가식은 같지만 이산화 오차가 함께 줄어든다. 즉 이번 결과는 "Gazebo 접촉 적분 + 플러그인 이산화"를 합한 step 민감도다.
2. **진단 표본 주기**: 1 ms에서는 20/21 ms 간격이었다. 0.5/0.25 ms에서는 20~20.5 ms로 달라질 수 있어 실제 간격을 보고한다. 0목표 step 수(`zero_steps`) 증가율로 **실제 적용 step을 실측**하며, step 수 자체는 dt마다 2·4배가 되므로 비교 지표로 쓰지 않는다.
3. **실시간 비율(RTF)**: 목표 갱신율이 2·4 kHz로 높아져 RTF가 떨어질 수 있다. 가상 MCU는 1 ms 실제 시간 타이머로 시뮬레이션 시각을 읽으므로, RTF가 낮으면 요청→safe→MCU→적용 목표의 **시뮬레이션 시각 지연**이 바뀔 수 있다. 새 실행은 ARM~종료 구간의 RTF를 기록하고, 재사용 1 ms 실행은 이 값이 없으므로 미기록으로 둔다. 명령 정렬(20 ms 격자 유지값 비교)과 마지막 0 요청 발행→MCU 0 목표→첫 0 진단 지연을 함께 보고한다.

### 왜 step을 확인하는가 (추정·근거의 한계)

후륜 두 개만 떼어 본 스칼라 모형에서 `J=4.225e-5 kg·m²`, 수동 근사의 캐리어 저항 계수 `b=Kp×r×ratio=0.04875 N·m/(rad/s)`, `J/b≈0.867 ms`다. 토크를 step 시작 속도로 계산해 한 step 동안 유지하면 비포화 감쇠의 한 step 배율은 대략 `1-dt/(J/b)`로, 1 ms에서 약 −0.15(매 step 부호 반전), 0.5 ms에서 약 0.42, 0.25 ms에서 약 0.71이다. 이는 **바퀴가 노면·차체와 분리돼 미끄러지는 비포화 구간에만 해당하는 추정**이다. 접촉으로 차체 관성이 결합되면 실효 관성은 훨씬 크다. 표면 속도 약 0.45 m/s 이상(캐리어 약 13.8 rad/s)에서는 제동 상한 0.045 N·m로 포화된다. 따라서 이 계산은 불안정의 증명이 아니라 step 민감도를 확인해야 하는 이유다. 진단은 약 20 ms 표본이므로 매 step의 부호 반전 자체는 관측할 수 없다.

### 실행 계획 (정상 계획 최대 8회 새 Gazebo 실행)

재사용 1 ms 기준(새 실행 아님):
- `../zero_speed_v1/flat_passive_damping`, `../zero_speed_v1/official_passive_damping`, `../zero_speed_v1/flat_legacy`.
- 재사용 조건: 보존 원시 해시·소스 ZIP·STOP·정상 종료 유효, 프로필 해시 재계산 일치, 현재 소스와의 차이가 위 계약에 등록된 launch/두 평가 스크립트뿐일 것, 구동 플러그인 `.so`가 `../zero_speed_v1/plugin.sha256`과 같을 것, 기본 경로 실행 사본 월드가 byte 동일함을 회귀로 확인. 하나라도 실패하면 1 ms를 같은 소스로 최소 재실행한다(그 경우 횟수를 다시 등록).

새 실행 순서(`run_case.sh <mode> <surface> <step> <name>`):

| 순서 | 이름 | 모드 | 노면 | step |
|---|---|---|---|---|
| A1 | `flat_passive_0p5ms` | `passive_damping` | flat | 0.5 ms |
| A2 | `flat_passive_0p25ms` | `passive_damping` | flat | 0.25 ms |
| A3 | `official_passive_0p5ms` | `passive_damping` | official | 0.5 ms |
| A4 | `official_passive_0p25ms` | `passive_damping` | official | 0.25 ms |
| A5 | `flat_legacy_0p25ms` | `legacy` | flat | 0.25 ms (기존식 1 ms 기준의 보조 확인) |
| A6 (조건부) | `flat_legacy_0p5ms` | `legacy` | flat | 0.5 ms. 아래 C를 0.5 ms로 수행해야 할 때만 |
| C1, C2 (조건부) | `g1_side_left_legacy_*`, `g1_side_left_passive_*` | 각 1회 | 왼쪽 잔디 옆차 가림 | 아래 관문에 따른 동일 step |

한 실행이 입력·STOP·정상 종료 중 하나라도 실패하면 다음 실행을 멈추고 원인을 본다. 단순 실행 오류(시작 실패·잔존 프로세스 등)는 실패 자료를 보존한 채 `_retry1` 새 이름으로 한 번만 재시도한다. 같은 시험을 결과가 마음에 들 때까지 반복하지 않는다.

### 실행 유효성 (실행별)

`report.passed`(입력 유효·안정 정지·정상 종료·STOP 확인·`software_stop`·소스 불변), 원시 해시·소스 ZIP, 모드 관측, 단일 모터축 피드백·좌우 엔코더 부재, 프로필 해시, 요청 감사, 진단 coverage(최대 간격 ≤50 ms), **선언 step = 요청 step = 실측 step(±1%)**, 1 ms 기준 대비 차량 SDF byte 동일·월드는 `max_step_size` 외 동일·시작 자세 동일·소스 차이 계약 등록. 실패 시 그 실행은 해석에서 제외하고 실패로 보존한다. 이 판정은 STOP/정지/종료를 각각 따로 기록하며, 시험 유효성 통과를 슬립 해소나 G1 성공으로 해석하지 않는다.

### 비교 지표

주요 지표(분류에 사용):

| 지표 | 정의 | 종류 |
|---|---|---|
| `before_zero_peak_difference_mps` | 제동 창 시작~첫 0 적용 진단 직전의 최대 \|평균 바퀴 표면 속도−차체 전진 속도\| | 속도 |
| `after_zero_peak_difference_mps` | 첫 0 진단~제동 창 끝의 같은 최대값 | 속도 |
| `brake_window_peak_difference_mps` | 제동 창 전체 최대값 | 속도 |
| `left_right_peak_difference_mps` | 제동 창 최대 좌우 바퀴 표면 속도 차 | 속도 |
| `request_zero_to_stable_chord_m` | 마지막 0 요청 **발행 시각** 이후 첫 진단부터 안정 정지 첫 표본까지 XY 현 길이 | 거리 |
| `first_zero_to_stable_chord_m` | **첫 0 적용 진단**부터 안정 정지 첫 표본까지 XY 현 길이(기존 정의) | 거리 |
| `request_zero_to_stable_s` | 마지막 0 요청 발행→안정 정지 첫 표본의 시뮬레이션 시간 | 시간 |

안정 정지는 기존 정의(차체 속도와 평균 바퀴 속도가 모두 0.05 m/s 미만으로 마지막 0.45 s 이상 유지되는 첫 표본)를 그대로 쓴다. 보조 지표: 0목표 뒤 최소 평균 바퀴 속도, 차체 전진 중 최소 단일 바퀴 표면 속도, 역회전 진단 표본 수와 표본 간격 합, 제동 창 PreUpdate 동시각 양의 동력 표본 수·최대값, 0목표 양의 일(J), 0목표 뒤 제동 상한 포화 표본 비율, 실측 step, 진단 간격, RTF, 0 요청→MCU 0 목표/첫 0 진단 지연.

`passive_damping`의 0목표 양의 일 0은 **계산 시점의 `τ×ω≤0`**이 만든 구성상 성질이다. 이것으로 적분 뒤 전체 에너지 보존이나 실물 수동성을 주장하지 않는다.

### 해석 기준 (사전 등록)

각 (노면, 모드) 묶음에서 가장 작은 step 값 `M(0.25)`를 기준으로 허용치 `tol`을 정한다: 속도 `max(0.05 m/s, 10%)`, 거리 `max(0.01 m, 20%)`, 시간 `max(0.05 s, 20%)`. 0.05 m/s는 직전 대조의 모드 간 차이(약 0.29 m/s)·노면 간 차이(약 0.2 m/s)보다 충분히 작아, 이 안의 차이는 기존 정성 결론을 바꾸지 않는 수준으로 본다. 진단 20 ms 표본 시점·속도 해상도 때문에 이보다 작은 차이를 물리적 의미로 해석하지 않는다.

- `consistent_including_1ms`: 모든 주요 지표에서 `|M(0.5)−M(0.25)|≤tol`이고 `|M(1)−M(0.25)|≤tol`.
- `fine_consistent_1ms_distorted`: 0.5/0.25는 허용치 안이나 1 ms가 하나 이상 벗어남.
- `not_converged`: 0.5/0.25 사이도 하나 이상 벗어나거나 지표를 계산할 수 없음.
- 두 step만 있는 기존식 보조(A5)는 `consistent_including_1ms` 또는 `differs_at_1ms`.
- 실행 유효성/비교 감사 중 하나라도 실패하면 해당 분석 전체를 `invalid_input_or_audit`로 둔다.

"차이가 작다"는 것은 **이 고정 요청·두 노면·한 시작 자세에서 수치 해가 step에 덜 민감하다**는 뜻일 뿐, 수동 근사가 물리적으로 타당하거나 실물 MD20A 제동을 재현한다는 뜻이 아니다. 특히 분할 노면의 **0명령 전 불일치(1 ms 0.592516 m/s)**가 step 감소 뒤에도 남는지를 별도로 서술한다.

### 왼쪽 가림 선택형 대조(C)의 관문

- **1 ms로 C 수행**: A1~A5 모두 유효하고, `flat/passive_damping`·`official/passive_damping`·`flat/legacy` 모두 `consistent_including_1ms`.
- **0.5 ms로 C 수행**: 모든 실행이 유효하고, 수동 근사 두 노면이 최소 `fine_consistent_1ms_distorted`이며, 기존식도 0.5 ms에서 0.25 ms와 허용치 안일 것(필요 시 A6로 확인). 이 경우 두 모드 모두 0.5 ms로 실행하며 기존 1 ms 실패와 별도 조건으로 표시하고, RTF/콜백 지연 차이를 함께 보고한다.
- **그 외**(`not_converged`·무효·수치 의심): C를 실행하지 않고 원인과 다음 최소 진단을 정리한다.

C는 `config/tests/g1_side_occlusion_v1.json`의 `side_left`, seed 0, `--mode g1 --record-braking-diagnostics --record-wheel-contacts`로 기존 실패와 같은 조건을 쓰고, 다른 것은 `--zero-speed-mode`(와 필요 시 `--physics-step`)뿐이다. 제어기(`local_follow.py`, 실행 전 SHA-256 `fddc808bae12f27b…`)·보호층·판정선·시험 시간을 바꾸지 않는다. 기존 평가기의 진행/간격/정체/앞섬 판정을 그대로 보고하고, 구동 진단으로 0명령 전후 바퀴–차체 차이를 추가 비교한다. G1 실패가 남아도 이 과제의 실패가 아니며 기본 모델 채택 근거로 쓰지 않는다. 10/7 실패 이후 현재 소스에는 구동 플러그인(영점 모드 추가, `legacy` 출력 동일성은 기존 50,000 step 검사)·launch·G1 실행기가 바뀌었으므로, C의 `legacy` 1회는 "현재 소스에서의 기존식 재실행"으로 기록하고 10/7 결과와 구분한다.

원시 고빈도 로그는 기존 정책대로 로컬 보존/Git 제외(`.gitignore`에 이 폴더 패턴 추가)하고 report·소스 ZIP·해시·분석을 보존한다. 커밋·푸시·PR·외부 게시·Jetson 접속·전원 설정 변경은 하지 않는다.

### 시작 상태 보존

`start_state/`에 작업 시작 시 `git status`, HEAD, 수정/미추적 소스·문서 파일의 SHA-256을 남겼다. 이번 작업의 변경은 이 기준과 대조해 구분한다.

## 사후 추가 등록 (A1~A5 결과 확인 뒤, 추가 실행 전 작성)

A1~A4는 모두 유효했다. `analysis_passive.json`의 분류는 단일 평면 `consistent_including_1ms`, 분할 노면 `not_converged`이다. 따라서 **사전 관문에 따라 C(왼쪽 가림 G1 대조)는 실행하지 않는다.** A5는 입력·STOP·정지는 유효했으나 Gazebo 서버가 정지 서비스 확인 뒤 코드 255로 끝나 정상 종료에 실패했다. 실패 자료는 그대로 보존한다.

결과 확인 중 사전 등록에서 예상하지 못한 혼입 요인을 발견했다. `/drive` 요청 발행 위상이 실행마다 달라, 프로필 전환 뒤 첫 발행까지의 지연이 실행별로 0~19 ms 범위에서 다르다. 각 발행값은 프로필과 일치한다. 1 ms 재사용 실행은 RTF도 기록되지 않았다. 따라서 분할 노면의 큰 차이가 **step 때문인지, 같은 step에서도 10~20 ms 명령 위상에 민감한 것인지** 현재 자료로는 분리할 수 없다. 남은 정상 예산 3회를 다음 최소 진단에 쓴다. 이것은 **결과를 본 뒤 정한 사후 진단**이며, 위 분류나 관문을 바꾸지 않는다.

| 순서 | 이름 | 목적 |
|---|---|---|
| A5 재시도 | `flat_legacy_0p25ms_retry1` | 등록한 단순 실행 오류 재시도 1회(기존식 보조 지표). 실패하면 더 반복하지 않는다 |
| D1 | `official_passive_1ms_rerun` | 현재 소스·기본 1 ms(`run_case_default.sh`, step 인자 없음)로 분할 노면 수동 근사를 재실행. RTF를 기록하고 기존 1 ms와의 차이로 같은 step의 실행 간 변동을 본다 |
| D2 | `official_passive_0p25ms_repeat` | 0.25 ms 반복. 0.5/0.25 사이 차이가 같은 step의 실행 간 변동보다 큰지 본다 |

해석: D1이 기존 1 ms의 0명령 전 불일치(0.592516 m/s)를 허용치 안에서 재현하고 D2가 A4를 허용치 안에서 재현하면, 분할 노면의 0명령 전 불일치는 **이 조건의 1 ms 수치 해에 강하게 의존**한다고 정리한다. 어느 쪽이든 크게 달라지면 명령 위상·타이밍에 대한 민감도가 커서 step 효과로 귀속할 수 없다고 정리한다. 어느 경우에도 C나 기본 모델 변경으로 넘어가지 않는다.

## 실행 결과

새 Gazebo 실행은 정상 계획 상한인 **8회**였다. 7회는 입력·실측 step·STOP(`software_stop`)·안정 정지·정상 종료·소스 불변을 모두 통과했다. `flat_legacy_0p25ms` 1회는 입력·STOP·정지는 유효했지만, Gazebo 서버가 정지 서비스를 확인한 뒤 코드 255로 끝나 **정상 종료에 실패**했다. 이 실행은 `passed=false`로 보존했고 주 분석에서 제외했다. 원인은 이번 자료로 확정하지 못했다. 등록한 재시도 1회(`_retry1`)는 통과했다. 잔존 시험 프로세스는 0개였다. 선언 step·요청 step·`zero_steps` 증가율로 잰 실측 step은 1/0.5/0.25 ms 모두 일치했다. 진단 간격은 20~21 ms에서 20~20.5 ms로 바뀌었다. 새 실행의 RTF는 단일 평면 수동 0.5/0.25 ms 0.095/0.094, 기존식 0.25 ms 재시도 0.095, 분할 노면 수동 1 ms 재실행 0.095·0.5 ms 0.076·0.25 ms 0.043/0.043(반복)이었다([`final_review_v2.json`](final_review_v2.json)). 재사용 1 ms 실행에는 RTF 기록이 없다.

### 수동 근사의 step 비교 (등록 분석 `analysis_passive.json`)

| 노면 / step | 0명령 전 최대 차 | 0명령 후 최대 차 | 좌우 표면 속도 차 | 첫 0 진단→정지 XY | 0 요청→정지 시간 |
|---|---:|---:|---:|---:|---:|
| 단일 평면 1 ms (재사용) | 0.009873 | 0.415015 | 0.034177 | 0.048941 m | 0.212 s |
| 단일 평면 0.5 ms | 0.009641 | 0.408491 | 0.034051 | 0.047891 m | 0.209 s |
| 단일 평면 0.25 ms | 0.008408 | 0.380214 | 0.034139 | 0.043618 m | 0.2185 s |
| 분할 노면 1 ms (재사용) | **0.592516** | 0.612975 | 1.155665 | 0.105801 m | 0.296 s |
| 분할 노면 0.5 ms | 0.063378 | 0.395662 | 0.769524 | 0.079292 m | 0.3885 s |
| 분할 노면 0.25 ms | 0.008121 | 0.398995 | 0.657831 | 0.047946 m | 0.2085 s |

속도 단위는 m/s다. 등록 분류는 단일 평면 `consistent_including_1ms`, 분할 노면 `not_converged`이다. 분할 노면은 0.5/0.25 ms 사이에서도 좌우 차(0.112 > 허용 0.066)·정지 이동·정지 시간이 허용치를 넘었다. 따라서 **관문에 따라 C(왼쪽 가림 G1 대조)는 실행하지 않았다.** 수동 근사 실행에서는 0목표의 양의 일이 모든 step에서 0이었는데, 이는 계산 시점 부호 조건에서 나온 구성상 성질이다.

### 사후 진단: 같은 step의 실행 간 변동

[`command_phase.json`](command_phase.json)을 보면, 각 실행의 요청값은 프로필과 같았지만 프로필 전환 뒤 첫 `/drive` 발행까지의 지연은 실행마다 0~19 ms 범위에서 달랐다. 마지막 0 요청 발행 지연은 기존 1 ms 분할 실행에서 8 ms, 1 ms 재실행에서 0 ms였다.

| 분할 노면 수동 근사 | 0명령 전 | 0명령 후 | 좌우 차 | 첫 0→정지 XY | 첫 0 진단 차체 속도 |
|---|---:|---:|---:|---:|---:|
| 1 ms 기존 | 0.592516 | 0.612975 | 1.155665 | 0.105801 m | 0.7613 |
| 1 ms 재실행(D1) | **0.019136** | 0.457905 | 0.832497 | 0.058748 m | 0.6026 |
| 0.25 ms (A4) | 0.008121 | 0.398995 | 0.657831 | 0.047946 m | 0.5461 |
| 0.25 ms 반복(D2) | 0.016199 | 0.400625 | 0.660228 | 0.047984 m | 0.5478 |

같은 소스·입력·1 ms에서 **0명령 전 0.593 m/s 불일치가 재현되지 않았다**(0.019 m/s). 0.25 ms 두 번은 주요 지표가 서로 가까웠다. 같은 step(1 ms) 반복에서 출력과 발행/전달 위상이 함께 달랐다. 위상은 후보 원인이며 원인 분리는 아직 끝나지 않았다. 0.25 ms 반복은 2회뿐이고 기존식 0.25 ms 두 실행도 서로 달랐으므로, 0.25 ms가 안정적이라고 확정하지 않는다. 앞선 분할 노면의 "0명령 전부터 남는 불일치"는 **한 위상에서 관측된 결과**로 낮춰 해석해야 한다. 두 번씩의 비교만으로 분포를 단정하지 않으며, step 효과와 위상 민감도를 이번 자료로 완전히 분리하지는 못했다.

### 기존식 보조 확인 (`analysis_legacy_aux.json`)

| 단일 평면 기존식 | 0명령 후 최대 차 | 0목표 뒤 최소 평균 바퀴 | 0목표 양의 일 | 첫 0→정지 XY |
|---|---:|---:|---:|---:|
| 1 ms (재사용) | 0.707520 | −0.2075 | 0.018059 J | 0.048569 m |
| 0.25 ms (종료 실패, 동역학 입력 유효) | 0.536594 | −0.0928 | 0.020018 J | 0.050010 m |
| 0.25 ms 재시도 | 0.422958 | −0.0441 | 0.014533 J | 0.040291 m |

분류는 `differs_at_1ms`다. 0.25 ms 두 실행 사이에도 0.114 m/s 차이가 났다. 원인(위상·step·기타)은 분리하지 못했다. 다만 기존식의 0목표 양의 일과 평균 역회전은 0.25 ms에서도 남았다. 40 ms 잔류 토크 효과가 있다는 정성 결론은 유지되지만, 크기(0.708 m/s 등)를 1 ms 한 실행의 대표값으로 쓰지 않는다. 종료 실패 실행의 수치는 보조로만 표시한다(`analysis_flat_legacy_failed_shutdown.json`, `audit_passed=false`).

### 결론과 다음 최소 진단

1. 단일 평면의 수동 근사는 1/0.5/0.25 ms가 등록 허용치 안이었다. 다만 0.25 ms 쪽으로 약간 줄어드는 경향(0.415→0.380 m/s)이 있다.
2. 분할 노면은 `not_converged`이고, 1 ms는 같은 step 재실행에서도 크게 달랐다. 분류는 사전 허용치에 따라 여러 실행의 지표를 나눈 것이며 수학적 수렴의 증명이 아니다. **1 ms 근사가 비교용으로 왜곡되지 않는다는 근거가 없으므로 G1 선택형 대조로 넘어가지 않았다.** G1 왼쪽 가림 실패·오른쪽 미실행은 그대로다.
3. 직전 0속도 대조의 모드 순위·크기와 분할 노면 0명령 전 불일치는 1 ms 한 위상의 단일 관측이다. 기본 모델·판정선·물성은 바꾸지 않았다.
4. 다음 최소 진단으로 권고하는 것은 다음과 같다. 고정 요청을 **시뮬레이션 시계 격자에 맞춰 결정적으로 발행**하도록 하되 `/drive`→보호층→MCU 경계는 유지한다. 그다음 소수의 등록 위상 오프셋에서 1 ms와 0.25 ms의 분포를 비교한다. 위상 변동이 큰 원인(접촉/분할 이음새/구동 이산화)을 좁힌 뒤에만 평가용 step을 정해 G1 대조를 다시 검토한다.

### 검증

- 사전 단위 회귀 100개 통과(`unit_v1.log`, `unit_v2.log`).
- 마감 회귀 **716개 통과**(`pytest_final.log`), 구동 플러그인 `.so`는 직전과 동일(`plugin_before.txt`, `plugin_after.txt`).
- [`final_audit.json`](final_audit.json) 통과. 8회 소스 ZIP·원시 해시, 원시 4종의 Git 제외·미추적, 등록 이후 실행 소스 불변(README만 결과 추가로 변경), 분석 스크립트·계약 해시 일치를 확인했다.

### 산출물

- 분석: [`analysis_passive.json`](analysis_passive.json), [`analysis_official_rerun_ref.json`](analysis_official_rerun_ref.json)(D1을 1 ms 기준으로 둠), [`analysis_official_repeat025.json`](analysis_official_repeat025.json)(D2를 0.25 ms로 둠), [`analysis_legacy_aux.json`](analysis_legacy_aux.json), [`analysis_flat_legacy_failed_shutdown.json`](analysis_flat_legacy_failed_shutdown.json), [`command_phase.json`](command_phase.json)
- 스크립트: [`run_case.sh`](run_case.sh), [`run_case_default.sh`](run_case_default.sh), [`run_analysis.sh`](run_analysis.sh), [`run_unit.sh`](run_unit.sh), [`final_checks.sh`](final_checks.sh), [`finalize.py`](finalize.py), [`command_phase.py`](command_phase.py)
- 동결·시작 상태: `pre_run_freeze.sha256`, `addendum_freeze.sha256`, `start_state/`
- 실행 폴더별 `report.json`·`source_snapshot.zip`·실행 SDF·`*.runner.log`·`simulation.log`는 보존한다. 고빈도 원시 4종은 로컬 보존/Git 제외다.

## 최종 검토 보완 (v2, Gazebo 재실행 없음)

최종 검토의 지적에 따라 기존 분석·판정·사전 기준·`final_audit.json`은 그대로 보존했다. 기존 분석 출력은 등록 당시 분석기 SHA-256 `d34e19b1…`의 증거로 남기고, 보완 결과는 새 이름으로만 기록했다.

- **두 판정의 분리.** 기존 `audit_passed`/`final_audit.json`은 **실행·보존 유효성**(입력 프로필, STOP, 정지, 종료, 소스, 실측 step, 해시)만 판정한다. **step 단독 효과 식별 가능성**은 적용 입력 동일성까지 요구하는 별도 판정이며, 이번 비교에서는 모두 `false`였다. 사후 검토(v2)에서 20 ms 격자로 다시 비교한 결과, 모든 step 비교에서 요청/safe/MCU 목표의 속도·조향 불일치 표본이 나왔다(MCU 목표 속도는 274~275표본 중 3~11개, 최대 차는 최초 step 비교·기존식 보조 비교에서 요청 0.589 m/s·MCU 0.230 m/s, 1 ms 재실행을 기준으로 한 비교까지 포함한 전체 최대는 요청·MCU 모두 0.589 m/s). 프로필 전환 근처에서 발행·전달 위상이 달라 생긴 차이로 보인다. 마지막 속도 0 전환에서 MCU 0 목표까지의 지연도 8~38.75 ms로 실행마다 달랐다(기존 분할 노면 1 ms는 33 ms, 1 ms 재실행은 8 ms). 기존 `audit_passed`는 실행·보존 유효성 판정일 뿐 적용 입력의 동일성은 판정하지 않으므로, **step 단독 효과는 식별되지 않았다**(`step_only_effect_identifiable=false`). 분류는 사전 허용치에 따라 여러 실행의 지표를 나눈 것이며 수학적 수렴의 증명이 아니다.
- **인과 표현 정정.** 같은 step(1 ms) 반복에서 출력과 발행/전달 위상이 함께 달랐다. 위상은 후보 원인이며 원인 분리는 아직 끝나지 않았다. 0.25 ms 반복은 2회뿐이고 기존식 0.25 ms 두 실행도 서로 달랐으므로, 0.25 ms가 안정적이라고 확정하지 않는다.
- **분석기 보강**(`scripts/analyze_physics_step.py`): 같은 노면/모드/step의 중복 입력을 명시적으로 거부한다(이전에는 덮어쓰기로 비교 대상이 사라질 수 있었다). 1 ms 기준이 없거나 등록되지 않은 step이 섞이거나 1 ms 하나뿐인 묶음은 분류하지 않는다. 비교마다 `aligned_mismatches`, `applied_input_identical`과 상위의 `execution_preservation_valid`, `step_only_effect_identifiable`, `classification_scope`를 출력한다. 지표 계산식과 허용치는 바꾸지 않았다.
- **명령 위상 v1 점검.** `command_phase.py`의 `final_zero_lag_s`는 마지막 상태 전환(속도·조향)을 속도 0 전환으로 암묵 가정했다. 현재 프로필에서는 두 시각이 모두 2.256 s로 같아 v1 값은 맞다. [`command_phase_v2.py`](command_phase_v2.py)는 속도 0 전환을 직접 계산하고 이 조건을 검사하며, 요청/safe/MCU 지연을 함께 기록한다([`command_phase_v2.json`](command_phase_v2.json)).
- **검사.** 사전 단위 회귀(`run_unit.sh unit_review_v1.log`)는 106개 통과했다. 기존 100개에 새 음성 회귀 6개(분류 거부 4, 중복 거부 1, 격자 불일치 계수 1)를 더한 것이다. 같은 입력에 v2 분석기를 다시 돌린 결과(`analysis_*_v2.json`)는 기존 출력과 지표·분류·감사·정렬 요약이 모두 같았다. 실제 자료의 중복 입력(`analysis_duplicate_negative_v2.log`)은 거부되어 출력이 생성되지 않았다. 종료 실패 실행이 포함된 분석은 v2에서도 `audit_passed=false`였다. 결과는 [`final_review_v2.json`](final_review_v2.json)에 모았고, 보존 산출물 해시는 `artifact_manifest_v2.sha256`으로 검증했다. 원래 마감 회귀 716개는 v2 변경 이전의 결과이며, 이번에 다시 돌리지 않았다. runtime 소스와 소스 계약은 바꾸지 않았다.
