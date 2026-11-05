# 이동 앞차 추종 속도 톱니 최소 보완 v1

- 날짜: 2026-11-05 (KST). 원인 분석·구현·회귀·폐루프 등록과 판정은 Claude Opus 5.5가 맡았다. 시뮬 실행·대기·회수는 Claude Haiku 5.5 보조(별칭 `haiku`)가 맡았다.
- **상태(10:40 KST):** r8 선택 옵션(`continuous_forward_pace`, 기본 OFF)을 켜고 기준과 같은 장면에서 폐루프 1회를 실행했다. 사전 등록 수용 조건을 모두 충족했다([폐루프 비교](#폐루프-비교-closed_loop_v2)). 기본값은 여전히 OFF이며 이 결과는 직선 0.30 m/s 앞차 한 장면·1회다.
- 01:18 KST에는 5시간 사용량 경고(92%)로 구현·회귀·replay·계약까지만 정리하고 인계했다. 아래 원인·변경·검증 절은 그때 기록이다.
- 추월(진입·측방 이동·복귀) 기능은 추가하지 않았다. 기준 주행 원본·등록 파일, r7/r8 계약, `continuity_before.zip`은 바꾸지 않았다. 이번 폐루프 작업에서 제어 소스는 바꾸지 않았다.

## 폐루프 비교 (`closed_loop_v2`)

### 실행 조건

- 등록: `closed_loop_v2/registered.json`(`prepare.py --register`). 기준 [moving_pass_baseline_v1](../moving_pass_baseline_v1/README.md)의 `registered.json` 인자에 `--handoff-continuous-pace`와 출력 경로만 다르다. 같은 `course.json`(해시 확인), `straight_moving_lead`, seed 0, 0.30 m/s·14 s, grid slot 3, JR60P20 센서 설정, ID49 0.140 m다.
- 판정 정의: 상위 `compare.py`(기준값 `baseline_metrics.json`과 같은 정의)와 `closed_loop_v2/review.py`. 둘 다 등록 해시에 포함된다.
- **소스 차이(등록에 명시):** 기준 run1은 r7 이전(`local_follow.py` r4 after)으로 실행됐다. 이번은 r7+r8 ON이다. r7은 일반 결측 광선이 있을 때의 구조 증속 후보만 바꾸므로 이 장면 영향은 작다고 보지만, r7 단독 효과를 분리하지는 않았다(추정).
- 설치 확인(`closed_loop_v1/install_check.log`): `install/`은 develop/symlink 설치라 런타임 모듈·launch가 `src/`를 직접 가리킨다. 빌드가 필요한 새 진입점·데이터가 없어 다시 빌드하지 않았다. 런타임 해시는 r7/r8 계약 after 6파일과 일치했다.
- 실행: `closed_loop_v2/run.sh`(기준 `run.sh`와 같은 형식: 사전 시뮬 검사, 입력 해시, `environment.sh`, `surface_contact_eval.py`, 잔여 검사). 01:30:13Z–01:34:52Z, `run1.exit`: `ACTUAL_EXIT_CODE=0 SURFACE=0 INTEGRITY=0 LEFTOVER_PGREP=1 WALL_S=279`. 잔여 시뮬 없음, `shutdown_clean=true`.

### 실행기 실패 보존 (`closed_loop_v1`)

`closed_loop_v1/run1`은 녹색 전에 중단됐다. 실행 보조가 `run.sh`를 자기 백그라운드 셸로 띄우고 곧바로 세션을 끝냈기 때문으로 보인다(시각 일치에 근거한 추정). 제어 결과가 없는 실행기 실패이므로 원본과 `run1_ABORTED.md`를 그대로 두었다. 같은 등록 조건을 `closed_loop_v2`에서 1회 다시 실행했고, 보조는 전경 실행으로 끝까지 기다렸다. 그 밖의 재시도는 없다.

### 결과 (`closed_loop_v2/review.json`)

| 항목 | 기준(r8 없음) | r8 ON | 판정 |
|---|---|---|---|
| 런타임 옵션 | — | 머리말 `continuous_forward_pace=true`, 인자 `evaluation_handoff_continuous_pace:=true`, `report.json` `source_sha256`이 r7/r8 after와 일치 | 확인 |
| 유효성·11관문·STOP | 통과 | `passed`/`run_valid`/`harness_valid`/`source_unchanged` 참, 11관문·STOP 6검사 모두 참 | 통과 |
| 접촉·노면 | 0 / 0 | 접촉 메시지 0, 차체 노면 밖 0, 벽 바퀴 0, 이탈 바퀴 0 | 통과 |
| 첫 3 s 이후 0.10 m/s 미만 | 0.95 s·7회 | **0 s·0회** | 개선 |
| 첫 3 s 이후 속도 최소/중앙/최대 | 0.009 / 0.331 / 0.522 m/s | 0.165 / 0.316 / 0.475 m/s | 진폭 감소 |
| 첫 3 s 이후 평균·표준편차·0.20 미만 표본(보충) | 0.299·0.134·61 | 0.305·0.067·15 | 보충 지표 |
| 명령 0.10 m/s 미만(첫 3 s 이후 550호출) | 35회, 최소 0.016 | **0회**, 최소 0.147 | 개선 |
| 고정 여유 분기(창 안) | 95회(1.82–13.70 s) | 15회(1.72–2.20 s만), `continuous_forward_pace` 80회 | — |
| 최종 간격(≤1.2 m) | 1.194 m(여유 6 mm) | **1.092 m(여유 108 mm)** | 통과 |
| 최소 표본 간격 / 쓸림 하한 | 1.083 / 1.082 m | 1.029 / 1.029 m | 관문 통과, 54 mm 가까움 |
| 창 안 자차 진행 | 4.400 m | 4.500 m | — |
| 앞차 이동 | 4.196 m, 1 s 속도 0.295–0.306 | 4.194 m, 1 s 속도 0.293–0.306 | 계속 이동 |
| STOP | 0.793 s 안정, 0.020 m | 0.822 s 안정, 0.017 m | 통과 |

- 제어기 입력은 센서뿐이었다(`integrity.sensor_only_controller_inputs=true`, 앞차 정답 자세 구독자는 평가기뿐).
- **판정:** 사전 등록 조건(유효·런타임 옵션·관문/접촉/노면·앞차 계속 이동·세 개선 지표)을 모두 만족했다(`verdict.accepted=true`). 같은 장면 1회 결과이며 다른 속도·코너·끼어들기·정지 앞차에는 확장하지 않는다.

### 남는 것

- **출발 직후 감속(등록 창 밖, 보충):** 첫 3 s 안 2.20–2.30 s에서 0.10 m/s 미만이 0.15 s 있었다(최저 0.048 m/s). 기준은 같은 구간 최저 0.14 m/s였다. 고정 여유 분기 15회가 모두 1.72–2.20 s에 몰려 있어, 전진율 표본(5스캔)이 아직 쌓이지 않았거나 `front_patch` 창 문제와 겹친 것으로 보인다(추정, 미조사). 첫 3 s 이동 거리는 1.151 m로 기준 1.122 m보다 길다. 출발마다 한 번 생기는 짧은 감속이므로 이번에는 수정하지 않았다(`closed_loop_v2/startup_supplement.log`).
- 앞차 정지 뒤 1스캔 전진율 지연은 이 장면에 정지 앞차가 없어 폐루프에서 확인하지 않았다.
- 보호층 `/drive`→`/drive/safe` 개입 비교는 등록 지표에 없어 따로 집계하지 않았다.
- 기본값 ON 전환 여부는 정하지 않았다. 다른 대표 교통 장면 확인과 함께 GPT 설계 판단으로 넘긴다.

## 원인

[기준 주행](../moving_pass_baseline_v1/README.md) 판단 기록과 `bypass_handoff.py`를 대조했다. 기준 구성은 `paced`와 관측 이동이 켜진 상태다(`moving/closing/low_motion_reserve`, `moving_pace_floor`, `observed_forward_pace` = true).

- 앞차 조각이 보정 후 6 cm를 넘게 움직이면 `BlockingPersistence`의 rows가 초기화되고, 그때의 이동률로 `motion_evidence()`가 켜진다. 0.30 m/s 앞차는 약 0.2 s마다 6 cm에 닿으므로 이 증거는 켜졌다 꺼지기를 반복한다.
- 증거가 꺼지고 rows가 3개 이상이면 `_follow_entry_space_current`의 마지막 분기만 남는다. 이 분기는 고정 우회 여유 상한 `(뒤축→앞면 − 1.2 m)/0.8 s`를 적용하며, 약 1.2–1.36 m 거리에서는 이 값이 0.02–0.2 m/s다. 기준 주행 창 안 95회(`observed_entry_distance_reserve`)가 여기에 해당한다.
- 같은 시각에도 rows 초기화와 별개인 5스캔 `ObservedForwardPace`는 양수 전진율을 내고 있었다. 그러나 이 값은 낮은 이동 분기의 가산에만 쓰이고 마지막 분기에서는 쓰이지 않았다.
- `front_patch` x≤1.60 m 창 이탈(출발 직후 1 s)은 별개 원인이다. 이번에는 다루지 않았고, 후속 과제로 남긴다.

## 변경 (기본 OFF, 명시 선택)

- `bypass_handoff.py`:
  - 새 필드 `HandoffExecutionConfig.continuous_forward_pace=False`(`observed_forward_pace` 필요).
  - 고정 여유 분기 직전에 조건을 추가했다. `ObservedForwardPace.rate > 0`이고 그 마지막 표본이 **이번 장면의 source 시각**일 때만 `min(G1 목표, 고정 여유 상한 + 전진율)`을 쓴다(사유 `continuous_forward_pace`).
  - 전진율은 기존 정의를 그대로 쓴다: 끝점 6 cm 차감, 마지막 간격 전진율 이하, 측방·역행·연관 범위 초과이면 0. 감속 중에도 전진이 관측되면 작은 양수는 남을 수 있다.
  - **최종 코드 검토 정정:** 추정기 `observe`를 같은 시각으로 직접 다시 호출하는 반례는 0을 확인했다. 실제 `BlockingPersistence.update`는 같은 스캔이면 추정기를 재호출하지 않으므로, 같은 유효 스캔을 쓰는 여러 제어 호출에는 전진율이 재사용된다. 매 제어 호출마다 새 스캔을 요구하거나 중복 호출마다 0으로 만든다는 뜻은 아니다. 새 스캔 관측·유효 시각과 센서 추정 지연을 구분한다.
  - G1 경로·점유·제동 출력(`target`)이 상한이다. 정체 증명, 자유 공간, 우회 허가와는 무관하다. `persistence`와 우회 진입 조건은 바꾸지 않았다.
- 연결(모두 기본 false):
  - `g1_g2_node.py`: 파라미터 `g1_g2_continuous_forward_pace`.
  - `simulation.launch.py`: 인자 `evaluation_handoff_continuous_pace`(paced와 관측 이동이 아니면 오류).
  - `g1_handoff_eval.py`: `continuous_pace` 인자와 헤더 확인 항목.
  - `validate_g1_traffic.py`: `--handoff-continuous-pace`.
- 과거 기록 헤더에는 이 필드가 없으므로 `from_mapping` 기본값(False)으로 당시 동작이 재생된다.

## 검증 (실행한 것만)

| 검사 | 결과 | 근거 |
|---|---|---|
| 새 반례 회귀 `tests/test_continuous_forward_pace.py` (15개) | 통과. 수정 전 코드 사본에서는 9 실패·6 통과(추정기 반례 6개는 기존 동작 고정용) | `tests.log`, `tests_new_on_before.log` |
| 관련 회귀 묶음 20파일 | 497 passed, 2 failed. 두 실패는 r7 시점과 실패 해시까지 같은 기존 실패(`test_r0_safety_contract`, `measured_lane_preference_provenance`) | `tests.sh` → `tests.log` / `tests.exit` |
| 기준 판단 replay, 선택 OFF | 740회 중 불일치 0 | `replay_baseline.json` `off` |
| 기준 판단 replay, 선택 ON(개방 루프) | 창 안 고정 여유 분기 95회 전부 `continuous_forward_pace`로 바뀜. 다른 분기·조향은 불변이고, 기록된 G1 목표를 넘은 경우 없음. 창 안 0.10 m/s 미만 명령 44 → 9회. 가산 전진율 중앙값 0.150 m/s(0.046–0.161) | 같은 파일 `on` |
| 앞차 정지 직후 | 정지(21.401 s) 뒤 첫 새 스캔(21.52–21.60 s, 5회)에서만 0.113 m/s 가산이 남았고, 다음 스캔부터는 0으로 돌아가 고정 여유가 적용됨. **1스캔(0.1 s) 지연이 있는 한계** | `replay_baseline.json` `changed_outside_window` |
| r8 소스 계약 | `config/tests/continuous_forward_pace_r8_sha256.json`(parent r7, before `continuity_before.zip`). `M3_SUCCESSORS`에 추가, 출처 회귀 통과 | `register_contract.log` |

개방 루프 replay는 원래 폐루프 입력을 그대로 쓰므로, 바뀐 명령 이후의 거리·속도 변화는 반영되지 않는다. 폐루프 결과는 위 [폐루프 비교](#폐루프-비교-closed_loop_v2)에 있다.

## 미완료와 재개 방법

1. ~~폐루프 비교 1회~~ → 완료(`closed_loop_v2`, 위 절). 초안 검토에서 `prepare.py`의 경로 변환은 정상이었다. 수용 조건에 런타임 옵션·앞차 이동 확인을 더하고 `review.py`를 추가한 뒤 등록했다.
2. 옵션 기본값은 OFF이므로 현재 모든 기존 실행·기록 재생의 동작은 이전과 같다(replay OFF 0 불일치로 확인).
3. 이후 소스를 다시 바꾸면 이번 r8·before·회귀/replay 증거는 보존하고 후속 계약을 만든다. 등록 스크립트가 다르다고 거절할 때 기존 계약을 지워 과거 시험 당시 소스를 새 상태로 바꾸지 않는다.

## 남는 한계

- 전진율은 6 cm를 빼는 보수값이라 0.30 m/s 앞차에서도 약 0.15 m/s다. 01:18의 "진폭이 남을 수 있다"는 추정은 폐루프 1회에서 첫 3 s 이후 0.10 m/s 미만 0회, 속도 표준편차 0.134→0.067 m/s로 나타났다. 다른 속도의 앞차는 미확인이다.
- 노이즈로 전진율이 0이 되는 스캔에서는 기존 고정 여유로 돌아간다. 개방 루프에서는 0.10 m/s 미만 명령이 9회 남았으나, 폐루프 첫 3 s 이후에는 0회였다.
- 앞차 정지 뒤 1스캔 지연이 있다. 이때도 G1 점유·제동과 독립 보호층이 상한이다.
- `front_patch` x 1.60 m 창(출발 직후 조각 소실)은 미해결 후속 과제다. 폐루프 출발 2.20–2.30 s의 짧은 감속과 관련이 있을 수 있다(추정).

## 파일

| 파일 | 내용 |
|---|---|
| `continuity_before.zip` | 수정 전 `bypass_handoff.py`·`g1_g2_node.py`·`simulation.launch.py`·`g1_handoff_eval.py`·`validate_g1_traffic.py`·`test_source_provenance_r0_plus_b3.py` |
| `replay_baseline.py` → `replay_baseline.json` (`.log`, `.exit`) | 기준 판단 replay(OFF 동일성 / ON 개방 루프) |
| `tests.sh` → `tests.log` / `.exit`, `tests_new_on_before.log` / `.exit` | 회귀 묶음, 수정 전 코드에서 새 회귀 실행 |
| `register_contract.py` (`.log`, `.exit`) | r8 계약 생성 |
| `compare.py`, `baseline_metrics.json` | 폐루프 비교 지표 정의와 기준값 |
| `closed_loop_v1/` | 검토한 `prepare.py`·`review.py`·`registered.json`·`run.sh`, `install_check.log`, `git_state_before_run1.txt`, **중단된 `run1`(실행기 실패)**과 `run1_ABORTED.md` |
| `closed_loop_v2/` | 같은 등록 조건의 1회 실행: `registered.json`, `run.sh`, `run1/`(원본), `run1.exit`, `run1_*.log`, `run1_inputs.sha256`, `run1_surface.json`, `review.py` → `review.json`(`.log`, `.exit`), `startup_supplement.log` |
