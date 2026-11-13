# Fable 5.2/high 자문 원문

## 결론

10 Hz 미달의 주된 원인은 계획 계산이 전반적으로 느린 것이 아니라, 선택기가 `RELEASED` 상태에 들어가면 계획 비용이 약 60 ms에서 130–150 ms로 뛰는 데 있습니다. 다만 이 미달이 실제 정지로 이어지는 경우는 드뭅니다. 독립 보호층이 정지시키는 300 ms 이상 발행 공백은 31초 재생당 0–2회(초과분 합계 0–0.31초)였습니다. 실제 주행 손실은 갱신 공백보다 `RELEASED` 구간의 저속·0 속도(r353, 다른 GPT 담당)가 지배하는 것으로 보입니다.

## 1) 원인 우선순위

1. **`RELEASED` 선택기 비용 (관측)**
   - r358 `jetson_2_shared`: `SOLO` 구간(t0–3)은 계획 중앙값 57–61 ms에 초당 9–10회 발행, `RELEASED` 구간(t6–22)은 110–150 ms에 초당 5–7회 발행입니다.
   - r359 프로파일: `pass_release_trial._select`가 `SOLO`에서 11–12.5 ms, `RELEASED`에서 79–152 ms입니다(cProfile 포함값). 큰 항목은 `motion_region` 80.9, `depth_support` 60.8, `released_body_gap.propose` 88.5 ms입니다.
   - 기본 경로 계산은 두 상태 모두 40–60 ms로 같습니다.

2. **`RELEASED`에서 버려지는 보관 경로 재검사 (관측 + 코드)**
   - `may_finish_current`가 `SOLO`만 유지하므로(`runtime_dual_state.py:34`), `RELEASED`에서는 새 계획마다 revision이 올라 진행 중 검사가 폐기됩니다(`:102`, `:112`).
   - t6–22에서 보관형 검사는 r358이 32회 중 29회, r363이 24회 중 23회 폐기됐습니다.
   - 이 검사가 current 슬롯을 점유하는 동안 새 계획 인계가 밀리고, `saved_plan`이 남아 있으면 `offer_plan`이 거부됩니다(`runtime_dual_process.py:178`).
   - 그 결과 계획 주기가 계산 중앙값 132 ms보다 긴 157 ms(r358), 163 ms(r363)입니다.

3. **빠듯한 기한 사슬 (관측)**
   - 발행 시점의 명령 기준 나이가 p50 92–99 ms라서 남은 수명은 약 108 ms이고, 최소 양수 발행 간격은 100 ms입니다.
   - `RELEASED`에서 새 계획 발행 시 경로 기하 나이는 p50 400, p95 447, 최대 462 ms로 500 ms 한도에 가깝습니다. 보관 경로를 한 번 더 쓸 여유가 거의 없습니다.

4. **동시 센서 부하 (관측)**
   - `SOLO` 계획이 무부하 57–75 ms에서 부하 중 67–105 ms로 늘어납니다.
   - `RELEASED`는 132–149 ms에서 147–166 ms로 늘어납니다.

5. **실행 간 편차: 계산 잡음보다 상태 분기 (관측)**
   - r360은 current 검사 283회가 모두 `rebase_current_follow` 계열로, `RELEASED`에 한 번도 들어가지 않았습니다. 계획 p50은 74.5 ms입니다.
   - r363은 r358과 같은 분기로 `rebase_current_follow` 42회 뒤 `rebase_current_released` 96회와 폐기 25회가 이어졌고, 계획 p50은 131.4 ms입니다.
   - 따라서 r360의 3.77초와 r363의 7.53초 차이는 카메라 fps 효과로 볼 수 없습니다.
   - 구간별 공백률은 `SOLO`(r360 t15–31)가 약 0.01 s/s, `RELEASED`가 약 0.30 s/s입니다. 0 속도 구간은 공백에 잡히지 않으므로, 총 공백 초는 구간을 나누지 않으면 실행 간 비교가 되지 않습니다.
   - (추론) r360의 분기 이탈은 t≈4–5초의 시작 부하로 스캔 선택이 밀려 측면 차량 증거를 놓친 결과로 보입니다. 그 시점에 456 ms 공백이 두 번 있었습니다.

**공백 지표와 실제 정지의 관계 (관측 + 코드)**
- 결정 대기 중 노드는 `/drive`를 발행하지 않고 이전 명령의 timestamp도 갱신하지 않습니다(`wall_follow.py:180–185`).
- 발행 stamp는 current 검사 기준 시각입니다(`wall_follow.py:205–207`).
- 보호층은 수신 시각부터 `command_timeout_s`(기본 0.30초) 동안 마지막 명령을 유지합니다(`tof_safety.py:105–113`).
- 양수 발행 간격 300 ms 이상은 r358 두 실행이 0회와 1회, r360이 2회, r363이 2회였습니다.

## 2) 대안과 권고

- **A. 스케줄링 한 줄 수정 (권고, 최우선)**
  - `offer_next_current`에서 계획이 진행 중이고 보관 경로가 유지 불가 의도(`RELEASED` 등)이면 보관형 재검사를 시작하지 않습니다.
  - 기한·증명·보호 의미는 바꾸지 않으며, 포기하는 것은 실행당 1–2회의 보관 양수 발행뿐입니다.
  - (추론, 제가 만든 단순 이산 모델) `RELEASED` 계획 주기가 157에서 약 135 ms로, 공백률이 0.31에서 약 0.20 s/s로 약 35% 줄어듭니다.
  - `runtime_dual_state.py`는 다른 GPT가 연결 중인 파일이므로 편집 소유권 조율이 필요합니다.

- **B. 같은 의도 유지 규칙을 `RELEASED`/baseline으로 확장 (차순위)**
  - 경로 기하 나이가 이미 400 ms라 500 ms 한도 안에서 얻는 것이 적습니다.
  - 측면 차량이 있는 상태에서 옛 검사를 유지하는 의미 변경이라 계약 재검증이 필요합니다.
  - r334에서 `SOLO`로 제한한 이유는 확인하지 못했습니다.

- **C. `RELEASED` 선택기 분기의 표적 절감 (출발 전 비권고)**
  - 매 스캔 반복되는 `motion_region`/`depth_support`/`band_check`를 줄여 계획을 약 90 ms 이하로 만드는 방법입니다.
  - `RELEASED`에서 실제 10 Hz에 도달할 유일한 경로이지만 계산 작업이라 일정 위험이 큽니다.
  - 정지 원인 담당이 같은 분기(우측 자기 가림 꼬리)를 수정한다면 그때 함께 다루는 편이 맞습니다.

A만 적용하고 10 Hz 작업은 거기서 멈추시기를 권합니다. 10 Hz 기준은 낮추지 않고 `RELEASED` 미달을 그대로 기록하며, 남은 시간은 `RELEASED` 저속·정지 해결과 실연결에 쓰는 편이 주행 손실을 더 줄입니다.

## 3) 다음 실험과 종료 조건

Jetson에서 r353 31초 재생을 무부하로, 현행과 A를 ABBA 순서로 2회씩 실행합니다.

- **선행 확인:** 네 실행의 `current_reason` 집계가 같은 분기(`RELEASED` 진입)인지 먼저 봅니다. 분기가 다르면 그 쌍은 비교하지 않습니다.
- **채택 조건 (`RELEASED` 구간만):**
  - 보관형 검사 폐기가 거의 0회입니다.
  - 계획 주기 중앙값이 140 ms 이하입니다.
  - 공백률이 30% 이상 줄어듭니다.
  - 300 ms 이상 간격 횟수가 늘지 않습니다.
  - `SOLO` 구간의 발행 수와 명령값이 동일하고, 기존 회귀·계약 시험을 통과합니다.
- **종료 조건:** 개선이 25% 미만이거나 두 쌍 모두 분기가 어긋나면 스케줄링 작업을 중단하고 `RELEASED` 10 Hz 미달을 그대로 기록합니다.
- **보고 형식:** 총 공백 초 대신 `SOLO`/`RELEASED`/0 속도 구간별 공백률과 300 ms 이상 간격 횟수를 함께 적습니다.

## 읽은 파일

- `docs/reports/RUNTIME_SENSOR_INTEGRATION_2026_11_12.md` (r355–r365 부분)
- `artifacts/validation/2026-11-13/` 아래:
  - `shared_ray_timed_r358_v1/`: `README.md`, `summary.json`, `timed_replay.py`, `jetson_results/jetson_{2,3}_shared/`의 `report.json`, `answers.json`, `events.jsonl`
  - `latest_plan_profile_r359_v1/`: `README.md`, `runtime/runtime_result/plan_profile.json`
  - `live_compute_overlap_r360_v1/`: `README.md`, `audit.json`, `bench.py`, `run.sh`, `remote_result/actual/calculation/*`
  - `camera_publish_r363_v1/`: `README.md`, `audit.json`, `remote_result/actual/calculation/*`
- `src/arena_autonomy/arena_autonomy/` 아래:
  - 전체: `runtime_dual_state.py`, `runtime_dual_process.py`, `runtime_dual_delivery.py`, `current_path_recheck/_released.py`
  - 일부: `pass_release_trial.py` (100–260, 556–700), `tof_safety.py` (28–187), `g1_g2_node.py` (150–370), `wall_follow.py` (150–211)

## 확인하지 못한 것

- 실차 설정의 `command_timeout_s` 값과 ESP32·브리지 쪽 명령 시간 제한은 보지 않았습니다. 더 짧다면 "공백이 정지로 거의 이어지지 않는다"는 판단이 바뀝니다.
- r334에서 `RELEASED`를 유지 규칙에서 제외한 근거는 읽지 않았습니다.
- r360 분기 이탈의 직접 원인은 추론이며, 스캔 단위로 대조하지 않았습니다.
- Jetson 전원 모드와 DVFS 상태는 확인하지 않았습니다.
- `docs/DEVELOPMENT_PLAN.md`, `docs/PROJECT_CONTEXT.md` 본문, r361/r362/r364/r365 산출물, `local_follow.py`, native 커널은 읽지 않았습니다.
- A의 효과 수치는 경로 기하 500 ms 한도를 뺀 제 단순 모델의 추론이며, 프로젝트 증거가 아닙니다.
- 재생은 기록 궤적에 대한 파일 발행 시험이므로 주행·ROS 전달의 증명이 아닙니다.

## 작업 보고

- **변경 파일:** 없습니다. Jetson·WSL·ROS·Gazebo 실행, subagent 사용, 외부 게시도 하지 않았습니다.
- **실행한 검사:** 저장된 `answers.json`/`events.jsonl`/`plan_profile.json`을 로컬 Python으로 읽기 전용 집계했고, 임시 타이밍 모델은 인라인으로만 돌렸습니다(종료 코드 0, 파일 생성 없음).
- **가정:** 재생의 `now_s` 간격을 `/drive` 수신 간격으로 보았고, 전송 지연은 무시했습니다.
