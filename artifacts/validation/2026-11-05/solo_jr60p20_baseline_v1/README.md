# JR60P20·Ø20 배선 단독 1랩 기준 v1

- 날짜: 2026-11-05 (KST). 주행 시각 12:06:28–12:15:18 KST(`run1_preflight.log`의 UTC 03:06:28Z–03:15:18Z), 실시간 530 s.
- 범위: 현재 팀 작업트리를 그대로 두고 **단독 고속 구성의 실제 Gazebo 1랩 기준을 1회** 측정했다. 차량은 JR60P20에 Ø20 mm 배선 예약 외곽, 후면 ID49 중심 z 0.140 m를 적용했다. 월드는 v2026.11.01 팀 파생본(`tl_post` 제거, 방지턱 비활성)이다. 추월·제어기·공식 월드·런타임 코드는 바꾸지 않았다. 새 증속 후보도 구현하지 않았다.
- 역할: 등록·실행 스크립트·사후 평가·보고서는 Claude Opus 5.5가 작성했다. 시뮬레이션 실행·대기·회수는 Claude Haiku 5.5 보조(별칭 `haiku`)가 감독되는 포그라운드 호출로 맡았다. `run.sh`는 1회 호출됐고 도구 시간 초과는 없었다.
- 근거 기준: 정답 자세·신호는 사후 평가에만 썼다. 제어기와 보호층 입력은 센서뿐이다(`report.json` `sensor_only_inputs=true`, `safety_sensor_only_inputs=true`).

## 결론

| 항목 | 결과(이번 실행) | 근거 |
|---|---|---|
| 실행 | 1회, 정상 종료. 재시도 없음 | `run1.exit`: `ACTUAL_EXIT_CODE=0 WALL_S=530 METRICS=0 SURFACE=0 MOTOR=0 ANALYSIS=0 INTEGRITY=0 LEFTOVER_PGREP=1` |
| 검사기 판정 | `passed=true`, `route_complete=true`, 1랩, `error=null` | `run1/report.json` |
| **랩 시간** | **32.949 s**(첫 초록 궤적 표본 8.077 s → 결승선 46.6329 m 교차) | `run1_metrics.json` `lap_s_from_first_green_sample` |
| 결승 거리 | 랩 길이 46.6329 m. 정지 후 진행 48.653 m. 랩 평균 1.415 m/s | `report.json` `lap_length_m`·`progress_m` |
| 지면 최고속도 | 2.393 m/s(8.61 km/h), 진행 2.28 m 지점 | `run1_analysis.json` `now.speed` |
| 최고 1초 평균 | 2.315 m/s | `run1_metrics.json` |
| ≥2.0 m/s | 총 5.316 s, 최장 연속 2.260 s(처음 9.801 s) | `now.speed.thresholds` |
| ≥10 km/h의 95%(2.639 m/s)·≥10 km/h | 0 s·0 m | 같은 곳, `time_above_95_percent_s=0` |
| 계획기 0명령 | 합 0.32 s. 출발 전 0.18 s, 출발 후 0.14 s(모두 43.41–43.60 m) | `now.zero_command` |
| 실제 지면 정지(출발 후 <0.05 m/s ≥0.1 s) | 0회 | `now.ground_stops_after_first_move=[]` |
| 42–44 m 손실 | 통과 2.411 s, 평균 0.829 m/s, 최저 0.094 m/s(43.67 m 부근) | `now.loss_42_44_m`, `run1_window_42_44.json` |
| 노면·접촉 | 21,841표본 중 차체 노면 밖 0, 조향 외곽 밖 0, 바퀴 이탈 0, 벽 바퀴 0, 충돌 표본 0(정지 중 0) | `run1_surface.json`, `report.json` |
| STOP | `stop_stable=true`. 정지 대기 1.814 s(시뮬), 결승 뒤 정지 이동 0.877 m. 조밀 자세 0.5 s 구간 최고 0.0092 m/s. MCU `ESTOP_LATCHED`/`software_stop` | `report.json` `dense_pose_stop_evidence`, `actuation` |
| 종료·잔여 | 자식 15/15 정상 종료, `remaining_test_pids=[]`, Gazebo 정지 서비스 확인, 주행 뒤 시뮬 프로세스 없음 | `shutdown_audit`, 빈 `run1_postflight.log` |
| 누락 | 궤적 표본 최대 간격 0.048 s, 노면 평가 간격 초과 0. 라이다 10.0 Hz(최대 간격 0.100 s), `/drive` 50 Hz, RGB 62.5 Hz, IMU 250 Hz | `run1_metrics.json`, `report.json` `lidar`·`drive_delivery`·`sensor_delivery` |

**해석:** 이번 구성의 단독 1랩 기준은 **32.949 s, 접촉·노면 이탈 0, STOP·정상 종료 통과**다. 다만 최고 속도가 목표 10 km/h의 95%에 한 번도 닿지 않았다. 2 m/s 이상 연속 주행도 최장 2.26 s에 그쳤다. 출발 후 계획기 0명령 0.14 s는 모두 43.4–43.6 m에 몰려 있다. 그 결과 42–44 m 2 m 구간에 2.41 s가 걸렸다. 출발 구간(0–2 m, 2.04 s)을 빼면 같은 길이의 다른 구간은 0.87–1.87 s였다. 실제 지면 정지(0.05 m/s 미만 0.1 s 이상)는 아니지만, 최저 0.094 m/s까지 떨어지는 **거의 정지에 가까운 감속**이다.

## 출발 — 실제 RGB 신호로 출발했는가

| 시각(시뮬 s) | 사건 | 근거 |
|---|---|---|
| 8.050 | 검사기가 초록 단계 적용 | `official_signal_evidence.green_applied_sim_s` |
| 8.077 | 첫 초록 궤적 표본(평가 정답, 랩 시작 기준) | `run1_metrics.json` `first_green_sample_s` |
| 8.261 | 자율 노드 `signal=green_confirmed`, `started=true`(RGB 처리 결과). 검사기 `first_start_sim_s`와 일치 | `now.start`, `official_signal_evidence` |
| 8.629 | 지면 속도 0.1 m/s 도달. 첫 초록 표본 뒤 0.552 s | `now.start.first_move_after_green_s` |

- 초록 전 자율 상태는 `unknown`, `waiting_red4`, `red_partial_or_ambiguous`, `red4_confirmed`뿐이었다. `false_start=false`, `start_after_actual_green=true`. 단계 7개(green_probe→off→red1–4→green)가 모두 수신 10회씩 확인됐다(`ten_receipts_each=true`).
- 출발 전 0명령 0.18 s(8.24 s부터. `follow_boundary_unconfirmed` 0.08 s, `follow_release_wait` 0.10 s)는 출발 반응 시간에 포함된다.

## 42–44 m 구간 (평가 전용 진단)

[`window_42_44.py`](window_42_44.py)는 진행 40–46 m의 follow 판단을 0.1 s 간격으로 요약한다. 주행 뒤에 추가한 집계이며 사전 등록 항목은 아니다.

- 41.9 m(37.3 s)부터 `swept_clearance_m`이 1.28 → 0.76 m로 줄며 명령이 1.76 → 1.25 m/s로 내려갔다. 43.0–43.4 m에서는 `local_follow_corridor_candidate`로 바뀌어 약 1.0 m/s를 유지했다.
- 43.5–43.7 m에서 오른쪽 벽 경로(`path_wall=right`)의 `swept_clearance_m`이 0.48–0.56 m로 줄었다. 명령은 0.5 m/s로 내려간 뒤 `follow_release_wait`(HOLD) 0.10 s와 `follow_forward_gap_too_wide` 0.02 s로 0이 됐다. 지면 속도는 38.98 s에 0.095 m/s까지 떨어졌다.
- 39.28 s(43.76 m)에 경로 기준 벽이 왼쪽으로 바뀐 뒤에도 명령은 1.07–1.25 m/s였다. 그러나 지면 속도는 약 0.43 m/s에 0.4 s가량 머물렀다. 44.0 m를 지나면서 `swept_clearance_m`이 1.3–2.9 m로 열리자 2.2 m/s대로 회복했다.
- 이 구간 내내 독립 보호층은 `clear`였다(`loss_42_44_m.safety_reason_s`). 감속은 보호층 개입이 아니라 계획기 명령에서 나왔다. 오른쪽 벽 경로에서 짧아진 쓸림 여유가 실제 무반환 때문인지, 벽 전환 과정에서 버린 근거 때문인지는 **이번 집계로 판정하지 않았다**.

## 고정 조건 확인

| 요구 | 확인 결과 | 근거 종류 |
|---|---|---|
| 현재 작업트리·사전 등록 | 실행 전 입력 830개(`src/ scripts/ config/`의 추적·미추적 파일, 환경·평가 스크립트)의 SHA-256과 진입점 인자를 등록했다. 실행 직전과 회수 뒤 재검사 결과 변경 0, 인자 일치. 검사기 자체 `source_unchanged=true` | 실행 기록 |
| 등록 당시 소스 보존 | HEAD `01d9e11c` + `worktree_diff.patch`(src/scripts/config/tests 작업트리 차이) + `registered_untracked_sources.zip`(미추적 소스 9개). 검사기가 만든 `run1/source_snapshot.zip`도 있다 | 파일 |
| 과거 소스·staged install 미사용 | 설치본은 작업트리를 가리키는 연결이다. Python 패키지는 egg-link → `build/*` → `src/*` 심볼릭 연결이고, launch·월드·`vehicle.yaml`·모터 플러그인도 연결이다(`registered.json` `install_links`). 환경은 `artifacts/validation/2026-11-01/turn_preview_v1/environment.sh`의 설치 진입점 확인을 거쳤다. 진입점은 `scripts/run_current_solo.py` 하나다 | 등록·실행 기록 |
| 모터 플러그인 | 빌드 산출물 `libarena-single-motor-system.so`(2026-10-31 빌드, SHA-256 등록). `src/arena_gazebo` 작업트리 변경 없음 | 등록 기록(재빌드 안 함) |
| C1 x=60 mm·광축 75 mm, 지주 4개 Ø7 | 런타임 `lidar_link` pose `0.06 0 0.075`. `c1_mount_post_0–3` 반지름 0.0035, (0.02, ±0.062), (−0.08, ±0.052) | `run1/runtime_vehicle.sdf` |
| Ø20 배선 예약 | `c1_mount_wire_0/1` 반지름 0.010, (−0.088906, ±0.055308). 구조 각도 6개(배선 반폭 4.109°) | 런타임 SDF·`run1/lidar_blind_sectors.json` |
| ID49 z 0.140 | `rear_marker_id49_board` z 0.0775(차체 링크 z 0.0625 기준) = 0.140 m. `test_arguments.pilot_marker_z=0.14` | 런타임 SDF·`report.json` |
| C1 12 m | `/scan` `range_max=12.0`, 500 표본, 10.0 Hz 순차 획득(`sequential_acquisition_verified=true`) | `report.json` `lidar` |
| 단일 28count·8.970588:1·65 mm | 모터 감사 9개 검사 모두 통과. 28 count/모터 회전, 기어비 8.970588235294118, 바퀴 반지름 0.0325 m(외경 65 mm), 바퀴당 0.813 mm/count. 제어기·보호층은 `/drive_motor/encoder`를 구독하고 `/wheel_states`는 구독하지 않는다 | `run1_motor_audit.json` |
| 독립 보호층 | `lidar_safety_enabled=true`. 보호층 구독은 `/clock,/drive,/drive_motor/encoder,/imu/data,/scan`이다. 랩 내내 `clear`였고, `/drive/safe`가 `/drive`보다 0.05 m/s 이상 낮은 궤적 표본은 131개다 | `report.json`·`now.safety` |
| GPU·렌더링 | `render_backend=wsl_nvidia`(기존값). 호스트 NVIDIA GeForce RTX 5070 Laptop GPU, 드라이버 617.42, WSL 커널 6.18.33.2 | `run1_host.log` |
| 월드 | v2026.11.01 팀 파생본, `speed_bump_enabled=false`. 런타임 월드 SHA-256 `2171176a…`(노면 평가 기록) | `report.json`·`run1_surface.json` |

## 기존값과의 구별

아래 기존값은 **11/4의 Ø2 mm 합성 배선 JR60P20 1회**에서 나왔다([원 보고](../../2026-11-04/lidar_jr_forward_v1/README.md)). 장착 모델(배선 Ø2 → Ø20)과 그 뒤의 코드가 다르므로, 차이를 장착이나 특정 변경의 효과로 해석하지 않는다. 두 기록 모두 1회씩이다. 같은 [`analyze.py`](analyze.py) 함수로 다시 계산했다(`run1_analysis.json` `prior_reference`).

| 항목 | 이번(11/5, Ø20) | 기존값(11/4, Ø2) |
|---|---:|---:|
| 랩 | 32.949 s | 33.225 s |
| 지면 최고 / 최고 1초 평균 | 2.393 / 2.315 m/s | 2.433 / 2.325 m/s |
| ≥2.0 m/s 총 / 최장 연속 | 5.316 / 2.260 s | 4.976 / 2.231 s |
| ≥95% 목표 | 0 s | 0 s |
| 계획기 0명령 합(출발 후) | 0.32 s(0.14 s) | 0.22 s(0.04 s) |
| 42–44 m 통과 / 최저 속도 | 2.411 s / 0.094 m/s | 2.010 s / 0.283 m/s |
| 첫 이동(초록 뒤) | 0.552 s | 0.553 s |

2 m 단위 구간 시간 차이(`progress_bins_now_minus_prior_s`)는 42–44 m만 +0.401 s로 크다. 나머지 구간은 −0.20…+0.08 s 범위에서 흩어진다. 랩 차이 −0.276 s는 1회 실행끼리의 차이이며 반복 분산은 측정하지 않았다.

## 영상

`run1/local_pursuit_third_person_drive.mp4`(1.32 MB)가 있다. 674프레임을 기록했고 674프레임이 디코딩됐다. 시뮬 시각 범위는 8.25–41.90 s로 출발과 결승선(41.03 s)을 포함한다. 존재와 시점만 확인했다. 별도 촬영이나 시뮬 시간 재생본 생성은 하지 않았다. 영상은 로컬에만 보존한다(이 폴더의 `.gitignore`).

## 다음 속도 작업 제안(한 단계)

**42–44 m 감속의 원인 분리를 이번 기록으로 먼저 한다.** 새 주행 없이 `run1`의 저장 입력을 재생한다. 43.4–43.9 m에서 오른쪽 벽 경로의 `swept_clearance_m` 0.48–0.56 m가 실제 반환 점(벽·코너 안쪽)에서 나왔는지 확인한다. 아니면 오른쪽→왼쪽 벽 전환 직전에 근거를 버려 생긴 것인지 확인한다. 이는 [개발 계획](../../../../docs/DEVELOPMENT_PLAN.md) M1의 "실제 무반환과 경로/벽 전환으로 버린 근거 분리"에 해당한다. 이 2 m에서 다른 구간 수준(약 1.0–1.5 s)으로 줄이면 랩당 약 0.9–1.4 s를 줄일 수 있다(추정: 구간 시간 차이에서 계산한 상한이며 폐루프 효과는 아니다). 이 값은 출발 후 0명령 전체와 최저 속도 구간을 함께 다룬다. 가속 상한·목표 속도 미달(최고 2.39 m/s)은 그다음 단계로 둔다.

> **후속(같은 날):** 저장 입력 진단은 [diagnosis_v1](diagnosis_v1/README.md)에 있다. 손실은 벽 전환이 아니라 43.41 m의 근거 전환 0명령(1호출)과 43.60 m의 실제 무반환 정지(12 m 밖 벽), 그리고 그 뒤 증속 재시작에서 나왔다.

## 한계

- 1회·단독·시뮬레이션 결과다. 반복성, 교통, 실차 성능은 확인하지 않았다. 판정은 시뮬레이션 시각 기준이다. 실시간 530 s는 처리율 비교에 쓰지 않는다.
- 지면 속도는 평가 정답의 종·횡 속도를 선형 보간한 값이다. 바퀴 속도나 물리 인증이 아니다. 노면 평가 기하(축거·윤거·타이어 폭)는 임시값이다(`run1_surface.json` `geometry_status`).
- `LEFTOVER_PGREP=1`은 `pgrep`이 일치 프로세스를 찾지 못했다는 뜻이다(잔여 없음).
- 저장소 루트 `.gitignore`의 `2026-11-0[2345]` 원시 기록 제외 규칙은 현재 작업트리에서 수정된 상태다(이 작업에서 바꾸지 않음). 이 규칙이 커밋되지 않으면 `run1/*.jsonl`·`trace.json` 등 약 86 MB가 추적 대상이 된다. 현재 추적 대상(미무시 파일)은 약 1.45 MB다.

## 파일

| 파일 | 내용 |
|---|---|
| `register.py` → `registered.json`, `registered_inputs.sha256` | 사전 등록(실행 전 1회): 입력 해시 830개, 진입점 인자, 고정 조건, 설치 연결, 측정 정의, 기존값 |
| `input_list.txt`, `git_state_registered.txt`, `worktree_diff.patch`, `registered_untracked_sources.zip` | 등록 당시 저장소 상태와 소스 |
| `verify.py` → `run1_verify_before.log`, `run1_verify_after.log` | 실행 직전·회수 뒤 해시·인자 재검사 |
| `run.sh` → `run1_sim.exit`, `run1_console.log`, `run1_preflight.log`, `run1_host.log` | 1회 실행(재실행 차단, 다른 시뮬 검사) |
| `post.sh` → `run1.exit`, `run1_metrics.json`, `run1_surface.json`, `run1_motor_audit.json`, `run1_post.log`, `run1_postflight.log` | 회수: 속도 측정, 노면, 모터 감사, 무결성, 잔여 프로세스 |
| `analyze.py` → `run1_analysis.json`, `run1_analysis.log` | 출발·속도 임계·0명령·42–44 m·보호층·기존값 재계산 |
| `window_42_44.py` → `run1_window_42_44.json`, `run1_window_42_44.log` | 주행 뒤 추가한 40–46 m 판단 요약(진단 입력) |
| `run1/` | 검사기 산출물. 원시 `*.jsonl`·`trace.json`·`actuation_trace.json` 등은 루트 `.gitignore`, 영상은 이 폴더 `.gitignore`로 로컬 보존 |
