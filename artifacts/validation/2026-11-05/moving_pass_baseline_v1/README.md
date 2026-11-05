# 이동 앞차 추월 전 기준 주행 v1 — JR60P20·직선 0.30 m/s 앞차 추종

- 날짜: 2026-11-05 (KST). 주행 시각 2026-11-05 00:33:02–00:37:53 KST(`run1_preflight.log`의 UTC 15:33:02Z–15:37:53Z).
- 범위: [개발 계획](../../../../docs/DEVELOPMENT_PLAN.md)의 이동 추월 ① 단계 전에 현재 추종 동작을 측정한 **기준 1회**다. 추월 제어는 꺼져 있으며(`passing_enabled=false`), 추월 알고리즘·설정·원본·범용 런타임은 바꾸지 않았다.
- 역할: 장면 등록(`prepare.py`·`course.json`·`registered.json`)은 GPT가 맡았다. 실행 스크립트·사후 평가·이 보고서는 Claude Opus 5.5가 작성했다. 시뮬레이션 실행·대기·회수는 Claude Haiku 5.5 보조(별칭 `haiku`)가 맡았다.
- 근거 기준: 정답 자세(`actor_poses`·`ego_poses`)는 평가에만 썼고, 제어기 입력은 센서뿐이다(`report.json` `integrity.sensor_only_controller_inputs=true`, `actor_pose_subscribers=["g1b_traffic_validator"]`).

## 결론

| 항목 | 결과 | 근거 |
|---|---|---|
| 실행 | 1회 정상 종료. 재시도 없음 | `run1.exit`: `ACTUAL_EXIT_CODE=0 SURFACE=0 INTEGRITY=0 LEFTOVER_PGREP=1 WALL_S=291` |
| 사전 등록 판정 | **통과**(`passed=true`, `run_valid=true`, `harness_valid=true`). 11개 관문 모두 통과 | `run1/report.json` `gates` |
| 접촉·간격 | 접촉 메시지 0. 최소 표본 간격 1.083 m, 보수적 쓸림 하한 1.082 m | `evaluation.gap` |
| 노면 | 차체 노면 밖 표본 0, 바퀴 이탈 0, 벽 바퀴 0 | `run1_surface.json`(`surface_contact_v1`) |
| STOP | 요청 21.600 s → 0.793 s 뒤 안정, 이동 0.020 m. MCU `ESTOP_LATCHED`/`software_stop`. 요청 전 래치 0 | `stop`, `stop_checks`, `actuation.stop_latch_timing` |
| 종료·잔여 | 자식 14개 정상 종료, 잔여 시험 PID 없음, 주행 뒤 시뮬 프로세스 없음 | `shutdown_audit`, `remaining_test_pids=[]`, 빈 `run1_postflight.log` |
| 순서 | 창 전체에서 자차가 뒤에 있었다. 순서 역전·나란히 달리기 없음 | `review.json` `order_gap_speed.ego_always_behind=true` |

**판정 해석:** 이 결과는 "움직이는 앞차를 접촉 없이 뒤따르며 영구 정체하지 않는다"는 기준선이다. 추월 성공·추월 가능성의 증거가 아니다. `final_gap` 관문(≤1.2 m)은 실제 1.194 m로 **여유가 6 mm**뿐이다. 아래 속도 톱니의 위상에 따라 같은 장면이 이 관문에서 경계 판정을 받을 수 있다(추정, 반복 실행 없음).

## 실행 조건 확인

| 요구 | 확인 결과 | 근거 종류 |
|---|---|---|
| 등록 무변경 | 실행 전 읽기 전용 재계산으로 `course.json`·`registered.json` 바이트 일치. `prepare.py --run` 자체 검사 통과. 실행 뒤 세 파일 SHA-256 불변 | 실행 기록 |
| 소스·설정 무변경 | 실행 전 `src/ scripts/ config/` 변경 없음(`git_state_before_run1.txt`, HEAD `01d9e11c`). 입력 해시 재검사 OK(`run1_post.log`), `report.json` `source_unchanged=true` | 실행 기록 |
| C1 x=60 mm, 광축 75 mm | `runtime_vehicle.sdf` `lidar_link` pose `0.06 0 0.075`(모델 기준, 바퀴 중심 z=0.0325=반지름이므로 모델 원점이 지면). 판단 기록 머리말 `motion_config.lidar_x_m=0.06` | 런타임 SDF·트레이스 |
| Ø20 배선 예약 | `c1_mount_wire_0/1` 반지름 0.010, 위치 (−0.088906, ±0.055308). 지주 4개 반지름 0.0035 | 런타임 SDF·`runtime_local_sensors.yaml` |
| ID49 140 mm | `rear_marker_id49_board` z 0.0775(차체 링크 z 0.0625 기준) = 0.140 m. `report.json` `pilot_marker_z_m=0.14` | 런타임 SDF |
| 후방 중앙 열림 | `masked_rear_half_angle_deg=0.0`. 구조 각도 `lidar_blind_sectors.json`(중심°, 반폭°)의 후방 끝 163.73° → 정후방 개방 **32.53°** | 런타임 파일·`review.json` `geometry_reference` |
| 계획기 구조 마스크 | 판단 입력 740회 모두 `sectors_deg`가 런타임 파일과 같고 `known_structure.policy=structure_aware_v1` | 판단 트레이스 |
| 보호층 구조 마스크 | `simulation.launch.py`가 같은 `lidar_blind_sectors_json`과 `known_structure_policy`를 `lidar_safety`에 넘긴다. 실행 중 파라미터 덤프는 없다 | **소스 경로 확인만** |
| 진입점 | `prepare.py --run` → 등록 인자로 `scripts/validate_g1_traffic.py`. 환경은 `artifacts/validation/2026-11-01/turn_preview_v1/environment.sh`(Jazzy). 사전 점검에서 다른 시뮬 없음(빈 `run1_preflight.log` 상단) | `run.sh` |

## 각본 추적 — 움직인 앞차였는가

- 녹색 전 7.37 s 동안 변위 0으로 대기했다. 이후 녹색(7.401 s)부터 14 s 동안 4.196 m 이동(등록 4.2 m)했다. 평균 0.2997 m/s, 1 s 구간 속도 0.295–0.306 m/s였다. 창이 끝난 뒤 변위는 0이다. 명령 추적 오차는 최대 6.9 mm(허용 35 mm)였다.
- 따라서 창 전체에서 **계속 움직인 앞차**였다. 정지차나 대기 후 출발 장면과 구별된다. 다만 상대는 평가기가 50 Hz로 순간 이동시키는 무한질량 상자(0.20×0.15×0.12 m)다. 차량 동역학과 후면 마커는 없다.

## 순서·간격·속도 (정답, 상대 진행축 기준)

| 시각 | 0 s(녹색) | 1 | 2 | 3 | 5 | 7 | 9 | 11 | 13 | 14 |
|---|---|---|---|---|---|---|---|---|---|---|
| 차체 간격 m | 1.400 | 1.580 | 1.190 | 1.181 | 1.194 | 1.169 | 1.177 | 1.163 | 1.175 | 1.194 |
| 자차 속도 m/s | 0.00 | 0.47 | 0.18 | 0.46 | 0.48 | 0.13 | 0.22 | 0.10 | 0.34 | 0.27 |

- 첫 움직임은 녹색 0.688 s 뒤(8.089 s)다. 창 안 자차 이동은 4.402 m(평균 0.314 m/s)로 상대보다 0.21 m 더 갔다. 측방 편차는 출발 때 0.09 m, 이후 수 mm였다.
- 3 s 이후 속도 중앙값은 0.33 m/s지만 범위가 0.009–0.52 m/s다. 0.10 m/s 미만으로 떨어진 구간이 **7회**였고, 각 0.10–0.15 s 정도였으며 최저 0.009 m/s(13.7 s)였다. 등록 정지 정의(지면 속도 <0.01 m/s·`/drive/safe` 0·HOLD 동시 성립)로는 정지 0 s다. 그러나 경기 목표(속도 연속, [ADR 0028](../../../../docs/decisions/0028-speed-continuity-and-final-gearing.md)) 관점에서는 **거의 정지에 가까운 톱니형 감가속**이다.
- 보호층은 창 내내 `clear`였다. `/drive/safe`가 `/drive`보다 0.03 m/s 이상 낮은 표본은 0이다. 즉, 감속은 보호층 개입이 아니라 제어기 명령에서 나왔다.

## 관측 가능성 — 센서로 실제 볼 수 있었던 범위

### 스캔 원자료(정답 교점과 대조)

`review.py`는 각 광선 시각(`stamp + i·time_increment`, 순차 획득에서 stamp=첫 광선)의 정답 자세로 상대 상자와의 교점을 구한다. 측정 거리가 교점과 0.05 m 안이면 적중으로 센다.

- 창 안 140스캔 **모두** 상대를 맞혔다. 스캔당 7–10광선(중앙 9), 기대 대비 적중 95%였고 구조 차폐 광선은 0이었다. 상대 방위는 −11.5°…+6.5°였다.
- 이번 장면은 처음부터 끝까지 정면 1.1–1.7 m 거리였다. **옆·뒤 관측은 한 번도 일어나지 않았다**(`selector_sides.*.seen` 항상 false). 방향·속도·시차에 따른 측후방 관측 가능성에는 이번 주행 증거가 없다.

### 제어기가 쓴 관측(판단 트레이스, 센서 입력만)

- **앞차 운동량:** `preparation.motion_rate_mps`의 중앙값은 0.2998 m/s(정답 0.2997)였고 범위는 0.215–0.346, 표본은 525개였다. 앞면 조각 `persistence.front_median_xy`(base 좌표; `rear_axle_to_patch_m − x = 0.0675`)는 정답 뒷면보다 x로 중앙 −0.020 m(−0.038…−0.005), y로 ≈0 차이였다. **창 안에 들어 있을 때 센서만으로 앞차 속도와 앞면 위치를 잘 잰다.**
- **정체 증명은 없다:** `identity_or_stationarity_proven`, `moving_object_identity_proven`, 창 안 `persistence.ready`는 항상 false였다. 현재 구조는 "같은 이동 물체"를 인증하지 않는다. 추월 진입에는 별도 근거가 필요하다.
- **출발 직후 앞차 조각 소실(7.82–8.80 s, 50회 판단):** `persistence.reason=no_front_patch`였다. 그런데 같은 시간 스캔에는 상대가 매번 7–8광선으로 찍혀 있었다(`review.json` `scans_during_no_front_patch`). 실제 적중점을 base 좌표로 옮기면 원인이 드러난다.
  - 7.70–8.50 s: 뒷면이 x 1.60–1.70 m에 있었다. 자차가 0.69 s 늦게 출발하는 동안 앞차가 멀어졌기 때문이다.
  - 8.50–8.70 s: 자차가 중앙으로 붙으며 요가 회전해 측방 |y|가 0.17–0.31 m가 됐다.

  두 경우 모두 `front_patch(contiguous_three_v0)`의 창(0.20≤x≤1.60 m, |y|≤0.24 m, |y|<0.15 m 점 필요; `bypass_handoff.py`)을 벗어났다. 이 근사(보정 전 원시 거리)로 재현한 실패 스캔은 창 안 11개로, 소실 구간과 일치한다. 소실 동안에는 G1 경로 거리만 남았다(`swept_clearance_m` 0.48→1.40). 명령은 0.4→**1.5 m/s(CRUISE, 8.62 s)**까지 올라갔고, 자차는 9.0 s에 최고 0.94 m/s에 이른 뒤 조각이 돌아오자 0.2 m/s 상한으로 다시 감속했다. 이번에는 간격 1.4 m 덕에 위험이 없었다(최소 1.08 m).
- **톱니의 출처:** `follow_entry_space.reason`이 `advancing_patch_close`(515회, 상한 약 0.35–0.50 m/s)와 `observed_entry_distance_reserve`(95회, 상한 0.05–0.20 m/s)를 0.1–1.6 s 주기로 오갔다. `reserve_m=1.2`(뒤축→앞면)에 붙어 있는 동안 상한이 크게 바뀌는 구조로 보인다(추정: 트레이스 값으로 본 해석이며 코드 경로 검증은 하지 않았다).
- **카메라:** 상대 상자에는 마커가 없다. 창 안 `marker_ids`는 비었거나 `[0, 45]`(시설 표지)뿐이었다. 자차 ID49는 다른 차가 볼 대상이며 이번 장면에서는 쓰이지 않았다.

### 기하 참고(실행 증거 아님)

`review.json` `geometry_reference`는 같은 구조 각도에서 같은 방향 20×15 cm 상대가 차지하는 광선 수를 정적으로 셌다. 측방 0.27 m는 정지 뒤 기록된 `bypass.corridor_plan` 행의 `offset_m` 크기를 참고값으로 쓴 것이다.

- 같은 차로 뒤(측방 0)에서는 x −0.3…−1.4 m 모두 구조 가림이 없다.
- 측방 0.27 m 뒤에서는 x −0.6/−0.8/−1.0 m일 때 24/18/14광선 중 12/7/9만 구조 밖이다. 전부 다시 보이는 것은 −1.4 m 무렵이다. 옆→뒤로 넘어가는 동안 **부분 가림 구간이 있다**. 개방각 32.5°는 다시 볼 기회일 뿐 항상 보인다는 뜻이 아니다. 거리 한계·다른 차폐·운동은 반영하지 않았다.

## 다음 개발(GPT)에 넘기는 핵심

1. **관측 창 1.6 m가 이동 앞차의 병목이다.** 정면 1.1–1.7 m의 앞차는 스캔에 늘 보였지만, 운동량·앞면 추정은 base x≤1.60 m, |y|≤0.24 m 안에서만 나온다. 앞차가 더 빠르거나 자차 출발이 늦거나 자차 요가 틀어지면 조각을 잃는다. 그동안에는 G1 거리 속도만 남아 1.5 m/s까지 명령한다. 중·고속 상대(② ③ 단계)에서는 이 상황이 더 잦을 것으로 예상한다(추정).
2. **센서 운동 추정은 쓸 만하다.** 창 안에서 앞차 속도 오차는 중앙값 기준 1 mm/s 미만, 앞면 x 오차는 약 2 cm다. 다만 동일 물체·지속 이동의 증명 필드는 늘 false다. "끝까지 움직이는 앞차"를 판단하려면 별도 근거가 필요하다.
3. **비교 기준선(seed 0, 1회):** 창 안 자차 이동 4.402 m(0.314 m/s), 최종 간격 1.194 m, 최소 간격 1.083 m, 3 s 이후 0.10 m/s 미만 감속 7회, 최고 0.94 m/s(출발 직후 조각 소실 중). 선택형 추월과 비교할 때 같은 `review.py` 필드로 비교할 수 있다.
4. **판정 유효성(이번 근거 필드):**
   - 무효로 볼 경우: `harness_valid`·`run_valid`·`source_unchanged`·STOP 검사 중 하나라도 거짓이면 제어기 결과로 세지 않는다.
   - 앞차가 "계속 움직였는가"는 `actor_script.per_second_speed_mps`와 창 뒤 변위로 확인한다.
   - 순서는 `order_gap_speed`의 상대 진행축 성분으로 확인한다.
   - 추월 성공 판정(뒤→옆→앞→복귀 후 전진)에는 이 기준 주행이 쓰는 `ego_stays_behind` 관문을 쓸 수 없다. 새 장면에 맞는 사전 등록이 필요하다.
5. **경계 판정 주의:** `final_gap` 여유가 6 mm다. 같은 기준으로 반복하거나 비교할 때 톱니 위상 때문에 결과가 뒤집힐 수 있다. 판정선은 바꾸지 않는다.

## 한계

- 1회·seed 0 결과다. 반복성, 다른 시작 오프셋, 실차는 확인하지 않았다.
- **호스트 실행 조건:** 주행은 00:33:02–00:37:53 KST에 이루어졌다. GPT 보고에 따르면 그때 Windows 전원 계획은 Silent였고, 주행이 끝난 뒤인 00:42:47 KST에 사용자 승인으로 Turbo(`6fecc5ae-…`)로 바뀌었다. 이 작업자는 전원 상태를 직접 조회하거나 변경하지 않았다. 팬·펌웨어 모드는 확인되지 않았다. 실시간 소요는 279.5 s(시뮬 약 22.4 s)였다. 이 값은 실제 처리율 비교에 쓰지 않는다. 판정은 시뮬레이션 시각 기준이다(라이다 10 Hz 시뮬 시각 측정, 마감 진단 0).
- 스캔 적중·`front_patch` 재현은 보정 전 원시 거리와 base 좌표를 가정한 근사다. 제어기 내부의 움직임 보정 장면과 수 cm 다를 수 있다.
- 기하 표는 정적 계산이다. 보호층 구조 마스크는 소스 경로만 확인했다.
- 정지 요청 뒤(22.22 s~) 판단 11회는 `bypass`(G2 진입 시험, HOLD `entry_initial_distinct_boundaries_wait`)를 기록했다. 앞차가 각본대로 멈춘 뒤이자 창 밖이므로 판정에 쓰지 않았다.

## 파일

| 파일 | 내용 |
|---|---|
| `prepare.py`, `course.json`, `registered.json` | GPT 등록본(수정하지 않음) |
| `run.sh` | 1회 실행·회수(재실행 차단, 사전 다른 시뮬 검사, 입력 해시, 노면 평가, 잔여 프로세스 검사) |
| `git_state_before_run1.txt`, `run1_inputs.sha256` | 실행 전 저장소·입력 상태 |
| `run1.exit`, `run1_console.log`, `run1_post.log`, `run1_preflight.log`, `run1_postflight.log` | 종료코드·콘솔·해시 재검사·시각·잔여 프로세스 |
| `run1/` | 검사기 산출물. 원시 `*.jsonl`(신호 기록 제외)·`trace.json` 등은 `.gitignore`로 로컬 보존 |
| `run1_surface.json` | `surface_contact_eval.py` 결과 |
| `review.py` → `review.json` (`review.exit`, `review.log`) | 평가 전용 사후 검토. `python -I -B review.py`로 재현 |
