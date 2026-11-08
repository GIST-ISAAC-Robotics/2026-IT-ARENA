# 새 코스 교통(G1/G2)에 기존 C1 오차/지연 입력 연결 — 실행 전 정적 감사

2026-11-08. 정적 읽기와 작은 JSON/해시 검사만 수행했다. Gazebo·빌드·ROS 실행, 공용 `src/ scripts/ config/ tests/ docs/` 수정은 하지 않았다(운영 시험 진행 중·소스 동결). 기계 가독 계획은 [RUN_PLAN.json](RUN_PLAN.json)에 있다.

## 1. 확인한 기존 근거

| 항목 | 경로 | 값 |
|---|---|---|
| 주입 입력 | `config/tests/lidar_uniform30_delay50_jitter20.json` | uniform ±0.03 m, 지연 0.05±0.02 s, seed 23 |
| 주입 무결성 검사 | `scripts/g1_lidar_stress.py` `audit_injection`/`require_injection_evidence` | 취득 기록↔수신 헤더 1:1, 순차 프레임 0.0998 s·500점·0.0002 s, 추가지연 [0.03, 0.07] s, 오차 ≤ 0.03 m, 실패 시 `run_valid`/`passed`를 거짓으로 만든다 |
| 과거 동일 입력 통과 | `artifacts/validation/2026-10-15/g1_stress_remaining_v1/first_curve_delay_s0/report.json` (g1) | verified, 395/395 스캔, 추가지연 0.032–0.070 s |
| 〃 | `artifacts/validation/2026-10-25/m3_execution_review_v1/run57_faster_lead_sensor_stress/run/report.json` (m3) | verified, 196/196, 0.032–0.070 s |
| 기준 주행 r27 | `artifacts/validation/2026-11-08/traffic_faster_reacquire_live_v1/two_actor_course_lap/` | g1g2, 0.8 m/s 두 앞차 60초, 48.14 m 한 랩. 원 판정 `HARNESS_EXIT=1`(최종 간격 1.246 m > 1.2 m), 보충 12조건 `ASSESSMENT_EXIT=0`, 886 입력 |
| r27 런치 설정 | 같은 폴더 `new_track_course.json` `launch` | `lidar_rate_hz` 10, `lidar_acquisition` sequential, `control_rate_hz` 50, virtual_mcu |
| r27 무주입 상태 | `run1/report.json`, `run1/acquisition.json` | `lidar_test_profile: null`, 취득 685행에 `impairment`·`injected_delay_s` 필드 없음, 수신 685행 |
| r27 제어 입력 시각(이번에 계산) | `run1/follow_decisions.jsonl` | 판단 3,047회, 고유 스캔 610개 모두 취득 기록에 존재, 판단 시각−첫 광선 시각 0.118/0.158/0.198/0.198 s(최소/중앙/p95/최대), 기록 footer dropped 0 |

## 2. 그대로 가능한 부분 / 막힌 부분

- **가능:** 프로파일 → `start_demo_process` → `g1_follow.launch.py` → `simulation.launch.py` → `rotating_lidar`(`test_impairment_profile`) 경로는 G1/G2와 공통이다. m3(같은 `g1_follow` 진입점) run57에서 실제 검증됐다. launch는 sequential만 확인하며 트랙·`local_sensor_config` 조건은 없다. r27의 g1g2 보고서에는 제어기와 보호층 모두 `/scan` 구독이 기록되어 있다. JR60P20 지주 가림은 각도 마스크(`known_structure`)여서 거리 오차와 충돌하지 않는다(정적 판단).
- **막힘(CLI 두 곳, `scripts/validate_g1_traffic.py`):**
  1. 532행 `--lidar-test-profile requires G1/M3` — 모드 `g1g2`를 거부한다.
  2. 584–591행 — `handoff_arguments`가 있는 등록 G1/G2 교통 비교에서 `lidar_profile`을 overlay로 간주해 거부한다(r27의 모든 인계·이동 추월 인자가 여기에 해당한다).
- **기준 표류:** 지금 트리는 r27 등록 해시 대비 12개 파일이 달라졌다(`g1_handoff_eval.py`, `g1_traffic_eval.py`, `validate_g1_traffic.py`, `local_pursuit.py`, `moving_pass.py`, `g1_follow.launch.py`, `simulation.launch.py` 등, r28–r32 작업). 따라서 r27을 "한 인자만 다른 기준"으로 직접 쓸 수 없다. `traffic_race_distance_v1`도 같은 12개가 달라졌다.

## 3. 최소 코드 변경과 표적 회귀(동결 해제 후, 주 에이전트 승인 필요)

1. 532행: 허용 모드를 `('g1', 'm3', 'g1g2')`로 넓힌다. 기록 필수 조건은 유지한다.
2. 584–591행: `lidar_profile`을 `(lidar_profile and not args.official_signal_course)`로 좁힌다. motion profile·seed·다른 overlay 금지와 609행 10 Hz sequential 검사는 그대로 둔다. 보호층·간격·STOP 한도는 바꾸지 않는다.
3. 출처 계약: r27 방식(`config/tests/traffic_standing_reacquire_r27_sha256.json`)처럼 `validate_g1_traffic.py` before/after 해시 계약을 새로 만든다. 체인 목록 갱신이 필요한지 함께 확인한다.
4. 표적 회귀(`tests/test_m3_lidar_stress_cli.py` 확장): (a) g1g2+공식 코스+기록이면 프로파일 검증에 도달하고, 584행 뒤의 단계까지 통과한다. 이를 위해 `check_route`처럼 뒤 단계 함수에 sentinel을 둔다. 537행 도달만으로는 2번 변경을 검사하지 못한다. (b) g1g2에서 공식 코스가 없으면 거부, (c) g1g2+motion profile 거부 유지, (d) 기록 없는 g1g2 거부, (e) 기존 g1/m3/r0plus 사례 불변. 함께 `tests/test_g1_lidar_stress.py`, `tests/test_m4_operation_cli.py`, `tests/test_g1_handoff_eval.py`, `tests/test_official_multi_actor_course.py`를 실행한다.

## 4. 실행과 판정

- **대표 1쌍(같은 등록 스냅숏):** A = r27 `registered.json` 인자 그대로(무주입), B = A + `--lidar-test-profile config/tests/lidar_uniform30_delay50_jitter20.json`. 소스 표류 때문에 A를 다시 실행하는 것을 권한다. A 재실행 대신 r27 3,047판단을 현 소스로 격리 재생해 동일함을 보이는 방법도 있다. 다만 이 재생 도구가 g1g2 인계 상태까지 재현하는지는 확인하지 않았다. r27/race_distance 원 결과는 이력으로 보존하며 소급하지 않는다.
- **무주입 확인(A):** `lidar_test_profile` null, 취득 행에 `impairment` 없음, 소스 목록에 프로파일 파일 없음.
- **유효 주입(B):** `integrity.lidar_stress_verified` true, `lidar_stress_evidence.checks` 6항목 true, 최소/최대 추가지연 ∈ [0.03, 0.07] s, `max_abs_error_m` ∈ (0, 0.03], `runtime_profile_unchanged`.
- **누락 검출:** 취득 수 = 수신 수, `unmatched_received_indices`·`invalid_received_indices`·`source_errors`가 비어 있어야 한다. 보충으로 제어기 판단의 스캔 시각이 모두 취득 기록에 있는지, footer dropped 0인지, 판단 시 스캔 나이 분포가 A(0.118–0.198 s) 대비 약 +0.03–0.07 s 이동했는지 확인한다(마지막 항목은 기대치이며 사전 등록 판정선은 아니다). 보호층(`lidar_safety`)의 스캔별 처리 증거는 없으므로 구독 확인까지만 주장한다.
- **원 판정:** 검사기 `passed`를 보존한다. r27과 같은 최종 간격 실패가 예상되므로 실패로 둔다. **보충 판정:** r27 `assess.py` 12조건(유효·한 랩 44.64 m·접촉 0·간격 ≥ 0.08 m·노면 이탈 ≤ 2바퀴·STOP/종료·입력 불변·최장 저속 ≤ 2.5 s)을 A와 B에 같게 적용한다. `valid`가 `run_valid`를 통해 주입 무결성을 포함한다. 판정선은 완화하지 않는다.
- **비용:** r27 실행은 벽시각 758초(60초 창)였다. 쌍 실행은 약 26분이 예상된다(추정).

## 한계

합성 반경 오차와 발행 지연일 뿐이며 C1 실측 잡음·USB 전송·Jetson 처리 시간을 보증하지 않는다. 두 각본 앞차 한 장면이며 독립 5대 경기나 실차 안전 증명이 아니다. 차량 상수(.505 m 단면, 모터축 28/8.9706/65 mm, 50 Hz 판단)는 r27 등록 설정을 그대로 쓴다는 전제이며, 이번에 개별 재확인하지는 않았다. IMU/엔코더 주입은 현행 CLI상 C1 주입과 동시에 켤 수 없으므로 이번 범위가 아니다.
