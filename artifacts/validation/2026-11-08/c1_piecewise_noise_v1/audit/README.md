# r35 C1 주입 B 결과 감사

2026-11-08. 작업은 Claude(Opus 5.5)가 했다. 완료된 원본을 읽기만 했다. Gazebo/ROS 재실행과 판단 재생은 하지 않았다. 원 `report.json`, `assessment_run1.json`, `run1.exit`와 [사전 판정](../PLAN.md)은 바꾸지 않았다.

- 스크립트: [audit_r35_result.py](audit_r35_result.py) (`python -I audit_r35_result.py`, 종료 코드 0, [audit_v1.exit](audit_v1.exit), [audit_v1.log](audit_v1.log))
- 결과: [r35_result_audit.json](r35_result_audit.json)(전체, 입력 파일 35개의 sha256 포함), [r35_low_speed_compact.json](r35_low_speed_compact.json)(저속 구간 요약)
- 비교 대상: r35 B(`../B_c1_injection`), r34 A/B(`../../g1g2_lidar_stress_pair_v1`). 이 비교는 서로 다른 실제 주행의 기록을 나란히 놓은 것이다. 같은 입력을 다시 돌린 결과가 아니다.

## 1. 종료·무결성·STOP 대조

| 항목 | r35 B 원시 근거 | 판단 |
|---|---|---|
| 종료 줄 | `HARNESS_EXIT=1 SURFACE_EXIT=0 INTEGRITY_EXIT=0 LEFTOVER_PGREP=1 ASSESSMENT_EXIT=1 WALL_S=775` | 원 판정과 보충 판정이 모두 실패했다. `LEFTOVER_PGREP=1`은 남은 프로세스가 없다는 뜻이다(`postflight_processes_run1.txt`는 빈 파일) |
| 원 판정 `report.passed` | false. 실패: `no_permanent_stall`, `final_gap`, `full_lap` | r34 B와 같은 세 항목 |
| 보충 판정 | false. 실패: `one_lap`(42.55 < 44.64 m), `sustained_motion`(6.764 > 2.5 s) | 나머지 16항목은 참 |
| 소스 무결성 | 입력 903개, 변경 0, 스냅숏 일치, `source_unchanged` 참 | — |
| 입력 경계 | 제어기·보호층·구동 입력 모두 센서 전용, `/drive`는 `local_pursuit`, `/drive/safe`는 `lidar_safety`만 발행 | — |
| STOP | 요청 67.847 s. 안정, 조밀 자세 근거 248구간, 0.5 s 동안 변위 5.1e-5 m. MCU `ESTOP_LATCHED`/`software_stop`. 정지 검사 6항목 모두 참 | 정상 정지 |
| 종료 | Gazebo 서버 종료 확인, 자식 14개 모두 정상 종료, 남은 시험 PID 0 | 정상 종료 |
| 판단 기록 | footer `accepted=written=3421`, `dropped=0`. 판단 호출 3,040회(사례창 안 2,989회, 밖 51회), 취득 누락 0 | r34 B의 사례창 밖 누락 2회는 이번에 없었다 |

## 2. r34 B와 r35 B 비교

| | r34 B(원 B) | r35 B |
|---|---|---|
| 생산 변경 | r34 | `piecewise_wall.py` 잡음 과분할 보완(r35) |
| 원 판정 실패 | `no_permanent_stall`, `final_gap`, `full_lap` | 같음 |
| 보충 판정 실패 | `one_lap`, `no_contacts`, `additional_valid`, `sustained_motion`, `decision_scans_acquired` | `one_lap`, `sustained_motion` |
| 진행 | 9.11 m | 42.55 m(s 41.66→84.21) |
| 최장 저속 | 48.396 s(18.965–67.361, s 6.13) | 6.764 s(60.595–67.359, s 39.57) |
| 저속 구간 수(규칙 동일) | 1 | 6 |
| 최고 속도 | 1.558 m/s | 1.563 m/s |
| 주 actor 최소 간격 / swept 하한 | 0.849 / 0.847 m | 0.809 / 0.808 m |
| 사례 끝 주 actor 간격 | 3.96 m | 6.83 m(`final_gap` 1.2 m 초과) |
| 접촉 | 추가 actor가 뒤에서 관통 1회(쌍 행 1,840개) | 주 actor 메시지 0, 추가 actor 사건 0 |
| 행동 수(평가기 표본) | FOLLOW 106 / HOLD 487 | FOLLOW 508 / HOLD 84 |
| 정체 직접 사유 | `follow_boundary_unconfirmed`, 선택자 `SOLO/baseline/baseline_not_normal` | `parallel_hold_partial_observation_outside_trial`, 선택자 `RELEASED/parallel/large_motion_region_unobserved` |
| 소스 입력 | 902개 불변 | 903개 불변 |
| STOP 요청 | 67.843 s, 정상 | 67.847 s, 정상 |

r35 B는 r34 B가 멈춘 첫 코너 앞(s 6.13)을 지나갔다. 대신 s 39.57에서 **다른 사유로** 사례 끝까지 멈췄다. 접촉 실패가 없어진 것은 r34 B의 접촉이 장시간 정체 뒤 각본 actor가 한 바퀴를 돌아 생긴 후속 결과였기 때문으로 보인다(추정). 이번 정체는 늦게 시작해 actor가 돌아올 시간이 없었다.

## 3. 주요 저속 구간(r35 B, 상위 3개)

규칙은 `assess.py`와 같다. 사례창 안에서 첫 0.10 m/s 초과 뒤 `trace.ground_speed_mps < 0.05`가 이어진 구간이다. 사유는 `autonomy_status.jsonl`(약 5 Hz 표본)에서 HOLD 시작 0.5 s 전부터 구간 끝까지 모았다. 명령은 `trace.json`의 제어기 명령, `/drive`, `/drive/safe`이고, 판단 출력은 `follow_decisions.jsonl` 호출의 첫 원소(속도)다. actor 위치는 자차 기준 (앞 +x, 왼쪽 +y)이다.

| 구간 | 진행 | 속도 평균/최대 | 명령 최대(제어기/drive/safe) | 판단 출력 | 지배 사유·선택자 | 보호층 | actor 위치 | 회복 |
|---|---|---|---|---|---|---|---|---|
| 60.595–67.359 s(6.764 s), s 39.57, HOLD 시작 60.394 | 0.00 m | 0.001 / 0.046 m/s | 0 / 0 / 0 | 338회 모두 0 | HOLD 중 상태 33행 모두 `parallel_hold_partial_observation_outside_trial`, `RELEASED/parallel/large_motion_region_unobserved`(직전 FOLLOW 2행 제외) | `clear` 68/68, 여유 0.84–0.88 m | 주 (1.20, 1.06)→(5.03, 4.88), 간격 1.38→6.78 m. 둘째 (5.49, 5.26)→(7.54, 9.12) | **없음.** 사례창 끝에서 잘렸다. STOP(67.847)까지 변위 5.0e-5 m |
| 44.695–45.197 s(0.502 s), s 26.77 | 0.01 m | 0.032 / 0.138 m/s | 0.22 / 0.24 / 0.22 | 25회 중 13회 양수, 최대 0.75 | `parallel_hold_baseline_unavailable`(`baseline_not_normal`) ↔ `follow_release_wait` ↔ `local_follow` 반복 | `clear` | 주 (0.59, 1.14)→(0.64, 1.53), 간격 1.05→1.44 m | 45.299 s 회복(+0.102 s) |
| 38.859–39.061 s(0.202 s), s 22.07 | 0.01 m | 0.079 / 0.158 m/s | 0.24 / 0.30 / 0.30 | 11회 모두 양수, 최대 1.2 | 직전 38.56 `parallel_hold_reference_not_straight`(`guard_curve_rejected_toward_right`), 이후 `local_follow` | `clear` | 주 (0.66, −1.19), 간격 1.14→1.30 m | 39.162 s 회복(+0.101 s) |

나머지 3개 구간은 각각 0.1 s 안팎이다.

### 6.764 s 정체의 원인 분류(기록 범위 안)

- **독립 보호층이 아니다.** 보호 사유는 내내 `clear`다. 제어기 명령이 먼저 0이었고, `/drive`와 `/drive/safe`도 0이었다. 보호층이 속도를 깎은 기록은 없다.
- **실제 선행차가 막은 것이 아니다.** 주 actor는 자차 좌전방에서 멀어졌다(간격 1.38→6.78 m). 둘째 actor는 7 m 이상 떨어져 있었다.
- **제어기가 붙잡은 정지 patch는 정적 벽과 겹친다.** 지속 patch 중앙값은 (0.641, −0.187) m다(직전 0.5 s 포함 35행. HOLD 중 33행은 x 0.61–0.67, y −0.21–−0.17 m). 63.98 s 판단 입력 스캔에서 가장 가까운 광선은 patch에서 4 mm 떨어져 있다. 이 광선은 22개 광선, 0.163 m 길이의 묶음에 속한다. 평가 전용 벽 메시(`batched_walls_0.obj`) XY 투영과 대조하면 patch 점은 벽 footprint 안에 있다(거리 0). 묶음 양끝은 벽에서 2.5 cm 떨어져 있다. 묶음 밖 이웃 광선은 끊기지 않았고, 5.3–5.5 cm 간격으로 이어진다. 따라서 이 묶음 끝은 연결 규칙(0.05 m) 경계에 걸린 잡음 간격으로 보인다(추정). `persistence.reason`은 `low_motion_patch_candidate`다. 이 값은 정지 상태로 증명된 것이 아니다(`identity_or_stationarity_proven=false`).
- **기록된 직접 사유는 미관측 계열이다.** 선택자가 `parallel`을 고른 채 `large_motion_region_unobserved`로 판단했다. 그 결과 `parallel_hold_partial_observation_outside_trial` HOLD가 사례 끝까지 이어졌다. 어떤 영역·광선이 「미관측」으로 판정됐는지는 상태 기록에 필드가 없어 **확인하지 못했다.**
- **경계 인식(`follow_boundary_unconfirmed`)은 이번 정체의 기록 사유가 아니다.** r34 B와는 사유가 다르다.
- **참고(r34 A, 다른 주행):** 무주입 r34 A도 같은 진행 위치를 60.4–61.0 s에 지났고, 같은 벽 patch (0.65, −0.20)을 보았다. 그때 선택자는 `SOLO/baseline/no_side_threat_seen`이었다. `follow_boundary_unconfirmed` HOLD는 1행만 나왔고, 최저 0.032 m/s에서 회복했다. 두 기록에서 드러나는 차이는 선택자 상태(`RELEASED/parallel`와 `SOLO`)다. r35 B는 59.54 s에 `released_large_motion_observed`였다. 주 actor가 좌전방을 가로지른 것이 이 상태의 원인일 수 있으나, 이것은 추정이다.

요약하면, 지속 patch는 평가 좌표계 가정에서 실제 actor보다 정적 벽과 겹쳤다. 이 patch가 현재 정지의 직접 원인이라는 뜻은 아니다. 그 patch를 놓아주지 않은 기록 사유는 `parallel` 선택 상태의 「큰 움직임 영역 미관측」이다. 보호층은 원인이 아니다. 미관측 판정의 구체 영역과 `parallel` 선택이 해제되지 않은 이유는 미확인이다. 짧은 두 구간도 `parallel` HOLD 직후에 생겼다. 두 구간 모두 actor가 측전방 약 1.1–1.2 m에 있었고, 0.1 s 만에 회복했다.

## 4. 처리시간·스캔 나이·주입 검증

| | r34 A(무주입) | r34 B | r35 B |
|---|---|---|---|
| `motion.processing_ms` 표본 수 | 288 | 285 | 291 |
| p50 / p95 / 최대(ms) | 0.506 / 0.703 / 1.024 | 0.499 / 0.673 / 1.058 | 0.506 / 0.724 / 1.080 |
| 스캔 나이(전체 호출) 호출 수 | 3,051 | 3,041 | 3,040 |
| p50 / p95 / 최대(s) | 0.156 / 0.196 / 0.196 | 0.197 / 0.257 / 0.257 | 0.197 / 0.257 / 0.257 |
| 취득 = 수신 | 686 = 686 | 682 = 682 | 683 = 683 |
| 주입 지연 최소 / 평균 / 최대(s) | 주입 없음 | 0.032 / 0.0503 / 0.070 | 0.032 / 0.0504 / 0.070 |
| 최대 거리 오차(m) | — | 0.0300 | 0.0300 |

- `motion.processing_ms`는 상태 기록(약 5 Hz 표본)의 움직임 보상 단계 값이다. 판단 호출마다 남은 값이 아니다. 판단 전체의 계산 시간 필드는 **기록되어 있지 않다.** 세 run의 값은 서로 비슷하다. r35 변경이 이 단계 시간을 늘렸다는 근거는 없다. 다만 `piecewise_wall` 계산 시간은 이 값에 들어가지 않으므로 따로 측정되지 않았다.
- 스캔 나이는 판단 입력의 `reference_s`에서 스캔 first-ray 스탬프를 뺀 값이다. 다시 계산한 값은 평가 기록과 같았다. r35 사례창 안 2,989회 값도 같다. B−A 차이 p50은 +0.041 s로, 등록 지연 0.05±0.02 s 범위와 맞는다.
- 주입 기록은 `acquisition.json`의 `injected_delay_s`와 `impairment.max_abs_error_m`에서 다시 계산했다. report의 `lidar_stress_evidence`(검증 참)와 일치한다.

## 5. 현재 원인과 다음 조사

- **현재 원인(기록 범위):** r35 보완 뒤 첫 코너 정체는 이 run에서 재현되지 않았다. 대신 s 39.57에서 제어기 HOLD가 사례 끝까지 이어졌다. 기록 사유는 `parallel` 선택 상태의 `large_motion_region_unobserved`이고, 붙잡은 patch는 정적 벽과 겹친다. 보호층과 실제 선행차는 원인이 아니다. 한 seed·한 주입 조합의 결과이며, 실패를 바꾸지 않는다.
- **다음 조사(새 주행 없이 가능한 것):**
  1. 60.36 s 전후 저장 판단 입력을 오프라인으로 같은 바이트 재생한다. 미관측으로 판정된 영역·광선과 `parallel` 선택이 해제되지 않은 조건을 확인한다. 이것은 같은 입력의 재현이며 새 주행이 아니다.
  2. 같은 벽 patch를 r34 A는 `SOLO` 상태로 지나갔다. 59.5–60.4 s에 선택자가 `RELEASED/parallel`로 들어간 경위를 확인한다.
  3. 0.5 s·0.2 s 짧은 구간도 `parallel` HOLD에서 시작했다. 같은 경로인지 함께 확인한다.
- 알고리즘 수정 설계는 이 감사의 범위가 아니다.

## 한계

- 상태 사유는 약 5 Hz 표본이다. 판단 호출 단위 사유는 판단 출력 속도만 전수 집계했다.
- patch 좌표계는 「광선 xy + lidar_x 0.06 m」의 차량 좌표로 가정했다. 가장 가까운 광선이 4 mm 안에 있어 이 가정과 모순되지는 않는다. 다만 좌표계 정의를 소스에서 확인하지는 않았다. 벽 대조에 쓴 자세 기준점과 차량 원점의 차이(수 cm)도 확인하지 않았다.
- 벽 메시 대조는 평가 전용 정답 지형을 쓴 사후 분석이며, 제어 입력과 무관하다.
- r34 A와의 같은 위치 비교는 다른 주행이다. actor와의 상대 시점이 다르다.
