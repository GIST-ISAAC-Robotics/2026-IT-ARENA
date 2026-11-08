# g1g2 C1 거리오차·지연 A/B — 실행 완료(A 원 실패·보충 통과, B 원·보충 실패)

2026-11-08. 사전 계획과 판정선은 [PLAN.md](PLAN.md)에 있다. 이 문서의 상태 표기 `AWAITING_RUN`은 실행 전 상태였다. 아래 「실행 전 준비」 절은 당시 기록이므로 그대로 보존한다.

## 결과(등록 후 A→B 순차 실행, 각 60초)

| | A 무주입 | B C1 주입(±0.03 m, 0.05±0.02 s) |
|---|---|---|
| 종료 기록 | `HARNESS_EXIT=1 … ASSESSMENT_EXIT=0` | `HARNESS_EXIT=1 … ASSESSMENT_EXIT=1` |
| 원 판정(`report.passed`) | 실패: `final_gap` | 실패: `no_permanent_stall`, `final_gap`, `full_lap` |
| 보충 판정([assess.py](assess.py)) | 통과 | 실패: `one_lap`, `no_contacts`, `additional_valid`, `sustained_motion`, `decision_scans_acquired` |
| 진행 / 최장 저속 | 48.14 m / 0.1 s | 9.11 m / 48.40 s(18.965 s부터 `HANDOFF_FOLLOW`/`HOLD`) |
| 주 actor 최소 표본 간격 | 0.696 m | 0.849 m |
| 소스 불변 | 902입력 불변·스냅숏 일치 | 902입력 불변·스냅숏 일치 |
| 주입 검증 | 무주입 상태 확인 | 추가지연 0.032–0.070 s, 최대 오차 0.0300 m, 취득=수신=682 |
| 판단 시점 스캔 나이 p50(기록만) | 0.156 s | 0.197 s(B−A +0.041 s) |

원 수치는 [pair_comparison.json](pair_comparison.json), [A](A_no_injection/assessment_run1.json), [B](B_c1_injection/assessment_run1.json)에 있다. 이 원본은 수정하지 않았다.

### 증거 감사([audit/](audit/README.md))

실행 없이 원본만 읽어 감사했다. 판정선은 바꾸지 않았다.
- **B `decision_scans_acquired`:** 취득 기록에 없는 판단 호출은 2회이고, 스캔으로는 1개(68.203 s)다. 원인은 반올림이나 출처 불일치가 아니라 **기록 범위 끝 경계**다. 이 스캔은 사례 종료(67.401)와 STOP 요청 이후, 평가기 수신 종료(68.36) 뒤인 (68.36, 68.38]에 해제되었다. 원 실패는 그대로 보존한다. 범위를 바꾸려면 별도 정정 평가를 사전 등록해야 한다. 정정하더라도 B의 결과는 같다.
- **B 접촉:** 연속 사건은 1회다(64.845–65.344 s, 메시지 500개, 쌍 행 1,840개). 자차가 18.965 s부터 정지한 뒤, 사전 일정으로 한 바퀴를 돈 운동학 추가 actor가 뒤에서 0.8 m/s로 자차를 관통했다. 접촉 중 자차 변위는 21 μm다. 실패로 보존하며, 정체의 후속 결과로 기록한다.
- **A:** 누락·접촉 0이다.

## 실행 전 준비(당시 기록, 보존)

아래는 등록·실행 전에 쓴 내용이다. 작업은 Claude(Opus 5.5)가 했다. 「소스 등록·Gazebo·colcon은 하지 않았다」는 문장도 그때 기준이다.

### 적용

| 파일 | 변경 | sha256 |
|---|---|---|
| `scripts/validate_g1_traffic.py` | [staging 패치](../g1g2_stress_staging_v1/README.md)를 원본 해시 `e3e7ae73…` 일치를 확인한 뒤 바이트 그대로 반영 | `77f97aef810e176bd96fc46a14c9365a9266fc1a03a2d57dd6ff78ae18447dce` |
| `tests/test_g1g2_lidar_stress_cli.py` | 새 파일, 24건. 출력과 사본 경로는 `artifacts/.<이름>_<pid>_<uuid>`로 고유화했고 출력은 생성하지 않음 | `d96724e8…` |
| `tests/test_source_provenance_r0_plus_b3.py` | `M3_SUCCESSORS += ('config/tests/g1g2_lidar_stress_r34_sha256.json',)` 한 행 추가(index 161) | `e3c89eba…` |
| `config/tests/g1g2_lidar_stress_r34_sha256.json` | 부모 r33(`534ebbca…`), before 스냅숏 [before.zip](before.zip)(`348d003a…`, 적용 전 두 파일), 평가 스크립트 하나 before→after | `05a8a7a9…` |

### 시험(WSL, `run_tests.sh`, python3 -B, 시뮬 없음) — 적용 당시 결과

| label | 파일 | 결과 |
|---|---|---|
| `provenance_before`(적용 전) | 출처 사슬 관련 6개 + `test_m3_lidar_stress_cli.py` | 229 passed, 2 failed |
| `after_apply`(적용 후) | 위 목록 + `test_g1g2_lidar_stress_cli.py` | 254 passed, 2 failed |

두 실패는 적용 전후에 같은 기존 실패이며, 이번 적용과 무관하다. 이 시험 결과는 적용 시점의 기록이며, 실행 완료 뒤에는 다시 돌리지 않았다(이 감사는 생산 코드·시험을 실행하거나 고치지 않았다).
- `test_g1_dense_stop.py::test_evaluation_changes_have_explicit_successor_not_rewritten_old_contract`
- `test_physics_step_option.py::test_contract_records_evaluation_script_changes`: 체인 목록이 `fresh_confirmation_r2`에서 끝난다.

새 24건과 `test_registered_oct26_and_nov01_successors_have_preserved_source_chain[161]`은 통과했다. 나머지 G1/M3 회귀는 같은 바이트로 staging에서 실행했다([결과](../g1g2_stress_staging_v1/README.md)).

### 도구 점검(쓰기 없음)

- `register.py --check`: 소스 902개, 두 run 인자(A = r27 인자에서 설정·overlay 경로만 사본으로 교체, B = A + 등록 프로필)를 확인했다. 아무것도 쓰지 않았다(`checks/register_check.log`).
- `check_runtime.py --dry`: 모듈 7개, share 5개, 실행 파일 5개, 총 17항목이 현재 src 바이트로 해석된다(`checks/runtime_dry.log`).
- `assess.py` 함수를 기존 결과에 읽기 전용으로 적용했다(`checks/assess_dry_check.json`).
  - r27 run1에서 12조건이 원 판정과 같았다. A 조건은 참이다. 판단 3,047회 중 누락은 0이었다.
  - m3 run57(등록 C1 주입)에서 B 조건은 참이고, A 조건은 거짓이다.
  - 추가지연 최대값 0.07000000000000028처럼 부동소수 표현만 흡수하도록 1e-8 허용을 두었다. 이는 `audit_injection`과 같은 허용값이다.
- `bash -n run.sh` 통과.
