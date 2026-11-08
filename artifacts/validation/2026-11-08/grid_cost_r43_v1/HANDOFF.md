# r43 격자 비용 절감 — 인계 상태 (five_hour 90% 경고로 중단)

**상태: 구현·소스 계약·대상 시험 완료. 수용 실행(관련 회귀, 전체 before/after 재생·시간·전수 대조)과 결과 README는 미완료.** 실행 중인 작업은 없다. 코드 취소·이전 증거 덮어쓰기는 하지 않았다.

## 소스 변경 (계약 `config/tests/grid_cost_r43_sha256.json`, 부모 r42, `M3_SUCCESSORS`에 r43 한 줄 추가)

| 파일 | before → after (SHA-256 앞 12자) | 내용 |
|---|---|---|
| `local_scene.py` | `6687baae7936` → `a599fcef1124` | C3. `scene_from_scan`이 첫 광선에서 `_blind_limits`로 한 번만 검증·파싱(같은 식·같은 예외 순서, 광선 0개면 미검증). `sectors_deg`가 list/tuple이 아닌 dict(예: 1회용 iterable)는 기존처럼 광선마다 `rear_masked`. 공개 `rear_masked`는 같은 헬퍼로 동일 동작 |
| `lateral_intent.py` | `4683fa9e3532` → `1c8b5f876b51` | 자유 표본 범위 제한 `_free_sample_window`(한 칸 넓힌 격자 slab 교차 → floor(lo/step)-2 ~ ceil(hi/step)+3, [0,counts) 고정, 원래 정수 번호·식 유지). 비유한·`scale·2⁻⁴⁰ ≥ r`·`free/step ≥ 2⁵²`이면 전체 표본. C2 비공개 키워드 `_same_call_returns=(scene, cfg, ReturnGrid)`: 객체 동일성+반환·나이 정확 일치+dtype/shape 확인 시 compact·components **복사** 재사용, 아니면 재계산 |
| `pass_release_trial.py` | `3da761e81ef3` → `69cb51033995` | `threat_view`가 같은 호출의 `light`를 위 키워드로 전달 |

- 원본: [before.zip](before.zip) SHA-256 `413bc8d6…c381c7`(3파일), 사본 `before_src/`. 생성기 [register_contract.py](register_contract.py). 계약 SHA-256 `cca16688…1a314`.
- 소스를 다시 고치면 계약을 삭제 후 재생성해야 한다(`open('x')`).

## 시험·검사 (실제 실행한 것)

| 항목 | 결과 | 근거 |
|---|---|---|
| 새 시험 `tests/test_grid_cost_r43.py` (+오라클 `tests/r43_reference_oracle.py`) | **108 통과**(Claude Opus 5.5가 WSL에서 1회 실행) | `checks/pytest_targeted_1.log/.exit` |
| 오라클 ↔ before.zip AST 비교 | 차이는 의도한 이름·계측 줄뿐 | `checks/oracle_diff.json` |
| 변이 검사(틀린 구간 3종은 실패해야, 맞는 좁은 구간 대조군은 통과해야) | 38/17/34건 실패, 대조군 0건 → 기대대로 | `checks/mutation_probe.log/.exit`(첫 실행 `*_first_run*`은 대조군을 변이로 잘못 분류한 판정식이라 보존만) |
| **smoke 재생**(r42 v3 픽스처 앞 100판단 r25·r41, ABBA+capture) | 명령 스트림 before=after 동일, source_check 5/5 ok. capture: 격자 75개 전체 상태 불일치 0, C2 guard·변형 0, threat_view 불일치 0, scene 200개 불일치 0. 표본 8,523,750 → 5,620,875(격자 안 5,588,028) | `raw_smoke/`, `raw_smoke/summary.json`. **하네스 점검용이며 수용 근거 아님** |

## 미실행 (이어서 할 일, 정확한 명령)

1. 전체 재생(긴 실행, 약 10분 예상·추정): r25 1,540 / r41 1,628 / 오목 0.50 m/s 1,137판단, ABBA 시간 + r41·전체 capture + compare.
   `wsl bash -lc "bash '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-08/grid_cost_r43_v1/run_replay.sh'"`
   → `raw/*.exit`, `raw/run_all.exit`, `summary.json`. 판정: `summary.json`의 `equality`(before_1 대비 전부 True, r25·r41은 `equal_to_r42` True), `capture.totals`의 mismatch/guard/mutated 0, `source_checks` 전부 ok, `timing_mean_before_after`.
2. 관련 회귀 1회(제안 집합, 미실행):
   `wsl bash -lc "bash '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-08/grid_cost_r43_v1/run_tests.sh' related_1 tests/test_grid_cost_r43.py tests/test_local_scene.py tests/test_lateral_intent.py tests/test_lateral_sweep_review.py tests/test_m3_execution_trial.py tests/test_m3_baseline_release.py tests/test_pass_release_trial.py tests/test_traffic_release_fringe.py tests/test_g1_partial_traffic.py tests/test_source_provenance_r0_plus_b3.py"`
   → `checks/pytest_related_1.log/.exit`. provenance 시험은 아직 한 번도 돌리지 않았다. 기존 무관 실패(r42 기록상 `test_measured_lane_preference_provenance` 등)는 기준선과 비교해 구분.
3. 결과 README.md(한국어) 작성: 실제 결과, 버린 표본 수·속도 향상, 위험, 하드웨어 미검증. **아직 없음.** `docs/PROJECT_CONTEXT.md`·계획은 고치지 않는다(지시).

## 위험·가정

- C2 재사용은 같은 `scene`·`cfg` 객체와 반환·나이 일치만 확인한다. 같은 호출 안에서만 전달되므로 compact 자체는 다시 비교하지 않는다(그 비교는 capture가 전수로 수행).
- 자유 표본 제한의 보수성은 반올림 오차 ≪ 한 칸이라는 가정에 기대며, 이를 넘는 좌표는 전체 표본으로 돌아간다. 원래 코드가 메모리 부족으로 실패할 만큼 큰 입력에서는 후보가 같은 상태를 계산해 낼 수 있다(원래와 다른 결과가 아니라 실패 여부 차이).
- smoke 시간 차(r41 p50 22.8/24.1 → 19.1/19.9 ms)는 100판단 단일 값이라 결론에 쓰지 않는다. Jetson·ROS 실시간 성능은 미검증.
