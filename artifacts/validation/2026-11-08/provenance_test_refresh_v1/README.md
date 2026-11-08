# 평가 스크립트 출처 검사 목록 낙후 정리 (2026-11-08)

## 문제

`tests/test_g1_dense_stop.py::test_evaluation_changes_have_explicit_successor_not_rewritten_old_contract`와
`tests/test_physics_step_option.py::test_contract_records_evaluation_script_changes`가 후속 계약 목록을
`fresh_confirmation_r2`까지만 각자 복제하고 있었습니다. 그 뒤 계약(`g1g2_lidar_stress_r34`까지)이 등록한
`scripts/validate_g1_traffic.py` 변경을 따라가지 못해 현재 해시(`77f97aef…`)와 목록 끝 해시(`61917b44…`)가 달라
r34 적용 전후 모두 실패했습니다. 원 실패 로그:
`../g1g2_lidar_stress_pair_v1/checks/pytest_after_apply.log` (254 passed, 2 failed, 수정하지 않음).

## 변경

- 두 검사가 `tests/test_source_provenance_r0_plus_b3.py`의 공통 목록 `M3_SUCCESSORS`(및 `CHAIN`, `R0E3`, `B3`)를
  import합니다. `M3_SUCCESSORS` 이전의 고정 접두(물리 step·scheduled_command·dense_stop 계열)만 각 검사에 남습니다.
  다음 후속 계약은 공통 목록에 한 번 추가하면 두 검사에도 반영됩니다.
- 유지한 의미: 부모 계약 바이트 해시 일치, 각 평가 스크립트의 `before == 이전 after` 연속성,
  기존 계약 미수정(부모 해시로 확인), 현재 파일 최종 해시 일치. 물리 step 검사의 `scheduled_command_r0..r7` 구간은
  기존처럼 부모 이름만 확인합니다.
- 부모 조건(2회차 확정): 모든 항목은 직전 계약을 부모로 가져야 합니다. 공통 목록에서 실제 기록된 분기
  `continuous_prefix_r1 → rotation_reobservation_r1` 한 건만 새 helper `tests/provenance_chain_helpers.py`의
  `expected_parent()`가 정확한 이름쌍으로 허용합니다. 공통 출처 검사에는 분기 map이 없어 helper에 두었습니다.
  1회차에서 쓴 "사슬에서 이미 지난 계약" 조건은 너무 느슨해서 2회차에 폐기했습니다(1회차 로그만 보존).
- 소스·config·공통 출처 검사·A/B 폴더는 수정하지 않았습니다(`checks/sha256_before.txt`, `checks/sha256_after_r2.txt`).

## 검사

2회차(현행 코드) 결과입니다. 1회차 결과(`checks/pytest_after*`, `checks/pytest_mutation_*`, `checks/change.diff`, `checks/sha256_after.txt`)는
완화된 부모 조건 기준이므로 참고용으로만 보존합니다.

| 항목 | 명령 | 결과 |
| --- | --- | --- |
| 공통 목록 연결 확인(읽기 전용) | `chain_check.sh` | 162개 중 직전이 아닌 부모는 `continuous_prefix_r1` 1건, `checks/chain_check.log` |
| 관련 범위(두 파일 + `test_source_provenance_r0_plus_b3.py`) | `run_all_r2.sh` → `run_tests.sh r2_after` | exit 0, 182 passed, `checks/pytest_r2_after.log` |
| 변이: 현재 스크립트 바이트 변경 | `PROVENANCE_MUTATION=script_bytes` | exit 1, 두 검사 모두 최종 해시 줄(90/167)에서 실패 |
| 변이: r33 계약 바이트 변경 | `PROVENANCE_MUTATION=parent_bytes` | exit 1, 부모 해시 줄(81/156)에서 실패 |
| 변이: r34 부모를 사슬 밖 이름으로 | `PROVENANCE_MUTATION=parent_link` | exit 1, 직전 부모 줄(80/154)에서 실패 |
| 변이: r34 부모를 r32로 바꾸고 r32 실제 해시까지 맞춤 | `PROVENANCE_MUTATION=fake_parent` | exit 1, 직전 부모 줄(80/154)에서 실패 |

변이는 `mutation_plugin.py`가 `Path.read_bytes/read_text` 결과만 가로채며 실제 파일은 바꾸지 않습니다.
동결 파일 해시 비교: `checks/sha256_before.txt` ↔ `checks/sha256_after_r2.txt`. 변경 diff: `checks/change_r2.diff`.

## 보존

- `before.zip`: 수정 전 두 검사 파일.
- `checks/change_r2.diff`: 현행 변경 diff(새 helper 포함). `checks/change.diff`는 1회차 diff.

## 한계

- 전체 테스트 묶음은 다시 돌리지 않았고, 두 파일과 공통 출처 검사만 실행했습니다.
- 두 검사는 이제 `test_source_provenance_r0_plus_b3.py`의 목록 정확성에 의존합니다.
- 앞으로 새 분기가 생기면 helper의 `BRANCH_PARENTS`에 정확한 이름쌍을 추가해야 하며, 추가하지 않으면 두 검사가 실패합니다.
