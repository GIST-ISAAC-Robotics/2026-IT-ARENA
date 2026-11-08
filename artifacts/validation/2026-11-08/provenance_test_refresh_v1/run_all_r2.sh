#!/usr/bin/env bash
# 2회차(직전 부모 + 명시 분기 1건): 관련 검사와 표적 변이 4종(WSL, 시뮬레이터 미실행). 1회차 로그는 그대로 둔다.
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
D=artifacts/validation/2026-11-08/provenance_test_refresh_v1
T1=tests/test_g1_dense_stop.py::test_evaluation_changes_have_explicit_successor_not_rewritten_old_contract
T2=tests/test_physics_step_option.py::test_contract_records_evaluation_script_changes
bash $D/run_tests.sh r2_after tests/test_g1_dense_stop.py tests/test_physics_step_option.py tests/test_source_provenance_r0_plus_b3.py
for m in script_bytes parent_bytes parent_link fake_parent; do
  PROVENANCE_MUTATION=$m PYTHONPATH="$PWD/$D" bash $D/run_tests.sh r2_mutation_$m -p mutation_plugin $T1 $T2
done
