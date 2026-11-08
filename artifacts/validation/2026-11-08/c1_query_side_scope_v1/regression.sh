#!/usr/bin/env bash
set -o pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-08/c1_query_side_scope_v1
bash artifacts/validation/2026-11-01/turn_preview_v1/environment.sh python3 -B -m pytest -q \
  tests/test_query_local_side_scope.py tests/test_same_path_side_scope.py \
  tests/test_same_path_forward.py tests/test_same_path_gap_probe.py \
  tests/test_same_path_speed_recheck.py tests/test_forward_same_path_provenance.py \
  tests/test_source_provenance_r0_plus_b3.py tests/test_g1_dense_stop.py \
  tests/test_physics_step_option.py > "$A/regression.log" 2>&1
status=$?
printf '%s\n' "$status" > "$A/regression.exit"
tail -n 12 "$A/regression.log"
exit "$status"
