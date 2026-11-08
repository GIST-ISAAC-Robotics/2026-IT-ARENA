#!/usr/bin/env bash
set -o pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-08/entry_preview_wiring_v1
bash artifacts/validation/2026-11-01/turn_preview_v1/environment.sh python3 -B -m pytest -q \
  tests/test_adaptive_entry_preview.py tests/test_stopped_entry_centering.py \
  tests/test_bypass_entry_trial.py tests/test_bypass_entry_sequence.py \
  tests/test_bypass_entry_path_conflict.py tests/test_bypass_entry_guide.py \
  tests/test_bypass_entry_follow.py tests/test_bypass_handoff.py \
  tests/test_bypass_handoff_corridor_trial.py \
  tests/test_g2_handoff_provenance.py tests/test_g2_handoff_corridor_provenance.py \
  tests/test_source_provenance_r0_plus_b3.py tests/test_g1_dense_stop.py \
  tests/test_physics_step_option.py > "$A/regression.log" 2>&1
status=$?
printf '%s\n' "$status" > "$A/regression.exit"
tail -n 8 "$A/regression.log"
exit "$status"
