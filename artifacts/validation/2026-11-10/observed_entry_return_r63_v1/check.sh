#!/usr/bin/env bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 2
A=artifacts/validation/2026-11-10/observed_entry_return_r63_v1
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
bash "$ENV" /usr/bin/python3 -B -m pytest -q \
 tests/test_bypass_observed_return.py tests/test_bypass_entry_guide.py \
 tests/test_bypass_handoff_corridor_trial.py tests/test_bypass_signed_hold_trial.py \
 tests/test_bypass_motion_pause_guide.py tests/test_adaptive_entry_preview.py \
 tests/test_bypass_entry_path_conflict.py tests/test_observed_rejoin.py \
 tests/test_stationary_rejoin_state.py tests/test_source_provenance_r0_plus_b3.py \
 > "$A/tests.log" 2>&1
code=$?; printf '%s\n' "$code" > "$A/tests.exit"
if (( code != 0 )); then exit "$code"; fi
bash "$ENV" /usr/bin/python3 -B "$A/replay.py" > "$A/replay.log" 2>&1
code=$?; printf '%s\n' "$code" > "$A/replay.exit"
exit "$code"
