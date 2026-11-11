#!/usr/bin/env bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 2
A=artifacts/validation/2026-11-10/released_straight_restore_r65_v1
if [[ -e "$A/tests.log" ]]; then exit 4; fi
bash artifacts/validation/2026-11-01/turn_preview_v1/environment.sh /usr/bin/python3 -B -m pytest -q tests/test_released_straight_recovery.py tests/test_pass_release_trial.py tests/test_guard_curve_trial.py tests/test_local_follow.py tests/test_bypass_observed_return.py tests/test_m3_execution_trial.py tests/test_source_provenance_r0_plus_b3.py > "$A/tests.log" 2>&1
code=$?; printf '%s\n' "$code" > "$A/tests.exit"
exit "$code"
