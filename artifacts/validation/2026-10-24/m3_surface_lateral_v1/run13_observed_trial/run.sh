#!/usr/bin/env bash
set -uo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
out='artifacts/validation/2026-10-24/m3_surface_lateral_v1/run13_observed_trial'
set +e
PYTHONPATH=scripts:src/arena_autonomy:src/arena_vehicle_interface python3 -B -m pytest -q -p no:cacheprovider --junitxml="$out/regression.xml" tests/test_observed_lateral_trial.py tests/test_lateral_intent.py tests/test_lateral_sweep_review.py tests/test_surface_contact_eval.py >"$out/regression.log" 2>&1
trc=$?
printf 'pytest_exit_code=%s\n' "$trc" >>"$out/regression.log"
cat "$out/regression.log"
if [ "$trc" -ne 0 ]; then printf 'screen_skipped_due_to_regression_failure\n'; exit "$trc"; fi
PYTHONPATH=scripts:src/arena_autonomy:src/arena_vehicle_interface python3 -B 'artifacts/validation/2026-10-24/m3_surface_lateral_v1/screen_observed_trial.py' "$out/screen.json" >"$out/screen.log" 2>&1
src=$?
printf 'screen_exit_code=%s\n' "$src" >>"$out/screen.log"
cat "$out/screen.log"
exit "$src"