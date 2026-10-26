#!/usr/bin/env bash
set -uo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
out='artifacts/validation/2026-10-24/m3_surface_lateral_v1/run16_follow_replay'
set +e
PYTHONPATH=scripts:src/arena_autonomy:src/arena_vehicle_interface python3 -B -m pytest -q -p no:cacheprovider --junitxml="$out/regression.xml" tests/test_m3_observed_follow.py tests/test_observed_lateral_trial.py >"$out/regression.log" 2>&1
trc=$?
printf 'pytest_exit_code=%s\n' "$trc" >>"$out/regression.log"
cat "$out/regression.log"
if [ "$trc" -ne 0 ]; then printf 'replay_skipped_due_to_regression_failure\n'; exit "$trc"; fi
PYTHONPATH=scripts:src/arena_autonomy:src/arena_vehicle_interface python3 -B 'artifacts/validation/2026-10-24/m3_surface_lateral_v1/replay_m3_follow.py' "$out/replay.json" >"$out/replay.log" 2>&1
rrc=$?
printf 'replay_exit_code=%s\n' "$rrc" >>"$out/replay.log"
cat "$out/replay.log"
exit "$rrc"