#!/usr/bin/env bash
set -uo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
out='artifacts/validation/2026-10-24/m3_surface_lateral_v1/run15_follow_contract'
set +e
PYTHONPATH=scripts:src/arena_autonomy:src/arena_vehicle_interface python3 -B -m pytest -q -p no:cacheprovider --junitxml="$out/regression.xml" tests/test_m3_observed_follow.py tests/test_observed_lateral_trial.py >"$out/regression.log" 2>&1
rc=$?
printf 'pytest_exit_code=%s\n' "$rc" >>"$out/regression.log"
cat "$out/regression.log"
exit "$rc"