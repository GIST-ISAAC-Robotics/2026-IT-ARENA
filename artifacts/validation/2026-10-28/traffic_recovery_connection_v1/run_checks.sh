#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
BASE="$PWD/artifacts/validation/2026-10-28/traffic_recovery_connection_v1"
OLD="$PWD/artifacts/validation/2026-10-28/traffic_residual_v1"
export PYTHONPATH="$BASE:$OLD:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
set +e
/usr/bin/python3 -B -m pytest -q -p production_test_plugin tests/test_timed_policy_probe.py tests/test_timed_missing_sweep_probe.py tests/test_joint_missing_sweep_probe.py tests/test_surface_anchor_probe.py tests/test_same_path_gap_probe.py "$OLD/test_recovery_policy.py" "$OLD/test_safety_shadow.py" "$BASE/test_selector_runtime.py" > "$BASE/counterexamples_v2.log" 2>&1
code=$?
printf '%s\n' "$code" > "$BASE/counterexamples_v2.exit"
if [ "$code" -ne 0 ]; then exit "$code"; fi
/usr/bin/python3 -B -m pytest -q tests > "$BASE/regression.log" 2>&1
code=$?
printf '%s\n' "$code" > "$BASE/regression.exit"
exit "$code"
