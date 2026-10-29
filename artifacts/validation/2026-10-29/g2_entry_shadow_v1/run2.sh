#!/usr/bin/env bash
set -o pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-29/g2_entry_shadow_v1
test ! -e "$base/run2_tests.log" || exit 90
/usr/bin/python3 -B -m pytest -q -p no:cacheprovider tests/test_bypass_entry_trial.py tests/test_parallel_lane_trial.py tests/test_pass_release_trial.py > "$base/run2_tests.log" 2>&1
tests=$?
/usr/bin/python3 -B scripts/probe_bypass_entry.py --output "$base/run2" > "$base/run2_probe.log" 2>&1
probe=$?
printf '{"tests":%s,"probe":%s}\n' "$tests" "$probe" > "$base/run2_exit.json"
test "$tests" -eq 0 && test "$probe" -eq 0
