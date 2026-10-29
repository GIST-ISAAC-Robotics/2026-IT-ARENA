#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-29/bump_traffic_review_v1
test ! -e "$base/rear_tolerance_v1.json"
test ! -e "$base/rear_tolerance_v1.log"
/usr/bin/python3 -B -m pytest -q -p no:cacheprovider "$base/test_rear_tolerance.py" tests/test_rear_fringe_trial.py > "$base/rear_tolerance_contracts_v1.log" 2>&1
/usr/bin/python3 -B "$base/probe_rear_tolerance.py" artifacts/validation/2026-10-28/bump_traffic_v1/run4/follow_decisions.jsonl --output "$base/rear_tolerance_v1.json" > "$base/rear_tolerance_v1.log" 2>&1
/usr/bin/python3 -B "$base/profile_latched_policy.py" --output "$base/policy_profile_v1" > "$base/policy_profile_v1.log" 2>&1
