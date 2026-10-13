#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-13/follow_profile_v2
test ! -e "$base/tests.log"
python3 -B -m pytest -q tests/test_follow_profile.py tests/test_follow_trace.py tests/test_fragmented_follow.py > "$base/tests.log" 2>&1
tail -10 "$base/tests.log"
python3 -B scripts/profile_follow_trace.py artifacts/validation/2026-10-13/gapped_boundary_v3/side_left_s0/follow_decisions.jsonl --output "$base/side" > "$base/side.log" 2>&1
tail -2 "$base/side.log"
python3 -B scripts/profile_follow_trace.py artifacts/validation/2026-10-13/g1_candidate_regression_v1/initial_s0/follow_decisions.jsonl --output "$base/initial" > "$base/initial.log" 2>&1
tail -2 "$base/initial.log"
