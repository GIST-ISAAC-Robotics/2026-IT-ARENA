#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-16/g1_corridor_followup_v1
test ! -e "$base/tests.log"
python3 -B -m pytest -q tests/test_corridor_input_recovery.py tests/test_corridor_follow.py tests/test_lidar_observation.py > "$base/tests.log" 2>&1
python3 -B scripts/profile_follow_trace.py artifacts/validation/2026-10-16/g1_corridor_solo_v1/run/follow_decisions.jsonl --output "$base/profile" --repeats 1 --warmups 0 > "$base/profile.log" 2>&1
tail -2 "$base/tests.log"
tail -2 "$base/profile.log"
