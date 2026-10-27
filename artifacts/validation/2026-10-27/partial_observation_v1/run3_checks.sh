#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-27/partial_observation_v1
test ! -e "$base/run3_tests.log"
python3 -B -m pytest -q tests/test_partial_observation.py tests/test_lidar_observation.py tests/test_lidar_safety.py tests/test_recent_observation_safety.py tests/test_g1_solo_candidate_validation.py > "$base/run3_tests.log" 2>&1
python3 -B scripts/probe_partial_observation.py artifacts/validation/2026-10-16/g1_corridor_solo_v1/run/follow_decisions.jsonl --output "$base/run3_saved_input" > "$base/run3_probe.log" 2>&1
tail -4 "$base/run3_tests.log"
cat "$base/run3_probe.log"
