#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_vehicle_interface:$PWD/src/arena_autonomy:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-16/g1_linear_observation_v1
test ! -e "$base/tests.log"
python3 -B -m pytest -q tests/test_linear_observation.py > "$base/tests.log" 2>&1
python3 -B scripts/probe_linear_observation.py artifacts/validation/2026-10-16/g1_corridor_solo_v1/run/follow_decisions.jsonl --output "$base/audit" > "$base/audit.log" 2>&1
tail -2 "$base/tests.log"
tail -1 "$base/audit.log"
