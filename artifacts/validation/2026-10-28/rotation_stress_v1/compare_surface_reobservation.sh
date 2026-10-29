#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-28/rotation_stress_v1
test ! -e "$base/surface_replay_stress_v1.log"
/usr/bin/python3 -B "$base/compare_surface_reobservation.py" "$base/run2_course/follow_decisions.jsonl" --output "$base/surface_replay_stress_v1" > "$base/surface_replay_stress_v1.log" 2>&1
test ! -e "$base/surface_replay_nominal_v1.log"
/usr/bin/python3 -B "$base/compare_surface_reobservation.py" artifacts/validation/2026-10-28/official_traffic_v1/run2_course/follow_decisions.jsonl --recorded-without-short-inner --output "$base/surface_replay_nominal_v1" > "$base/surface_replay_nominal_v1.log" 2>&1
