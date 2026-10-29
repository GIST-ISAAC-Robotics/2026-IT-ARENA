#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-28/rotation_reobservation_review_v1
/usr/bin/python3 -B "$base/validate_connected.py" artifacts/validation/2026-10-28/rotation_stress_v1/run3_course/follow_decisions.jsonl "$base/full_stress_v1/decisions.jsonl" --output "$base/connected_stress_v1" > "$base/connected_stress_v1.log" 2>&1
/usr/bin/python3 -B "$base/validate_connected.py" artifacts/validation/2026-10-28/official_traffic_v1/run2_course/follow_decisions.jsonl "$base/full_nominal_v1/decisions.jsonl" --output "$base/connected_nominal_v1" > "$base/connected_nominal_v1.log" 2>&1
