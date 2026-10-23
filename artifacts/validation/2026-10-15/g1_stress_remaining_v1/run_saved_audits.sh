#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-15/g1_stress_remaining_v1
test ! -e "$base/occupancy_audit_tests.log"
python3 -B -m pytest -q tests/test_follow_occupancy_audit.py tests/test_occupancy_timing.py > "$base/occupancy_audit_tests.log" 2>&1
python3 -B scripts/audit_follow_occupancy.py artifacts/validation/2026-10-13/g1_lidar_stress_v1/side_left_uniform_s0/follow_decisions.jsonl --mode oldest --output "$base/prior_oldest.occupancy_audit.json" > "$base/prior_oldest.occupancy_audit.log" 2>&1
python3 -B scripts/audit_follow_occupancy.py artifacts/validation/2026-10-13/g1_per_ray_v1/side_left_uniform_s0/follow_decisions.jsonl --mode per_ray_candidate --output "$base/prior_per_ray.occupancy_audit.json" > "$base/prior_per_ray.occupancy_audit.log" 2>&1
tail -3 "$base/occupancy_audit_tests.log"
tail -1 "$base/prior_oldest.occupancy_audit.log"
tail -1 "$base/prior_per_ray.occupancy_audit.log"
