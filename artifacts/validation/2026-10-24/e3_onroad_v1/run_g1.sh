#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
base=artifacts/validation/2026-10-24/e3_onroad_v1
test ! -e "$base/g1"
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
set +e
python3 -B scripts/validate_g1_traffic.py --case slow_lead --seed 0 --mode g1 --record-follow-decisions --boundary-evidence corner_distance_candidate --occupancy-timing per_ray_candidate --corridor-fallback --actor-overlay config/tests/e3_onroad_encounter_v1.json --output "$base/g1" > "$base/g1_runner.log" 2>&1
status=$?
set -e
printf '%s\n' "$status" > "$base/g1_exit.txt"
tail -8 "$base/g1_runner.log"
exit "$status"
