#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
base=artifacts/validation/2026-10-24/e3_harness_v1
test ! -e "$base/run"
test ! -e "$base/tests.log"
bash "$base/checks.sh" > "$base/tests.log" 2>&1
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
python3 -B "$base/prepare.py" > "$base/prepare.log" 2>&1
set +e
python3 -B scripts/validate_g1_traffic.py --case stopped_center --seed 0 --mode g1 --record-follow-decisions --boundary-evidence corner_distance_candidate --occupancy-timing per_ray_candidate --corridor-fallback --actor-overlay config/tests/e3_two_actors_harness_v1.json --output "$base/run" > "$base/runner.log" 2>&1
status=$?
set -e
printf '%s\n' "$status" > "$base/exit.txt"
tail -8 "$base/runner.log"
exit "$status"
