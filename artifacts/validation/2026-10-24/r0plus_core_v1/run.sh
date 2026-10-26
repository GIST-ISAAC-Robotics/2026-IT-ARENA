#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
base=artifacts/validation/2026-10-24/r0plus_core_v1
test ! -e "$base/stopped_center"
test ! -e "$base/build.log"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select arena_autonomy arena_bringup > "$base/build.log" 2>&1
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
python3 -B "$base/prepare.py" > "$base/prepare.log" 2>&1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
set +e
python3 -B scripts/validate_g1_traffic.py --case stopped_center --seed 0 --mode r0plus --record-follow-decisions --occupancy-timing per_ray_candidate --actor-overlay config/tests/e3_two_actors_harness_v1.json --output "$base/stopped_center" > "$base/runner.log" 2>&1
status=$?
set -e
printf '%s\n' "$status" > "$base/exit.txt"
tail -8 "$base/runner.log"
exit "$status"
