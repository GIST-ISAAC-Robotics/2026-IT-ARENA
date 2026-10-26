#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
base=artifacts/validation/2026-10-24/r0plus_core_v1
test ! -e "$base/stop_restart"
test ! -e "$base/restart_exit.txt"
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
set +e
python3 -B scripts/validate_g1_traffic.py --case stop_restart --seed 0 --mode r0plus --record-follow-decisions --occupancy-timing per_ray_candidate --actor-overlay config/tests/e3_stop_restart_v1.json --output "$base/stop_restart" > "$base/restart_runner.log" 2>&1
status=$?
set -e
printf '%s\n' "$status" > "$base/restart_exit.txt"
tail -8 "$base/restart_runner.log"
exit "$status"
