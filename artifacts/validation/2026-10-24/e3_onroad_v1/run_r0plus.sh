#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
base=artifacts/validation/2026-10-24/e3_onroad_v1
test ! -e "$base/r0plus"
test ! -e "$base/r0plus_exit.txt"
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
python3 -B -c 'import json; r=json.load(open("artifacts/validation/2026-10-24/e3_onroad_v1/g1/report.json")); assert r["run_valid"], "First run integrity invalid; do not compare"'
set +e
python3 -B scripts/validate_g1_traffic.py --case slow_lead --seed 0 --mode r0plus --record-follow-decisions --occupancy-timing per_ray_candidate --actor-overlay config/tests/e3_onroad_encounter_v1.json --output "$base/r0plus" > "$base/r0plus_runner.log" 2>&1
status=$?
set -e
printf '%s\n' "$status" > "$base/r0plus_exit.txt"
tail -8 "$base/r0plus_runner.log"
exit "$status"
