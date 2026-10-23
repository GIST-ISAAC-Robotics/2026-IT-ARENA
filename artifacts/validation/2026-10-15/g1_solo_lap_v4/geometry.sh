#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-15/g1_solo_lap_v4
test ! -e "$base/geometry.log"
python3 -B scripts/inspect_corner_geometry.py "$base/run/follow_decisions.jsonl" --seq 2166 2171 2176 2181 2201 2231 --output "$base/geometry" > "$base/geometry.log" 2>&1
tail -1 "$base/geometry.log"
