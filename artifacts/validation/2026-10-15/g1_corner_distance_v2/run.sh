#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-15/g1_corner_distance_v2
test ! -e "$base/probe.log"
test ! -e "$base/tests.log"
python3 -B -m pytest -q tests/test_corner_distance_probe.py > "$base/tests.log" 2>&1
python3 -B "$base/probe.py" > "$base/probe.log" 2>&1
echo completed
