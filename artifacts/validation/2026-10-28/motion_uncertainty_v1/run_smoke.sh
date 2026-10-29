#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-28/motion_uncertainty_v1
test ! -e "$base/smoke_v1.log"
/usr/bin/python3 -B "$base/smoke_replay.py" > "$base/smoke_v1.log" 2>&1
cat "$base/smoke_v1.log"
