#!/usr/bin/env bash
set -e
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-31/g2_wall_reference_probe_v1
test ! -e "$base/run1.log"
python3 -B "$base/probe.py" > "$base/run1.log" 2>&1
