#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-27/g1_partial_integration_v1
test ! -e "$base/run6_surface_probe.log"
/usr/bin/python3 -B "$base/run6_surface_probe.py" > "$base/run6_surface_probe.log" 2>&1
tail -2 "$base/run6_surface_probe.log"
