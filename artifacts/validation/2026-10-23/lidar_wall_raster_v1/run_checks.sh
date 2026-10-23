#!/usr/bin/env bash
set -eo pipefail
cd -- /mnt/c/Users/Jinhyeong/Documents/ChatGPT/'IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
out=artifacts/validation/2026-10-23/lidar_wall_raster_v1
test ! -e "$out/checks_exit.txt"
finish() { result=$?; printf '%s\n' "$result" > "$out/checks_exit.txt"; }
trap finish EXIT
python3 -B "$out/run_fixture.py" > "$out/run.log" 2>&1
