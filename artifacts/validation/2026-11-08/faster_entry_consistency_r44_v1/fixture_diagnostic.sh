#!/usr/bin/env bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
A=artifacts/validation/2026-11-08/faster_entry_consistency_r44_v1
python3 -B "$A/fixture_diagnostic.py" > "$A/fixture_diagnostic.log" 2>&1
