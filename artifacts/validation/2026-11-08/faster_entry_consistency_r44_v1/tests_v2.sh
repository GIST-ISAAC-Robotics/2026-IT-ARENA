#!/usr/bin/env bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
A=artifacts/validation/2026-11-08/faster_entry_consistency_r44_v1
python3 -B -m pytest -q -p no:cacheprovider --tb=short tests/test_faster_entry_consistency.py > "$A/tests_v2.log" 2>&1
rc=$?
printf '%s\n' "$rc" > "$A/tests_v2.exit"
exit "$rc"
