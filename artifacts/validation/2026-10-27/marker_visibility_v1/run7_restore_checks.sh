#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-27/marker_visibility_v1
/usr/bin/python3 -B "$base/run7_snapshot.py" > "$base/run7_snapshot.log" 2>&1
set +e
/usr/bin/python3 -B -m pytest -q tests > "$base/run7_all_tests.log" 2>&1
unit_exit=$?
/usr/bin/python3 -B scripts/validate_sensor_input_ros.py --partial-observation-policy bounded_local_v3 --output "$base/run7_ros" > "$base/run7_ros.log" 2>&1
ros_exit=$?
/usr/bin/python3 -B "$base/run7_snapshot.py" --check > "$base/run7_source_check.log" 2>&1
source_exit=$?
set -e
printf 'unit=%s\nros=%s\nsource=%s\n' "$unit_exit" "$ros_exit" "$source_exit" > "$base/run7_exit.txt"
tail -8 "$base/run7_all_tests.log"
if (( unit_exit != 0 || ros_exit != 0 || source_exit != 0 )); then exit 1; fi
