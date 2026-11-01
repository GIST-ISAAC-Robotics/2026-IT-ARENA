#!/usr/bin/env bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
python3 -B artifacts/validation/2026-11-01/corner_approach_v1/run_ten.py
result=$?
printf 'ACTUAL_EXIT_CODE=%s\n' "$result" > artifacts/validation/2026-11-01/corner_approach_v1/solo_10kmh_exit.log
exit "$result"