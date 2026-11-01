#!/usr/bin/env bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
python3 -B scripts/surface_contact_eval.py artifacts/validation/2026-11-01/race_follow_pair_v1/slow_moving --output artifacts/validation/2026-11-01/race_follow_pair_v1/slow_moving_surface.json > artifacts/validation/2026-11-01/race_follow_pair_v1/slow_moving_surface.log 2>&1
result=$?
printf 'ACTUAL_EXIT_CODE=%s\n' "$result" >> artifacts/validation/2026-11-01/race_follow_pair_v1/slow_moving_surface.log
if [ "$result" -ne 0 ]; then exit "$result"; fi
python3 -B artifacts/validation/2026-11-01/race_follow_pair_v1/slow_summary.py artifacts/validation/2026-11-01/race_follow_pair_v1/slow_moving artifacts/validation/2026-11-01/race_follow_pair_v1/slow_moving_summary.json > artifacts/validation/2026-11-01/race_follow_pair_v1/slow_moving_summary.log 2>&1
result=$?
printf 'ACTUAL_EXIT_CODE=%s\n' "$result" >> artifacts/validation/2026-11-01/race_follow_pair_v1/slow_moving_summary.log
exit "$result"