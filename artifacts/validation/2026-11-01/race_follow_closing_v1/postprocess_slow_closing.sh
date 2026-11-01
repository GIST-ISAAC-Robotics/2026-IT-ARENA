#!/usr/bin/env bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
python3 -B scripts/surface_contact_eval.py artifacts/validation/2026-11-01/race_follow_closing_v1/slow --output artifacts/validation/2026-11-01/race_follow_closing_v1/slow_surface.json > artifacts/validation/2026-11-01/race_follow_closing_v1/slow_surface.log 2>&1
result=$?
printf 'ACTUAL_EXIT_CODE=%s\n' "$result" >> artifacts/validation/2026-11-01/race_follow_closing_v1/slow_surface.log
if [ "$result" -ne 0 ]; then exit "$result"; fi
python3 -B artifacts/validation/2026-11-01/race_follow_closing_v1/compare.py artifacts/validation/2026-11-01/race_follow_pair_v1/slow_moving artifacts/validation/2026-11-01/race_follow_closing_v1/slow artifacts/validation/2026-11-01/race_follow_closing_v1/slow_compare.json > artifacts/validation/2026-11-01/race_follow_closing_v1/slow_compare.log 2>&1
result=$?
printf 'ACTUAL_EXIT_CODE=%s\n' "$result" >> artifacts/validation/2026-11-01/race_follow_closing_v1/slow_compare.log
exit "$result"