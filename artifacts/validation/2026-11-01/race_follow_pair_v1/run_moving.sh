#!/usr/bin/env bash
set -o pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
python3 -B scripts/validate_g1_traffic.py --config artifacts/validation/2026-11-01/race_follow_pair_v1/stop_restart.json --mode g1g2 --case stop_restart --seed 0 --record-follow-decisions --handoff-reserve moving --output artifacts/validation/2026-11-01/race_follow_pair_v1/moving 2>&1 | tee artifacts/validation/2026-11-01/race_follow_pair_v1/moving_console.log
result=$?
printf 'ACTUAL_EXIT_CODE=%s\n' "$result" >> artifacts/validation/2026-11-01/race_follow_pair_v1/moving_console.log
exit "$result"