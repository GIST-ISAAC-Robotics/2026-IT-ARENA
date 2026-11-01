#!/usr/bin/env bash
set -o pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
python3 -B scripts/validate_basic_autonomy.py --autonomy-mode local_pursuit --g1-follow --g1-boundary-evidence corner_distance_candidate --g1-occupancy-timing per_ray_candidate --record-follow-decisions --g1-corridor-fallback --g1-race-path --partial-observation-policy bounded_local_v4 --speed-profile g1_open_2p0_approach --disable-speed-bump --lidar-rate-hz 10 --lidar-acquisition sequential --lidar-compensation both --actuation-mode virtual_mcu --control-rate-hz 50 --render-backend wsl_nvidia --shutdown-mode service --grid-slot 3 --laps 1 --max-sim-seconds 110 --wall-timeout-s 1200 --stop-wall-timeout-s 90 --output "$PWD/artifacts/validation/2026-11-01/corner_approach_v1/solo_2p0" 2>&1 | tee artifacts/validation/2026-11-01/corner_approach_v1/solo_2p0_console.log
result=$?
printf 'ACTUAL_EXIT_CODE=%s\n' "$result" >> artifacts/validation/2026-11-01/corner_approach_v1/solo_2p0_console.log
exit "$result"