#!/usr/bin/env bash
set -o pipefail
cd "/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local" || exit 90
if [ -e "/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-01/race_continuity_v1/solo_1mps" ]; then printf 'OUTPUT_EXISTS: %s\n' "/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-01/race_continuity_v1/solo_1mps"; exit 93; fi
source /opt/ros/jazzy/setup.bash || exit 91
source install/local_setup.bash || exit 92
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
printf '%s\n' '$ python3 -B scripts/validate_basic_autonomy.py --autonomy-mode local_pursuit --g1-follow --g1-boundary-evidence corner_distance_candidate --g1-occupancy-timing per_ray_candidate --record-follow-decisions --g1-corridor-fallback --partial-observation-policy bounded_local_v4 --speed-profile g1_compare_1mps --disable-speed-bump --lidar-rate-hz 10 --lidar-acquisition sequential --lidar-compensation both --actuation-mode virtual_mcu --control-rate-hz 50 --render-backend wsl_nvidia --shutdown-mode service --grid-slot 3 --laps 1 --max-sim-seconds 110 --wall-timeout-s 1200 --stop-wall-timeout-s 90 --output artifacts/validation/2026-11-01/race_continuity_v1/solo_1mps'
python3 -B scripts/validate_basic_autonomy.py --autonomy-mode local_pursuit --g1-follow --g1-boundary-evidence corner_distance_candidate --g1-occupancy-timing per_ray_candidate --record-follow-decisions --g1-corridor-fallback --partial-observation-policy bounded_local_v4 --speed-profile g1_compare_1mps --disable-speed-bump --lidar-rate-hz 10 --lidar-acquisition sequential --lidar-compensation both --actuation-mode virtual_mcu --control-rate-hz 50 --render-backend wsl_nvidia --shutdown-mode service --grid-slot 3 --laps 1 --max-sim-seconds 110 --wall-timeout-s 1200 --stop-wall-timeout-s 90 --output "/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-01/race_continuity_v1/solo_1mps" 2>&1 | tee artifacts/validation/2026-11-01/race_continuity_v1/solo_1mps_console.log
rc=${PIPESTATUS[0]}
printf 'ACTUAL_EXIT_CODE=%s\n' "$rc" | tee -a artifacts/validation/2026-11-01/race_continuity_v1/solo_1mps_console.log
exit "$rc"
