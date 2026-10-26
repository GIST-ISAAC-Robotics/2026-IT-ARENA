#!/usr/bin/env bash
set -o pipefail
repo="/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local"
cd "$repo" || exit 90
unset PYTHONPATH
source /opt/ros/jazzy/setup.bash || exit 90
source install/setup.bash || exit 90
printf "ROS_DISTRO=%s\nPYTHONPATH=%s\n" "${ROS_DISTRO:-}" "${PYTHONPATH:-}" > artifacts/validation/2026-10-26/m4_second_start_v1/run5_second_visual_start/environment.txt
O="$repo/artifacts/validation/2026-10-26/m4_second_start_v1/run5_second_visual_start"
echo "START $(date -Is)" > "$O/preflight_status.txt"
python3 "$O/preflight.py" "$O" > "$O/preflight_stdout.txt" 2> "$O/preflight_stderr.txt"
pre=$?
printf "%s\n" "$pre" > "$O/preflight_exit_code.txt"
echo "END $(date -Is)" >> "$O/preflight_status.txt"
if [ "$pre" -ne 0 ]; then exit "$pre"; fi
cmd='python3 -B scripts/validate_g1_traffic.py --case stop_restart --seed 0 --mode g1 --race-operation-test new_race_restart --record-follow-decisions --record-braking-diagnostics --boundary-evidence corner_distance_candidate --occupancy-timing per_ray_candidate --corridor-fallback --config config/tests/g1_traffic_v1.json --output artifacts/validation/2026-10-26/m4_second_start_v1/run5_second_visual_start/run'
printf "%s\n" "$cmd" > "$O/command.txt"
if [ -e "$O/run" ]; then echo "OUTPUT_ALREADY_EXISTS" > "$O/guard_failure.txt"; exit 91; fi
echo "START $(date -Is)" > "$O/run_status.txt"
python3 -B scripts/validate_g1_traffic.py --case stop_restart --seed 0 --mode g1 --race-operation-test new_race_restart --record-follow-decisions --record-braking-diagnostics --boundary-evidence corner_distance_candidate --occupancy-timing per_ray_candidate --corridor-fallback --config config/tests/g1_traffic_v1.json --output artifacts/validation/2026-10-26/m4_second_start_v1/run5_second_visual_start/run > "$O/run_stdout.txt" 2> "$O/run_stderr.txt"
code=$?
printf "%s\n" "$code" > "$O/run_exit_code.txt"
echo "END $(date -Is)" >> "$O/run_status.txt"
exit "$code"
