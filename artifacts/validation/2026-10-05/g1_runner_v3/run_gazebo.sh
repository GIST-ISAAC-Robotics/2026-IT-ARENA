#!/usr/bin/env bash
repo='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
cd "$repo" || exit 90
source /opt/ros/jazzy/setup.bash || exit 91
source install/local_setup.bash || exit 92
export PYTHONPATH="$repo/src/arena_autonomy:$repo/src/arena_vehicle_interface${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
logs='artifacts/validation/2026-10-05/g1_runner_v3'
/usr/bin/python3 -B scripts/validate_basic_autonomy.py --autonomy-mode local_pursuit --g1-follow --lidar-acquisition sequential --lidar-compensation both --control-rate-hz 50 --render-backend wsl_nvidia --collision-detector configured --shutdown-mode service --sensor-wall-timeout-s 30 --disable-speed-bump --red-duration-s 1.0 --wall-timeout-s 1200 --stop-wall-timeout-s 120 --speed-profile exploratory --target-progress-m 12 --max-sim-seconds 60 --grid-slot 3 --actuation-mode virtual_mcu --output artifacts/validation/2026-10-05/g1_gazebo_solo_v2 > "$logs/gazebo.stdout.log" 2> "$logs/gazebo.stderr.log"
rc=$?
printf '%s\n' "$rc" > "$logs/gazebo.exitcode"
tail -20 "$logs/gazebo.stdout.log"
printf 'returncode=%s\n' "$rc"
exit "$rc"
