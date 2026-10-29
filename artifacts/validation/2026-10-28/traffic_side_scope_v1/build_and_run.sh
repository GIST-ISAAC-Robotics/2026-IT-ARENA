#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-28/traffic_side_scope_v1
colcon build --packages-select arena_autonomy --symlink-install --event-handlers console_direct+ > "$base/build.log" 2>&1
python3 -B "$base/run1_snapshot.py" > "$base/run1_snapshot.log" 2>&1
bash "$base/run1_trial.sh"
python3 -B "$base/summarize_run.py" > "$base/summarize_run.log" 2>&1
