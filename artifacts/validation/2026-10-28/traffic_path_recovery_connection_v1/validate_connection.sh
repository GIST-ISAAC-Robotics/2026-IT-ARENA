#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-28/traffic_path_recovery_connection_v1
test ! -e "$base/off_replay.json"
test ! -e "$base/connection_comparison"
python3 -B scripts/replay_follow_trace.py artifacts/validation/2026-10-28/traffic_release_fringe_v1/run1_course/follow_decisions.jsonl --output "$base/off_replay.json" > "$base/off_replay.log" 2>&1
python3 -B "$base/verify_connection.py" > "$base/connection_comparison.log" 2>&1
colcon build --packages-select arena_autonomy arena_bringup --symlink-install > "$base/build.log" 2>&1
source install/local_setup.bash
python3 -B -c 'from arena_autonomy.local_follow import FollowConfig; from arena_autonomy.traffic_path_recovery_trial import SuspendedPassageSelector; import inspect; print(inspect.getfile(SuspendedPassageSelector)); print(FollowConfig().traffic_path_recovery_trial)' > "$base/installed_import.log"
