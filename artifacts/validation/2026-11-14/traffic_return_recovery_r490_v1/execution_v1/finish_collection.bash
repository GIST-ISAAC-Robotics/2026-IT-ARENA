#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
a='artifacts/validation/2026-11-14/traffic_return_recovery_r490_v1'
python3 -B "$a/collect_result.py"
python3 -B "$a/prepare.py" --check
python3 -B "$a/execution_v1/check_owned_processes.py"
