#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="artifacts/validation/2026-11-11/parallel_rebase_r133_v1:src/arena_autonomy:src/arena_vehicle_interface:scripts:tests:${PYTHONPATH:-}"
/usr/bin/python3 -B artifacts/validation/2026-11-11/parallel_rebase_r133_v1/probe_saved.py > build/r133_saved.log 2>&1
