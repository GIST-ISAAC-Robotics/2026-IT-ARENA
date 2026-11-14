#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
python3 artifacts/validation/2026-11-14/field_guard_replay_r531_v1/stage_replay.py --name baseline_extended
python3 artifacts/validation/2026-11-14/field_guard_replay_r531_v1/stage_replay.py --stage artifacts/validation/2026-11-14/guard_memory_crawl_r528_v1/stage --name r528_extended