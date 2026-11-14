#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
python3 artifacts/validation/2026-11-14/guard_memory_relevance_r538_v1/replay.py --name common_replay
python3 artifacts/validation/2026-11-14/guard_memory_relevance_r538_v1/replay.py --stage artifacts/validation/2026-11-14/guard_memory_relevance_r538_v1/stage --name candidate_replay
