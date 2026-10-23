#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
python3 -B scripts/probe_recent_observation_follow.py artifacts/validation/2026-10-16/g1_corridor_solo_v1/run/follow_decisions.jsonl --output artifacts/validation/2026-10-16/g1_recent_observation_replay_v1/enabled > artifacts/validation/2026-10-16/g1_recent_observation_replay_v1/enabled.log 2>&1
tail -1 artifacts/validation/2026-10-16/g1_recent_observation_replay_v1/enabled.log
