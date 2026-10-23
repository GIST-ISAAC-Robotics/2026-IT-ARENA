#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_vehicle_interface:$PWD/src/arena_autonomy:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-15/g1_corridor_integration_v1
test ! -e "$base/tests.log"
python3 -B -m pytest -q tests > "$base/tests.log" 2>&1
python3 -B scripts/replay_follow_trace.py artifacts/validation/2026-10-15/g1_solo_lap_v4/run/follow_decisions.jsonl --output "$base/baseline_replay.json" > "$base/baseline_replay.log" 2>&1
tail -3 "$base/tests.log"
tail -1 "$base/baseline_replay.log"
