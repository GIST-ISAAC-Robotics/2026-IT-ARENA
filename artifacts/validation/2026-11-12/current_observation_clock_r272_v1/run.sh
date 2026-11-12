#!/usr/bin/env bash
set -eo pipefail
cd /home/q/it_arena_conditional_r271_20261112/worktree
source /opt/ros/humble/setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests:${PYTHONPATH:-}"
/usr/bin/python3 -m pytest -q artifacts/validation/2026-11-12/current_observation_clock_r272_v1/test_timeline.py > artifacts/validation/2026-11-12/current_observation_clock_r272_v1/jetson_tests.log 2>&1
cat artifacts/validation/2026-11-12/current_observation_clock_r272_v1/jetson_tests.log
for mode in held delay20; do
 /usr/bin/python3 -B artifacts/validation/2026-11-12/current_observation_clock_r272_v1/timed_replay.py --trace artifacts/validation/2026-11-12/retained_route_execution_r266_v1/prefix_trace.jsonl --mode "$mode" --plan-ms 0 --current-ms 0 --out "artifacts/validation/2026-11-12/current_observation_clock_r272_v1/jetson_$mode" > "artifacts/validation/2026-11-12/current_observation_clock_r272_v1/jetson_$mode.log" 2>&1
 cat "artifacts/validation/2026-11-12/current_observation_clock_r272_v1/jetson_$mode.log"
done
