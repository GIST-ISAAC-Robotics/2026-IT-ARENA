#!/usr/bin/env bash
set -eo pipefail
cd /home/q/it_arena_conditional_r271_20261112/worktree
source /opt/ros/humble/setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests:${PYTHONPATH:-}"
/usr/bin/python3 -m pytest -q artifacts/validation/2026-11-12/conditional_route_execution_r271_v1/test_submission.py artifacts/validation/2026-11-12/conditional_route_execution_r271_v1/test_route_delivery.py > artifacts/validation/2026-11-12/conditional_route_execution_r271_v1/jetson_tests.log 2>&1
cat artifacts/validation/2026-11-12/conditional_route_execution_r271_v1/jetson_tests.log
/usr/bin/python3 -B artifacts/validation/2026-11-12/conditional_route_execution_r271_v1/timed_replay.py --trace artifacts/validation/2026-11-12/retained_route_execution_r266_v1/prefix_trace.jsonl --mode new_only --plan-ms 0 --current-ms 0 --out artifacts/validation/2026-11-12/conditional_route_execution_r271_v1/jetson_new_only > artifacts/validation/2026-11-12/conditional_route_execution_r271_v1/jetson_new_only.log 2>&1
cat artifacts/validation/2026-11-12/conditional_route_execution_r271_v1/jetson_new_only.log
/usr/bin/python3 -B artifacts/validation/2026-11-12/conditional_route_execution_r271_v1/timed_replay.py --trace artifacts/validation/2026-11-12/retained_route_execution_r266_v1/prefix_trace.jsonl --mode scheduled --plan-ms 0 --current-ms 0 --out artifacts/validation/2026-11-12/conditional_route_execution_r271_v1/jetson_scheduled > artifacts/validation/2026-11-12/conditional_route_execution_r271_v1/jetson_scheduled.log 2>&1
cat artifacts/validation/2026-11-12/conditional_route_execution_r271_v1/jetson_scheduled.log
