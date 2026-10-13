#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/scripts:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-13/follow_trace_v1
test ! -e "$base/tests.log"
python3 -B -m pytest -q tests/test_follow_trace.py tests/test_local_follow.py tests/test_motor_observability.py tests/test_queued_log.py > "$base/tests.log" 2>&1
tail -15 "$base/tests.log"
