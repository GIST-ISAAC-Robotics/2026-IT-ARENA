#!/usr/bin/env bash
set -u
export PYTHONDONTWRITEBYTECODE=1
cd "$(dirname "$0")"
export PYTHONPATH="$PWD/stage/src/arena_autonomy:$PWD/stage/src/arena_vehicle_interface"
python3 -B -m pytest -q -p no:cacheprovider test_current_route_priority.py > pytest_v1.log 2>&1
rc=$?
printf '%s\n' "$rc" > pytest_v1.exit
exit "$rc"
