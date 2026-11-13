#!/usr/bin/env bash
set -u
export PYTHONDONTWRITEBYTECODE=1
cd "$(dirname "$0")"
export PYTHONPATH="$PWD/stage/src/arena_autonomy:$PWD/stage/src/arena_vehicle_interface"
ROOT="$(cd ../../../.. && pwd)"
python3 -B -m pytest -q -p no:cacheprovider "$ROOT/tests/test_runtime_dual_contract.py::test_generation_invalidation_clears_deceleration_cap" "$ROOT/tests/test_runtime_dual_contract.py::test_stop_between_selection_and_publish_cancels_pending_ack" > pytest_stop_v1.log 2>&1
rc=$?
printf '%s\n' "$rc" > pytest_stop_v1.exit
exit "$rc"
