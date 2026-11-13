#!/usr/bin/env bash
set -euo pipefail
export PYTHONDONTWRITEBYTECODE=1
cd "$(dirname "$0")"
export PYTHONPATH="$PWD/stage/src/arena_autonomy:$PWD/stage/src/arena_vehicle_interface:$PWD/stage/tests"
python3 -m pytest -q stage/tests/test_released_structure_region.py stage/tests/test_current_structure_shadow.py stage/tests/test_pass_release_trial.py stage/tests/test_known_structure_policy.py stage/tests/test_rear_structure_memory.py stage/tests/test_guard_curve_trial.py stage/tests/test_current_path_recheck.py > regression_v1.log 2>&1
