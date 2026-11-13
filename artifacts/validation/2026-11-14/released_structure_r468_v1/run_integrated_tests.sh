#!/usr/bin/env bash
set -euo pipefail
set +u
source /opt/ros/jazzy/setup.bash
set -u
export PYTHONDONTWRITEBYTECODE=1
cd "$(dirname "$0")/../../../.."
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/tests:${PYTHONPATH:-}"
python3 -m pytest -q tests/test_released_structure_region.py tests/test_current_structure_shadow.py tests/test_pass_release_trial.py tests/test_known_structure_policy.py tests/test_rear_structure_memory.py tests/test_guard_curve_trial.py tests/test_current_path_recheck.py > artifacts/validation/2026-11-14/released_structure_r468_v1/integration_pytest_v1.log 2>&1
