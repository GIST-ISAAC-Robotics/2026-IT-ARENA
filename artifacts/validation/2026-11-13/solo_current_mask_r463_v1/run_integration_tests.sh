#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/tests:$PWD/scripts:$PWD/artifacts/validation/2026-11-13/complete_plan_trace_r449_v1/test_dependencies${PYTHONPATH:+:$PYTHONPATH}"
a=artifacts/validation/2026-11-13/solo_current_mask_r463_v1
set +e
python3 -B -m pytest -q -p no:cacheprovider tests/test_current_structure_shadow.py tests/test_known_structure_policy.py tests/test_rear_structure_memory.py tests/test_current_path_recheck.py tests/test_current_wall_refresh.py tests/test_traffic_boundary_refresh.py > "$a/integration_pytest_v1.log" 2>&1
code=$?
printf '%s\n' "$code" > "$a/integration_pytest_v1.exit"
tail -c 8000 "$a/integration_pytest_v1.log"
exit "$code"
