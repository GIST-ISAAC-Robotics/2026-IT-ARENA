#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
out=artifacts/validation/2026-10-28/rotation_stress_v1/inner_prefix_checks_v1.log
test ! -e "$out"
/usr/bin/python3 -B -m pytest -q -p no:cacheprovider tests/test_short_inner_prefix.py tests/test_short_prefix_connection.py tests/test_short_prefix_origin.py tests/test_corner_corridor_module.py tests/test_corridor_observer.py tests/test_corridor_roles.py > "$out" 2>&1
cat "$out"
