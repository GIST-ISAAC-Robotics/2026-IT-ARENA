#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/tests:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
python3 -m pytest -q tests/test_short_prefix_origin.py tests/test_short_prefix_connection.py tests/test_corner_follow.py tests/test_corridor_roles.py tests/test_g1_dense_stop.py tests/test_physics_step_option.py tests/test_source_contract_helpers.py tests/test_source_provenance_r0_plus_b3.py
