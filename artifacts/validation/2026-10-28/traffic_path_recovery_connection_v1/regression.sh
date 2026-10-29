#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
python3 -B -m pytest -q -p no:cacheprovider tests/test_traffic_path_recovery.py tests/test_traffic_release_fringe.py tests/test_corridor_roles.py tests/test_corridor_observer.py tests/test_corridor_follow.py tests/test_short_prefix_connection.py tests/test_pass_release_trial.py tests/test_guard_curve_trial.py tests/test_source_provenance_r0_plus_b3.py tests/test_corner_follow.py tests/test_g1_dense_stop.py tests/test_physics_step_option.py
