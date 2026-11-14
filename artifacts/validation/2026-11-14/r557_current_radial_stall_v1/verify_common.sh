#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/tests:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export ROS_DOMAIN_ID=223
export ROS_LOCALHOST_ONLY=1
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_STATIC_PEERS=''
art=artifacts/validation/2026-11-14/r557_current_radial_stall_v1
python3 -B -m pytest -q tests/test_observed_crawl_recheck.py tests/test_reacquire_radial_recheck.py tests/test_current_wall_reacquire.py tests/test_lidar_wall_bearings.py tests/test_current_route_proof.py tests/test_current_path_recheck.py tests/test_guard_observation_rebase.py tests/test_junction_priority.py tests/test_independent_guard_route.py tests/test_lidar_safety.py tests/test_swept_radial_motion.py --junitxml="$art/common_tests.xml" > "$art/common_pytest.log" 2>&1
