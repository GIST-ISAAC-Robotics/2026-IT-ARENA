#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/tests:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export ROS_DOMAIN_ID=223
export ROS_LOCALHOST_ONLY=1
art=artifacts/validation/2026-11-14/radial_motion_reacquire_r552_v1/recheck_verification
python3 -B -m pytest -q tests/test_reacquire_radial_recheck.py --junitxml="$art/junit.xml" > "$art/pytest.log" 2>&1
