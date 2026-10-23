#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
python3 -c 'import ackermann_msgs; print(ackermann_msgs.__file__)' > artifacts/validation/2026-10-16/g1_recent_observation_regression_v2/environment.log 2>&1
python3 -B -m pytest -q tests > artifacts/validation/2026-10-16/g1_recent_observation_regression_v2/tests.log 2>&1
tail -3 artifacts/validation/2026-10-16/g1_recent_observation_regression_v2/tests.log
