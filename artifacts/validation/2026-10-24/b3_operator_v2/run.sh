#!/usr/bin/env bash
set -e
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=123
export ROS_LOCALHOST_ONLY=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PYTHONPATH"
out=artifacts/validation/2026-10-24/b3_operator_v2
sha256sum scripts/race_operator.py tests/test_race_operator.py tests/test_race_operator_ros.py src/arena_autonomy/arena_autonomy/race_session.py src/arena_autonomy/arena_autonomy/local_pursuit.py src/arena_autonomy/arena_autonomy/wall_follow.py > "$out/before.sha256"
zip -q "$out/source.zip" scripts/race_operator.py tests/test_race_operator.py tests/test_race_operator_ros.py src/arena_autonomy/arena_autonomy/race_session.py src/arena_autonomy/arena_autonomy/local_pursuit.py src/arena_autonomy/arena_autonomy/wall_follow.py
set +e
python3 -m pytest -q tests/test_race_operator.py tests/test_race_operator_ros.py tests/test_race_session.py tests/test_race_session_node.py tests/test_race_permission.py --junitxml="$out/junit.xml" > "$out/tests.log" 2>&1
code=$?
set -e
printf '%s\n' "$code" > "$out/exit.txt"
sha256sum -c "$out/before.sha256" > "$out/source_check.txt"
exit "$code"
