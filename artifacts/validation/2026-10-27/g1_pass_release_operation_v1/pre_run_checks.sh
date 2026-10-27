#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
set -u
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-27/g1_pass_release_operation_v1
test ! -e "$base/related_tests.log"
set +e
/usr/bin/python3 -B -m pytest -q -p no:cacheprovider $(cat "$base/related_tests.txt") > "$base/related_tests.log" 2>&1
tests_exit=$?
colcon build --symlink-install --packages-select arena_autonomy arena_bringup > "$base/build.log" 2>&1
build_exit=$?
set -e
printf 'tests=%s\nbuild=%s\n' "$tests_exit" "$build_exit" > "$base/pre_run_exit.txt"
