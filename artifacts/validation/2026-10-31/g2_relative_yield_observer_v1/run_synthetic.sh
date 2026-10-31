#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
# ROS의 Python 경로를 덮어쓰지 않고 저장소 경로만 앞에 추가한다.
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-31/g2_relative_yield_observer_v1
test ! -e "$base/closed_run2"
test ! -e "$base/closed_run2.log"
set +e
python3 -B -c 'import sys,ackermann_msgs; print(sys.executable); print(ackermann_msgs.__file__)' > "$base/closed_run2_environment.log" 2>&1
env_exit=$?
if [ "$env_exit" != 0 ]; then printf 'environment=%s\n' "$env_exit" > "$base/closed_run2_exit.txt"; exit "$env_exit"; fi
python3 -B "$base/closed_loop.py" --output "$base/closed_run2" > "$base/closed_run2.log" 2>&1
result=$?
printf 'synthetic=%s\n' "$result" > "$base/closed_run2_exit.txt"
exit "$result"
