#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-12/solver_compare_v1
test ! -e "$base/diagnosis.log"
python3 -B "$base/diagnose.py" > "$base/diagnosis.log" 2>&1
tail -80 "$base/diagnosis.log"
# 실행 중인 redirect 로그를 자신의 artifact hash에 넣지 않는다.
python3 -B "$base/finalize.py"
